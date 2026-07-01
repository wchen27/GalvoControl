// main.cpp
// Minimal Dear ImGui motor control for the GalvoCam MR-J5 servos (RSI RMP / EtherCAT).
//
// Style follows moments-behavior/orange: snake_case functions with gui_/draw_ prefixes,
// PascalCase types, g_ globals, 1TBS braces, create_new_frame()/render_a_frame() loop.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include <GLFW/glfw3.h>

#include "rsi.h"

using namespace RSI::RapidCode;

// Path passed to CreateFromSoftware(), injected by CMake per-platform (RMP_RTA_PATH).
// Windows: the RMP install dir (holds RMP.rta). Linux: usually empty -> RapidServer default.
#ifndef RMP_RTA_PATH
#define RMP_RTA_PATH ""
#endif

// Counts per motor revolution (encoder resolution). UserUnits = this / 360 so the UI
// works in degrees. If a commanded angle doesn't match the physical rotation, adjust
// "counts/rev" live in the UI - it's 2^(encoder bits): 8388608 (2^23), 16777216 (2^24),
// 33554432 (2^25), 67108864 (2^26 - HK-KT, confirmed on this rig).
static double g_counts_per_rev = 67108864.0;

// ---------------------------------------------------------------------------
// types
// ---------------------------------------------------------------------------
struct MotorAxis {
    Axis*   axis   = nullptr;
    int32_t number = 0;

    // cached readback (degrees / deg-per-sec), refreshed each frame
    double actual_position  = 0.0;
    double command_position = 0.0;
    double actual_velocity  = 0.0;
    RSIState state          = RSIState::RSIStateIDLE;
    bool   amp_enabled      = false;
    std::string source_name;                 // reason for an ERROR state

    // UI-editable move parameters (degrees)
    float target_deg = 90.0f;   // absolute target
    float jog_deg    = 10.0f;   // relative jog increment
    float velocity   = 90.0f;   // deg/s
    float accel      = 360.0f;  // deg/s^2
    float jerk_pct   = 50.0f;   // %

    // safety limits (degrees)
    float error_limit_deg = 5.0f;      // following-error trip (runaway protection)
    float pos_min_deg     = -180.0f;   // software travel limits
    float pos_max_deg     = 180.0f;
    bool  limits_enabled  = false;     // travel limits off until real range is set + coupled

    // speed limits: commanded velocity/accel are clamped to these
    float max_velocity    = 360.0f;    // deg/s
    float max_accel       = 3600.0f;   // deg/s^2
};

// ---------------------------------------------------------------------------
// globals
// ---------------------------------------------------------------------------
static MotionController*      g_controller = nullptr;
static std::vector<MotorAxis> g_axes;
static std::string            g_status = "not connected";

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

// Run a RapidCode call, capturing any RsiError into g_status instead of taking down the UI.
template <class Fn>
static void motor_try(const char* what, Fn&& fn) {
    try {
        fn();
    } catch (const RsiError& e) {
        g_status = std::string(what) + ": " + e.what();
    }
}

static const char* state_name(RSIState state) {
    switch (state) {
        case RSIState::RSIStateIDLE:           return "IDLE";
        case RSIState::RSIStateMOVING:         return "MOVING";
        case RSIState::RSIStateSTOPPING:       return "STOPPING";
        case RSIState::RSIStateSTOPPED:        return "STOPPED";
        case RSIState::RSIStateSTOPPING_ERROR: return "STOPPING_ERROR";
        case RSIState::RSIStateERROR:          return "ERROR";
        default:                               return "?";
    }
}

// ---------------------------------------------------------------------------
// motor system
// ---------------------------------------------------------------------------

// Push the axis' safety limits to the drive. Call after UserUnitsSet and whenever
// the user edits them. Values are in degrees.
static void motor_apply_limits(MotorAxis& m) {
    motor_try("apply limits", [&] {
        // Following-error limit: abort (drop torque) on a runaway / stall.
        m.axis->ErrorLimitTriggerValueSet(m.error_limit_deg);
        m.axis->ErrorLimitActionSet(RSIAction::RSIActionABORT);

        // Software travel limits: e-stop before the mirror can reach a mechanical stop.
        RSIAction act = m.limits_enabled ? RSIAction::RSIActionE_STOP : RSIAction::RSIActionNONE;
        m.axis->SoftwarePosLimitTriggerValueSet(m.pos_max_deg);
        m.axis->SoftwarePosLimitActionSet(act);
        m.axis->SoftwareNegLimitTriggerValueSet(m.pos_min_deg);
        m.axis->SoftwareNegLimitActionSet(act);

        // No physical limit switches are wired on this rig, so the hardware limit
        // inputs float to "tripped" and abort every move (this is the fault seen in
        // RapidSetup). Disable them; the software travel limits above are the real
        // protection once the motors are coupled to the mirrors.
        m.axis->HardwarePosLimitActionSet(RSIAction::RSIActionNONE);
        m.axis->HardwareNegLimitActionSet(RSIAction::RSIActionNONE);
    });
}

static void motor_connect() {
    motor_try("connect", [&] {
        // RapidCode locates the RMP runtime from this path (Windows: install dir with
        // RMP.rta). Set by CMake via RMP_RTA_PATH; empty -> default CreateFromSoftware()
        // (Linux connects to the running RapidServer).
        const char* rtaPath = RMP_RTA_PATH;
        g_controller = (rtaPath && rtaPath[0])
            ? MotionController::CreateFromSoftware(rtaPath)
            : MotionController::CreateFromSoftware();
        if (g_controller == nullptr) {
            g_status = "CreateFromSoftware returned null";
            return;
        }
        g_controller->ThrowExceptions(true);

        if (g_controller->NetworkStateGet() != RSINetworkState::RSINetworkStateOPERATIONAL) {
            g_controller->NetworkStart();
        }
        if (g_controller->NetworkStateGet() != RSINetworkState::RSINetworkStateOPERATIONAL) {
            std::string msg = "network not operational; nodes=" +
                              std::to_string(g_controller->NetworkNodeCountGet());
            int n = g_controller->NetworkLogMessageCountGet();
            for (int i = 0; i < n; i++) {
                msg += "\n  ";
                msg += g_controller->NetworkLogMessageGet(i);
            }
            g_status = msg;
            return;
        }

        int32_t count = g_controller->AxisCountGet();
        g_axes.clear();
        for (int32_t i = 0; i < count; i++) {
            MotorAxis m;
            m.number = i;
            m.axis   = g_controller->AxisGet(i);
            m.axis->ThrowExceptions(true);
            m.axis->UserUnitsSet(g_counts_per_rev / 360.0);   // work in degrees
            motor_apply_limits(m);                     // following-error + travel limits
            g_axes.push_back(m);
        }
        g_status = "connected: " + std::to_string(count) + " axes";
    });
}

static void motor_refresh(MotorAxis& m) {
    motor_try("read", [&] {
        m.actual_position  = m.axis->ActualPositionGet();
        m.command_position = m.axis->CommandPositionGet();
        m.actual_velocity  = m.axis->ActualVelocityGet();
        m.state            = m.axis->StateGet();
        m.amp_enabled      = m.axis->AmpEnableGet();
        m.source_name      = m.axis->SourceNameGet(m.axis->SourceGet());
    });
}

// Re-apply UserUnits (and the limits, which are stored in user units) to every axis.
static void motor_apply_units() {
    for (MotorAxis& m : g_axes) {
        motor_try("apply units", [&] { m.axis->UserUnitsSet(g_counts_per_rev / 360.0); });
        motor_apply_limits(m);
    }
}

// ---------------------------------------------------------------------------
// targeting: world 3D point -> pan/tilt motor angles
// ---------------------------------------------------------------------------
static constexpr double RAD2DEG = 57.29577951308232;

struct TargetConfig {
    float base[3] = {0.0f, 0.0f, 0.0f};   // GalvoCam optical center in world coords
    int   pan_axis  = 0;
    int   tilt_axis = 1;
    // motor_deg = sign * gaze_deg * scale + offset.
    // scale defaults 0.5: a mirror rotates half the gaze deflection (2x reflection).
    float pan_sign  = 1.0f, pan_scale  = 0.5f, pan_offset  = 0.0f;
    float tilt_sign = 1.0f, tilt_scale = 0.5f, tilt_offset = 0.0f;
};
static TargetConfig g_tgt;
static float g_target[3]     = {0.0f, 0.0f, 1000.0f};   // world target (units arbitrary)
static bool  g_moving_target = false;

struct PanTilt { double pan_motor, tilt_motor, az, el; };

// Unit frame assumed axis-aligned with world (+Z forward, +Y up, +X right), origin at base.
// A full orientation R can be added when units are placed in the arena.
static PanTilt targeting_solve(const float* target) {
    double dx = target[0] - g_tgt.base[0];
    double dy = target[1] - g_tgt.base[1];
    double dz = target[2] - g_tgt.base[2];
    double az = atan2(dx, dz) * RAD2DEG;                       // azimuth   -> pan gaze
    double el = atan2(dy, sqrt(dx * dx + dz * dz)) * RAD2DEG;  // elevation -> tilt gaze
    PanTilt pt;
    pt.az = az; pt.el = el;
    pt.pan_motor  = g_tgt.pan_sign  * az * g_tgt.pan_scale  + g_tgt.pan_offset;
    pt.tilt_motor = g_tgt.tilt_sign * el * g_tgt.tilt_scale + g_tgt.tilt_offset;
    return pt;
}

static void targeting_aim(const float* target) {
    PanTilt pt = targeting_solve(target);
    auto cmd = [&](int idx, double angle) {
        if (idx < 0 || idx >= (int)g_axes.size()) return;
        MotorAxis& m = g_axes[idx];
        float v = std::min(m.velocity, m.max_velocity);
        float a = std::min(m.accel, m.max_accel);
        motor_try("aim", [&] { m.axis->MoveSCurve(angle, v, a, a, m.jerk_pct); });
    };
    cmd(g_tgt.pan_axis,  pt.pan_motor);
    cmd(g_tgt.tilt_axis, pt.tilt_motor);
}

// ---------------------------------------------------------------------------
// gui
// ---------------------------------------------------------------------------
static void gui_draw_axis(MotorAxis& m) {
    motor_refresh(m);

    ImGui::PushID(m.number);
    ImGui::SeparatorText(("Axis " + std::to_string(m.number)).c_str());

    ImGui::Text("State: %s    Amp: %s", state_name(m.state), m.amp_enabled ? "ON" : "off");
    if (m.state == RSIState::RSIStateERROR || m.state == RSIState::RSIStateSTOPPING_ERROR) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Fault: %s",
                           m.source_name.empty() ? "(unknown)" : m.source_name.c_str());
    }
    ImGui::Text("Actual:   %10.3f deg",   m.actual_position);
    ImGui::Text("Command:  %10.3f deg",   m.command_position);
    ImGui::Text("Velocity: %10.3f deg/s", m.actual_velocity);

    // amp / fault controls
    if (m.amp_enabled) {
        if (ImGui::Button("Disable")) {
            motor_try("disable", [&] { m.axis->AmpEnableSet(false); });
        }
    } else {
        if (ImGui::Button("Enable")) {
            motor_try("enable", [&] {
                m.axis->ClearFaults();
                m.axis->AmpEnableSet(true);
            });
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Faults")) {
        motor_try("clear faults", [&] { m.axis->ClearFaults(); });
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop")) {
        motor_try("stop", [&] { m.axis->Abort(); });
    }

    // motion parameters
    ImGui::InputFloat("velocity (deg/s)", &m.velocity);
    ImGui::InputFloat("accel (deg/s^2)",  &m.accel);
    ImGui::SliderFloat("jerk %", &m.jerk_pct, 0.0f, 100.0f);

    // commanded velocity/accel are clamped to the speed limits (see "Limits")
    float v = std::min(m.velocity, m.max_velocity);
    float a = std::min(m.accel, m.max_accel);

    // homing: absolute encoder + no home switch, so "zero here" then go-to-zero
    if (ImGui::Button("Set Zero (home here)")) {
        motor_try("set zero", [&] { m.axis->PositionSet(0.0); m.axis->HomeStateSet(true); });
    }
    ImGui::SameLine();
    if (ImGui::Button("Go Home (0)")) {
        motor_try("go home", [&] { m.axis->MoveSCurve(0.0, v, a, a, m.jerk_pct); });
    }

    // absolute move
    ImGui::InputFloat("target (deg)", &m.target_deg);
    if (ImGui::Button("Move Abs")) {
        motor_try("move abs", [&] {
            m.axis->MoveSCurve(m.target_deg, v, a, a, m.jerk_pct);
        });
    }

    // relative jog
    ImGui::InputFloat("jog (deg)", &m.jog_deg);
    if (ImGui::Button("Jog -")) {
        motor_try("jog -", [&] {
            m.axis->MoveRelative(-m.jog_deg, v, a, a, m.jerk_pct);
        });
    }
    ImGui::SameLine();
    if (ImGui::Button("Jog +")) {
        motor_try("jog +", [&] {
            m.axis->MoveRelative(m.jog_deg, v, a, a, m.jerk_pct);
        });
    }

    // limits: speed (clamped in-app) + travel/following-error (pushed to the drive)
    if (ImGui::TreeNode("Limits")) {
        ImGui::TextDisabled("Speed limits (clamp commanded motion)");
        ImGui::InputFloat("max velocity (deg/s)", &m.max_velocity);
        ImGui::InputFloat("max accel (deg/s^2)",  &m.max_accel);
        ImGui::Separator();
        ImGui::TextDisabled("Travel + following-error (Apply pushes to drive)");
        ImGui::InputFloat("error limit (deg)", &m.error_limit_deg);
        ImGui::InputFloat("travel min (deg)",  &m.pos_min_deg);
        ImGui::InputFloat("travel max (deg)",  &m.pos_max_deg);
        ImGui::Checkbox("travel limits enabled", &m.limits_enabled);
        if (ImGui::Button("Apply Limits")) {
            motor_apply_limits(m);
        }
        ImGui::TreePop();
    }

    ImGui::PopID();
}

static void gui_draw() {
    if (ImGui::Begin("GalvoCam Motor Control")) {
        if (g_controller == nullptr) {
            if (ImGui::Button("Connect")) {
                motor_connect();
            }
        } else {
            if (ImGui::Button("Abort All")) {
                for (MotorAxis& m : g_axes) {
                    motor_try("abort all", [&] { m.axis->Abort(); });
                }
            }
        }
        ImGui::TextWrapped("Status: %s", g_status.c_str());

        if (g_controller != nullptr) {
            ImGui::Separator();
            ImGui::InputDouble("counts/rev", &g_counts_per_rev, 0.0, 0.0, "%.0f");
            ImGui::SameLine();
            ImGui::Text("(%.1f counts/deg)", g_counts_per_rev / 360.0);
            if (ImGui::Button("Apply units")) {
                motor_apply_units();
            }

            // --- targeting: aim the camera line-of-sight at a world 3D point ---
            ImGui::SeparatorText("Targeting (world 3D -> pan/tilt)");
            if (g_moving_target) {
                // test path: target circles in the world X-Y plane at a fixed forward distance
                double t = glfwGetTime();
                g_target[0] = g_tgt.base[0] + 300.0f * cosf((float)t);
                g_target[1] = g_tgt.base[1] + 300.0f * sinf((float)t);
                g_target[2] = g_tgt.base[2] + 1000.0f;
            }
            ImGui::InputFloat3("target x,y,z", g_target);
            PanTilt pt = targeting_solve(g_target);
            ImGui::Text("az %.2f  el %.2f deg  ->  pan %.2f  tilt %.2f deg",
                        pt.az, pt.el, pt.pan_motor, pt.tilt_motor);
            if (ImGui::Button("Aim at target")) {
                targeting_aim(g_target);
            }
            ImGui::SameLine();
            ImGui::Checkbox("moving target (test)", &g_moving_target);
            if (g_moving_target) {
                static double last_cmd = 0.0;
                double now = glfwGetTime();
                if (now - last_cmd > 0.1) {   // re-aim at ~10 Hz
                    targeting_aim(g_target);
                    last_cmd = now;
                }
            }

            if (ImGui::TreeNode("Targeting calibration")) {
                ImGui::InputFloat3("base pos (world)", g_tgt.base);
                ImGui::InputInt("pan axis",  &g_tgt.pan_axis);
                ImGui::InputInt("tilt axis", &g_tgt.tilt_axis);
                ImGui::InputFloat("pan sign",    &g_tgt.pan_sign);
                ImGui::InputFloat("pan scale",   &g_tgt.pan_scale);
                ImGui::InputFloat("pan offset",  &g_tgt.pan_offset);
                ImGui::InputFloat("tilt sign",   &g_tgt.tilt_sign);
                ImGui::InputFloat("tilt scale",  &g_tgt.tilt_scale);
                ImGui::InputFloat("tilt offset", &g_tgt.tilt_offset);
                ImGui::TreePop();
            }
        }

        for (MotorAxis& m : g_axes) {
            gui_draw_axis(m);
        }
    }
    ImGui::End();
}

// ---------------------------------------------------------------------------
// render plumbing (ImGui + GLFW + OpenGL3)
// ---------------------------------------------------------------------------
static void create_new_frame() {
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

static void render_a_frame(GLFWwindow* window) {
    ImGui::Render();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.10f, 0.10f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
}

static void glfw_error_callback(int code, const char* desc) {
    printf("GLFW error %d: %s\n", code, desc ? desc : "(none)");
}

int main() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        printf("glfwInit failed\n");
        return 1;
    }
    // No GL version hints: this PC has no GPU driver (Microsoft Basic Display Adapter
    // = OpenGL 1.1 only), so take the default context and use the legacy OpenGL2 backend.
    GLFWwindow* window = glfwCreateWindow(560, 820, "GalvoCam Motor Control", nullptr, nullptr);
    if (window == nullptr) {
        const char* desc = nullptr;
        int code = glfwGetError(&desc);
        printf("glfwCreateWindow failed (code %d): %s\n", code, desc ? desc : "(no description)");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);   // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        create_new_frame();
        gui_draw();
        render_a_frame(window);
    }

    // drop torque on the way out
    for (MotorAxis& m : g_axes) {
        motor_try("shutdown", [&] {
            m.axis->Abort();
            m.axis->AmpEnableSet(false);
        });
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
