// main.cpp
// Minimal Dear ImGui motor control for the GalvoCam MR-J5 servos (RSI RMP / EtherCAT).
//
// Style follows moments-behavior/orange: snake_case functions with gui_/draw_ prefixes,
// PascalCase types, g_ globals, 1TBS braces, create_new_frame()/render_a_frame() loop.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

// networking (UDP target receiver) + threading
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#include <atomic>
#include <mutex>
#include <thread>

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
// works in degrees. 67108864 = 2^26 (HK-KT). Editable live + persisted in the config.
static double g_counts_per_rev = 67108864.0;

// ---------------------------------------------------------------------------
// types
// ---------------------------------------------------------------------------
struct MotorAxis {
    Axis*   axis   = nullptr;
    int32_t number = 0;

    // cached raw readback (degrees / deg-per-sec), refreshed each frame
    double actual_position  = 0.0;   // raw (before zero_ref)
    double command_position = 0.0;   // raw
    double actual_velocity  = 0.0;
    RSIState state          = RSIState::RSIStateIDLE;
    bool   amp_enabled      = false;
    std::string source_name;         // reason for an ERROR state

    // UI-editable move parameters (degrees, displayed frame)
    float target_deg = 90.0f;
    float jog_deg    = 10.0f;
    float velocity   = 90.0f;   // deg/s
    float accel      = 360.0f;  // deg/s^2
    float jerk_pct   = 50.0f;   // %

    // safety limits (degrees)
    float error_limit_deg = 5.0f;
    float pos_min_deg     = -180.0f;
    float pos_max_deg     = 180.0f;
    bool  limits_enabled  = false;

    // speed limits: commanded velocity/accel are clamped to these
    float max_velocity    = 360.0f;    // deg/s
    float max_accel       = 3600.0f;   // deg/s^2

    // persistent zero: displayed position = raw - zero_ref; absolute cmd = displayed + zero_ref
    double zero_ref = 0.0;
};

// ---------------------------------------------------------------------------
// globals
// ---------------------------------------------------------------------------
static MotionController*      g_controller = nullptr;
static std::vector<MotorAxis> g_axes;
static std::string            g_status = "not connected";

// ---------------------------------------------------------------------------
// network target receiver (UDP, one-way Linux -> Windows). See PROTOCOL.md.
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct GcTargetPacket {
    char     magic[4];      // 'G','C','T','1'
    uint16_t version;       // = 1
    uint16_t flags;         // bit0 = target valid
    uint32_t seq;           // monotonic sequence number
    uint32_t target_id;     // GalvoCam/target id (0 = this unit)
    uint64_t timestamp_ns;  // sender timestamp (ns), informational
    double   x, y, z;       // world coordinates, mm
};
#pragma pack(pop)
static_assert(sizeof(GcTargetPacket) == 48, "GcTargetPacket must be 48 bytes");

#ifdef _WIN32
typedef SOCKET sock_t;
static const sock_t BAD_SOCK = INVALID_SOCKET;
#else
typedef int sock_t;
static const sock_t BAD_SOCK = -1;
#endif

static std::atomic<bool> g_net_running{false};
static std::thread       g_net_thread;
static std::mutex        g_net_mutex;
static int      g_net_port = 5005;
static bool     g_net_aim  = false;   // aim at received targets
// shared under g_net_mutex:
static float    g_net_target[3] = {0.0f, 0.0f, 1000.0f};
static bool     g_net_valid     = false;
static uint32_t g_net_seq       = 0;
static uint32_t g_net_id        = 0;
static uint64_t g_net_count     = 0;
static double   g_net_last_recv = 0.0;

static void net_recv_loop(int port) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { g_status = "net: WSAStartup failed"; g_net_running = false; return; }
#endif
    sock_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == BAD_SOCK) { g_status = "net: socket failed"; g_net_running = false; return; }
#ifdef _WIN32
    DWORD to = 200; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
#else
    timeval to{0, 200000}; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));
#endif
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)port);
    if (bind(s, (sockaddr*)&addr, sizeof(addr)) != 0) {
        g_status = "net: bind failed (port in use?)";
#ifdef _WIN32
        closesocket(s); WSACleanup();
#else
        close(s);
#endif
        g_net_running = false;
        return;
    }
    g_status = "net: listening on UDP " + std::to_string(port);

    GcTargetPacket pkt;
    while (g_net_running) {
        int n = (int)recvfrom(s, (char*)&pkt, sizeof(pkt), 0, nullptr, nullptr);
        if (n == (int)sizeof(pkt) &&
            pkt.magic[0] == 'G' && pkt.magic[1] == 'C' && pkt.magic[2] == 'T' && pkt.magic[3] == '1' &&
            pkt.version == 1) {
            std::lock_guard<std::mutex> lk(g_net_mutex);
            // accept the first packet, then only newer sequence numbers (int32 diff => wraps ok)
            if (g_net_count == 0 || (int32_t)(pkt.seq - g_net_seq) > 0) {
                g_net_seq       = pkt.seq;
                g_net_id        = pkt.target_id;
                g_net_valid     = (pkt.flags & 0x1) != 0;
                g_net_target[0] = (float)pkt.x;
                g_net_target[1] = (float)pkt.y;
                g_net_target[2] = (float)pkt.z;
                g_net_count++;
                g_net_last_recv = glfwGetTime();
            }
        }
    }
#ifdef _WIN32
    closesocket(s); WSACleanup();
#else
    close(s);
#endif
}

static void net_start(int port) {
    if (g_net_running) return;
    g_net_running = true;
    g_net_thread = std::thread(net_recv_loop, port);
}
static void net_stop() {
    if (!g_net_running) return;
    g_net_running = false;
    if (g_net_thread.joinable()) g_net_thread.join();
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
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
static void motor_apply_limits(MotorAxis& m) {
    motor_try("apply limits", [&] {
        m.axis->ErrorLimitTriggerValueSet(m.error_limit_deg);
        m.axis->ErrorLimitActionSet(RSIAction::RSIActionABORT);

        RSIAction act = m.limits_enabled ? RSIAction::RSIActionE_STOP : RSIAction::RSIActionNONE;
        // Travel limits are in raw coordinates -> shift by zero_ref.
        m.axis->SoftwarePosLimitTriggerValueSet(m.pos_max_deg + m.zero_ref);
        m.axis->SoftwarePosLimitActionSet(act);
        m.axis->SoftwareNegLimitTriggerValueSet(m.pos_min_deg + m.zero_ref);
        m.axis->SoftwareNegLimitActionSet(act);

        // No physical limit switches on this rig -> disable hardware limit inputs
        // (they float to "tripped" and abort every move).
        m.axis->HardwarePosLimitActionSet(RSIAction::RSIActionNONE);
        m.axis->HardwareNegLimitActionSet(RSIAction::RSIActionNONE);
    });
}

static void motor_connect() {
    motor_try("connect", [&] {
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
            m.axis->UserUnitsSet(g_counts_per_rev / 360.0);
            motor_apply_limits(m);
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

static void motor_apply_units() {
    for (MotorAxis& m : g_axes) {
        motor_try("apply units", [&] { m.axis->UserUnitsSet(g_counts_per_rev / 360.0); });
        motor_apply_limits(m);
    }
}

// ---------------------------------------------------------------------------
// targeting: world 3D point -> pan/tilt motor angles (displayed frame, degrees)
// ---------------------------------------------------------------------------
static constexpr double RAD2DEG = 57.29577951308232;

struct TargetConfig {
    float base[3] = {0.0f, 0.0f, 0.0f};   // GalvoCam optical center, in the INCOMING world frame
    float rot[3]  = {0.0f, 0.0f, 0.0f};   // residual world->galvo rotation after g_xf, Euler XYZ deg (R = Rz*Ry*Rx)
    int   pan_axis  = 0;
    int   tilt_axis = 1;
    float pan_sign  = 1.0f, pan_scale  = 0.5f, pan_offset  = 0.0f;
    float tilt_sign = 1.0f, tilt_scale = 0.5f, tilt_offset = 0.0f;
};
static TargetConfig g_tgt;

// Incoming-coordinate transform: map the sender's world frame -> app frame
// (app: X=right, Y=up, Z=forward). ax[i] = which incoming axis feeds app axis i;
// sgn[i] = its sign; scale = uniform unit scale. Default assumes the sender is
// Z-up (X right, Y forward, Z up): app Y<-in Z, app Z<-in Y.
struct CoordXform {
    int   ax[3]  = {0, 2, 1};                 // appX<-in0(X), appY<-in2(Z), appZ<-in1(Y)
    float sgn[3] = {1.0f, 1.0f, 1.0f};
    float scale  = 1.0f;
};
static CoordXform g_xf;

static float g_target[3]     = {0.0f, 0.0f, 1000.0f};
static bool  g_moving_target = false;
static float g_net_max_step  = 15.0f;   // max pan/tilt change per aim update (deg): slew guard (discrete mode)

// Network-target tracking mode. Velocity pursuit (default) commands
// MoveVelocity every frame with a TIME-OPTIMAL law: full max_velocity toward
// the goal, braking only on the max-accel curve (v = sqrt(2*a*d)). No
// smoothing anywhere -- speed is the priority; the per-axis max velocity /
// max accel fields are the only limits. (Repeated MoveSCurve point-to-point
// moves each plan to STOP at the goal, capping a short segment at sqrt(d*a),
// which is why a fast mover outruns that mode.)
static bool  g_track_velocity = true;
static bool  g_track_was      = false;  // velocity pursuit active last frame

struct PanTilt { double pan_motor, tilt_motor, az, el; };

// Fitted affine map az/el -> pan/tilt (from correspondence calibration). When inactive,
// the simple diagonal sign*scale+offset model is used.
static bool   g_cal_active  = false;
static double g_cal_pan[3]  = {1.0, 0.0, 0.0};   // pan  = [0]*az + [1]*el + [2]
static double g_cal_tilt[3] = {0.0, 1.0, 0.0};   // tilt = [0]*az + [1]*el + [2]

// az/el of a world point as seen from an explicit turret rotation center `base`.
// After the g_xf axis mapping, the residual rotation g_tgt.rot is applied as
// R^T*d -- identity by default, remotely fittable (SET_CALIB) for a frame
// alignment finer than g_xf's 90-degree permutations.
static void world_to_azel_base(const float* target, const float* base, double& az, double& el) {
    double dIn[3] = { (target[0] - base[0]) * g_xf.scale,
                      (target[1] - base[1]) * g_xf.scale,
                      (target[2] - base[2]) * g_xf.scale };
    double d[3] = { g_xf.sgn[0] * dIn[g_xf.ax[0]],
                    g_xf.sgn[1] * dIn[g_xf.ax[1]],
                    g_xf.sgn[2] * dIn[g_xf.ax[2]] };
    double dx = d[0], dy = d[1], dz = d[2];
    if (g_tgt.rot[0] != 0.0f || g_tgt.rot[1] != 0.0f || g_tgt.rot[2] != 0.0f) {
        const double DEG2RAD = 1.0 / RAD2DEG;
        double rx = g_tgt.rot[0] * DEG2RAD;
        double ry = g_tgt.rot[1] * DEG2RAD;
        double rz = g_tgt.rot[2] * DEG2RAD;
        double cx = cos(rx), sx = sin(rx);
        double cy = cos(ry), sy = sin(ry);
        double cz = cos(rz), sz = sin(rz);
        // rows of R = Rz*Ry*Rx; v = R^T * d
        double R[3][3] = {
            {cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx},
            {sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx},
            {-sy,     cy * sx,                cy * cx},
        };
        dx = R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2];
        dy = R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2];
        dz = R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2];
    }
    az = atan2(dx, dz) * RAD2DEG;
    el = atan2(dy, sqrt(dx * dx + dz * dz)) * RAD2DEG;
}

static void world_to_azel(const float* target, double& az, double& el) {
    world_to_azel_base(target, g_tgt.base, az, el);
}

static PanTilt targeting_solve(const float* target) {
    double az, el;
    world_to_azel(target, az, el);
    PanTilt pt;
    pt.az = az; pt.el = el;
    if (g_cal_active) {
        pt.pan_motor  = g_cal_pan[0]  * az + g_cal_pan[1]  * el + g_cal_pan[2];
        pt.tilt_motor = g_cal_tilt[0] * az + g_cal_tilt[1] * el + g_cal_tilt[2];
    } else {
        pt.pan_motor  = g_tgt.pan_sign  * az * g_tgt.pan_scale  + g_tgt.pan_offset;
        pt.tilt_motor = g_tgt.tilt_sign * el * g_tgt.tilt_scale + g_tgt.tilt_offset;
    }
    return pt;
}

// Velocity pursuit: one time-optimal step toward displayed-frame goal angles.
// Uses MoveVelocity (like the joystick jog), re-issued every frame: full
// speed toward the goal, braking on the max-accel curve. Overshoot guards:
// 0.8 margin on the braking curve (the loop only updates at frame rate), a
// per-update crossing cap, and a small deadband so it doesn't hunt at rest.
static void track_velocity_step(double goal_pan, double goal_tilt) {
    auto vcmd = [&](int idx, double goal) {
        if (idx < 0 || idx >= (int)g_axes.size()) return;
        MotorAxis& m = g_axes[idx];
        goal = std::min((double)m.pos_max_deg, std::max((double)m.pos_min_deg, goal));
        double err  = goal - (m.actual_position - m.zero_ref);
        double aerr = fabs(err);
        double a    = (double)m.max_accel;
        double v    = 0.0;
        if (aerr > 0.02) {                       // deadband (deg)
            v = sqrt(2.0 * a * aerr * 0.8);      // brake curve, with margin
            v = std::min(v, aerr * 60.0);        // don't cross the goal in one ~16ms update
            v = std::min(v, (double)m.max_velocity);
            if (err < 0.0) v = -v;
        }
        motor_try("track", [&] { m.axis->MoveVelocity(v, a); });
    };
    vcmd(g_tgt.pan_axis,  goal_pan);
    vcmd(g_tgt.tilt_axis, goal_tilt);
}

static void track_velocity_stop() {
    auto stop = [&](int idx) {
        if (idx < 0 || idx >= (int)g_axes.size()) return;
        MotorAxis& m = g_axes[idx];
        motor_try("track stop", [&] { m.axis->MoveVelocity(0.0, m.max_accel); });
    };
    stop(g_tgt.pan_axis);
    stop(g_tgt.tilt_axis);
}

// Command explicit displayed pan/tilt angles, each clamped to its axis's travel range.
static void aim_pan_tilt(double pan_disp, double tilt_disp) {
    auto cmd = [&](int idx, double disp) {
        if (idx < 0 || idx >= (int)g_axes.size()) return;
        MotorAxis& m = g_axes[idx];
        disp = std::min((double)m.pos_max_deg, std::max((double)m.pos_min_deg, disp));
        float v = std::min(m.velocity, m.max_velocity);
        float a = std::min(m.accel, m.max_accel);
        motor_try("aim", [&] { m.axis->MoveSCurve(disp + m.zero_ref, v, a, a, m.jerk_pct); });
    };
    cmd(g_tgt.pan_axis,  pan_disp);
    cmd(g_tgt.tilt_axis, tilt_disp);
}

static void targeting_aim(const float* target) {
    PanTilt pt = targeting_solve(target);
    aim_pan_tilt(pt.pan_motor, pt.tilt_motor);
}

// ---------------------------------------------------------------------------
// correspondence ("teach") calibration: X-Y jog to center a known object, capture
// the (jitter-averaged) live coord + the pan/tilt that centered it, then least-squares
// fit az/el -> pan/tilt. Needs >= 3 non-collinear points; more = better.
// ---------------------------------------------------------------------------
struct CalPoint { double x, y, z, pan, tilt; };   // world coord (avg) + displayed pan/tilt
static std::vector<CalPoint> g_cal_points;
static double g_cal_rms    = -1.0;
static double g_cal_std_az = 0.0, g_cal_std_el = 0.0;   // az/el spread of the captures
static float  g_cal_std_min = 1.0f;                     // inputs varying < this (deg) are suppressed
static bool   g_cal_solve_base = true;                  // jointly solve the turret center during Fit
static float  g_cal_base_reg   = 0.02f;                 // anchor solved base to typed base (deg^2/pt at full spread)
static bool   g_cal_base_solved = false;                // last Fit actually moved the base

static float g_xyjog_step      = 2.0f;   // deg per press
static float g_xyjog_pan_sign  = 1.0f;
static float g_xyjog_tilt_sign = 1.0f;
static int   g_xyjog_rot       = 1;      // view rotation before mapping: 0/1/2/3 = 0/90/180/270 deg CCW
static float g_joy_rate        = 30.0f;  // deg/s at full joystick deflection

static bool   g_capturing  = false;      // averaging the live target
static double g_cap_end    = 0.0;
static double g_cap_sum[3] = {0.0, 0.0, 0.0};
static int    g_cap_n      = 0;

// Rotate a view vector by g_xyjog_rot*90 deg CCW.
static void rot_view(double x, double y, double& rx, double& ry) {
    switch (g_xyjog_rot & 3) {
        case 1: rx = -y; ry =  x; break;   // 90 CCW
        case 2: rx = -x; ry = -y; break;   // 180
        case 3: rx =  y; ry = -x; break;   // 270
        default: rx = x; ry = y; break;    // 0
    }
}

// Step jog (buttons): relative move. view-X -> pan, view-Y -> tilt (rotated + signed).
static void xy_jog(double dview_x, double dview_y) {
    double rx, ry; rot_view(dview_x, dview_y, rx, ry);
    auto rel = [&](int idx, double d) {
        if (idx < 0 || idx >= (int)g_axes.size() || d == 0.0) return;
        MotorAxis& m = g_axes[idx];
        float v = std::min(m.velocity, m.max_velocity);
        float a = std::min(m.accel, m.max_accel);
        motor_try("xy jog", [&] { m.axis->MoveRelative(d, v, a, a, m.jerk_pct); });
    };
    rel(g_tgt.pan_axis,  g_xyjog_pan_sign  * rx);
    rel(g_tgt.tilt_axis, g_xyjog_tilt_sign * ry);
}

// Continuous jog (joystick): velocity control. Call with 0,0 to stop.
static void xy_jog_velocity(double vx_view, double vy_view) {
    double rx, ry; rot_view(vx_view, vy_view, rx, ry);
    auto vel = [&](int idx, double v) {
        if (idx < 0 || idx >= (int)g_axes.size()) return;
        MotorAxis& m = g_axes[idx];
        float a = std::min(m.accel, m.max_accel);
        motor_try("joy", [&] { m.axis->MoveVelocity(v, a); });
    };
    vel(g_tgt.pan_axis,  g_xyjog_pan_sign  * rx);
    vel(g_tgt.tilt_axis, g_xyjog_tilt_sign * ry);
}

// Solve 3x3 A x = b (Gaussian elimination, partial pivot). false if singular.
static bool solve3(double A[3][3], double b[3], double x[3]) {
    for (int i = 0; i < 3; i++) {
        int p = i;
        for (int r = i + 1; r < 3; r++) if (fabs(A[r][i]) > fabs(A[p][i])) p = r;
        if (fabs(A[p][i]) < 1e-12) return false;
        if (p != i) { for (int c = 0; c < 3; c++) std::swap(A[i][c], A[p][c]); std::swap(b[i], b[p]); }
        for (int r = i + 1; r < 3; r++) {
            double f = A[r][i] / A[i][i];
            for (int c = i; c < 3; c++) A[r][c] -= f * A[i][c];
            b[r] -= f * b[i];
        }
    }
    for (int i = 2; i >= 0; i--) {
        double s = b[i];
        for (int c = i + 1; c < 3; c++) s -= A[i][c] * x[c];
        x[i] = s / A[i][i];
    }
    return true;
}

// Closed-form least-squares affine fit az/el -> pan/tilt for a FIXED turret center.
// Returns false if singular; on success fills the coefficients, the sum of squared
// pan+tilt residuals (deg^2), and the az/el spreads.
static bool cal_fit_affine(const float base[3], double panc[3], double tiltc[3],
                           double& sse, double& std_az, double& std_el) {
    int n = (int)g_cal_points.size();
    double AtA[3][3] = {{0}}, AtPan[3] = {0}, AtTilt[3] = {0};
    double sum_az = 0, sum_el = 0, sum_az2 = 0, sum_el2 = 0;
    for (const CalPoint& c : g_cal_points) {
        float tp[3] = { (float)c.x, (float)c.y, (float)c.z };
        double az, el; world_to_azel_base(tp, base, az, el);
        double r[3] = { az, el, 1.0 };
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) AtA[i][j] += r[i] * r[j];
            AtPan[i]  += r[i] * c.pan;
            AtTilt[i] += r[i] * c.tilt;
        }
        sum_az += az; sum_el += el; sum_az2 += az * az; sum_el2 += el * el;
    }
    double maz = sum_az / n, mel = sum_el / n;
    std_az = sqrt(std::max(0.0, sum_az2 / n - maz * maz));
    std_el = sqrt(std::max(0.0, sum_el2 / n - mel * mel));
    // Suppress an input that barely varied across captures - its coefficient would be
    // garbage and amplify jitter (e.g. el ~ 0). A huge diagonal drives that coef -> 0.
    if (std_az < g_cal_std_min) AtA[0][0] += 1e12;
    if (std_el < g_cal_std_min) AtA[1][1] += 1e12;
    double tr = AtA[0][0] + AtA[1][1] + AtA[2][2];   // tiny ridge for numerical robustness
    double eps = 1e-9 * (tr > 0.0 ? tr : 1.0);
    AtA[0][0] += eps; AtA[1][1] += eps; AtA[2][2] += eps;
    double A1[3][3], A2[3][3], b1[3], b2[3];
    memcpy(A1, AtA, sizeof(AtA)); memcpy(A2, AtA, sizeof(AtA));
    memcpy(b1, AtPan, sizeof(b1)); memcpy(b2, AtTilt, sizeof(b2));
    if (!solve3(A1, b1, panc) || !solve3(A2, b2, tiltc)) return false;
    sse = 0.0;
    for (const CalPoint& c : g_cal_points) {
        float tp[3] = { (float)c.x, (float)c.y, (float)c.z };
        double az, el; world_to_azel_base(tp, base, az, el);
        double dp = (panc[0]*az + panc[1]*el + panc[2]) - c.pan;
        double dt = (tiltc[0]*az + tiltc[1]*el + tiltc[2]) - c.tilt;
        sse += dp*dp + dt*dt;
    }
    return true;
}

static bool cal_fit() {
    int n = (int)g_cal_points.size();
    if (n < 3) { g_status = "cal: need >= 3 points"; return false; }

    // The affine az/el -> pan/tilt model is only exact when `base` is the true turret
    // rotation center. A wrong base injects a curved (parallax) residual that a plane
    // can absorb when points fill a 3D volume, but NOT when they are coplanar (e.g. all
    // on the floor) - that is the classic high-error case. So we solve for `base` too:
    // for any candidate base the affine fit above is closed-form, leaving a smooth 3D
    // cost we minimize by pattern search, lightly anchored to the typed base so the
    // in-plane (weakly observable) directions stay put.
    const float base0[3] = { g_tgt.base[0], g_tgt.base[1], g_tgt.base[2] };
    float lo[3] = { 1e30f, 1e30f, 1e30f }, hi[3] = { -1e30f, -1e30f, -1e30f };
    for (const CalPoint& c : g_cal_points) {
        float p[3] = { (float)c.x, (float)c.y, (float)c.z };
        for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
    }
    float spread[3]; double diag2 = 0.0;
    for (int k = 0; k < 3; k++) { spread[k] = hi[k] - lo[k]; diag2 += (double)spread[k] * spread[k]; }
    float diag = (float)sqrt(diag2); if (diag <= 0.0f) diag = 1.0f;

    double panc[3], tiltc[3], sse, saz, sel;
    // regularized cost at a candidate base (affine SSE + gentle anchor to base0)
    auto cost = [&](const float b[3], double pc[3], double tc[3], double& az_s, double& el_s) -> double {
        double s; if (!cal_fit_affine(b, pc, tc, s, az_s, el_s)) return 1e300;
        double reg = 0.0;
        for (int k = 0; k < 3; k++) { double d = (b[k] - base0[k]) / (double)diag; reg += d * d; }
        return s + (double)g_cal_base_reg * (double)n * reg;
    };

    float best[3] = { base0[0], base0[1], base0[2] };
    double bestc = cost(best, panc, tiltc, saz, sel);
    g_cal_base_solved = false;

    // Base adds 3 unknowns on top of the 6 affine ones; require enough points to constrain
    // them (>=5 points -> 10 residuals >= 9 unknowns) before daring to move the center.
    if (g_cal_solve_base && n >= 5 && bestc < 1e299) {
        float step[3];
        for (int k = 0; k < 3; k++) step[k] = std::max(spread[k] * 0.5f, diag * 0.05f);
        double tol = (double)diag * 1e-4;
        double pc[3], tc[3], a, e;
        for (int iter = 0; iter < 300; iter++) {
            bool improved = false;
            for (int k = 0; k < 3; k++) {
                for (int s = -1; s <= 1; s += 2) {
                    float trial[3] = { best[0], best[1], best[2] };
                    trial[k] += s * step[k];
                    double c = cost(trial, pc, tc, a, e);
                    if (c < bestc - 1e-9) {
                        bestc = c; best[k] = trial[k];
                        memcpy(panc, pc, sizeof(panc)); memcpy(tiltc, tc, sizeof(tiltc));
                        saz = a; sel = e; improved = true;
                    }
                }
            }
            if (!improved) {   // converged along current step scale -> refine, stop when tiny
                bool done = true;
                for (int k = 0; k < 3; k++) { step[k] *= 0.5f; if (step[k] > tol) done = false; }
                if (done) break;
            }
        }
        g_cal_base_solved = (best[0] != base0[0] || best[1] != base0[1] || best[2] != base0[2]);
    }

    // commit the winning base + its affine fit (recompute for the raw, un-regularized RMS)
    if (!cal_fit_affine(best, panc, tiltc, sse, saz, sel)) {
        g_status = "cal: singular - vary azimuth AND elevation (points lie on a line in az/el)";
        return false;
    }
    g_tgt.base[0] = best[0]; g_tgt.base[1] = best[1]; g_tgt.base[2] = best[2];
    for (int i = 0; i < 3; i++) { g_cal_pan[i] = panc[i]; g_cal_tilt[i] = tiltc[i]; }
    g_cal_std_az = saz; g_cal_std_el = sel;
    g_cal_rms = sqrt(sse / (2.0 * n));
    g_cal_active = true;
    g_status = "cal: RMS " + std::to_string(g_cal_rms) + " deg" +
               (g_cal_base_solved ? " (base solved)" : "") +
               "  (az spread " + std::to_string(g_cal_std_az) +
               ", el spread " + std::to_string(g_cal_std_el) + ")";
    return true;
}

static void cal_capture_begin() {
    g_capturing = true;
    g_cap_end = glfwGetTime() + 0.5;         // 0.5 s averaging window (jitter)
    g_cap_sum[0] = g_cap_sum[1] = g_cap_sum[2] = 0.0; g_cap_n = 0;
}
static void cal_capture_tick() {
    if (!g_capturing) return;
    { std::lock_guard<std::mutex> lk(g_net_mutex);
      g_cap_sum[0] += g_net_target[0]; g_cap_sum[1] += g_net_target[1]; g_cap_sum[2] += g_net_target[2]; }
    g_cap_n++;
    if (glfwGetTime() >= g_cap_end && g_cap_n > 0) {
        CalPoint cp;
        cp.x = g_cap_sum[0] / g_cap_n; cp.y = g_cap_sum[1] / g_cap_n; cp.z = g_cap_sum[2] / g_cap_n;
        cp.pan = 0.0; cp.tilt = 0.0;
        if (g_tgt.pan_axis  >= 0 && g_tgt.pan_axis  < (int)g_axes.size())
            cp.pan  = g_axes[g_tgt.pan_axis].actual_position  - g_axes[g_tgt.pan_axis].zero_ref;
        if (g_tgt.tilt_axis >= 0 && g_tgt.tilt_axis < (int)g_axes.size())
            cp.tilt = g_axes[g_tgt.tilt_axis].actual_position - g_axes[g_tgt.tilt_axis].zero_ref;
        g_cal_points.push_back(cp);
        g_capturing = false;
        g_status = "cal: captured point " + std::to_string(g_cal_points.size());
    }
}

// (PVT streaming for smooth tracking is deferred until it can be brought up on live
//  hardware - the MovePVT buffer/time semantics need validation. Tracking below uses
//  discrete re-aim of both axes, which is proven to move pan+tilt together.)

// ---------------------------------------------------------------------------
// control channel (UDP request/reply, GCC1 in -> GCS1 out). See PROTOCOL.md.
// Polled on the UI thread each frame so all RapidCode calls stay there.
// ---------------------------------------------------------------------------
static void config_save();

#pragma pack(push, 1)
struct GcCommandPacket {
    char     magic[4];      // 'G','C','C','1'
    uint16_t version;       // = 1
    uint16_t cmd;
    uint32_t seq;
    uint32_t reserved;
    double   args[14];
};
struct GcStatusPacket {
    char     magic[4];      // 'G','C','S','1'
    uint16_t version;       // = 1
    uint16_t cmd;           // echoed
    uint32_t seq;           // echoed
    uint32_t flags;         // bit0 ok, bit1 in_position, bit2 calib_mode,
                            // bit3 motors_enabled, bit4 remote_allowed
    int32_t  err;           // 0 ok, 1 bad args, 2 remote disabled,
                            // 3 not connected, 4 clamped
    uint32_t reserved;
    double   pan_deg, tilt_deg;
    double   pan_min, pan_max, tilt_min, tilt_max;
    double   reserved2[3];
};
#pragma pack(pop)
static_assert(sizeof(GcCommandPacket) == 128, "GcCommandPacket must be 128 bytes");
static_assert(sizeof(GcStatusPacket)  == 96,  "GcStatusPacket must be 96 bytes");

enum GcCommand : uint16_t {
    GC_CMD_PING        = 0,
    GC_CMD_SET_ANGLES  = 1,
    GC_CMD_CALIB_MODE  = 2,
    GC_CMD_SET_CALIB   = 3,
    GC_CMD_SAVE_CONFIG = 4,
    GC_CMD_STOP        = 5,
};

static sock_t g_ctrl_sock      = BAD_SOCK;
static int    g_ctrl_port      = 5006;
static bool   g_ctrl_enabled   = true;    // listener on at launch (persisted)
static bool   g_remote_allowed = true;    // gates motion/config commands (persisted)
static bool   g_calib_mode     = false;   // suppresses target-stream aiming
static double g_ctrl_last_recv = 0.0;
static uint64_t g_ctrl_count   = 0;

static void ctrl_stop() {
    if (g_ctrl_sock == BAD_SOCK) return;
#ifdef _WIN32
    closesocket(g_ctrl_sock); WSACleanup();
#else
    close(g_ctrl_sock);
#endif
    g_ctrl_sock = BAD_SOCK;
    g_calib_mode = false;
}

static void ctrl_start(int port) {
    if (g_ctrl_sock != BAD_SOCK) return;
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { g_status = "ctrl: WSAStartup failed"; return; }
#endif
    sock_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == BAD_SOCK) { g_status = "ctrl: socket failed"; return; }
    // nonblocking: drained on the UI thread each frame
#ifdef _WIN32
    u_long nb = 1; ioctlsocket(s, FIONBIO, &nb);
#else
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)port);
    if (bind(s, (sockaddr*)&addr, sizeof(addr)) != 0) {
        g_status = "ctrl: bind failed (port in use?)";
#ifdef _WIN32
        closesocket(s); WSACleanup();
#else
        close(s);
#endif
        return;
    }
    g_ctrl_sock = s;
    g_status = "ctrl: listening on UDP " + std::to_string(port);
}

static MotorAxis* targeting_axis(int idx) {
    return (idx >= 0 && idx < (int)g_axes.size()) ? &g_axes[idx] : nullptr;
}

// Absolute move of one targeting axis to a displayed-frame angle, clamped to
// its travel limits. Returns true if the command had to be clamped.
static bool ctrl_move_axis(MotorAxis& m, double angle_deg) {
    double lo = m.pos_min_deg, hi = m.pos_max_deg;
    bool clamped = angle_deg < lo || angle_deg > hi;
    double a_deg = angle_deg < lo ? lo : (angle_deg > hi ? hi : angle_deg);
    float v = std::min(m.velocity, m.max_velocity);
    float a = std::min(m.accel, m.max_accel);
    motor_try("ctrl move", [&] { m.axis->MoveSCurve(a_deg + m.zero_ref, v, a, a, m.jerk_pct); });
    return clamped;
}

static GcStatusPacket ctrl_make_status(uint16_t cmd, uint32_t seq, int32_t err) {
    GcStatusPacket rep;
    memset(&rep, 0, sizeof(rep));
    rep.magic[0] = 'G'; rep.magic[1] = 'C'; rep.magic[2] = 'S'; rep.magic[3] = '1';
    rep.version = 1;
    rep.cmd = cmd;
    rep.seq = seq;
    rep.err = err;

    MotorAxis* pan  = targeting_axis(g_tgt.pan_axis);
    MotorAxis* tilt = targeting_axis(g_tgt.tilt_axis);
    bool connected = g_controller != nullptr && pan != nullptr && tilt != nullptr;
    bool in_pos = false, enabled = false;
    if (connected) {
        // cached readback refreshed by gui_draw_axis each frame
        rep.pan_deg  = pan->actual_position  - pan->zero_ref;
        rep.tilt_deg = tilt->actual_position - tilt->zero_ref;
        rep.pan_min  = pan->pos_min_deg;  rep.pan_max  = pan->pos_max_deg;
        rep.tilt_min = tilt->pos_min_deg; rep.tilt_max = tilt->pos_max_deg;
        enabled = pan->amp_enabled && tilt->amp_enabled;
        in_pos  = enabled &&
                  pan->state  == RSIState::RSIStateIDLE &&
                  tilt->state == RSIState::RSIStateIDLE;
    }
    rep.flags = (err == 0 ? 0x01 : 0x00) |
                (in_pos ? 0x02 : 0x00) |
                (g_calib_mode ? 0x04 : 0x00) |
                (enabled ? 0x08 : 0x00) |
                (g_remote_allowed ? 0x10 : 0x00);
    return rep;
}

// Executes one command; returns the reply err code.
static int32_t ctrl_dispatch(const GcCommandPacket& pkt) {
    MotorAxis* pan  = targeting_axis(g_tgt.pan_axis);
    MotorAxis* tilt = targeting_axis(g_tgt.tilt_axis);
    bool connected = g_controller != nullptr && pan != nullptr && tilt != nullptr;

    switch (pkt.cmd) {
        case GC_CMD_PING:
            return 0;
        case GC_CMD_STOP:
            // always allowed -- it's the safe direction
            if (!connected) return 3;
            motor_try("ctrl stop", [&] { pan->axis->Abort(); });
            motor_try("ctrl stop", [&] { tilt->axis->Abort(); });
            return 0;
        case GC_CMD_SET_ANGLES: {
            if (!g_remote_allowed) return 2;
            if (!connected) return 3;
            bool clamped = ctrl_move_axis(*pan, pkt.args[0]);
            clamped     |= ctrl_move_axis(*tilt, pkt.args[1]);
            return clamped ? 4 : 0;
        }
        case GC_CMD_CALIB_MODE:
            if (!g_remote_allowed) return 2;
            g_calib_mode = pkt.args[0] != 0.0;
            return 0;
        case GC_CMD_SET_CALIB:
            if (!g_remote_allowed) return 2;
            g_tgt.base[0]     = (float)pkt.args[0];
            g_tgt.base[1]     = (float)pkt.args[1];
            g_tgt.base[2]     = (float)pkt.args[2];
            g_tgt.rot[0]      = (float)pkt.args[3];
            g_tgt.rot[1]      = (float)pkt.args[4];
            g_tgt.rot[2]      = (float)pkt.args[5];
            g_tgt.pan_sign    = (float)pkt.args[6];
            g_tgt.pan_scale   = (float)pkt.args[7];
            g_tgt.pan_offset  = (float)pkt.args[8];
            g_tgt.tilt_sign   = (float)pkt.args[9];
            g_tgt.tilt_scale  = (float)pkt.args[10];
            g_tgt.tilt_offset = (float)pkt.args[11];
            // last calibration wins: the uploaded model must actually drive
            // aiming, so retire an active teach (affine) calibration and
            // reset the coord-frame mapping -- the uploaded rot IS the full
            // world->galvo alignment, fitted on raw sender coordinates
            g_cal_active = false;
            g_xf.ax[0] = 0; g_xf.ax[1] = 1; g_xf.ax[2] = 2;
            g_xf.sgn[0] = g_xf.sgn[1] = g_xf.sgn[2] = 1.0f;
            g_xf.scale = 1.0f;
            return 0;
        case GC_CMD_SAVE_CONFIG:
            if (!g_remote_allowed) return 2;
            config_save();
            return 0;
        default:
            return 1;
    }
}

// Drain pending commands (a few per frame is plenty at 2 Hz polls + button
// presses) and reply to each sender. Also expires calib mode on silence.
static void ctrl_poll() {
    if (g_ctrl_sock == BAD_SOCK) {
        return;
    }
    for (int i = 0; i < 8; i++) {
        GcCommandPacket pkt;
        sockaddr_in from;
#ifdef _WIN32
        int fromlen = (int)sizeof(from);
#else
        socklen_t fromlen = sizeof(from);
#endif
        int n = (int)recvfrom(g_ctrl_sock, (char*)&pkt, sizeof(pkt), 0,
                              (sockaddr*)&from, &fromlen);
        if (n != (int)sizeof(pkt) ||
            pkt.magic[0] != 'G' || pkt.magic[1] != 'C' || pkt.magic[2] != 'C' ||
            pkt.magic[3] != '1' || pkt.version != 1) {
            break; // no (valid) packet waiting
        }
        g_ctrl_last_recv = glfwGetTime();
        g_ctrl_count++;
        int32_t err = ctrl_dispatch(pkt);
        GcStatusPacket rep = ctrl_make_status(pkt.cmd, pkt.seq, err);
        sendto(g_ctrl_sock, (const char*)&rep, sizeof(rep), 0,
               (sockaddr*)&from, fromlen);
    }
    // dead-man switch: the sender polls at 2 Hz, so 5s of silence means it's
    // gone -- drop back to normal aiming rather than staying paused forever
    if (g_calib_mode && glfwGetTime() - g_ctrl_last_recv > 5.0) {
        g_calib_mode = false;
        g_status = "ctrl: calib mode expired (no control traffic)";
    }
}

// ---------------------------------------------------------------------------
// config file (persists zeros + tunables next to the exe)
// ---------------------------------------------------------------------------
static const char* CONFIG_PATH = "motor_control.cfg";
static std::map<std::string, double> g_cfg;

static double cfg_get(const std::string& k, double def) {
    auto it = g_cfg.find(k);
    return it == g_cfg.end() ? def : it->second;
}

static void config_load_file() {
    g_cfg.clear();
    std::ifstream f(CONFIG_PATH);
    std::string line;
    while (std::getline(f, line)) {
        auto eq = line.find('=');
        if (eq != std::string::npos)
            g_cfg[line.substr(0, eq)] = atof(line.substr(eq + 1).c_str());
    }
}

static void config_load_globals() {
    g_counts_per_rev  = cfg_get("counts_per_rev", g_counts_per_rev);
    g_tgt.base[0]     = (float)cfg_get("tgt_base_x", g_tgt.base[0]);
    g_tgt.base[1]     = (float)cfg_get("tgt_base_y", g_tgt.base[1]);
    g_tgt.base[2]     = (float)cfg_get("tgt_base_z", g_tgt.base[2]);
    g_tgt.rot[0]      = (float)cfg_get("tgt_rot_x", g_tgt.rot[0]);
    g_tgt.rot[1]      = (float)cfg_get("tgt_rot_y", g_tgt.rot[1]);
    g_tgt.rot[2]      = (float)cfg_get("tgt_rot_z", g_tgt.rot[2]);
    g_ctrl_port       = (int)  cfg_get("ctrl_port", g_ctrl_port);
    g_ctrl_enabled    = cfg_get("ctrl_enabled", g_ctrl_enabled ? 1 : 0) != 0.0;
    g_remote_allowed  = cfg_get("remote_allowed", g_remote_allowed ? 1 : 0) != 0.0;
    g_tgt.pan_axis    = (int)  cfg_get("tgt_pan_axis", g_tgt.pan_axis);
    g_tgt.tilt_axis   = (int)  cfg_get("tgt_tilt_axis", g_tgt.tilt_axis);
    g_tgt.pan_sign    = (float)cfg_get("tgt_pan_sign", g_tgt.pan_sign);
    g_tgt.pan_scale   = (float)cfg_get("tgt_pan_scale", g_tgt.pan_scale);
    g_tgt.pan_offset  = (float)cfg_get("tgt_pan_offset", g_tgt.pan_offset);
    g_tgt.tilt_sign   = (float)cfg_get("tgt_tilt_sign", g_tgt.tilt_sign);
    g_tgt.tilt_scale  = (float)cfg_get("tgt_tilt_scale", g_tgt.tilt_scale);
    g_tgt.tilt_offset = (float)cfg_get("tgt_tilt_offset", g_tgt.tilt_offset);
    g_xf.ax[0]  = (int)  cfg_get("xf_ax0", g_xf.ax[0]);
    g_xf.ax[1]  = (int)  cfg_get("xf_ax1", g_xf.ax[1]);
    g_xf.ax[2]  = (int)  cfg_get("xf_ax2", g_xf.ax[2]);
    g_xf.sgn[0] = (float)cfg_get("xf_sgn0", g_xf.sgn[0]);
    g_xf.sgn[1] = (float)cfg_get("xf_sgn1", g_xf.sgn[1]);
    g_xf.sgn[2] = (float)cfg_get("xf_sgn2", g_xf.sgn[2]);
    g_xf.scale  = (float)cfg_get("xf_scale", g_xf.scale);
    g_net_max_step = (float)cfg_get("net_max_step", g_net_max_step);
    g_track_velocity = cfg_get("track_velocity", g_track_velocity ? 1 : 0) != 0.0;
    g_xyjog_rot       = (int)  cfg_get("xyjog_rot", g_xyjog_rot);
    g_xyjog_step      = (float)cfg_get("xyjog_step", g_xyjog_step);
    g_xyjog_pan_sign  = (float)cfg_get("xyjog_pan_sign", g_xyjog_pan_sign);
    g_xyjog_tilt_sign = (float)cfg_get("xyjog_tilt_sign", g_xyjog_tilt_sign);
    g_joy_rate        = (float)cfg_get("joy_rate", g_joy_rate);
    g_cal_std_min     = (float)cfg_get("cal_std_min", g_cal_std_min);
    g_cal_solve_base  = cfg_get("cal_solve_base", g_cal_solve_base ? 1 : 0) != 0.0;
    g_cal_base_reg    = (float)cfg_get("cal_base_reg", g_cal_base_reg);
    g_cal_active = cfg_get("cal_active", g_cal_active ? 1 : 0) != 0.0;
    g_cal_pan[0]  = cfg_get("cal_pan0",  g_cal_pan[0]);
    g_cal_pan[1]  = cfg_get("cal_pan1",  g_cal_pan[1]);
    g_cal_pan[2]  = cfg_get("cal_pan2",  g_cal_pan[2]);
    g_cal_tilt[0] = cfg_get("cal_tilt0", g_cal_tilt[0]);
    g_cal_tilt[1] = cfg_get("cal_tilt1", g_cal_tilt[1]);
    g_cal_tilt[2] = cfg_get("cal_tilt2", g_cal_tilt[2]);
    int np = (int)cfg_get("cal_np", 0);
    g_cal_points.clear();
    for (int i = 0; i < np; i++) {
        std::string p = "cal" + std::to_string(i) + "_";
        CalPoint c;
        c.x = cfg_get(p + "x", 0); c.y = cfg_get(p + "y", 0); c.z = cfg_get(p + "z", 0);
        c.pan = cfg_get(p + "pan", 0); c.tilt = cfg_get(p + "tilt", 0);
        g_cal_points.push_back(c);
    }
}

static void config_apply_axes() {
    for (size_t i = 0; i < g_axes.size(); i++) {
        MotorAxis& m = g_axes[i];
        std::string p = "axis" + std::to_string(i) + "_";
        m.zero_ref       = cfg_get(p + "zero_ref", m.zero_ref);
        m.velocity       = (float)cfg_get(p + "velocity", m.velocity);
        m.accel          = (float)cfg_get(p + "accel", m.accel);
        m.jerk_pct       = (float)cfg_get(p + "jerk", m.jerk_pct);
        m.max_velocity   = (float)cfg_get(p + "max_velocity", m.max_velocity);
        m.max_accel      = (float)cfg_get(p + "max_accel", m.max_accel);
        m.error_limit_deg= (float)cfg_get(p + "error_limit", m.error_limit_deg);
        m.pos_min_deg    = (float)cfg_get(p + "pos_min", m.pos_min_deg);
        m.pos_max_deg    = (float)cfg_get(p + "pos_max", m.pos_max_deg);
        m.limits_enabled = cfg_get(p + "limits_enabled", m.limits_enabled ? 1 : 0) != 0.0;
        motor_apply_limits(m);
    }
}

static void config_save() {
    std::ofstream f(CONFIG_PATH);
    if (!f) return;
    f << "counts_per_rev=" << g_counts_per_rev << "\n";
    f << "tgt_base_x=" << g_tgt.base[0] << "\n";
    f << "tgt_base_y=" << g_tgt.base[1] << "\n";
    f << "tgt_base_z=" << g_tgt.base[2] << "\n";
    f << "tgt_rot_x=" << g_tgt.rot[0] << "\n";
    f << "tgt_rot_y=" << g_tgt.rot[1] << "\n";
    f << "tgt_rot_z=" << g_tgt.rot[2] << "\n";
    f << "ctrl_port=" << g_ctrl_port << "\n";
    f << "ctrl_enabled=" << (g_ctrl_enabled ? 1 : 0) << "\n";
    f << "remote_allowed=" << (g_remote_allowed ? 1 : 0) << "\n";
    f << "tgt_pan_axis=" << g_tgt.pan_axis << "\n";
    f << "tgt_tilt_axis=" << g_tgt.tilt_axis << "\n";
    f << "tgt_pan_sign=" << g_tgt.pan_sign << "\n";
    f << "tgt_pan_scale=" << g_tgt.pan_scale << "\n";
    f << "tgt_pan_offset=" << g_tgt.pan_offset << "\n";
    f << "tgt_tilt_sign=" << g_tgt.tilt_sign << "\n";
    f << "tgt_tilt_scale=" << g_tgt.tilt_scale << "\n";
    f << "tgt_tilt_offset=" << g_tgt.tilt_offset << "\n";
    f << "xf_ax0=" << g_xf.ax[0] << "\n";
    f << "xf_ax1=" << g_xf.ax[1] << "\n";
    f << "xf_ax2=" << g_xf.ax[2] << "\n";
    f << "xf_sgn0=" << g_xf.sgn[0] << "\n";
    f << "xf_sgn1=" << g_xf.sgn[1] << "\n";
    f << "xf_sgn2=" << g_xf.sgn[2] << "\n";
    f << "xf_scale=" << g_xf.scale << "\n";
    f << "net_max_step=" << g_net_max_step << "\n";
    f << "track_velocity=" << (g_track_velocity ? 1 : 0) << "\n";
    f << "xyjog_rot=" << g_xyjog_rot << "\n";
    f << "xyjog_step=" << g_xyjog_step << "\n";
    f << "xyjog_pan_sign=" << g_xyjog_pan_sign << "\n";
    f << "xyjog_tilt_sign=" << g_xyjog_tilt_sign << "\n";
    f << "joy_rate=" << g_joy_rate << "\n";
    f << "cal_std_min=" << g_cal_std_min << "\n";
    f << "cal_solve_base=" << (g_cal_solve_base ? 1 : 0) << "\n";
    f << "cal_base_reg=" << g_cal_base_reg << "\n";
    f << "cal_active=" << (g_cal_active ? 1 : 0) << "\n";
    f << "cal_pan0=" << g_cal_pan[0] << "\n";
    f << "cal_pan1=" << g_cal_pan[1] << "\n";
    f << "cal_pan2=" << g_cal_pan[2] << "\n";
    f << "cal_tilt0=" << g_cal_tilt[0] << "\n";
    f << "cal_tilt1=" << g_cal_tilt[1] << "\n";
    f << "cal_tilt2=" << g_cal_tilt[2] << "\n";
    f << "cal_np=" << g_cal_points.size() << "\n";
    for (size_t i = 0; i < g_cal_points.size(); i++) {
        const CalPoint& c = g_cal_points[i];
        std::string p = "cal" + std::to_string(i) + "_";
        f << p << "x=" << c.x << "\n";
        f << p << "y=" << c.y << "\n";
        f << p << "z=" << c.z << "\n";
        f << p << "pan=" << c.pan << "\n";
        f << p << "tilt=" << c.tilt << "\n";
    }
    for (size_t i = 0; i < g_axes.size(); i++) {
        MotorAxis& m = g_axes[i];
        std::string p = "axis" + std::to_string(i) + "_";
        f << p << "zero_ref="      << m.zero_ref       << "\n";
        f << p << "velocity="      << m.velocity       << "\n";
        f << p << "accel="         << m.accel          << "\n";
        f << p << "jerk="          << m.jerk_pct       << "\n";
        f << p << "max_velocity="  << m.max_velocity   << "\n";
        f << p << "max_accel="     << m.max_accel      << "\n";
        f << p << "error_limit="   << m.error_limit_deg<< "\n";
        f << p << "pos_min="       << m.pos_min_deg    << "\n";
        f << p << "pos_max="       << m.pos_max_deg    << "\n";
        f << p << "limits_enabled="<< (m.limits_enabled ? 1 : 0) << "\n";
    }
}

// ---------------------------------------------------------------------------
// connect + auto-enable
// ---------------------------------------------------------------------------
static void motor_enable_all() {
    for (MotorAxis& m : g_axes) {
        motor_try("enable", [&] { m.axis->ClearFaults(); m.axis->AmpEnableSet(true); });
    }
}

// Connect, restore saved config, then clear faults + enable every axis.
static void startup_connect() {
    motor_connect();
    config_apply_axes();
    motor_enable_all();
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
    // displayed = raw - zero_ref
    ImGui::Text("Actual:   %10.3f deg",   m.actual_position  - m.zero_ref);
    ImGui::Text("Command:  %10.3f deg",   m.command_position - m.zero_ref);
    ImGui::Text("Velocity: %10.3f deg/s", m.actual_velocity);

    if (m.amp_enabled) {
        if (ImGui::Button("Disable")) { motor_try("disable", [&] { m.axis->AmpEnableSet(false); }); }
    } else {
        if (ImGui::Button("Enable")) {
            motor_try("enable", [&] { m.axis->ClearFaults(); m.axis->AmpEnableSet(true); });
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Faults")) { motor_try("clear faults", [&] { m.axis->ClearFaults(); }); }
    ImGui::SameLine();
    if (ImGui::Button("Stop")) { motor_try("stop", [&] { m.axis->Abort(); }); }

    ImGui::InputFloat("velocity (deg/s)", &m.velocity);
    ImGui::InputFloat("accel (deg/s^2)",  &m.accel);
    ImGui::SliderFloat("jerk %", &m.jerk_pct, 0.0f, 100.0f);

    float v = std::min(m.velocity, m.max_velocity);
    float a = std::min(m.accel, m.max_accel);

    // homing: zero here (persisted) + go to zero
    if (ImGui::Button("Set Zero (home here)")) {
        m.zero_ref = m.actual_position;   // raw position becomes the new 0
        config_save();
    }
    ImGui::SameLine();
    if (ImGui::Button("Go Home (0)")) {
        motor_try("go home", [&] { m.axis->MoveSCurve(m.zero_ref, v, a, a, m.jerk_pct); });
    }

    ImGui::InputFloat("target (deg)", &m.target_deg);
    if (ImGui::Button("Move Abs")) {
        motor_try("move abs", [&] { m.axis->MoveSCurve(m.target_deg + m.zero_ref, v, a, a, m.jerk_pct); });
    }

    ImGui::InputFloat("jog (deg)", &m.jog_deg);
    if (ImGui::Button("Jog -")) {
        motor_try("jog -", [&] { m.axis->MoveRelative(-m.jog_deg, v, a, a, m.jerk_pct); });
    }
    ImGui::SameLine();
    if (ImGui::Button("Jog +")) {
        motor_try("jog +", [&] { m.axis->MoveRelative(m.jog_deg, v, a, a, m.jerk_pct); });
    }

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
        if (ImGui::Button("Apply Limits")) { motor_apply_limits(m); config_save(); }
        ImGui::TreePop();
    }

    ImGui::PopID();
}

static void gui_draw() {
    if (ImGui::Begin("GalvoCam Motor Control")) {
        if (g_controller == nullptr) {
            if (ImGui::Button("Connect")) {
                startup_connect();
            }
        } else {
            if (ImGui::Button("Abort All")) {
                for (MotorAxis& m : g_axes) {
                    motor_try("abort all", [&] { m.axis->Abort(); });
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Home both (0,0)")) {
                g_net_aim = false; g_moving_target = false;   // stop aiming, then go home
                aim_pan_tilt(0.0, 0.0);
            }
            ImGui::SameLine();
            if (ImGui::Button("Save Settings")) { config_save(); }
        }
        ImGui::TextWrapped("Status: %s", g_status.c_str());

        if (g_controller != nullptr) {
            cal_capture_tick();
            ImGui::Separator();
            ImGui::InputDouble("counts/rev", &g_counts_per_rev, 0.0, 0.0, "%.0f");
            ImGui::SameLine();
            ImGui::Text("(%.1f counts/deg)", g_counts_per_rev / 360.0);
            if (ImGui::Button("Apply units")) { motor_apply_units(); config_save(); }

            // --- targeting ---
            ImGui::SeparatorText("Targeting (world 3D -> pan/tilt)");
            if (g_moving_target) {
                double t = glfwGetTime();
                g_target[0] = g_tgt.base[0] + 300.0f * cosf((float)t);
                g_target[1] = g_tgt.base[1] + 300.0f * sinf((float)t);
                g_target[2] = g_tgt.base[2] + 1000.0f;
            }
            ImGui::InputFloat3("target x,y,z", g_target);
            PanTilt pt = targeting_solve(g_target);
            ImGui::Text("az %.2f  el %.2f deg  ->  pan %.2f  tilt %.2f deg",
                        pt.az, pt.el, pt.pan_motor, pt.tilt_motor);
            if (ImGui::Button("Aim at target")) { targeting_aim(g_target); }
            ImGui::SameLine();
            ImGui::Checkbox("track (moving target)", &g_moving_target);

            // network target receiver (UDP, one-way from the Linux triangulation box)
            ImGui::SeparatorText("Network target (UDP one-way)");
            ImGui::InputInt("UDP port", &g_net_port);
            bool rx = g_net_running.load();
            if (ImGui::Checkbox("receive targets", &rx)) {
                if (rx) net_start(g_net_port); else net_stop();
            }
            float ntgt[3] = {0.0f, 0.0f, 0.0f};
            {
                std::lock_guard<std::mutex> lk(g_net_mutex);
                double age = (g_net_count > 0) ? (glfwGetTime() - g_net_last_recv) : -1.0;
                ImGui::Text("rx %llu pkts  seq %u  id %u  %s  age %.2fs",
                            (unsigned long long)g_net_count, g_net_seq, g_net_id,
                            g_net_valid ? "VALID" : "no-target", age);
                ntgt[0] = g_net_target[0]; ntgt[1] = g_net_target[1]; ntgt[2] = g_net_target[2];
                ImGui::Text("last target: %.1f, %.1f, %.1f", ntgt[0], ntgt[1], ntgt[2]);
            }
            {   // live preview: what the received target solves to (verify before aiming)
                PanTilt npt = targeting_solve(ntgt);
                ImGui::Text("  solves to: az %.1f el %.1f  ->  pan %.1f tilt %.1f",
                            npt.az, npt.el, npt.pan_motor, npt.tilt_motor);
            }
            ImGui::Checkbox("aim at network target", &g_net_aim);
            ImGui::SameLine();
            if (ImGui::Checkbox("velocity pursuit", &g_track_velocity)) { config_save(); }
            if (g_track_velocity) {
                ImGui::TextDisabled("time-optimal: full speed, brakes at max accel -- raise the "
                                    "per-axis max velocity / max accel to go faster");
            }

            // control channel (request/reply, orange commands angles / uploads calibration)
            ImGui::SeparatorText("Control channel (UDP request/reply)");
            bool ctrl_on = g_ctrl_sock != BAD_SOCK;
            if (ctrl_on) ImGui::BeginDisabled();
            ImGui::InputInt("control port", &g_ctrl_port);
            if (ctrl_on) ImGui::EndDisabled();
            if (ImGui::Checkbox("control channel", &ctrl_on)) {
                if (ctrl_on) ctrl_start(g_ctrl_port); else ctrl_stop();
                g_ctrl_enabled = g_ctrl_sock != BAD_SOCK;
                config_save();
            }
            if (ImGui::Checkbox("allow remote control (moves + calibration)", &g_remote_allowed)) {
                config_save();
            }
            {
                double age = (g_ctrl_count > 0) ? (glfwGetTime() - g_ctrl_last_recv) : -1.0;
                ImGui::Text("rx %llu cmds  age %.2fs  %s",
                            (unsigned long long)g_ctrl_count, age,
                            g_calib_mode ? "CALIB MODE (target aiming paused)" : "");
            }

            // network-target aiming. Suppressed in calib mode so remote
            // SET_ANGLES moves aren't fought.
            bool net_tracking = g_net_aim && g_net_running && !g_calib_mode;
            double goal_pan = 0.0, goal_tilt = 0.0;   // no valid target -> home (0,0)
            if (net_tracking) {
                float tgt[3]; bool valid;
                { std::lock_guard<std::mutex> lk(g_net_mutex);
                  tgt[0] = g_net_target[0]; tgt[1] = g_net_target[1]; tgt[2] = g_net_target[2];
                  valid = g_net_valid; }
                if (valid) {
                    PanTilt pt = targeting_solve(tgt);
                    goal_pan = pt.pan_motor; goal_tilt = pt.tilt_motor;
                }
            }
            static double last_t = 0.0;
            double now = glfwGetTime();
            if (net_tracking && g_track_velocity) {
                // velocity pursuit: every frame, no throttle
                track_velocity_step(goal_pan, goal_tilt);
                g_track_was = true;
            } else {
                if (g_track_was) { track_velocity_stop(); g_track_was = false; }
                // discrete fallback: slew-limited point-to-point re-aim at ~30 Hz
                if (now - last_t >= 0.033 && !g_calib_mode) {
                    if (net_tracking) {
                        static double cur_pan = 0.0, cur_tilt = 0.0; static bool have = false;
                        if (!have) { cur_pan = goal_pan; cur_tilt = goal_tilt; have = true; }
                        double s = (double)g_net_max_step;   // slew cap per update
                        cur_pan  += std::min(s, std::max(-s, goal_pan  - cur_pan));
                        cur_tilt += std::min(s, std::max(-s, goal_tilt - cur_tilt));
                        aim_pan_tilt(cur_pan, cur_tilt);
                    } else if (g_moving_target && !g_calib_mode) {
                        targeting_aim(g_target);
                    }
                    last_t = now;
                }
            }

            if (ImGui::TreeNode("Targeting calibration")) {
                ImGui::TextDisabled("Filled by orange's calibration upload; hand-edit only for bench tests.");
                ImGui::InputFloat3("base pos (world)", g_tgt.base);
                ImGui::InputFloat3("rot xyz (deg)", g_tgt.rot);
                ImGui::InputInt("pan axis",  &g_tgt.pan_axis);
                ImGui::InputInt("tilt axis", &g_tgt.tilt_axis);
                ImGui::InputFloat("pan sign",    &g_tgt.pan_sign);
                ImGui::InputFloat("pan scale",   &g_tgt.pan_scale);
                ImGui::InputFloat("pan offset",  &g_tgt.pan_offset);
                ImGui::InputFloat("tilt sign",   &g_tgt.tilt_sign);
                ImGui::InputFloat("tilt scale",  &g_tgt.tilt_scale);
                ImGui::InputFloat("tilt offset", &g_tgt.tilt_offset);
                if (ImGui::Button("Save calibration")) { config_save(); }
                ImGui::TreePop();
            }

            if (ImGui::TreeNode("Coordinate frame (world -> app)")) {
                const char* axnames[] = { "in X", "in Y", "in Z" };
                ImGui::TextDisabled("map each app axis to an incoming axis + sign");
                ImGui::Combo("app X (right) <-", &g_xf.ax[0], axnames, 3);
                ImGui::SameLine(); { bool n = g_xf.sgn[0] < 0; if (ImGui::Checkbox("neg##x", &n)) g_xf.sgn[0] = n ? -1.0f : 1.0f; }
                ImGui::Combo("app Y (up) <-", &g_xf.ax[1], axnames, 3);
                ImGui::SameLine(); { bool n = g_xf.sgn[1] < 0; if (ImGui::Checkbox("neg##y", &n)) g_xf.sgn[1] = n ? -1.0f : 1.0f; }
                ImGui::Combo("app Z (fwd) <-", &g_xf.ax[2], axnames, 3);
                ImGui::SameLine(); { bool n = g_xf.sgn[2] < 0; if (ImGui::Checkbox("neg##z", &n)) g_xf.sgn[2] = n ? -1.0f : 1.0f; }
                ImGui::InputFloat("coord scale", &g_xf.scale);
                ImGui::InputFloat("max step/update (deg)", &g_net_max_step);
                if (ImGui::Button("Save coord frame")) { config_save(); }
                ImGui::TreePop();
            }

            if (ImGui::TreeNode("Correspondence calibration (teach)")) {
                ImGui::TextDisabled("X-Y jog to center the object in the camera, then capture");
                if (ImGui::Button(" Left "))  xy_jog(-g_xyjog_step, 0.0);
                ImGui::SameLine(); if (ImGui::Button(" Right ")) xy_jog(+g_xyjog_step, 0.0);
                ImGui::SameLine(); if (ImGui::Button(" Up "))    xy_jog(0.0, +g_xyjog_step);
                ImGui::SameLine(); if (ImGui::Button(" Down "))  xy_jog(0.0, -g_xyjog_step);
                ImGui::InputFloat("jog step (deg)", &g_xyjog_step);
                { bool n = g_xyjog_pan_sign  < 0; if (ImGui::Checkbox("invert L/R", &n)) g_xyjog_pan_sign  = n ? -1.0f : 1.0f; }
                ImGui::SameLine();
                { bool n = g_xyjog_tilt_sign < 0; if (ImGui::Checkbox("invert U/D", &n)) g_xyjog_tilt_sign = n ? -1.0f : 1.0f; }
                if (ImGui::Button("Rotate view 90")) g_xyjog_rot = (g_xyjog_rot + 1) & 3;
                ImGui::SameLine(); ImGui::Text("view rot: %d deg CCW", (g_xyjog_rot & 3) * 90);

                // on-screen joystick: hold + drag to jog continuously (velocity control)
                {
                    ImVec2 sz(140.0f, 140.0f);
                    ImVec2 p0 = ImGui::GetCursorScreenPos();
                    ImGui::InvisibleButton("joy", sz);
                    bool active = ImGui::IsItemActive();
                    ImVec2 center(p0.x + sz.x * 0.5f, p0.y + sz.y * 0.5f);
                    float radius = sz.x * 0.5f - 10.0f;
                    ImVec2 knob = center;
                    double jx = 0.0, jy = 0.0;
                    if (active) {
                        ImVec2 mp = ImGui::GetIO().MousePos;
                        float dx = mp.x - center.x, dy = mp.y - center.y;
                        float len = sqrtf(dx * dx + dy * dy);
                        if (len > radius && len > 0.0f) { dx *= radius / len; dy *= radius / len; }
                        knob = ImVec2(center.x + dx, center.y + dy);
                        jx =  (double)dx / radius;    // +right
                        jy = -(double)dy / radius;    // +up (screen y is down)
                    }
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    dl->AddCircle(center, radius, IM_COL32(130, 130, 130, 255), 32, 2.0f);
                    dl->AddCircleFilled(knob, 11.0f, active ? IM_COL32(90, 200, 120, 255) : IM_COL32(110, 110, 110, 255));
                    static bool joy_was = false;
                    if (active)          { xy_jog_velocity(jx * g_joy_rate, jy * g_joy_rate); joy_was = true; }
                    else if (joy_was)    { xy_jog_velocity(0.0, 0.0); joy_was = false; }   // stop on release
                }
                ImGui::SliderFloat("joystick rate (deg/s)", &g_joy_rate, 1.0f, 180.0f);
                ImGui::Separator();
                if (g_capturing) {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "capturing... %d samples", g_cap_n);
                } else if (ImGui::Button("Capture live point (0.5s avg)")) {
                    cal_capture_begin();
                }
                ImGui::Text("points: %d   RMS: %s", (int)g_cal_points.size(),
                            g_cal_rms < 0 ? "-" : (std::to_string(g_cal_rms) + " deg").c_str());
                ImGui::Text("az spread: %.2f   el spread: %.2f deg  (suppressed if < min)",
                            g_cal_std_az, g_cal_std_el);
                ImGui::InputFloat("min spread to use (deg)", &g_cal_std_min);
                ImGui::Checkbox("solve base position (fixes coplanar/floor points)", &g_cal_solve_base);
                if (g_cal_solve_base) {
                    ImGui::InputFloat("base anchor strength", &g_cal_base_reg);
                    ImGui::SameLine();
                    ImGui::TextDisabled(g_cal_base_solved ? "(base moved)" : "(needs >= 5 pts)");
                }
                ImGui::BeginChild("callog", ImVec2(0.0f, 120.0f), ImGuiChildFlags_Borders);
                for (size_t i = 0; i < g_cal_points.size(); i++) {
                    const CalPoint& c = g_cal_points[i];
                    float tp[3] = { (float)c.x, (float)c.y, (float)c.z };
                    double az, el; world_to_azel(tp, az, el);
                    ImGui::Text("%2d: xyz(%.0f, %.0f, %.0f)  az %.1f el %.1f  -> pan %.1f tilt %.1f",
                                (int)i, c.x, c.y, c.z, az, el, c.pan, c.tilt);
                }
                ImGui::EndChild();
                if (ImGui::Button("Delete last") && !g_cal_points.empty()) { g_cal_points.pop_back(); g_cal_rms = -1.0; }
                ImGui::SameLine();
                if (ImGui::Button("Fit")) { if (cal_fit()) config_save(); }
                ImGui::SameLine();
                if (ImGui::Button("Clear points")) { g_cal_points.clear(); g_cal_rms = -1.0; g_cal_active = false; }
                ImGui::Checkbox("use calibration", &g_cal_active);
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
// render plumbing (ImGui + GLFW + OpenGL2)
// ---------------------------------------------------------------------------
static void glfw_error_callback(int code, const char* desc) {
    printf("GLFW error %d: %s\n", code, desc ? desc : "(none)");
}

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

int main() {
    config_load_file();
    config_load_globals();

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        printf("glfwInit failed\n");
        return 1;
    }

    // No GL version hints: this PC has no GPU driver (Microsoft Basic Display Adapter
    // = OpenGL 1.1 only), so take the default context and use the legacy OpenGL2 backend.
    GLFWwindow* window = glfwCreateWindow(560, 900, "GalvoCam Motor Control", nullptr, nullptr);
    if (window == nullptr) {
        const char* desc = nullptr;
        int code = glfwGetError(&desc);
        printf("glfwCreateWindow failed (code %d): %s\n", code, desc ? desc : "(no description)");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    startup_connect();   // auto-connect + clear faults + enable on startup
    if (g_ctrl_enabled) ctrl_start(g_ctrl_port);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ctrl_poll();
        create_new_frame();
        gui_draw();
        render_a_frame(window);
    }

    ctrl_stop();
    net_stop();
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
