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
    float base[3] = {0.0f, 0.0f, 0.0f};
    float rot[3]  = {0.0f, 0.0f, 0.0f};   // world->galvo frame, Euler XYZ deg (R = Rz*Ry*Rx)
    int   pan_axis  = 0;
    int   tilt_axis = 1;
    float pan_sign  = 1.0f, pan_scale  = 0.5f, pan_offset  = 0.0f;
    float tilt_sign = 1.0f, tilt_scale = 0.5f, tilt_offset = 0.0f;
};
static TargetConfig g_tgt;
static float g_target[3]     = {0.0f, 0.0f, 1000.0f};
static bool  g_moving_target = false;

struct PanTilt { double pan_motor, tilt_motor, az, el; };

// v_galvo = R^T * (target - base), R = Rz(rz)*Ry(ry)*Rx(rx). Identity rot
// reproduces the old axis-aligned behavior.
static void targeting_world_to_galvo(const float* target, double* v) {
    const double DEG2RAD = 1.0 / RAD2DEG;
    double rx = g_tgt.rot[0] * DEG2RAD;
    double ry = g_tgt.rot[1] * DEG2RAD;
    double rz = g_tgt.rot[2] * DEG2RAD;
    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);
    // rows of R = Rz*Ry*Rx
    double R[3][3] = {
        {cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx},
        {sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx},
        {-sy,     cy * sx,                cy * cx},
    };
    double d[3] = {target[0] - g_tgt.base[0],
                   target[1] - g_tgt.base[1],
                   target[2] - g_tgt.base[2]};
    // v = R^T * d
    for (int i = 0; i < 3; i++) {
        v[i] = R[0][i] * d[0] + R[1][i] * d[1] + R[2][i] * d[2];
    }
}

static PanTilt targeting_solve(const float* target) {
    double v[3];
    targeting_world_to_galvo(target, v);
    double az = atan2(v[0], v[2]) * RAD2DEG;
    double el = atan2(v[1], sqrt(v[0] * v[0] + v[2] * v[2])) * RAD2DEG;
    PanTilt pt;
    pt.az = az; pt.el = el;
    pt.pan_motor  = g_tgt.pan_sign  * az * g_tgt.pan_scale  + g_tgt.pan_offset;
    pt.tilt_motor = g_tgt.tilt_sign * el * g_tgt.tilt_scale + g_tgt.tilt_offset;
    return pt;
}

// Discrete single-point aim (point-to-point). raw command = displayed angle + zero_ref.
static void targeting_aim(const float* target) {
    PanTilt pt = targeting_solve(target);
    auto cmd = [&](int idx, double angle) {
        if (idx < 0 || idx >= (int)g_axes.size()) return;
        MotorAxis& m = g_axes[idx];
        float v = std::min(m.velocity, m.max_velocity);
        float a = std::min(m.accel, m.max_accel);
        motor_try("aim", [&] { m.axis->MoveSCurve(angle + m.zero_ref, v, a, a, m.jerk_pct); });
    };
    cmd(g_tgt.pan_axis,  pt.pan_motor);
    cmd(g_tgt.tilt_axis, pt.tilt_motor);
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
            if (ImGui::Button("Save Settings")) { config_save(); }
        }
        ImGui::TextWrapped("Status: %s", g_status.c_str());

        if (g_controller != nullptr) {
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
            {
                std::lock_guard<std::mutex> lk(g_net_mutex);
                double age = (g_net_count > 0) ? (glfwGetTime() - g_net_last_recv) : -1.0;
                ImGui::Text("rx %llu pkts  seq %u  id %u  %s  age %.2fs",
                            (unsigned long long)g_net_count, g_net_seq, g_net_id,
                            g_net_valid ? "VALID" : "no-target", age);
                ImGui::Text("last target: %.1f, %.1f, %.1f",
                            g_net_target[0], g_net_target[1], g_net_target[2]);
            }
            ImGui::Checkbox("aim at network target", &g_net_aim);

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

            // re-aim BOTH axes at ~30 Hz; a valid network target takes priority over the test
            // circle. Suppressed in calib mode so remote SET_ANGLES moves aren't fought.
            static double last_t = 0.0;
            double now = glfwGetTime();
            if (now - last_t >= 0.033 && !g_calib_mode) {
                if (g_net_aim && g_net_running) {
                    float tgt[3]; bool valid;
                    { std::lock_guard<std::mutex> lk(g_net_mutex);
                      tgt[0] = g_net_target[0]; tgt[1] = g_net_target[1]; tgt[2] = g_net_target[2];
                      valid = g_net_valid; }
                    if (valid) targeting_aim(tgt);
                } else if (g_moving_target) {
                    targeting_aim(g_target);
                }
                last_t = now;
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
