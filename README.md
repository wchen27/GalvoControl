# GalvoCam Motor Control

Windows/Linux Dear ImGui app that drives the GalvoCam pan/tilt mirror servos
(Mitsubishi MR-J5 over EtherCAT, via RSI RMP / RapidCode) and aims the
mirror-steered camera at live 3D targets streamed from the
[orange](https://github.com/moments-behavior/orange) multi-camera rig.

```
 orange (Linux)                          this app (Windows)
 ┌─────────────────────────┐             ┌──────────────────────────────────┐
 │ cameras → YOLO → 3D     │  GCT1 UDP   │ receiver thread → tracking thread│
 │ triangulation           ├────:5005───▶│  (extrapolate → solve → filter → │
 │                         │             │   velocity pursuit)              │
 │ calibration wizard,     │  GCC1/GCS1  │ control listener (UI thread)     │
 │ status, remote control  ├────:5006───▶│  angles / calib / status         │
 └─────────────────────────┘             └───────────────┬──────────────────┘
                                            RapidCode / EtherCAT
                                                 MR-J5 × 2 (pan, tilt)
```

Style follows `moments-behavior/orange` (CMake, ImGui + GLFW + OpenGL2,
snake_case functions, PascalCase types, `g_` globals).

Wire formats live in [PROTOCOL.md](PROTOCOL.md). Sender-side setup (static
IPs, reference sender code) lives in
[LINUX_SENDER_GUIDE.md](LINUX_SENDER_GUIDE.md) — note that orange now speaks
both protocols natively, so the guide is only needed for custom senders.

---

## Build

### Windows (x64, the deployment target)

Dependencies come from vcpkg:

```
vcpkg install glfw3 imgui[glfw-binding,opengl2-binding]

cmake -B build -S . -A x64 -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

Build **x64** — it links `RapidCode64.lib`. `RapidCode64.dll` is copied next
to the exe automatically. If your RMP is not at `C:/RSI/10.4.4`, pass
`-DRSI_DIR=<path>`. Links `ws2_32` (UDP) and `winmm` (1 ms timer resolution
for the tracking thread).

### Linux

See [UBUNTU_SETUP.md](UBUNTU_SETUP.md), then `./build.sh` (needs `VCPKG_ROOT`
and an RMP-Linux install; adjust `RSI_DIR`).

## Run

- The **RMP RTOS must be running** and the EtherCAT network operational.
- **Close RapidSetup / MR Configurator2 first** — only one master can own the
  drives at a time, or you'll get access errors.
- Launch → the app auto-connects, clears faults, and enables every axis. The
  control channel starts automatically (persisted setting).
- The config (`motor_control.cfg`, next to the exe) persists everything:
  zeros, limits, calibration, filters, network settings.

### First-time setup checklist

1. **Connect** (automatic on launch). Each axis appears with live state.
2. **Units**: UserUnits are `counts_per_rev / 360` counts/deg so everything
   in the UI is **degrees**. Default 2^23 (23-bit encoder); edit + *Apply
   units* if your motor differs.
3. **Zero**: aim the mirrors at the arena ("facing forward"), then per axis
   click **Set Zero (home here)**. The zeroed pose (0,0) is the reference
   everything else uses — calibration sweeps, park-at-home, remote angles.
4. **Travel limits**: with the mirrors coupled, set per-axis min/max degrees,
   tick *travel limits enabled*, **Apply Limits**. Remote/track commands are
   always clamped to these.
5. **Speed limits**: set per-axis *max velocity* / *max accel* to what the
   mechanics tolerate — these are the **only** limits on tracking speed (see
   Tuning below).
6. **Calibrate** (see Calibration below), then enable *receive targets* and
   *aim at network target*.

---

## The tracking pipeline

What happens between a UDP packet and mirror motion — and where each knob
acts:

1. **Receiver thread** (UDP :5005) keeps the newest-seq target. v2 packets
   carry position *at capture time*, velocity, and the sender's measured
   capture→send age; v1 packets still work (velocity 0).
2. **Tracking thread** (~500 Hz, dedicated — never blocked by the UI) each
   iteration:
   - **Extrapolates** the target to *now*:
     `pos + vel × (sender age + link age + servo lead)`. The link term is
     capped at 200 ms; a packet older than 1 s ⇒ target invalid ⇒ return to
     home. *(knob: servo lead)*
   - **World tracker**: dead-reckons on the packet velocity and blends each
     new measurement in over `track_tau` instead of snapping — packet-rate
     innovation steps never reach the mirrors. Innovations > 500 mm snap
     (re-acquisition). *(knob: target smooth tau)*
   - **Targeting solve**: world point → pan/tilt motor degrees through the
     calibrated model (see Calibration).
   - **One Euro goal filter**: adaptive low-pass — heavy smoothing while the
     goal is quasi-static (no buzz at rest), opens with goal rate (no lag at
     speed). *(knobs: min cutoff, speed beta)*
   - **Velocity pursuit**: time-optimal law — full `max_velocity` toward the
     goal, braking on the max-accel curve `v = √(2·a·d)`, a stable 40/s
     P-gain near the goal, a 0.02° soft deadband, plus **goal-rate
     feed-forward** from the packet velocity so a constant-speed target is
     ridden with ~zero chase error.
   - **Command hygiene**: `MoveVelocity` is sent only when the command
     meaningfully changed (100 ms keepalive), with accel scheduled to the
     size of the velocity change — small corrections are gentle, large
     maneuvers get full accel. Loop relaxes to 10 ms when settled.
3. **Safety**: goals always clamped to travel limits; velocity commanded to
   zero on every tracking exit (aim off, calib mode, link lost); calib mode
   auto-expires after 5 s without control traffic.

The **discrete fallback** ("velocity pursuit" unchecked) is the old
slew-limited 30 Hz point-to-point mode — keep it only for debugging; it
cannot follow fast targets (each S-curve plans to stop).

---

## Calibration

The targeting model maps a world point to motor angles:

```
v  = R(rot)ᵀ · (target − base)          base: pivot, world mm
az = atan2(v.x, v.z), el = elevation     rot: world→galvo Euler XYZ deg
pan  = pan_sign  · az · pan_scale  + pan_offset      (scale ≈ 0.5,
tilt = tilt_sign · el · tilt_scale + tilt_offset      mirror half-angle)
```

Two ways to fill it:

**1. Automated ChArUco calibration from orange (preferred).** Orange sweeps
the mirrors over the control channel while detecting a ChArUco board in the
galvo camera and a calibrated fixed camera, fits the full model, and uploads
it (`SET_CALIB`). Nothing to do on this side except *allow remote control*.
The upload resets the coordinate-frame mapping to identity and deactivates
any teach calibration — **last calibration wins**. See orange's
`docs/galvo_calibration_plan.md`.

**2. Teach calibration (manual fallback).** Under *Correspondence
calibration (teach)*: jog (buttons or the on-screen joystick) to center a
tracked object in the galvo camera, **Capture live point** (0.5 s average),
repeat for ≥ 3 (better 5+) positions spread in azimuth *and* elevation, then
**Fit**. The fit is an affine az/el→pan/tilt map; *solve base position* also
pattern-searches the pivot (needs ≥ 5 points; fixes the coplanar-points
case). Works entirely on this machine, no board needed.

The *Coordinate frame (world → app)* tree maps the sender's axis convention
manually — only relevant for teach calibration; the ChArUco upload supersedes
it.

---

## Tuning guide (in this order)

1. **Per-axis max accel / max velocity** — the only speed limits in velocity
   pursuit. Raise max accel aggressively (MR-J5 with a mirror load handles
   10000–20000 °/s²); if the following-error limit (5° default) trips,
   you've found the mechanical truth — back off.
2. **Step test** (below) — measures the servo's actual rise time and
   overshoot after every accel change.
3. **Servo lead (ms)** — the unmeasured share of latency: camera exposure/
   readout (before the sender's timestamp) + command transport + servo
   response. Best set empirically: track something moving at constant speed
   and watch the galvo image — trailing ⇒ raise, leading ⇒ lower
   (`offset = speed × lead error`, so one measurement converges). Err low.
4. **Goal filter** — with beta fixed, lower *min cutoff* until a stationary
   target shows no mirror buzz; then raise *speed beta* until fast motion
   shows no perceptible lag. Defaults 1.5 Hz / 0.3.
5. **Target smooth tau** — raise (60–80 ms) if a *moving* target still
   shimmers, lower (20 ms) if reactions to genuine maneuvers feel delayed.
6. Re-check the servo lead after any accel/drive-tuning change.

## Instrumentation

**HUD** (under *velocity pursuit*, while tracking):

| field | meaning |
|---|---|
| `rx` | target packets/s reaching the receiver |
| `sender age` | capture→send latency measured by orange (detection cost, live) |
| `link` | age of the newest packet (network + queueing) |
| `extrap` | total prediction applied = sender age + link + servo lead |
| `innov` | distance between prediction and each new measurement — your live measurement-noise meter; large values while moving = motion blur on the detection side |
| `err pan/tilt` | goal vs commanded position, deg — should be small and *steady*; flickering sign means loop trouble |

**Step test**: square-waves both axes (amplitude / half-period / cycles),
logs every 2 ms control iteration to `step_test.csv`
(`t,goal,cmd_pan,act_pan,cmd_tilt,act_tilt`), reports rise time (to 90 %) and
overshoot per axis in the panel, then parks at home. It bypasses all filters
— it measures the servo, not the smoothing. Run it after changing accel
limits or drive gains; its rise time is a good starting value for the servo
lead.

---

## Remote control (orange integration)

The GCC1/GCS1 control channel (UDP :5006) lets orange query status, command
raw angles, run the calibration sweep, and upload calibration — full spec in
[PROTOCOL.md](PROTOCOL.md). On this side:

- **allow remote control** gates every motion/config command (refused with
  an error code when off). `PING` and `STOP` always work.
- **Calib mode** (entered remotely for sweeps) pauses target-stream aiming
  and auto-expires after 5 s without control traffic — a dead sender can't
  leave the unit paused.
- Angles on the wire are in the **displayed (zeroed) frame** — the same
  numbers the UI shows.

## Config file

Everything persists in `motor_control.cfg` (flat `key=value`, next to the
exe; delete it to factory-reset). Highlights:

| key(s) | what |
|---|---|
| `counts_per_rev` | encoder counts per motor revolution |
| `axisN_zero_ref`, `axisN_pos_min/max`, `axisN_limits_enabled` | homing + travel limits |
| `axisN_velocity/accel/jerk`, `axisN_max_velocity/max_accel` | manual-move params + tracking speed limits |
| `axisN_error_limit` | following-error abort threshold (deg) |
| `tgt_base_*`, `tgt_rot_*`, `tgt_pan_*`, `tgt_tilt_*` | targeting model (uploaded by orange or hand-set) |
| `xf_*` | manual world→app axis mapping (teach path only) |
| `cal_*` | teach calibration (affine coefficients + captured points) |
| `track_velocity`, `servo_lead_ms`, `track_tau_ms` | tracking mode + prediction |
| `filt_on`, `filt_min_cutoff`, `filt_beta` | One Euro goal filter |
| `net_max_step` | discrete-fallback slew cap |
| `ctrl_port`, `ctrl_enabled`, `remote_allowed` | control channel |

## Troubleshooting

| symptom | check |
|---|---|
| "network not operational" / no axes on connect | RMP RTOS running? EtherCAT cabled + drives powered? RapidSetup/MR Configurator2 closed? |
| axis faults the moment it moves fast | following-error limit tripping: real mechanical limit reached, or drive gains too soft for the commanded accel — tune the drive, or lower max accel |
| `rx 0 Hz` while orange is streaming | port/IP mismatch (default :5005), firewall, or packet-size/version skew — a v2-sending orange needs this app ≥ the v2 receiver (48- and 80-byte packets both accepted here) |
| mirrors aim the wrong way / off by a lot | stale calibration: re-run orange's ChArUco calibration and upload; check the live "solves to" preview line against expectation before enabling aim |
| target valid but mirrors parked at home | packet older than 1 s (sender stalled), or target outside travel limits after solve |
| buzz at rest | lower goal-filter *min cutoff*; confirm `innov` is small (else detection noise) |
| shimmer only while moving | raise *target smooth tau*; if `innov` is large while moving, fix the detection side (shorter exposure — motion blur) |
| trailing/leading a fast target | servo lead too low/high (see Tuning) |
| remote commands refused (err 2) | *allow remote control* unchecked |
| UI sluggish while tracking | shouldn't happen (readback throttled, commands change-detected) — check the status line for a fault spamming exceptions |

## Safety notes

- The **following-error limit** (default 5°, applied on connect) aborts and
  drops torque on a runaway/stall.
- **Software travel limits** e-stop before a mechanical stop; every remote
  and tracking command is additionally clamped to them in software.
- Hardware limit inputs are disabled (none wired on this rig — they float to
  "tripped" otherwise).
- On exit the app stops tracking, aborts, and disables all amps.
