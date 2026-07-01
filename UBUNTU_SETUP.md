# GalvoCam Motor Control — Ubuntu 24.04 Setup & Run Guide

Guide for setting up, building, and running the GalvoCam motor-control app on an
**Ubuntu 24.04** machine. Written for an engineer or coding agent doing the install.

---

## 1. What this software is

A minimal **Dear ImGui** desktop app that controls **Mitsubishi MR-J5 servo drives**
over **EtherCAT** via **RSI RMP / RapidCode**. Two axes drive a two-mirror pan/tilt
"GalvoCam" that aims a camera's line of sight. Features:

- Per-axis enable / clear-faults / stop, jog, absolute move (all in **degrees**).
- Live numeric position/velocity/state readout + fault-reason display.
- Homing (set-zero-here + go-to-zero), speed limits, software travel limits.
- **Targeting**: given a world 3D point, computes pan/tilt and aims (a moving-target
  test mode is included).

Control stack:
```
ImGui + GLFW + OpenGL2 backend  →  RapidCode (client)  →  RMP RapidServer (RT)  →  EtherCAT  →  MR-J5 drives
```

## 2. Repo layout
```
motor_control/
├─ CMakeLists.txt      # cross-platform; Linux knobs are near the top
├─ build.sh / run.sh   # Linux build/run
├─ build.bat / run.bat # Windows (ignore on Linux)
├─ src/main.cpp        # the whole app
├─ README.md           # Windows-oriented notes
└─ UBUNTU_SETUP.md     # this file
```

---

## 3. Prerequisites (do these BEFORE building)

### 3a. RMP-Linux from RSI  ← the critical dependency
RMP runs on Linux as a supported product (PREEMPT_RT + a `RapidServer` service).
You must obtain from RSI (Robotic Systems Integration):
- The **RMP-Linux package** (RapidCode shared library `.so` + headers + `RapidServer`),
  version-compatible with the Windows install (10.4.4).
- A **Linux license**. Licenses are node-locked; the Windows license (SN-184880) likely
  does **not** transfer — confirm/obtain via RSI support. (An "RMP Support Access" contact
  exists for this project.)
- Note the **install path** (this guide assumes `/rsi`) and the **RapidCode `.so` name**.

### 3b. Real-time kernel
RMP-Linux needs **PREEMPT_RT** for deterministic EtherCAT. Easiest on Ubuntu 24.04:
```bash
sudo pro attach <token>            # Ubuntu Pro
sudo pro enable realtime-kernel
sudo reboot
uname -a                           # confirm "PREEMPT_RT"
```
RSI's tuning: isolate a CPU core for the RT task and use SCHED_FIFO priority (per RMP-Linux docs).

### 3c. Dedicated EtherCAT NIC
Like the INtime port on Windows, `RapidServer` binds a NIC for EtherCAT. Identify a spare
NIC (`ip link`), keep it off the normal network, and configure it per RMP-Linux docs.
The motor box's EtherCAT cable plugs into that NIC.

### 3d. Build tools + GUI dependencies
```bash
sudo apt update
sudo apt install -y build-essential cmake git curl zip unzip pkg-config \
                    libgl1-mesa-dev xorg-dev            # GL + X11 headers for GLFW

# vcpkg (no root):
git clone https://github.com/microsoft/vcpkg "$HOME/vcpkg"
"$HOME/vcpkg/bootstrap-vcpkg.sh"
export VCPKG_ROOT="$HOME/vcpkg"                          # add to ~/.bashrc
"$VCPKG_ROOT/vcpkg" install glfw3 'imgui[glfw-binding,opengl2-binding]'
```

---

## 4. Adjustable values (the only things Linux-specific)

All in **`CMakeLists.txt`** (the `else()` / Linux branch near the top), overridable on the
command line or in `build.sh`:

| Knob | Default | What to set it to |
|---|---|---|
| `RSI_DIR` | `/rsi` | Root of your RMP-Linux install (has `include/`, the `.so`, `RapidServer`). |
| RapidCode lib name | `find_library(NAMES RapidCode rapidcode RapidCode64 …)` | Add the exact `.so` base name if it differs (e.g. add to `NAMES`). |
| `RMP_RTA_PATH` | `""` (empty) | Leave empty to use `CreateFromSoftware()` (connects to RapidServer). Set only if RMP-Linux docs require a path. |

In code (`src/main.cpp`) the connect call already honors `RMP_RTA_PATH`; no code edit
needed unless the Linux RapidCode connect API differs — then adjust `motor_connect()`.

---

## 5. Build
```bash
cd motor_control
export VCPKG_ROOT="$HOME/vcpkg"
export RSI_DIR="/rsi"          # ADJUST to your RMP-Linux install
bash build.sh                  # or: chmod +x build.sh && ./build.sh
```
Produces `build/motor_control`. If `find_library` errors, the RapidCode `.so` name/path is
wrong — fix `RSI_DIR` or add the name to `find_library(NAMES …)` in CMakeLists.txt.

## 6. Run
```bash
# 1) Start RapidServer (per RMP-Linux docs; usually a service or a binary under $RSI_DIR).
#    Confirm the EtherCAT network reaches OPERATIONAL and sees the 2 MR-J5 drives.
# 2) Launch the app (may need sudo / rt-group membership for RapidServer access):
cd motor_control
bash run.sh                    # or: chmod +x run.sh && ./run.sh
```
Then in the app: **Connect** → the two axes appear → **Enable** → jog/move/target.

---

## 7. Using the app (same on all platforms)

- **counts/rev = 67108864** (2^26, HK-KT 26-bit encoder). This is the default; the field is
  live-editable if a commanded angle ≠ physical rotation. **Do not use 8388608 / "23-bit"** —
  the RSI quote's spec is wrong; the correct value is 67,108,864 (≈186,413.5 counts/deg).
- **Enable** does Clear Faults + amp enable. On connect the app **auto-disables the drive's
  hardware limit inputs** (no physical limit switches are wired on this rig — otherwise every
  move faults on a phantom limit).
- **Homing**: "Set Zero (home here)" defines the current shaft angle as 0°; "Go Home (0)"
  returns to 0°. (Absolute encoders; no home switch.)
- **Limits** section: speed limits (clamp commanded velocity/accel) and software travel
  limits (min/max degrees, off by default; enable + Apply once mirrors are coupled).
- **Targeting**: enter a world 3D `x,y,z` → shows az/el and pan/tilt → **Aim at target**;
  or tick **moving target** for a circling test. Calibration params (base pos, per-axis
  sign/scale/offset, pan/tilt axis assignment) are in "Targeting calibration". `scale`
  defaults 0.5 (2× mirror reflection). Orientation `R` and sampled calibration are TODO.

---

## 8. Troubleshooting (lessons carried from the Windows bring-up)

- **Connect fails: "device error … MemoryGet"** → RapidServer isn't running, or the app lacks
  privileges, or `RSI_DIR`/`.so` is wrong. Start RapidServer; run with sudo/rt group; verify
  the `.so` loaded (`ldd build/motor_control`).
- **Connect: "network not operational"** → EtherCAT not up: check the dedicated NIC, cabling
  (shielded, drive IN port), drive power (bus ≈ 300 V, not ~16), STO satisfied, E-stop cleared.
  The status box prints the RMP network log.
- **No window / OpenGL error** → Ubuntu ships Mesa, so this is usually fine. On a headless /
  GPU-less box, force software GL: `export LIBGL_ALWAYS_SOFTWARE=1` before `run.sh`
  (the Linux analog of the Windows Mesa `opengl32.dll` workaround).
- **Fault: hardware positive/negative limit** → should not occur (app disables hardware limits
  on connect); if it does, a different drive config is present — check `HardwarePosLimitActionSet`.
- **Move faults on position/following-error** → lower accel or raise the axis error limit (in
  "Limits"). The red **Fault:** line names the exact source bit.
- **Commanded 90° ≠ 90°** → wrong counts/rev; set 67108864.

---

## 9. Verification checklist
1. `uname -a` shows PREEMPT_RT.
2. `echo $VCPKG_ROOT` set; `vcpkg list` shows glfw3 + imgui.
3. RMP-Linux installed; RapidCode `.so` present under `$RSI_DIR`; RapidServer runs.
4. Dedicated EtherCAT NIC identified; motor box cabled to it; drives powered (bus ≈300 V).
5. `bash build.sh` → `build/motor_control` exists.
6. RapidServer running → `bash run.sh` → window opens → **Connect** → "connected: 2 axes".
7. **Enable** an axis, small **jog** moves the motor; **Set Zero**; a **Move Abs 90°** turns
   a clean quarter-turn (confirms counts/rev).
8. Targeting: enter a point / moving-target → both axes track.

If anything in steps 3–6 fails, it's almost always the **RMP-Linux install / RapidServer /
license / NIC** (section 3), not the app.
