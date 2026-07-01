# GalvoCam Motor Control (minimal Dear ImGui)

A minimal Dear ImGui app to control the GalvoCam MR-J5 servos through RSI RMP
(RapidCode) over EtherCAT. Connects to the running RMP controller, binds every
axis, and gives per-axis enable / clear-faults / stop plus absolute-move and jog.

Style follows `moments-behavior/orange` (CMake, ImGui + GLFW + OpenGL3,
snake_case functions, PascalCase types, `g_` globals).

## Build (Windows, x64)

Dependencies come from vcpkg:

```
vcpkg install glfw3 imgui[glfw-binding,opengl3-binding]

cmake -B build -S . -A x64 -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

Build **x64** — it links `RapidCode64.lib`. `RapidCode64.dll` is copied next to
the exe automatically. If your RMP is not at `C:/RSI/10.4.4`, pass
`-DRSI_DIR=<path>`.

## Run

- The **RMP RTOS must be running** and the EtherCAT network operational.
- **Close RapidSetup / MR Configurator2 first** — only one master can own the
  drives at a time, or you'll get access errors.
- Click **Connect**. Each axis appears with live position/velocity/state.
- Per axis: **Enable** (clears faults + enables amp), set velocity/accel, then
  **Move Abs** or **Jog +/-**. **Abort All** stops everything.

## Notes / current config

- UserUnits are set to `8388608 / 360` counts/deg, so everything is in **degrees**
  (23-bit encoder). Actual = a full rev is 360.
- **Safety limits** are built in (per-axis "Safety Limits" section):
  - A **following-error limit** (default 5°) is applied on connect and **aborts**
    (drops torque) on a runaway/stall — runaway protection.
  - **Software travel limits** (min/max degrees) e-stop before the mirror can hit a
    mechanical stop. They ship **disabled** so full-range bench moves aren't blocked;
    once a motor is coupled to a mirror, set the real min/max, tick **travel limits
    enabled**, and click **Apply Limits**.
- Single-threaded: RapidCode calls run on the UI thread each frame, which is fine
  for this scale. Don't run it alongside RapidSetup.
