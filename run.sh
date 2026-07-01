#!/usr/bin/env bash
# Launch the GalvoCam motor control app (Linux).
#
# The RMP RapidServer must be running first (see UBUNTU_SETUP.md). Depending on your
# RMP-Linux setup, RapidServer and/or this client may need elevated / real-time
# privileges (run via sudo or as a user in the configured rt group).
set -euo pipefail

BIN="./build/motor_control"
[ -x "$BIN" ] || { echo "Not built. Run ./build.sh first."; exit 1; }

# If no GPU / OpenGL driver is present (headless box), force Mesa software GL:
#   export LIBGL_ALWAYS_SOFTWARE=1
exec "$BIN" "$@"
