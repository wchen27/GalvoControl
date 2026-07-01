#!/usr/bin/env bash
# GalvoCam motor control - Linux build.
#
# Prereqs (see UBUNTU_SETUP.md):
#   - cmake + a C++ toolchain (build-essential)
#   - vcpkg with VCPKG_ROOT exported, and:
#       vcpkg install glfw3 'imgui[glfw-binding,opengl2-binding]'
#   - GL/X11 dev packages: libgl1-mesa-dev xorg-dev
#   - RMP-Linux installed (RapidCode .so + headers + RapidServer)
set -euo pipefail

: "${VCPKG_ROOT:?Set VCPKG_ROOT to your vcpkg install, e.g. export VCPKG_ROOT=\$HOME/vcpkg}"

# ADJUST if your RMP-Linux install is not at /rsi:
RSI_DIR="${RSI_DIR:-/rsi}"

cmake -B build -S . \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
  -DRSI_DIR="$RSI_DIR"

cmake --build build -j"$(nproc)"
echo "Built: build/motor_control"
