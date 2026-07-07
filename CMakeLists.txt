cmake_minimum_required(VERSION 3.20)
project(galvocam_motor_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# RSI RMP / RapidCode linkage (platform-specific).
#
#   *** LINUX USERS: adjust RSI_DIR, the RapidCode library name, and RMP_RTA_PATH
#       to match your RMP-Linux install. See UBUNTU_SETUP.md. ***
# ---------------------------------------------------------------------------
if(WIN32)
    set(RSI_DIR "C:/RSI/10.4.4" CACHE PATH "RSI RMP (Windows) install directory")
    set(RSI_INCLUDE_DIR "${RSI_DIR}/include")
    set(RSI_LIB "${RSI_DIR}/lib/x64/RapidCode64.lib")
    # RapidCode resolves the RMP RTA files (RMP.rta) relative to this path.
    set(RMP_RTA_PATH "${RSI_DIR}" CACHE STRING "Path passed to CreateFromSoftware()")
else()
    # ---- ADJUST to your RMP-Linux install (from the RSI Linux package) ----
    set(RSI_DIR "/rsi" CACHE PATH "RSI RMP-Linux install directory")
    set(RSI_INCLUDE_DIR "${RSI_DIR}/include")
    # Confirm the exact .so name in the RMP-Linux package. find_library fails loudly
    # (rather than silently) if none of these names are present under the hints.
    find_library(RSI_LIB
        NAMES RapidCode rapidcode RapidCode64
        HINTS "${RSI_DIR}" "${RSI_DIR}/lib" "${RSI_DIR}/lib/x64"
        DOC "RapidCode shared library (.so)")
    # On Linux the client connects to the running RapidServer; empty -> CreateFromSoftware()
    # with no args. Set a path only if your RMP-Linux install documents one.
    set(RMP_RTA_PATH "" CACHE STRING "Path passed to CreateFromSoftware() (empty = default)")
endif()

# ---------------------------------------------------------------------------
# GUI dependencies (same on both platforms, via vcpkg):
#   vcpkg install glfw3 imgui[glfw-binding,opengl2-binding]
# ---------------------------------------------------------------------------
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(motor_control src/main.cpp)

target_include_directories(motor_control PRIVATE "${RSI_INCLUDE_DIR}")
target_compile_definitions(motor_control PRIVATE RMP_RTA_PATH="${RMP_RTA_PATH}")

target_link_libraries(motor_control PRIVATE
    imgui::imgui
    glfw
    OpenGL::GL
    "${RSI_LIB}")

if(UNIX)
    # RapidCode-Linux typically needs threads + libdl; harmless if unused.
    find_package(Threads REQUIRED)
    target_link_libraries(motor_control PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
    # Find the RapidCode .so at runtime without exporting LD_LIBRARY_PATH each run.
    set_target_properties(motor_control PROPERTIES
        BUILD_RPATH   "${RSI_DIR};${RSI_DIR}/lib;${RSI_DIR}/lib/x64"
        INSTALL_RPATH "${RSI_DIR};${RSI_DIR}/lib;${RSI_DIR}/lib/x64")
endif()

if(WIN32)
    target_link_libraries(motor_control PRIVATE ws2_32)   # UDP target receiver (Winsock)
endif()

# NOTE (Windows): RapidCode64.dll and its dependencies (KDL.dll, VMPlugin*, RMP.rta) are
# loaded from RSI_DIR via PATH - see run.bat. Do NOT copy RapidCode64.dll next to the exe;
# that breaks RapidCode's relative resolution of the RMP runtime.
