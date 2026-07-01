@echo off
setlocal

REM Build the GalvoCam motor control app (Windows, x64).
REM
REM Requires:
REM   - CMake and a C++ toolchain (Visual Studio 2019/2022)
REM   - vcpkg, with:  vcpkg install glfw3 imgui[glfw-binding,opengl3-binding]
REM   - RSI RMP installed (default C:\RSI\10.4.4; override with RSI_DIR below)

if "%VCPKG_ROOT%"=="" (
    echo ERROR: VCPKG_ROOT is not set.
    echo   set VCPKG_ROOT=C:\vcpkg
    exit /b 1
)

set BUILD_DIR=build
if not "%RSI_DIR%"=="" set RSI_ARG=-DRSI_DIR=%RSI_DIR%

cmake -B %BUILD_DIR% -S . -A x64 ^
    -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake ^
    %RSI_ARG%
if errorlevel 1 exit /b 1

cmake --build %BUILD_DIR% --config Release
if errorlevel 1 exit /b 1

echo.
echo Build complete: %BUILD_DIR%\Release\motor_control.exe
endlocal
