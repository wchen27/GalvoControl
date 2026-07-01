@echo off
REM Launch the motor control app. Build first with build.bat.
REM Close RapidSetup / MR Configurator2 first - only one master can own the drives.

REM RapidCode64.dll pulls in KDL.dll, VirtualMachine64.dll, the VMPlugin* DLLs, etc.
REM Putting the RMP install dir on PATH lets the exe resolve them at runtime.
set RSI_DIR=C:\RSI\10.4.4
set PATH=%RSI_DIR%;%PATH%

REM This PC has no GPU driver / OpenGL ICD, so we ship Mesa's opengl32.dll next to
REM the exe and force its software (llvmpipe) renderer.
set GALLIUM_DRIVER=llvmpipe

set EXE=build\Release\motor_control.exe
if not exist %EXE% (
    echo Not built yet. Run build.bat first.
    exit /b 1
)
%EXE% %*
