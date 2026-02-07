@echo off
REM Build script for PC Rocket Simulator
REM Requires Microsoft Visual C++ compiler (cl.exe)

echo Building Rocket Simulator...

pushd "%~dp0..\pc\src"

cl.exe /std:c++17 /EHsc /W3 /O2 rocket_sim.cpp setupapi.lib /Fe:..\..\rocket_sim.exe

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Build successful! Run with: rocket_sim.exe
) else (
    echo.
    echo Build failed!
)

popd
pause
