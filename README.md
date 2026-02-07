# Rocket Thrust Vectoring Simulator

A 2D rocket simulation system demonstrating real-time thrust vector control (TVC) using an ESP32 microcontroller as the flight computer.

## Overview

This project simulates a simple rocket with thrust vectoring capability. The PC runs a physics simulation at 1kHz and sends IMU data to an ESP32, which calculates trajectory corrections and sends actuator commands back. The rocket gradually tilts from -5° to +5° over 30 seconds using proportional control.

## Architecture

```
┌─────────────────┐     Serial 921600 baud      ┌─────────────────┐
│   PC Simulator  │ ◄─────────────────────────► │     ESP32       │
│   (1kHz Physics)│                             │ (Flight Control)│
└─────────────────┘                             └─────────────────┘
        │                                               │
        │  Sends: I,seq,velocity,angle                  │
        │  Recv:  A,tilt                                │
        └───────────────────────────────────────────────┘
```

## Project Structure

```
Sine Stream/
├── README.md              # This file
├── platformio.ini         # PlatformIO configuration for ESP32
├── .gitignore
├── src/
│   └── main.cpp           # ESP32 flight controller firmware
├── pc/
│   └── src/
│       └── rocket_sim.cpp # PC-side physics simulator
├── scripts/
│   └── build_pc.bat       # Build script for Windows
├── include/               # ESP32 header files (empty)
├── lib/                   # ESP32 libraries (empty)
└── test/                  # ESP32 tests (empty)
```

## Hardware Requirements

- ESP32 development board (any variant with USB-Serial)
- USB cable for programming and communication

## Software Requirements

- [PlatformIO](https://platformio.org/) (for ESP32 firmware)
- Microsoft Visual C++ compiler (for PC simulator)
  - Included with Visual Studio or Build Tools for Visual Studio

## Building

### ESP32 Firmware

Using PlatformIO CLI:
```bash
pio run --target upload
```

Or use the PlatformIO IDE extension in VS Code.

### PC Simulator (Windows)

Run the build script:
```bash
scripts\build_pc.bat
```

Or manually:
```bash
cd pc\src
cl.exe /std:c++17 /EHsc /W3 /O2 rocket_sim.cpp setupapi.lib /Fe:..\..\rocket_sim.exe
```

## Usage

1. Upload firmware to ESP32
2. Build and run the PC simulator: `rocket_sim.exe`
3. The simulator will auto-detect the ESP32 and begin communication
4. Watch the rocket gradually tilt from -5° to +5° over 30 seconds

## Serial Protocol

| Direction | Format | Description |
|-----------|--------|-------------|
| PC → ESP32 | `I,seq,velocity,angle\n` | IMU data: sequence number, velocity (m/s), body angle (deg) |
| ESP32 → PC | `A,tilt\n` | Actuator command: thrust vector tilt angle (deg) |

## Physics Model

- **Gravity**: 9.81 m/s²
- **Thrust**: 15 m/s² acceleration
- **Timestep**: 1ms (1kHz simulation rate)
- **Actuator range**: ±15° tilt

The rocket starts at Y=10m with the engine on. Thrust is applied along the rocket body axis, offset by the actuator tilt angle. Body angle responds to thrust vectoring over time.

## Control Algorithm

The ESP32 uses simple proportional control:

```cpp
targetAngle = START_ANGLE + (END_ANGLE - START_ANGLE) * (elapsed / TRANSITION_TIME)
error = targetAngle - bodyAngle
tiltCommand = error * CONTROL_GAIN
```

- **Start angle**: -5°
- **End angle**: +5°
- **Transition time**: 30 seconds
- **Control gain**: 0.5

## License

MIT License - feel free to use and modify.
