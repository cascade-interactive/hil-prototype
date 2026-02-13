![Status](https://img.shields.io/badge/Status-In%20Progress-ffb703) ![License](https://img.shields.io/badge/License-MIT-green)

# Hardware In The Loop - A proof of concept

Weekend project to see how far I can push a quick-and-dirty thrust vectoring loop: PC runs a 1 kHz 2D rocket sim, the ESP32 pretends to be the flight computer and outputs back a tilt command.

## What you get
- PC side: physics step + serial link at 921600 baud.
- ESP32 side: proportional controller that nudges the rocket from -5 deg to +5 deg over ~30 seconds.
- Metrics are printed so you can watch the loop behave.

## Fast setup
1. Plug in an ESP32 dev board.
2. Flash the firmware in `src/main.cpp` using PlatformIO. 
3. Build the PC sim on Windows with `scripts\build_pc.bat` (uses `cl.exe`).
4. Run `rocket_sim.exe`; it should auto-find the board and start communicating.

## How the link talks
- PC sends: `I,seq,velocity,angle` once per physics tick.
- ESP32 replies: `A,tilt`.
- Baud: 921600. If your cable/port flakes out, drop it in `pc/src/rocket_sim.cpp` and `platformio.ini` to match.

## Quick notes
- Tested on random ESP32 dev kits with USB serial.
- If prints stutter, close other serial monitors or try a lower baud.
- The physics is intentionally lightweight so timing is the thing we measure, not frame rate.
