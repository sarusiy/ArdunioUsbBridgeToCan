# ArdunioUsbBridgeToCan

Arduino Uno + MCP2515/TJA1050 CAN module firmware, acting as a USB-to-CAN
bridge for testing the CAN bus interface on the
[JC-ESP32P4-M3](https://github.com/sarusiy/JC-ESP32P4-M3) project.

The ESP32-P4 board has its own SPI-connected MCP2515/TJA1050 module for its
CAN bus interface. This Arduino, with a second MCP2515 module, sits on the
same physical CAN bus (CAN-H/CAN-L wired together, 120 ohm termination
jumper closed on both ends) and is used to simulate/test CAN traffic without
a real vehicle bus.

## Status: Phase 2 - Simulated OBD-II ECU (default), Phase 1 echo test preserved

`src/main.cpp` is now dual-mode via a compile-time build flag
(`ECHO_TEST_MODE`), selected per PlatformIO environment in `platformio.ini`:

- **`env:uno` (default)** — `ECHO_TEST_MODE=0`. Simulates a real vehicle ECU
  speaking standard SAE J1979 OBD-II over CAN:
  - Listens for OBD-II requests on `0x7DF` (functional/broadcast) and
    `0x7E0` (physical addressing for ECU1).
  - Responds on `0x7E8` using ISO 15765-4 single-frame format
    (`[len, mode+0x40, PID, data...]`) for mode `0x01` ("show current data"),
    supporting PIDs `0x00` (supported PIDs bitmask), `0x05` (coolant temp),
    `0x0C` (RPM), `0x0D` (vehicle speed), `0x11` (throttle position).
  - Internally simulates a plausible drive cycle (20s sine-wave RPM/speed/
    throttle oscillation) and a 60s coolant warm-up ramp, printing the
    simulated state over serial every 2 seconds.
  - Broadcasts generic simulated engine (`0x120`, 50 Hz), vehicle (`0x180`,
    20 Hz), and body (`0x220`, 2 Hz) frames to exercise passive CAN capture.
    These IDs and payload layouts are test definitions, not manufacturer data.
- **`env:uno_echo_test`** — `ECHO_TEST_MODE=1`. Preserves the original
  Phase 1 hardware-validation test: every second, sends a CAN frame
  (ID `0x100`) with a single data byte cycling through ASCII `'0'`-`'9'`,
  and logs whether the JC-ESP32P4-M3 board's echo matches what was last
  sent.

Previously confirmed on hardware: the echo test reports `TX ... -> OK`
and `RX ... (echo matches last TX)` with zero errors across many frames,
and the base ECU simulator boots and prints evolving state. The newly added
periodic broadcast traffic and full P4 request/response integration still need
hardware validation.

Build/upload a specific environment with `-e uno` or `-e uno_echo_test`
(see Build section below).

The matching ESP32-P4 side lives in `src/main.c` of the JC-ESP32P4-M3 repo.
It actively queries the simulated ECU on `0x7DF`, consumes responses on
`0x7E8`, and captures periodic broadcasts. Only the dedicated Phase 1 test
ID `0x100` is echoed; arbitrary vehicle traffic is not echoed.

## Wiring

| MCP2515 pin | Uno pin |
|---|---|
| SCK | D13 |
| SI (MOSI) | D11 |
| SO (MISO) | D12 |
| CS | D10 |
| VCC | 5V |
| GND | GND |
| CAN-H / CAN-L | shared bus with the ESP32-P4's MCP2515 |

See `Doc/electrical drawing.vsdx` in the JC-ESP32P4-M3 repo for the original
wiring diagram.

## Build

```
platformio run -e uno              # simulated OBD-II ECU (default)
platformio run -e uno_echo_test     # original echo test
platformio run -e uno -t upload --upload-port COM4
platformio device monitor -p COM4 -b 115200
```

## Next steps

- Build and validate periodic broadcasts together with P4 OBD requests.
- Add simulated Mode 02 freeze-frame and Mode 03 DTC responses.
- Add ISO-TP multi-frame behavior for responses larger than one CAN frame.
- Add configurable fault, ignition, door, and driving scenarios.
