# ArdunioUsbBridgeToCan

Arduino Uno + MCP2515/TJA1050 CAN module firmware, acting as a USB-to-CAN
bridge for testing the CAN bus interface on the
[JC-ESP32P4-M3](https://github.com/sarusiy/JC-ESP32P4-M3) project.

The ESP32-P4 board has its own SPI-connected MCP2515/TJA1050 module for its
CAN bus interface. This Arduino, with a second MCP2515 module, sits on the
same physical CAN bus (CAN-H/CAN-L wired together, 120 ohm termination
jumper closed on both ends) and is used to simulate/test CAN traffic without
a real vehicle bus.

## Status: Phase 1 - HW connectivity test

`src/main.cpp` currently just proves the bus works end-to-end:
- Every second, sends a CAN frame (ID `0x100`) with a single data byte
  cycling through ASCII `'0'`-`'9'`, logged over USB serial.
- Listens for CAN frames and logs them; when the JC-ESP32P4-M3 board echoes
  the frame back, this reports whether the echoed byte matches what was
  last sent.

The matching ESP32-P4 side lives in `src/main.c` of the JC-ESP32P4-M3 repo,
which listens on its MCP2515 and echoes back any frame it receives.

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
platformio run
platformio run -t upload
platformio device monitor -b 115200
```

## Next steps (Phase 2)

Replace the test payload with an actual USB<->CAN bridge protocol (e.g.
SLCAN-style ASCII framing over serial) so a PC can inject/observe real CAN
frames via tools like `python-can`.
