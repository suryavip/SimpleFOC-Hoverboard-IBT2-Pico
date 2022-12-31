# SimpleFOC x Hoverboard x IBT2 x Pico
Examples for implementing [SimpleFOC library](https://www.simplefoc.com) for driving hoverboard motor with IBT2 modules and Raspberry Pi Pico.

## Hardware:
- Raspberry Pi Pico.
- 2pcs of IBT2 H bridge driver module (each module have 2 BTS7960).
- Hoverboard motor.
- AS5600 magnetic sensor module.
- 12V power supply. 24V might do better for hoverboard motor.

## Software & Libraries:
- Arduino IDE (at the time of writing, I use version 2.0.3).
- [SimpleFOC library](https://www.simplefoc.com).

## Wiring Connections:
You can see Raspberry Pi Pico's pinout diagram [here](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf).
| Raspberry Pi Pico | 1st IBT2 | 2nd IBT2 |
| --- | --- | --- |
| 3V3_OUT | VCC | VCC |
| GND | GND | GND |
| GP12 | R_EN | - |
| GP13 | RPWM | - |
| GP14 | L_EN | - |
| GP15 | LPWM | - |
| GP16 | - | R_EN |
| GP17 | - | RPWM |

As for now, I haven't continue to close-loop test yet. So the AS5600 magnetic sensor module will be unused until then.
