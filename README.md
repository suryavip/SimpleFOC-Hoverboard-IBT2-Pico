# SimpleFOC x Hoverboard x IBT2 x Pico
Examples for implementing [SimpleFOC library](https://www.simplefoc.com) for driving hoverboard motor with 2x IBT2 modules and Raspberry Pi Pico.

## Warning:
Powering Hoverboard motor is involving a lot of power. You can use my code and follow my guide, but at your own risk. Don't proceed until you understand the risk and how to mitigate and handle it.

Also, this project is still in progress. So far I'm able to make it work with open-loop velocity mode. The goal of this project is close-loop mode for Force Feedback Steering Wheel application.

## Hardware:
- Raspberry Pi Pico.
- 2x of IBT2 H bridge driver module (each module have 2x BTS7960).
- Hoverboard motor.
- AS5600 magnetic sensor module.
- 12V power supply. 24V might do better for hoverboard motor.

## Software & Libraries:
- Arduino IDE (at the time of writing, I use version 2.0.3).
- [Arduino-Pico board library](https://github.com/earlephilhower/arduino-pico) by @earlephilhower.
- [SimpleFOC library](https://www.simplefoc.com).

## Software Preparation:
At least on SimpleFOC v2.2.3, there will be compiling errors when using Arduino-Pico board library by @earlephilhower (Something about `pwm_gpio_to_slice_num` is not declared in this scope). To fix that, locate this file
```
<your arduino libraries folder>/Simple_FOC/src/drivers/hardware_specific/rp2040_mcu.cpp
```
and add this line after other `#include`:
```cpp
#include "hardware/pwm.h"
```

## Wiring Connections:
You can see Raspberry Pi Pico's pinout diagram [here](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf).
| Pi Pico | 1st IBT2 | 2nd IBT2 | Power Supply | Motor | AS5600 |
| --- | --- | --- | --- | --- | --- |
| 3V3_OUT | VCC | VCC | | | VCC |
| GND | GND | GND | | | GND |
| GP0 | | | | | SDA |
| GP1 | | | | | SCL |
| GP12 | R_EN |
| GP13 | RPWM |
| GP14 | L_EN |
| GP15 | LPWM |
| GP16 | | R_EN |
| GP17 | | RPWM |
| | B+ | B+ | + terminal |
| | B- | B- | - terminal |
| | M+ | | | Phase A |
| | M- | | | Phase B |
| | | M+ | | Phase C |
| | | M- |