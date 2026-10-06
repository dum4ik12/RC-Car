---
name: esp32-pins
description: Choose, check or change ESP32 GPIO pins for the RC car (motor driver, servo, CRSF receiver UART, sensors, LEDs, battery ADC). Use whenever a pin is assigned, wired, moved or questioned, e.g. "which pin for X", "can I use GPIO N", boot or flashing problems caused by pins, or wiring questions.
---

# ESP32 (WROOM-32 DevKit) pin selection

## Current allocation (source of truth: `src/main.cpp` constants + the CLAUDE.md hardware map)
| GPIO | Used for | Notes |
|---|---|---|
| 25 | Steering servo | LEDC HS ch0 |
| 26 | BTS7960 RPWM | LEDC HS ch1 |
| 27 | BTS7960 LPWM | LEDC HS ch2 |
| 15 | BTS7960 R_EN | strapping pin, HIGH or toggling at boot (see below) |
| 2 | BTS7960 L_EN | strapping pin + on-board LED, 10 kΩ pull-down |
| 13 | CRSF RX (UART2) ← receiver TX | in use, 420000 baud; moved from 16 to match the wiring (2026-10) |
| 17 | CRSF TX (UART2) → receiver RX | configured by `CrsfReceiver`; needed only for telemetry |

## Pin classes
| Class | GPIOs | Rule |
|---|---|---|
| **Don't exist** | 20, 24, 28–31 | There is no "D28". Users sometimes read these from other boards. |
| **SPI flash** | 6–11 | **Never use.** Touching them crashes the chip. |
| **Input-only** | 34, 35, 36 (VP), 39 (VN) | No output, no internal pull-up/down. Good for ADC and receiver signals. |
| **UART0 / USB** | 1 (TX0), 3 (RX0) | Used for flashing and logs. Avoid. |
| **Strapping** | 0, 2, 5, 12, 15 | Their level at reset changes boot behavior (see the next table). |
| **Glitch at boot** | 0, 1, 3, 5, 14, 15 | These output a signal or PWM, or go HIGH, during boot. Don't put anything that must stay off at boot on them unless something else keeps it safe. |
| **ADC2** | 0, 2, 4, 12–15, 25–27 | `analogRead` fails while WiFi is on. Use **ADC1 (32–39)** for analog. |
| **Safe general I/O** | 4, 13, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 | First choice for outputs. 16/17 are only free on WROOM (WROVER uses them for PSRAM). |

## Strapping pins
| GPIO | At reset | Consequence |
|---|---|---|
| 0 | internal pull-up; LOW = download mode | the BOOT button. Don't pull it LOW. |
| 2 | must be LOW/floating to enter download mode | a pull-down is fine. **A pull-up breaks flashing.** |
| 5 | pull-up; SDIO timing | outputs PWM at boot |
| 12 (MTDI) | selects the flash voltage | **HIGH at boot → 1.8 V flash → boot loop.** Never pull it up. |
| 15 (MTDO) | pull-up; LOW silences the ROM boot log | a pull-down only hides the boot messages, which is harmless |

## Why the BTS7960 enables are safe on 15 and 2
The motor only gets current when **both** half-bridges are enabled. GPIO2 has a 10 kΩ pull-down, so L_EN stays LOW through reset and boot, even while GPIO15 (R_EN) floats HIGH or toggles. **If you ever move L_EN off GPIO2, the new pin needs a pull-down, and you must re-check this reasoning.**

## Electrical limits
- Logic is 3.3 V and the pins are **not 5 V tolerant**. Level-shift or use a divider for 5 V signals.
- Keep each pin at 12 mA or less. Use a driver or transistor for LEDs above ~10 mA, relays or buzzers.
- An ELRS receiver's TX is 3.3 V logic, so it connects directly to GPIO13.
- Battery voltage: use a resistor divider into ADC1 (e.g. GPIO34/35), and keep the pin below ~3.1 V at full charge.

## Procedure when assigning or changing a pin
1. Pick from **Safe general I/O** that isn't already in the allocation table. Explain the choice if you use anything else.
2. For anything that moves the car, decide how the pin behaves at boot, and add a pull-down or pull-up if needed.
3. Update **all three**: the `constexpr` pin constant in `src/main.cpp`, the CLAUDE.md hardware map, and the allocation table above.
4. Tell the user exactly what to wire, including resistors, power and a common GND.
