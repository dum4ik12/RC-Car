# RC_CAR: ESP32 RC car firmware

ESP32 DevKit (`esp32dev`, WROOM-32) · PlatformIO · Arduino core 2.0.17 on ESP-IDF 4.4.7 · C++17 · FreeRTOS.
An ExpressLRS receiver (CRSF on UART2) drives a steering servo and a brushed DC motor through a BTS7960 (IBT-2) H-bridge. Use the `crsf-elrs` skill for radio work.

## Commands (PowerShell; `pio` is NOT on PATH)
```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run                        # build (esp32dev only)
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload              # flash: ASK THE USER FIRST
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor             # serial log, 115200, never exits
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" test -e native             # host unit tests (needs MSYS2 gcc on PATH)
```
- Builds must stay **warning-free**: `-Wall -Wextra` is on for `src/` (`build_src_flags`) and for every `lib/*` (`library.json`).
- The firmware build also runs **compile-time self-tests** (`static_assert`): CRC vector, channel packing, parser scenarios (`crsf/SelfTest.h`) and the arming/failsafe scenarios (`DriveControllerSelfTest.h`). A failed scenario is a build error.
- **Flashing moves the motor.** Never upload without the user's OK, and remind them to keep the wheels off the ground.
- Details and troubleshooting are in the `pio-build` skill.

## Layout
```
UART2 ─(rx-timeout IRQ)→ crsfRx task (CrsfReceiver) ─mailbox + notify→ control task (ControlTask)
                                                      → DriveController → ServoOutput / Bts7960
```
- `src/main.cpp`: pin map, channel map, `constexpr` tuning values (failsafe, arming, speed limit), object construction and `setup()`. `loop()` deletes its own task.
- `src/ControlTask.*`: the control loop as a FreeRTOS task. Waits for a frame notification or the failsafe deadline, runs `DriveController::update()`, writes the actuators, then logs (state changes, 1 Hz status). Subscribed to the task watchdog.
- `lib/Control/src/`: `DriveController`, pure logic (no ESP headers, header-only, `constexpr`): arming state machine, failsafe timeout, link-statistics check, direction switch, stick→µs/permille mapping. `DriveControllerSelfTest.h` holds the compile-time scenarios.
- `lib/CrsfProtocol/src/crsf/`: pure C++17 CRSF protocol: `Crc8`, `Frame` (constants), `Parser` (byte-fed, 64 B buffer), `Channels` (pack/unpack, ticks↔µs), `LinkStats`, `Builder` (frames for tests), `SelfTest`.
- `lib/Radio/src/`: `CrsfReceiver`: IDF UART driver with event queue, `crsfRx` task, 1-slot mailbox of `RcFrame` (µs + link stats + timestamps), consumer notification, counters, `sendFrame()` telemetry hook.
- `lib/Actuators/src/`: hardware drivers, no application logic.
  - `LedcPwm`: RAII wrapper over one IDF LEDC channel (timer + channel): thread-safe `setDuty()`, single-owner `setDutyNoWait()`, hardware fade engine. Neither write blocks for a PWM period (verified in IDF 4.4.7); they wait only for a running fade.
  - `ServoOutput`: servo/ESC control in µs. Input is clamped to [min, max], neutral is applied at `begin()`, µs→ticks uses Q16 fixed point, unchanged values are skipped. 50 Hz, 16-bit.
  - `Bts7960`: speed as signed permille [-1000, 1000]; `rampTo()` uses the hardware fade; `brake()` and `coast()`. Direction changes are safe. 20 kHz, 11-bit.
- `test/test_crsf`, `test/test_control`: Unity tests for the two pure libraries (`[env:native]`).
- `platformio.ini`: `default_envs = esp32dev`; `-std=gnu++17 -O2 -DCORE_DEBUG_LEVEL=3` (enables `log_i`/`log_e`), the `esp32_exception_decoder` monitor filter, and the `native` test env.

## Control policy (safety-critical; change only with the user's consent)
- Channels: **CH1 steering, CH3 gas, CH5 (AUX1) arm switch, CH6 (AUX2) direction switch** (stick radio, gas stick rests at the bottom). Mapping `us = 1500 + (ticks − 992) · 5 / 8`; gas `permille = us − 1000` (`kThrottleZeroUs`), zero up to 1020 µs, clamped to `kMaxSpeedPermille` (**500 during bring-up**, raise to 1000 once failsafe is verified).
- Direction: CH6 is a 3-position switch: > 1700 µs forward, < 1300 µs reverse, in between **stop** (motor braked at once). A drive direction is accepted **only with the gas at zero**; moving the switch with the gas applied brakes the motor until the stick is back at zero. `kThrottleReversed` swaps forward and reverse.
- States: boot in **FAILSAFE** (steering neutral, motor braked) → **DISARMED** when a trusted frame arrives (steering follows, motor braked) → **ARMED** on an OFF→ON edge of CH5 (> 1700 µs after < 1300 µs) **only with the gas at zero**. The edge is consumed even if refused. CH5 OFF disarms. Leaving ARMED clears the selected direction.
- Failsafe: no CRC-valid RC frame for **250 ms** → brake + neutral + disarm; the check runs on a timer, so it works when no frame ever arrives. A frame without link statistics newer than 400 ms, or with uplink LQ 0, is also failsafe (`kLinkStatsStaleMs`, 0 disables). After a failsafe the switch must be cycled again.

## Tasks (core 1; higher number = higher priority)
| Task | Prio | Stack | Wakes on |
|---|---|---|---|
| `crsfRx` | 10 | 4096 B | UART2 event queue |
| `control` | 9 | 4096 B | notification from `crsfRx`, or the failsafe deadline (≤ 50 ms) |

## Hardware map (keep in sync with `src/main.cpp`)
| Function | GPIO | Notes |
|---|---|---|
| Steering servo signal | 25 | 50 Hz, 1000–2000 µs, held at neutral 1500 µs |
| BTS7960 RPWM | 26 | forward side |
| BTS7960 LPWM | 27 | reverse side |
| BTS7960 R_EN | 15 | strapping pin, HIGH or toggling at boot. Safe only because L_EN holds the other half-bridge off |
| BTS7960 L_EN | 2 | strapping pin. Needs a **10 kΩ pull-down** (keeps the motor off at boot and is required for flashing). Drives the on-board LED |
| CRSF: ESP32 RX ← receiver TX | 13 | UART2 (routed through the GPIO matrix), 420000 baud 8N1, not inverted |
| CRSF: ESP32 TX → receiver RX | 17 | UART2, configured; only needed once telemetry is sent |

BTS7960 VCC goes to **3.3 V** (not 5 V), and all grounds are shared. The receiver takes 5 V (VIN/BEC) and GND; its TX is 3.3 V logic and connects directly to GPIO13. Free output pins: 4, 14, 16, 18, 19, 21, 22, 23, 32, 33. For analog input (battery or current sense) use ADC1 only (GPIO 32–39), because ADC2 stops working when WiFi is on. Check the `esp32-pins` skill before assigning any pin.

## LEDC allocation (all high-speed mode)
| Output | Timer | Channel | Freq / resolution |
|---|---|---|---|
| Steering servo | 0 | 0 | 50 Hz / 16-bit |
| Motor RPWM | 1 | 1 | 20 kHz / 11-bit |
| Motor LPWM | 1 | 2 | 20 kHz / 11-bit |

Don't call Arduino `analogWrite`, `ledcSetup`/`ledcWrite` or `tone` anywhere. They grab LEDC channels behind our back and clash with this table.

## Code conventions
- **Drivers are classes** with a nested `Config` struct that has defaults. The constructor is `explicit ... noexcept` and never touches hardware. Setup happens in `[[nodiscard]] esp_err_t begin()`. Drivers are non-copyable, and the destructor puts the hardware in a safe state.
- **Errors** are returned as `esp_err_t` and logged with `log_e(... esp_err_to_name(err))`. No exceptions, and no heap allocation after `setup()` (the one accepted exception: Arduino's `log_x` mallocs for lines ≥ 64 chars and blocks ~90 µs per character, so log after the actuator writes and at ≤ 2 Hz).
- **Pure logic goes in a hardware-free lib** (`CrsfProtocol`, `Control`): no ESP/Arduino headers, `constexpr` where possible, covered by `static_assert` self-tests and native Unity tests.
- **Performance:** use ESP-IDF drivers (`driver/ledc.h`, `driver/gpio.h`, `driver/uart.h`) rather than Arduino wrappers. Hot paths use integer or Q16 fixed-point math, with no float and no divide. Skip writes when the value hasn't changed. Let hardware do the work (fade engine, UART FIFO + event queue).
- **Tasks** are pinned to core 1 (core 0 is kept for WiFi/BT). They block on queues, notifications or semaphores and never busy-poll. Priorities, stacks and data sharing are covered in the `freertos-tasks` skill.
- **Naming:** constants are `kCamelCase`, members `name_`, and units are suffixed (`Us`, `Ms`, `Hz`). Speed is always permille.
- **Style:** match the file you're editing. `lib/` uses attached braces. `src/main.cpp` is IDE-formatted (Allman braces, indented namespace), so keep it that way.

## Safety rules (this firmware moves a real vehicle)
- Never drive RPWM and LPWM high at the same time. `Bts7960` writes the old side to 0 before the new side; both channels share timer 1 and latch on the same period edge. That write order must stay.
- The boot state must be safe: enable pins LOW before PWM is configured, motor braked, steering at neutral. `setup()` starts no task if any `begin()` fails.
- Every control input needs a **failsafe**. On lost signal the motor brakes and steering returns to neutral (see "Control policy"). Arm only when the gas is at zero, and only on a switch edge. Start or reverse the motor only from zero gas.
- Clamp every command (servo µs range, speed ±1000). Never use a frame whose CRC or payload length is wrong.
- Only the control task touches the actuators, and it is subscribed to the task watchdog.
- Before flashing any change to actuators, control or failsafe code, run the `rc-safety-review` skill.

## Skills (`.claude/skills/`)
`pio-build` · `esp32-pins` · `actuator-driver` · `freertos-tasks` · `crsf-elrs` · `rc-safety-review`
