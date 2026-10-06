---
name: crsf-elrs
description: Add, debug or extend the ExpressLRS (ELRS) radio receiver on the RC car via the CRSF serial protocol, covering UART setup, frame parsing, CRC, RC channel mapping to steering and throttle, arming, failsafe, link statistics (RSSI/LQ) and telemetry such as battery voltage back to the radio. Use for anything mentioning ELRS, ExpressLRS, CRSF, receiver, transmitter/radio, channels, failsafe, arming, link quality or telemetry.
---

# ELRS receiver over CRSF

**Status: implemented** (receive path). GPIO13 (RX) and GPIO17 (TX) on UART2 are in use; nothing is sent yet.
Protocol constants, frame layouts and the test vectors are in [reference.md](reference.md).

## Decisions already taken with the user (2026-09, radio layout revised 2026-10-02)
1. Stick radio (not a surface radio): **CH1 = steering (right stick), CH3 = gas (left stick, rests at the bottom = zero), CH5 (AUX1) = arm switch, CH6 (AUX2) = 3-position direction switch forward / stop / reverse** (0-based indices 0/2/4/5 in `src/main.cpp`). The first layout (centred throttle on CH2) did not fit the user's radio.
2. Arming by a CH5 OFF→ON edge with the gas at zero; brake on failsafe; no telemetry yet.
3. Own parser (`lib/CrsfProtocol`), not an Arduino library.
Ask again only if the user changes radio, receiver or wiring.

## Where things live
| Piece | File | Notes |
|---|---|---|
| CRC, constants, parser, channels, link stats, builders, self-test | `lib/CrsfProtocol/src/crsf/*.h` | pure C++17, header-only, `constexpr`; `static_assert(crsf::selftest::run())` in `CrsfReceiver.cpp` |
| UART driver + `crsfRx` task + mailbox | `lib/Radio/src/CrsfReceiver.*` | `receive()` gives each frame once; `sendFrame()` is the telemetry hook (single writer) |
| Arming / failsafe / mapping | `lib/Control/src/DriveController.h` | pure, `constexpr`; scenarios in `DriveControllerSelfTest.h` |
| Control loop task | `src/ControlTask.*` | deadline wait, WDT, actuators first then logs |
| Pins, channel map, thresholds, speed limit | `src/main.cpp` | `kMaxSpeedPermille = 500` until failsafe is verified on the car |
| Host tests | `test/test_crsf`, `test/test_control` | `pio test -e native` (needs MSYS2 gcc) |

## Wiring
| Receiver pad | ESP32 |
|---|---|
| TX | GPIO13 / D13 (UART2 RX) |
| RX | GPIO17 (UART2 TX), only needed for telemetry |
| 5V | 5 V (ESP32 VIN or a BEC). Check the receiver's input range. |
| GND | GND |

Receiver settings (ELRS web UI or Lua): serial protocol **CRSF**, baud **420000** (the ELRS default), not inverted, failsafe mode **"No Pulses"** (the default), "Lock on first connection" left at default. Model match off, or matching the radio.

## Pipeline (see the `freertos-tasks` skill for the task details)
```
UART2 ─(rx-timeout IRQ, ~3 idle symbols)→ crsfRx: Parser → RcFrame{µs[16], receivedMs, linkStats, linkStatsMs}
        → xQueueOverwrite(mailbox) + xTaskNotifyGive(control)
control: receive() → RcInput{steer, throttle, arm, timestamps, LQ} → DriveController::update() → actuators
```
- `uart_driver_install` (1024 B ring buffer, 16-event queue) → `uart_param_config` → `uart_set_pin` → `uart_set_rx_timeout(3)` (must follow `uart_param_config`) → `uart_flush_input`.
- `UART_DATA`: drain with `uart_read_bytes(…, 0)` until it returns 0 (the ISR drops events silently when the queue is full). `UART_FIFO_OVF`/`UART_BUFFER_FULL`: flush, reset the queue and the parser, count. Frame/parity errors: count only; the CRC rejects the bytes.
- The rx task never logs. Counters (`rcFrames`, `parserErrors`, `badLength`, `overflows`, …) are atomics read by the control task's 1 Hz status line.
- A 0x16 frame is used only if `payloadSize() == 22`; a 0x14 only if `>= 10`. The reference parser doesn't check sizes, the receiver must.

## Control policy (safety-critical; don't weaken it without the user's consent)
- **Mapping:** `us = 1500 + (ticks − 992) * 5 / 8`.
  - Steering: optional reverse (`2·1500 − us`), then clamp to the `ServoOutput` endpoints. Follows the stick while DISARMED; neutral in FAILSAFE.
  - Gas (CH3): one-directional, `permille = us − 1000` (`kThrottleZeroUs`), 0 up to 1020 µs (`kThrottleDeadbandUs`), clamped to `kMaxSpeedPermille` (500 for bring-up, 1000 afterwards).
  - Direction (CH6, 3-position): > 1700 µs forward, < 1300 µs reverse, in between stop (brake at once). Forward/reverse is accepted **only with the gas at zero**; a switch move with the gas applied brakes the motor until the stick is back at zero (`DirectionBlocked`, logged once). `kThrottleReversed` swaps the two ends. Leaving ARMED clears the direction. AUX2 is sent round-robin by ELRS, so it updates a few packets later than CH1–CH5; that is fine for a direction switch.
- **States:** boot in **FAILSAFE** → **DISARMED** on the first trusted frame (`LinkUp`) → **ARMED** on a CH5 OFF→ON edge (< 1300 µs then > 1700 µs; 1300..1700 = no change) **only if** the gas is at zero. The edge is consumed even when refused (`ArmRefusedThrottle`), so a refused attempt needs another switch cycle. CH5 OFF → DISARMED (brake).
- **Failsafe:** no CRC-valid 0x16 for **250 ms** → brake, steering neutral, FAILSAFE; the check runs on every control step, so it also fires when no frame ever arrives. Every entry into FAILSAFE clears the switch history: a switch left ON never re-arms by itself.
  - **Link statistics rule:** a frame is trusted only if a 0x14 arrived within the last 400 ms with uplink LQ > 0 (`kLinkStatsStaleMs`; 0 disables). ELRS sends 0x14 at 10 Hz while connected and stops on link loss, even in the "Set/Last position" failsafe modes that keep sending 0x16. This catches a link loss the timeout alone would miss, and it also blocks a false arming edge from pre-link frames. Cost: DISARMED is reached ≤ 100 ms after the link is up.
  - The TBS spec suggests 1 s before failsafe, which suits aircraft. Keep ≤ 250 ms for the car.
- Use **CH5 (AUX1) for arming**, because ELRS sends it with every packet.

## Bring-up checklist (wheels off the ground; run `rc-safety-review` first)
1. Boot log: `CRSF on UART2: RX GPIO13 …`, `control: FAILSAFE, waiting for the radio link`.
2. Radio on → `LINK UP -> DISARMED (...)`; 1 Hz status lines show `fps` ≈ packet rate, `err=0 ovf=0`, `steer` ≈ 1500 centred, `gas` ≈ 988 with the stick down, `arm` ≈ 988/2011, `dir` ≈ 988/1500/2011, `lq` 90–100. Steering follows, motor stays braked.
3. Switch ON with the gas raised → `ARM REFUSED`, no motion even after lowering it. OFF, then ON with the gas at the bottom → `ARMED`. CH6 forward + gas → motor forward (`spd` > 0 in the status line); CH6 middle → brake; CH6 back + gas → reverse (`spd` < 0). Moving CH6 with the gas raised → `DIRECTION switch moved ...`, braked until the stick is down. Flip `kThrottleReversed`/`kSteeringReversed` if a direction is wrong. If `spd` shows a command but the motor stands still in one direction only, check that side's PWM wire (RPWM GPIO26 for `spd` > 0, LPWM GPIO27 for `spd` < 0).
4. Radio off while driving → `FAILSAFE: no frame for 25x ms` within 250 ms, motor braked, steering centred. Radio on with the switch still ON → `LINK UP -> DISARMED`, no motion until the switch is cycled.
5. Then raise `kMaxSpeedPermille` to 1000.

## Libraries
Existing Arduino libraries (CRSFforArduino, AlfredoCRSF) poll `HardwareSerial` from `loop()`. They don't follow this project's rules (event-driven, IDF drivers, no polling, testable pure parser), which is why `lib/CrsfProtocol` exists. Use them only as a reference.

## Adding telemetry later
Build the frame in `crsf/Builder.h` (battery 0x08 from reference.md, flight mode 0x21 with the state name), send it from a low-priority task with `CrsfReceiver::sendFrame()` at 1–5 Hz. That task is the only UART writer. Wire receiver RX to GPIO17 (already configured as UART2 TX).
