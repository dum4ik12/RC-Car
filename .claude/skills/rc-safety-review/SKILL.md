---
name: rc-safety-review
description: Review RC_CAR firmware for safety hazards that could make the car move unexpectedly, run away, or damage hardware (motor driver, steering, boot state, failsafe, arming, pins, task timing). Use before flashing any change to actuator, control, radio/failsafe, pin or task code, or when the user asks for a safety check or review.
argument-hint: "[file or area to focus on]"
---

# RC car safety review

Focus: `$ARGUMENTS` (default: every file changed in this session, plus `src/main.cpp` and `lib/Actuators/`).

Read the actual code. Don't review from memory. For each item below, decide **OK / ISSUE / N/A**, and cite `file:line` as evidence.

## 1. Boot and init state
- [ ] Every enable pin is driven LOW **before** the PWM channels are configured, and PWM starts at 0 duty or neutral µs.
- [ ] Pins that float or toggle during reset (strapping 0/2/5/12/15, boot-glitch pins 1/3/14) can't enable motion. The `esp32-pins` skill covers the L_EN pull-down reasoning.
- [ ] If `begin()` fails, the firmware doesn't start any task that drives that actuator, and it logs the error.
- [ ] The steering servo starts at neutral. The motor starts braked or coasting, never moving.

## 2. H-bridge integrity (Bts7960)
- [ ] There is no code path where RPWM and LPWM are both > 0. On a direction change the old side is zeroed and latched **before** the new side rises (`setDuty` / `fadeTo` complete in order).
- [ ] `speed_` always matches the hardware after an error path, or the error leaves both sides at 0.
- [ ] Frequency stays ≤ 25 kHz (BTS7960 limit), and duty never exceeds `2^res`.

## 3. Command limits
- [ ] Every external input is clamped before conversion: servo µs to [min, max], speed to ±1000.
- [ ] Fixed-point conversions can't overflow 32 bits for the largest allowed input, and there's a comment proving it.
- [ ] No sign or narrowing bugs (int16 negation of −32768, unsigned wrap in `abs` or subtraction).

## 4. Failsafe and arming (once a radio input exists)
- [ ] Losing the input (no valid frame for `kFailsafeTimeoutMs` ≤ 250 ms) → motor brake/coast + steering neutral + disarm. **Check that this path runs even when no frames arrive at all** (timeout-based, not frame-triggered only).
- [ ] The car starts disarmed. Arming needs the arm switch **and** throttle inside the deadband. After a failsafe, arming again is required.
- [ ] CRC-invalid or malformed frames are never used. A partial or garbled frame can't produce full throttle.
- [ ] The clock used for timeouts can't wrap badly: use unsigned differences `now - last`.

## 5. Tasks and timing
- [ ] No busy-waiting, and no blocking actuator calls (`rampTo`, the blocking servo `setDuty`) inside a fixed-rate control loop.
- [ ] The control loop subscribes to the task watchdog, so a hang causes a reset and the motor ends up off.
- [ ] Priorities: input and control above logging and telemetry. No logging in ISRs, in critical sections, or above ~10 Hz.
- [ ] Shared data between tasks goes through a mailbox, notification, atomic or critical section. There are no torn reads of multi-field structs.
- [ ] Stack sizes have headroom, especially in tasks that call `log_x`/`printf`.

## 6. Demo and test code
- [ ] Any demo that moves the motor by itself (e.g. `motorDemoTask`) is either intended or removed. It must not coexist with radio control.
- [ ] The user has been warned to keep the wheels off the ground before flashing.

## 7. Electrical notes (tell the user if relevant)
- The motor's current spikes can brown out the ESP32. Use a separate regulator or BEC, a bulk capacitor at the BTS7960, and a common ground.
- BTS7960 VCC must be 3.3 V. Receiver and servo power come from 5 V, not from a 3.3 V pin.

## Report format
Start with a one-line verdict: **SAFE TO FLASH**, **FLASH WITH CAUTION**, or **DO NOT FLASH**. Then list only the ISSUE items, most severe first. Each one gets:
- `file:line`: what is wrong
- the failure scenario (concrete input or state, and what the car does)
- the fix (a small code change, or a question for the user)

Don't pad the report with passing items. If there are no issues, give the verdict and a list of what you checked.
