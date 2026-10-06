---
name: freertos-tasks
description: Create or change FreeRTOS tasks, control loops, timing and inter-task communication in the RC_CAR ESP32 firmware (core pinning, priorities, stack sizes, watchdog, queues, task notifications, ISRs). Use when adding a task (e.g. CRSF receiver, control loop, telemetry, battery monitor), sharing data between tasks, fixing stack overflow, watchdog or timing problems, or reviewing real-time behavior.
---

# FreeRTOS on the ESP32 (Arduino core 2.0.17 / IDF 4.4)

## Platform facts (from this project's sdkconfig)
- **Tick = 1 kHz** (`CONFIG_FREERTOS_HZ=1000`), so `pdMS_TO_TICKS(1)` = 1 tick.
- **Stack sizes are in BYTES** on ESP-IDF, unlike vanilla FreeRTOS, both in `xTaskCreatePinnedToCore` and in `uxTaskGetStackHighWaterMark()`.
- Stack overflow detection uses a canary (`CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY`).
- The task watchdog is 5 s and watches **only core 0's idle task** by default. A busy loop on core 1 is therefore not caught automatically, but it still starves every lower-priority task.
- Core 0 runs WiFi/BT/lwIP/esp_timer. **Our tasks go on core 1** (`CONFIG_ARDUINO_RUNNING_CORE=1`).
- Arduino's `loopTask` (8 KB) is deleted by `loop()` calling `vTaskDelete(nullptr)`, which frees its stack.

## Task plan (keep this table, `src/main.cpp` and CLAUDE.md in sync)
| Task | Core | Prio | Stack (B) | Wakes on | Status |
|---|---|---|---|---|---|
| `crsfRx` | 1 | 10 | 4096 | UART2 driver event queue (`CrsfReceiver::run`) | current |
| `control` | 1 | 9 | 4096 | notification from `crsfRx`, or the failsafe deadline, ≤ 50 ms (`ControlTask::run`) | current |
| `telemetry` | 1 | 3 | 3072 | `vTaskDelayUntil` 200 ms; the only `CrsfReceiver::sendFrame()` caller | planned |

The `control` task's 1 Hz status line prints `uxTaskGetStackHighWaterMark()`; keep it ≥ 512 B.

Higher numbers mean higher priority. The input path (receiver, then control) must be higher than anything cosmetic (logging, telemetry, LEDs).

## Rules
- **Never busy-wait.** Block on a queue, notification, semaphore, `vTaskDelay` or `vTaskDelayUntil`. `delayMicroseconds()` spins, so keep it to a few µs at most.
- **Fixed-rate loops use `vTaskDelayUntil`**, not `vTaskDelay`, so the period doesn't drift:
  ```cpp
  TickType_t last = xTaskGetTickCount();
  for (;;) { vTaskDelayUntil(&last, pdMS_TO_TICKS(kPeriodMs)); step(); }
  ```
- **No blocking actuator calls in the control loop.** `Bts7960::rampTo()` and `ServoOutput::sweepTo(..., true)` block for the whole fade. `setSpeed()` and `writeMicroseconds()` return at once (verified in IDF 4.4.7: `ledc_set_duty_and_update` waits only for a running fade, never for the period), so they are the per-frame calls.
- **Logging is slow** (UART at 115200 baud): at most ~10 Hz, or only when something changes. Never log in an ISR or a critical section.
- **Task functions** are `[[noreturn]] void nameTask(void* arg)`. The object is passed through `arg`, and the function never returns (call `vTaskDelete(nullptr)` if it must end).
- **Create tasks in `setup()` only after the drivers' `begin()` succeeded**, and check for `pdPASS`. Keep the `TaskHandle_t` if another task has to notify it.
- **Stack sizing:** start at 3072 B, and add ~2 KB if the task uses `log_x`/`printf`. During development, log `uxTaskGetStackHighWaterMark(nullptr)` once and keep at least 512 B free.
- **Safety-critical loops subscribe to the watchdog.** Call `esp_task_wdt_add(nullptr)` once and `esp_task_wdt_reset()` every iteration. If the control loop hangs, the chip resets, and boot leaves the motor off with the enables LOW.
- **ISRs:** mark them `IRAM_ATTR`, use only `...FromISR` APIs, and end with `portYIELD_FROM_ISR(woken)`. Prefer driver event queues (UART) over custom ISRs.

## Sharing data between tasks (pick the lightest one that fits)
| Need | Mechanism |
|---|---|
| Wake a task ("new frame arrived") | `xTaskNotifyGive(handle)` → `ulTaskNotifyTake(pdTRUE, timeout)`; from an ISR use `vTaskNotifyGiveFromISR` |
| Latest value of a struct (e.g. RC channels) | 1-slot mailbox: `xQueueCreate(1, sizeof(T))` in `setup()`, producer `xQueueOverwrite`, consumer `xQueuePeek`/`xQueueReceive` |
| Stream of events that must not be lost | a normal queue with enough depth; check the send result |
| One 32-bit value (flag, timestamp) | `std::atomic<uint32_t>` (lock-free on Xtensa) |
| Small struct, both sides very fast | `portMUX_TYPE` + `portENTER_CRITICAL`/`portEXIT_CRITICAL` around a plain copy only |

Create queues and mutexes in `setup()`, which is the only place heap allocation is allowed.

## Radio → actuator pipeline (event-driven, minimal latency; implemented)
```
UART2 FIFO ─(rx-timeout IRQ)→ crsfRx task: parse, CRC, xQueueOverwrite(mailbox), xTaskNotifyGive(control)
control task: ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(waitMs))     waitMs = time left to the failsafe deadline, ≤ 50 ms
   ├─ receive() new frame → DriveController::update(&input, now) → writeMicroseconds() / setSpeed()
   └─ no frame            → DriveController::update(nullptr, now) → FAILSAFE at exactly 250 ms (brake, neutral)
```
The control task runs once per received frame, with no polling, and the deadline wait makes the failsafe fire at 250 ms rather than 250 ms + a tick. `esp_task_wdt_reset()` runs every iteration. The mailbox is consumed with `xQueueReceive`, so `receive()` returns true only for a new frame. Protocol details are in the `crsf-elrs` skill.
