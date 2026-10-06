---
name: actuator-driver
description: Add or modify a hardware output driver in lib/Actuators (servo, ESC, BTS7960 motor, LEDs/lights, buzzer, any PWM) built on the LedcPwm class and the ESP32 LEDC peripheral. Use when writing or changing LedcPwm, ServoOutput, Bts7960 or a new actuator class, choosing PWM frequency/resolution, allocating LEDC timers/channels, or using the hardware fade engine.
---

# Actuator drivers on the ESP32 LEDC peripheral

## Layering
```
LedcPwm      raw ticks, one LEDC channel + its timer, thread-safe duty, HW fade   (LedcPwm.h)
  ├─ ServoOutput  µs API, clamp, Q16 µs→ticks, 50 Hz / 16-bit                      (ServoOutput.h)
  └─ Bts7960      two LedcPwm on one timer, permille speed, brake/coast, EN GPIOs (Bts7960.h)
```
A new actuator is a **domain class that owns one or more `LedcPwm` members**. It exposes domain units (µs, permille, percent), never raw ticks. Don't edit `LedcPwm` to add domain logic.

## LEDC facts (ESP32, IDF 4.4)
- The peripheral has two speed modes (HS, LS). Each has **8 channels and 4 timers**. Channels on the same timer share frequency and resolution.
- Every output in this project uses **HS mode**, where a new duty latches glitch-free at the next period edge. The allocation table is in CLAUDE.md. Add new rows there **before** writing code.
- **Frequency limit:** freq × 2^resolution ≤ 80 MHz (APB clock). Pick the highest resolution that fits:

| Frequency | Max resolution | Typical use |
|---|---|---|
| 50 Hz | 16 bit (≤20) | analog servo / ESC |
| 333 Hz | 16 bit | digital servo |
| 1 kHz | 16 bit | LEDs |
| 5 kHz | 13 bit | LEDs, buzzer tone |
| 10 kHz | 12 bit | |
| 20 kHz | 11 bit | motor (inaudible). **BTS7960 max is 25 kHz.** |
| 40 kHz | 10 bit | |

- The duty range is `[0, 2^res]`. `2^res` means constant HIGH (100 %).

## Driver API behavior (important for timing)
| Call (via LedcPwm) | IDF function | Behavior |
|---|---|---|
| `begin()` | `ledc_timer_config` + `ledc_channel_config` + fade install | Installs the fade ISR **once per process** through a function-local static. Never call `ledc_fade_func_install` anywhere else. |
| `setDuty()` | `ledc_set_duty_and_update` | Thread-safe (takes the channel's fade semaphore). **Returns at once**; the hardware latches the duty at the next period edge. Waits only while a fade is running on that channel. (Verified in the IDF 4.4.7 source; the old v4.4.0 version did block for a period.) |
| `setDutyNoWait()` | `ledc_set_duty` + `ledc_update_duty` | Same latch behaviour without the fade-service dependency. Single owner, never during a fade. This is what `ServoOutput::writeMicroseconds()` uses. |
| `fadeTo(..., true)` | `ledc_set_fade_time_and_start(WAIT_DONE)` | The task sleeps on a semaphore until the fade ends, using 0 % CPU. Fade steps happen per PWM period, so a 50 Hz fade updates at most every 20 ms. |
| second `begin()` on a shared timer | `ledc_timer_config` | Reconfigures the shared timer. Only safe while its outputs are at 0 (at startup). |

**Ordering on shared timers:** two channels on one timer latch on the same period edge. `Bts7960` relies on that (plus write order) to keep both half-bridges from being high together. Don't move RPWM/LPWM onto different timers.

## Class template (follow the existing drivers exactly)
```cpp
class Thing {
public:
    struct Config {                       // aggregate with defaults; pins first
        gpio_num_t     pin;
        uint32_t       frequencyHz = 5000;
        ledc_timer_t   timer       = LEDC_TIMER_2;   // from the CLAUDE.md allocation table
        ledc_channel_t channel     = LEDC_CHANNEL_3;
    };
    explicit Thing(const Config& config) noexcept;   // no hardware access here
    Thing(const Thing&)            = delete;
    Thing& operator=(const Thing&) = delete;

    [[nodiscard]] esp_err_t begin();                 // validate config, safe initial output
    esp_err_t set(DomainUnit value);                 // clamp → convert → LedcPwm, skip if unchanged
private:
    static constexpr ledc_timer_bit_t kResolution = LEDC_TIMER_13_BIT;
    const Config config_;
    LedcPwm      pwm_;
};
```
Rules:
- **Validate in `begin()`** and return `ESP_ERR_INVALID_ARG` with a `log_e` explaining why. Log one `log_i` line with the final configuration when it succeeds.
- **Clamp every input** before converting. Out-of-range input must never reach hardware.
- **Conversion to ticks uses Q16 fixed point.** Precompute the scale, as a `static constexpr` if the resolution is fixed, or once in the constructor. The hot path is then `(value * scaleQ16 + 0x8000) >> 16`. Prove in a comment that `maxValue * scaleQ16 + 0x8000` fits in 32 bits, or check it in `begin()` like `ServoOutput` does.
- **The safe state comes first:** configure outputs at 0 or neutral before enabling anything that can move. The destructor returns the hardware to its safe state.
- **Motor-type outputs:** never allow two outputs that would fight each other (both H-bridge sides) to be high together. Zero the old side before raising the new one; `setDuty` returning after the latch guarantees the order.
- No `float` or division in per-command paths. No heap. No Arduino `analogWrite`/`ledcWrite`.
- Give the class a short header comment: what it drives, its units, and its thread-safety (usually "drive from a single task").

## Checklist before finishing
1. LEDC timer/channel added to the CLAUDE.md allocation table, with no conflicts.
2. Pins chosen per the `esp32-pins` skill, and boot-time behavior considered.
3. The CLAUDE.md "Layout" list mentions the new class.
4. Build is warning-free (`pio-build` skill).
5. If it can move the car, run the `rc-safety-review` skill.
