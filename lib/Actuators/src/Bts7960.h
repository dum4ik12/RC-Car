#pragma once

#include <cstdint>

#include <driver/gpio.h>

#include "LedcPwm.h"

/**
 * @brief BTS7960 / IBT-2 H-bridge driver for one brushed DC motor, both directions.
 *
 * RPWM and LPWM are two LEDC channels sharing one hardware timer (default 20 kHz,
 * 11-bit), so both latch on the same period edge. R_EN and L_EN are on their own
 * GPIOs but are always switched together.
 *
 *   speed > 0 : RPWM = |speed|, LPWM = 0
 *   speed < 0 : RPWM = 0,       LPWM = |speed|
 *   speed = 0 : both 0, EN high -> both low-side switches on, motor braked
 *   coast()   : both 0, EN low  -> bridge high-impedance, motor free-wheels
 *
 * Speed is in permille: [-kMaxSpeed, kMaxSpeed], sign = direction.
 * On a direction change the old side is written to 0 before the new side is
 * written. Both channels sit on the same timer and latch on the same period
 * edge, so the old side can never latch later than the new one: the two
 * half-bridges are never driven high together. Keep that write order.
 *
 * Not thread-safe: drive one instance from a single task.
 */
class Bts7960 {
public:
    struct Config {
        gpio_num_t     rpwmPin;
        gpio_num_t     lpwmPin;
        gpio_num_t     rEnablePin;              // R_EN
        gpio_num_t     lEnablePin;              // L_EN
        uint32_t       frequencyHz = 20000;     // inaudible; BTS7960 limit is 25 kHz
        bool           reversed    = false;     // swap directions in software
        ledc_timer_t   timer       = LEDC_TIMER_1;
        ledc_channel_t rChannel    = LEDC_CHANNEL_1;
        ledc_channel_t lChannel    = LEDC_CHANNEL_2;
    };

    static constexpr int16_t kMaxSpeed = 1000;

    explicit Bts7960(const Config& config) noexcept;
    ~Bts7960();

    Bts7960(const Bts7960&)            = delete;
    Bts7960& operator=(const Bts7960&) = delete;

    /** Bridge off -> both PWM at 0 -> bridge on. Leaves the motor braked. */
    [[nodiscard]] esp_err_t begin();

    /** Set speed immediately; applied at the next PWM period. */
    esp_err_t setSpeed(int16_t speed);

    /**
     * Ramp to @p speed over @p durationMs using the hardware fade engine.
     * Blocks the calling task (it sleeps, no CPU used) until the ramp is done.
     */
    esp_err_t rampTo(int16_t speed, uint32_t durationMs);

    /** Active brake: motor terminals shorted through the low-side switches. */
    esp_err_t brake() { return setSpeed(0); }

    /** Disable the bridge: outputs high-impedance, motor free-wheels. */
    esp_err_t coast();

    int16_t speed() const noexcept { return speed_; }
    bool    isEnabled() const noexcept { return enabled_; }

private:
    static constexpr ledc_timer_bit_t kResolution     = LEDC_TIMER_11_BIT;  // max for 20 kHz on the 80 MHz APB clock
    static constexpr uint32_t         kMinFrequencyHz = 100;
    static constexpr uint32_t         kMaxFrequencyHz = 25000;

    // Permille -> ticks in Q16 fixed point; kMaxSpeed maps exactly to 2^11 (100 % duty).
    static constexpr uint32_t kTicksPerPermilleQ16 =
        (((1UL << kResolution) << 16) + kMaxSpeed / 2) / kMaxSpeed;

    static int16_t clampSpeed(int16_t speed) noexcept {
        return speed > kMaxSpeed ? kMaxSpeed : (speed < -kMaxSpeed ? -kMaxSpeed : speed);
    }

    static uint32_t toTicks(int16_t speed) noexcept {
        const uint32_t magnitude = static_cast<uint32_t>(speed < 0 ? -speed : speed);
        return (magnitude * kTicksPerPermilleQ16 + 0x8000UL) >> 16;
    }

    static bool sameDirection(int16_t a, int16_t b) noexcept {
        return a != 0 && b != 0 && (a > 0) == (b > 0);
    }

    /** The half-bridge that drives @p speed's direction. */
    LedcPwm& sideFor(int16_t speed) noexcept {
        return ((speed > 0) != config_.reversed) ? rPwm_ : lPwm_;
    }

    esp_err_t setEnabled(bool enabled);
    esp_err_t writeEnablePins(uint32_t level);

    const Config config_;
    LedcPwm      rPwm_;
    LedcPwm      lPwm_;
    int16_t      speed_   = 0;
    bool         enabled_ = false;
    bool         started_ = false;
};
