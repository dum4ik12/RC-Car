#pragma once

#include <cstdint>

#include "LedcPwm.h"

/**
 * @brief RC servo / ESC output: standard pulse-width PWM on a LEDC hardware channel.
 *
 * Commands are pulse widths in microseconds and are always clamped to
 * [minUs, maxUs], so an out-of-range value can never reach the servo/ESC.
 * The output starts at neutralUs, which is what a car ESC expects in order to arm.
 *
 * The µs -> ticks conversion is Q16 fixed point: one 32-bit multiply and a shift,
 * no float and no divide on the hot path. writeMicroseconds() returns at once
 * (the hardware latches at the next period) and skips the write when the value
 * has not changed, so it is safe to call once per received radio frame.
 *
 * Drive from a single task, and don't mix sweepTo() with per-frame writes.
 */
class ServoOutput {
public:
    struct Config {
        gpio_num_t     pin;
        uint16_t       minUs       = 1000;
        uint16_t       maxUs       = 2000;
        uint16_t       neutralUs   = 1500;
        uint16_t       frequencyHz = 50;  // analog servos/ESCs: 50 Hz; many digital servos accept up to 333 Hz
        ledc_timer_t   timer       = LEDC_TIMER_0;
        ledc_channel_t channel     = LEDC_CHANNEL_0;
    };

    explicit ServoOutput(const Config& config) noexcept;

    /** Validate the config and start the output at neutralUs. */
    [[nodiscard]] esp_err_t begin();

    /** Set the pulse width; applied at the start of the next PWM period. Unchanged values are skipped. */
    esp_err_t writeMicroseconds(uint16_t us);

    /** Return to the neutral pulse width (servo centre / ESC stop). */
    esp_err_t writeNeutral() { return writeMicroseconds(config_.neutralUs); }

    /** Ramp from the current pulse width to @p us in hardware over @p durationMs. */
    esp_err_t sweepTo(uint16_t us, uint32_t durationMs, bool waitDone = true);

    /** Pulse width of the current PWM period (tracks an in-progress sweep). */
    uint16_t readMicroseconds() const;

    const Config& config() const noexcept { return config_; }
    bool          isRunning() const noexcept { return pwm_.isRunning(); }

private:
    static constexpr ledc_timer_bit_t kResolution   = LEDC_TIMER_16_BIT;
    static constexpr uint32_t         kUsPerSecond  = 1000000UL;
    static constexpr uint32_t         kUnknownTicks = UINT32_MAX;  // forces the next write through

    static uint32_t computeTicksPerUsQ16(uint16_t frequencyHz) noexcept;

    uint16_t clamp(uint16_t us) const noexcept {
        return us < config_.minUs ? config_.minUs : (us > config_.maxUs ? config_.maxUs : us);
    }

    uint32_t toTicks(uint16_t us) const noexcept {
        // begin() guarantees maxUs * ticksPerUsQ16_ + 0x8000 fits in 32 bits.
        return (static_cast<uint32_t>(clamp(us)) * ticksPerUsQ16_ + 0x8000UL) >> 16;
    }

    const Config   config_;
    const uint32_t ticksPerUsQ16_;
    LedcPwm        pwm_;
    uint32_t       lastTicks_ = kUnknownTicks;  // last duty written, to skip unchanged values
};
