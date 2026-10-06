#pragma once

#include <cstdint>

#include <driver/ledc.h>
#include <esp_err.h>

/**
 * @brief Hardware PWM channel backed by the ESP32 LEDC peripheral.
 *
 * The waveform is produced entirely by hardware: once started it costs zero CPU.
 * Duty ramps use the LEDC hardware fade engine, so a fade also costs zero CPU;
 * a blocking fade sleeps the calling task on a driver semaphore until done.
 *
 * Duty values are raw timer ticks in the range [0, maxDuty()].
 * setDuty() and fadeTo() go through the driver's thread-safe API, so one channel
 * may be driven from several tasks. setDutyNoWait() skips that lock and is meant
 * for a single owner writing at a high rate (e.g. a control loop). A duty write
 * issued while a fade is running waits for that fade to finish.
 *
 * Neither duty write blocks until the value latches (verified in IDF 4.4.7): both
 * return right away and the hardware applies the new duty at the next period edge.
 */
class LedcPwm {
public:
    struct Config {
        gpio_num_t       pin;
        uint32_t         frequencyHz;
        ledc_timer_bit_t resolution;
        ledc_timer_t     timer   = LEDC_TIMER_0;
        ledc_channel_t   channel = LEDC_CHANNEL_0;
        ledc_mode_t      mode    = LEDC_HIGH_SPEED_MODE;  // ESP32 high-speed group: glitch-free HW latch
    };

    explicit LedcPwm(const Config& config) noexcept;
    ~LedcPwm();

    LedcPwm(const LedcPwm&)            = delete;
    LedcPwm& operator=(const LedcPwm&) = delete;

    /** Configure timer + channel and start output at @p initialDuty. */
    [[nodiscard]] esp_err_t begin(uint32_t initialDuty);

    /** Set duty; applied at the start of the next PWM period. Thread-safe (takes the fade lock). */
    esp_err_t setDuty(uint32_t ticks);

    /**
     * Set duty without the fade lock; applied at the start of the next PWM period.
     * Single owner only, and never while a fade is running on this channel.
     */
    esp_err_t setDutyNoWait(uint32_t ticks);

    /** Hardware fade from the current duty to @p ticks over @p durationMs. */
    esp_err_t fadeTo(uint32_t ticks, uint32_t durationMs, bool waitDone);

    /** Duty of the current PWM period (tracks an in-progress fade). */
    uint32_t duty() const;

    /** Stop output and drive the pin to @p idleLevel. */
    esp_err_t stop(uint32_t idleLevel = 0);

    uint32_t   maxDuty() const noexcept { return 1UL << config_.resolution; }
    uint32_t   frequencyHz() const noexcept { return config_.frequencyHz; }
    uint32_t   resolutionBits() const noexcept { return config_.resolution; }
    gpio_num_t pin() const noexcept { return config_.pin; }
    bool       isRunning() const noexcept { return running_; }

private:
    static esp_err_t installFadeService();

    const Config config_;
    bool         running_ = false;
};
