#include "ServoOutput.h"

#include <esp32-hal-log.h>

ServoOutput::ServoOutput(const Config& config) noexcept
    : config_(config),
      ticksPerUsQ16_(computeTicksPerUsQ16(config.frequencyHz)),
      pwm_(LedcPwm::Config{config.pin, config.frequencyHz, kResolution, config.timer, config.channel}) {}

uint32_t ServoOutput::computeTicksPerUsQ16(uint16_t frequencyHz) noexcept {
    // ticks/µs = 2^resolution * f / 1e6, stored as Q16 and rounded to nearest.
    const uint64_t ticksPerSecondQ16 = (static_cast<uint64_t>(frequencyHz) << kResolution) << 16;
    return static_cast<uint32_t>((ticksPerSecondQ16 + kUsPerSecond / 2) / kUsPerSecond);
}

esp_err_t ServoOutput::begin() {
    const uint32_t periodUs = config_.frequencyHz ? kUsPerSecond / config_.frequencyHz : 0;
    const bool     rangeOk  = config_.minUs <= config_.neutralUs && config_.neutralUs <= config_.maxUs;
    const bool     fitsQ16  = static_cast<uint64_t>(config_.maxUs) * ticksPerUsQ16_ + 0x8000U <= UINT32_MAX;

    if (periodUs == 0 || !rangeOk || config_.maxUs >= periodUs || !fitsQ16) {
        log_e("Invalid servo config: %u Hz, %u..%u us, neutral %u us",
              static_cast<unsigned>(config_.frequencyHz), static_cast<unsigned>(config_.minUs),
              static_cast<unsigned>(config_.maxUs), static_cast<unsigned>(config_.neutralUs));
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t  neutralTicks = toTicks(config_.neutralUs);
    const esp_err_t err          = pwm_.begin(neutralTicks);
    if (err == ESP_OK) {
        lastTicks_ = neutralTicks;
        log_i("Servo on GPIO%d: %u Hz, %u-bit, %u..%u us, neutral %u us (%.4f ticks/us)",
              static_cast<int>(config_.pin), static_cast<unsigned>(config_.frequencyHz),
              static_cast<unsigned>(pwm_.resolutionBits()), static_cast<unsigned>(config_.minUs),
              static_cast<unsigned>(config_.maxUs), static_cast<unsigned>(config_.neutralUs),
              ticksPerUsQ16_ / 65536.0);
    }
    return err;
}

esp_err_t ServoOutput::writeMicroseconds(uint16_t us) {
    const uint32_t ticks = toTicks(us);
    if (ticks == lastTicks_) {
        return ESP_OK;
    }
    const esp_err_t err = pwm_.setDutyNoWait(ticks);
    if (err == ESP_OK) {
        lastTicks_ = ticks;
    }
    return err;
}

esp_err_t ServoOutput::sweepTo(uint16_t us, uint32_t durationMs, bool waitDone) {
    // The fade engine moves the duty on its own, so the cached value is no longer trustworthy.
    lastTicks_ = kUnknownTicks;
    return pwm_.fadeTo(toTicks(us), durationMs, waitDone);
}

uint16_t ServoOutput::readMicroseconds() const {
    if (!pwm_.isRunning()) {
        return 0;
    }
    const uint64_t ticks          = pwm_.duty();
    const uint64_t ticksPerSecond = static_cast<uint64_t>(config_.frequencyHz) << kResolution;
    return static_cast<uint16_t>((ticks * kUsPerSecond + ticksPerSecond / 2) / ticksPerSecond);
}
