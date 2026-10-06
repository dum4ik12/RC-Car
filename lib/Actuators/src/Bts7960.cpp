#include "Bts7960.h"

#include <esp32-hal-log.h>

Bts7960::Bts7960(const Config& config) noexcept
    : config_(config),
      rPwm_(LedcPwm::Config{config.rpwmPin, config.frequencyHz, kResolution, config.timer, config.rChannel}),
      lPwm_(LedcPwm::Config{config.lpwmPin, config.frequencyHz, kResolution, config.timer, config.lChannel}) {}

Bts7960::~Bts7960() {
    if (started_) {
        writeEnablePins(0);
    }
    // rPwm_ / lPwm_ destructors then stop both channels with the outputs LOW.
}

esp_err_t Bts7960::begin() {
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (config_.frequencyHz < kMinFrequencyHz || config_.frequencyHz > kMaxFrequencyHz) {
        log_e("BTS7960 frequency %u Hz out of range [%u, %u]", static_cast<unsigned>(config_.frequencyHz),
              static_cast<unsigned>(kMinFrequencyHz), static_cast<unsigned>(kMaxFrequencyHz));
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Bridge off first: the output latches are set LOW before the pins become outputs.
    esp_err_t err = writeEnablePins(0);
    if (err != ESP_OK) {
        return err;
    }
    gpio_config_t enable{};
    enable.pin_bit_mask = (1ULL << config_.rEnablePin) | (1ULL << config_.lEnablePin);
    enable.mode         = GPIO_MODE_OUTPUT;
    enable.pull_up_en   = GPIO_PULLUP_DISABLE;
    enable.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enable.intr_type    = GPIO_INTR_DISABLE;
    err = gpio_config(&enable);
    if (err != ESP_OK) {
        log_e("BTS7960 enable pins GPIO%d/GPIO%d config failed: %s", static_cast<int>(config_.rEnablePin),
              static_cast<int>(config_.lEnablePin), esp_err_to_name(err));
        return err;
    }

    // 2. Both half-bridge inputs at 0 %.
    err = rPwm_.begin(0);
    if (err != ESP_OK) {
        return err;
    }
    err = lPwm_.begin(0);
    if (err != ESP_OK) {
        return err;
    }
    started_ = true;

    // 3. Bridge on: both low sides conduct, so the motor is held braked.
    err = setEnabled(true);
    if (err == ESP_OK) {
        log_i("BTS7960 ready: RPWM GPIO%d, LPWM GPIO%d, R_EN GPIO%d, L_EN GPIO%d, %u Hz, %u-bit",
              static_cast<int>(config_.rpwmPin), static_cast<int>(config_.lpwmPin),
              static_cast<int>(config_.rEnablePin), static_cast<int>(config_.lEnablePin),
              static_cast<unsigned>(config_.frequencyHz), static_cast<unsigned>(kResolution));
    }
    return err;
}

esp_err_t Bts7960::setSpeed(int16_t speed) {
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    speed = clampSpeed(speed);
    if (speed == speed_ && enabled_) {
        return ESP_OK;
    }

    // Release the old side first. Both sides share one timer and latch on the same
    // period edge, so writing the old side to 0 before the new side guarantees the
    // old side is LOW no later than the new side rises.
    if (speed_ != 0 && !sameDirection(speed_, speed)) {
        const esp_err_t err = sideFor(speed_).setDuty(0);
        if (err != ESP_OK) {
            return err;
        }
        speed_ = 0;
    }

    esp_err_t err = setEnabled(true);
    if (err != ESP_OK) {
        return err;
    }
    if (speed != 0) {
        err = sideFor(speed).setDuty(toTicks(speed));
        if (err != ESP_OK) {
            return err;
        }
    }
    speed_ = speed;
    return ESP_OK;
}

esp_err_t Bts7960::rampTo(int16_t speed, uint32_t durationMs) {
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    speed = clampSpeed(speed);
    if (durationMs == 0 || (speed == speed_ && enabled_)) {
        return setSpeed(speed);
    }

    if (speed_ != 0 && speed != 0 && !sameDirection(speed_, speed)) {
        // Crossing zero: ramp the current side down, then the other side up,
        // splitting the time in proportion to the distance covered on each side.
        const uint32_t down   = static_cast<uint32_t>(speed_ < 0 ? -speed_ : speed_);
        const uint32_t up     = static_cast<uint32_t>(speed < 0 ? -speed : speed);
        const uint32_t downMs = static_cast<uint32_t>(static_cast<uint64_t>(durationMs) * down / (down + up));
        const esp_err_t err   = rampTo(0, downMs);
        if (err != ESP_OK) {
            return err;
        }
        return rampTo(speed, durationMs - downMs);
    }

    esp_err_t err = setEnabled(true);
    if (err != ESP_OK) {
        return err;
    }
    if (speed == 0 && speed_ == 0) {
        return ESP_OK;
    }

    // Same direction, or to/from zero: only one side moves, the other is already 0.
    err = sideFor(speed != 0 ? speed : speed_).fadeTo(toTicks(speed), durationMs, /*waitDone=*/true);
    if (err != ESP_OK) {
        return err;
    }
    speed_ = speed;
    return ESP_OK;
}

esp_err_t Bts7960::coast() {
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    // Bridge off first so the motor free-wheels immediately, then zero the PWM
    // so the next enable starts from a stop.
    esp_err_t err = setEnabled(false);
    if (err != ESP_OK) {
        return err;
    }
    if (speed_ != 0) {
        err = sideFor(speed_).setDuty(0);
        if (err != ESP_OK) {
            return err;
        }
        speed_ = 0;
    }
    return ESP_OK;
}

esp_err_t Bts7960::setEnabled(bool enabled) {
    if (enabled == enabled_) {
        return ESP_OK;
    }
    const esp_err_t err = writeEnablePins(enabled ? 1 : 0);
    if (err == ESP_OK) {
        enabled_ = enabled;
    }
    return err;
}

esp_err_t Bts7960::writeEnablePins(uint32_t level) {
    const esp_err_t err = gpio_set_level(config_.rEnablePin, level);
    return err != ESP_OK ? err : gpio_set_level(config_.lEnablePin, level);
}
