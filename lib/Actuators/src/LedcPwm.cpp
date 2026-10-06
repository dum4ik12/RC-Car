#include "LedcPwm.h"

#include <esp32-hal-log.h>

namespace {

esp_err_t logIfError(esp_err_t err, const char* what) {
    if (err != ESP_OK) {
        log_e("%s failed: %s", what, esp_err_to_name(err));
    }
    return err;
}

}  // namespace

LedcPwm::LedcPwm(const Config& config) noexcept : config_(config) {}

LedcPwm::~LedcPwm() {
    if (running_) {
        stop();
    }
}

esp_err_t LedcPwm::begin(uint32_t initialDuty) {
    if (running_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (initialDuty > maxDuty()) {
        return logIfError(ESP_ERR_INVALID_ARG, "LedcPwm::begin (duty out of range)");
    }

    ledc_timer_config_t timer{};
    timer.speed_mode      = config_.mode;
    timer.duty_resolution = config_.resolution;
    timer.timer_num       = config_.timer;
    timer.freq_hz         = config_.frequencyHz;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t err = logIfError(ledc_timer_config(&timer), "ledc_timer_config");
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel{};
    channel.gpio_num   = config_.pin;
    channel.speed_mode = config_.mode;
    channel.channel    = config_.channel;
    channel.intr_type  = LEDC_INTR_DISABLE;
    channel.timer_sel  = config_.timer;
    channel.duty       = initialDuty;
    channel.hpoint     = 0;
    err = logIfError(ledc_channel_config(&channel), "ledc_channel_config");
    if (err != ESP_OK) {
        return err;
    }

    err = logIfError(installFadeService(), "ledc_fade_func_install");
    if (err != ESP_OK) {
        return err;
    }

    running_ = true;
    return ESP_OK;
}

esp_err_t LedcPwm::setDuty(uint32_t ticks) {
    if (!running_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ticks > maxDuty()) {
        return ESP_ERR_INVALID_ARG;
    }
    return ledc_set_duty_and_update(config_.mode, config_.channel, ticks, 0);
}

esp_err_t LedcPwm::setDutyNoWait(uint32_t ticks) {
    if (!running_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ticks > maxDuty()) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t err = ledc_set_duty(config_.mode, config_.channel, ticks);
    return err != ESP_OK ? err : ledc_update_duty(config_.mode, config_.channel);
}

esp_err_t LedcPwm::fadeTo(uint32_t ticks, uint32_t durationMs, bool waitDone) {
    if (durationMs == 0) {
        return setDuty(ticks);
    }
    if (!running_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ticks > maxDuty()) {
        return ESP_ERR_INVALID_ARG;
    }
    return ledc_set_fade_time_and_start(config_.mode, config_.channel, ticks, durationMs,
                                        waitDone ? LEDC_FADE_WAIT_DONE : LEDC_FADE_NO_WAIT);
}

uint32_t LedcPwm::duty() const {
    return ledc_get_duty(config_.mode, config_.channel);
}

esp_err_t LedcPwm::stop(uint32_t idleLevel) {
    const esp_err_t err = ledc_stop(config_.mode, config_.channel, idleLevel);
    if (err == ESP_OK) {
        running_ = false;
    }
    return err;
}

esp_err_t LedcPwm::installFadeService() {
    // The fade ISR is shared by every LEDC channel and must be installed once per
    // process; a function-local static gives us a thread-safe one-shot.
    // ESP_ERR_INVALID_STATE means another component already installed it.
    static const esp_err_t result = [] {
        const esp_err_t err = ledc_fade_func_install(0);
        return err == ESP_ERR_INVALID_STATE ? ESP_OK : err;
    }();
    return result;
}
