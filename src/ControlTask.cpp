#include "ControlTask.h"

#include <esp32-hal-log.h>
#include <esp_task_wdt.h>

#include <DriveControllerSelfTest.h>
#include <crsf/Frame.h>

static_assert(drive_selftest::run(), "DriveController self-test failed");
static_assert(configTICK_RATE_HZ == 1000, "the control loop assumes a 1 kHz FreeRTOS tick");

namespace
{

    const char *stateName(DriveController::State state) noexcept
    {
        switch (state)
        {
        case DriveController::State::Failsafe:
            return "FAILSAFE";
        case DriveController::State::Disarmed:
            return "DISARMED";
        case DriveController::State::Armed:
            return "ARMED";
        }
        return "?";
    }

} // namespace

ControlTask::ControlTask(ServoOutput &steering, Bts7960 &motor, CrsfReceiver &receiver, DriveController &controller,
                         const Config &config) noexcept
    : steering_(steering), motor_(motor), receiver_(receiver), controller_(controller), config_(config)
{
}

esp_err_t ControlTask::start()
{
    if (handle_ != nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (config_.steeringChannel >= crsf::kChannelCount || config_.throttleChannel >= crsf::kChannelCount ||
        config_.armChannel >= crsf::kChannelCount || config_.directionChannel >= crsf::kChannelCount ||
        config_.idleWaitMs == 0 || config_.maxWaitMs == 0)
    {
        log_e("control: invalid config (channels %u/%u/%u/%u, waits %u/%u ms)",
              static_cast<unsigned>(config_.steeringChannel), static_cast<unsigned>(config_.throttleChannel),
              static_cast<unsigned>(config_.armChannel), static_cast<unsigned>(config_.directionChannel),
              static_cast<unsigned>(config_.idleWaitMs), static_cast<unsigned>(config_.maxWaitMs));
        return ESP_ERR_INVALID_ARG;
    }
    if (xTaskCreatePinnedToCore(entry, "control", config_.stackBytes, this, config_.priority, &handle_,
                                config_.core) != pdPASS)
    {
        log_e("control: task creation failed");
        handle_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void ControlTask::entry(void *arg)
{
    static_cast<ControlTask *>(arg)->run();
}

void ControlTask::run()
{
    // Subscribe to the task watchdog from inside the task. A hang now reboots the chip,
    // and the boot state has the motor off.
    const esp_err_t wdt = esp_task_wdt_add(nullptr);
    if (wdt != ESP_OK)
    {
        log_e("control: esp_task_wdt_add failed: %s (running without watchdog cover)", esp_err_to_name(wdt));
    }

    apply(controller_.command()); // explicit safe state: steering neutral, motor braked
    log_i("control: %s, waiting for the radio link", stateName(controller_.state()));

    uint32_t waitMs = config_.idleWaitMs;
    for (;;)
    {
        // Sleep until the receiver publishes a frame or the failsafe deadline is due.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(waitMs));
        esp_task_wdt_reset();

        const uint32_t nowMs = xTaskGetTickCount();
        CrsfReceiver::RcFrame frame{};
        const bool fresh = receiver_.receive(frame);
        if (fresh)
        {
            lastFrame_ = frame;
            lastInput_ = toInput(frame);
        }

        const DriveCommand command = controller_.update(fresh ? &lastInput_ : nullptr, nowMs);
        apply(command); // actuators first, logging afterwards

        if (controller_.lastEvent() != DriveController::Event::None)
        {
            logEvent(controller_.lastEvent(), nowMs);
        }
        if (config_.statusPeriodMs != 0 && nowMs - lastStatusMs_ >= config_.statusPeriodMs)
        {
            logStatus(nowMs);
        }

        waitMs = nextWaitMs(xTaskGetTickCount());
    }
}

RcInput ControlTask::toInput(const CrsfReceiver::RcFrame &frame) const noexcept
{
    RcInput input{};
    input.receivedMs   = frame.receivedMs;
    input.steeringUs   = frame.channelsUs[config_.steeringChannel];
    input.throttleUs   = frame.channelsUs[config_.throttleChannel];
    input.armUs        = frame.channelsUs[config_.armChannel];
    input.directionUs  = frame.channelsUs[config_.directionChannel];
    input.linkStatsMs  = frame.linkStatsMs;
    input.uplinkLq     = frame.linkStats.uplinkLq;
    input.hasLinkStats = frame.hasLinkStats;
    return input;
}

void ControlTask::apply(const DriveCommand &command)
{
    const esp_err_t steeringErr = steering_.writeMicroseconds(command.steeringUs);
    esp_err_t       motorErr    = motor_.setSpeed(command.speedPermille);
    if (motorErr != ESP_OK && command.speedPermille == 0)
    {
        // Braking through the PWM path failed: disable the bridge through the GPIOs instead.
        motorErr = motor_.coast();
    }
    if (steeringErr != ESP_OK || motorErr != ESP_OK)
    {
        // Log the first failure and then every 256th, so a persistent fault can't flood the UART.
        if ((applyErrors_++ & 0xFFU) == 0)
        {
            log_e("control: actuator write failed (steering: %s, motor: %s)", esp_err_to_name(steeringErr),
                  esp_err_to_name(motorErr));
        }
    }
}

uint32_t ControlTask::nextWaitMs(uint32_t nowMs) const noexcept
{
    if (controller_.state() == DriveController::State::Failsafe)
    {
        return config_.idleWaitMs;
    }
    const uint32_t remaining = controller_.msUntilFailsafe(nowMs);
    if (remaining == 0)
    {
        return 1; // deadline already passed: re-check on the next tick
    }
    return remaining < config_.maxWaitMs ? remaining : config_.maxWaitMs;
}

void ControlTask::logEvent(DriveController::Event event, uint32_t nowMs) const
{
    using Event = DriveController::Event;
    switch (event)
    {
    case Event::LinkUp:
        log_i("LINK UP -> DISARMED (steer=%u gas=%u arm=%u dir=%u us, lq=%u%%). Cycle the arm switch to arm",
              static_cast<unsigned>(lastInput_.steeringUs), static_cast<unsigned>(lastInput_.throttleUs),
              static_cast<unsigned>(lastInput_.armUs), static_cast<unsigned>(lastInput_.directionUs),
              static_cast<unsigned>(lastInput_.uplinkLq));
        break;
    case Event::Armed:
        log_i("ARMED (speed limit %d permille)", static_cast<int>(controller_.config().maxSpeedPermille));
        break;
    case Event::Disarmed:
        log_i("DISARMED: arm switch off, motor braked");
        break;
    case Event::ArmRefusedThrottle:
        log_w("ARM REFUSED: gas %u us is not at zero. Lower the stick and cycle the switch again",
              static_cast<unsigned>(lastInput_.throttleUs));
        break;
    case Event::DirectionBlocked:
        log_w("DIRECTION switch moved with the gas applied (%u us): motor braked until the stick is at zero",
              static_cast<unsigned>(lastInput_.throttleUs));
        break;
    case Event::FailsafeTimeout:
        log_w("FAILSAFE: no frame for %u ms -> motor braked, steering neutral",
              static_cast<unsigned>(controller_.frameAgeMs(nowMs)));
        break;
    case Event::FailsafeLinkStats:
        log_w("FAILSAFE: link statistics missing or stale (lq=%u%%) -> motor braked, steering neutral",
              static_cast<unsigned>(lastInput_.uplinkLq));
        break;
    case Event::None:
        break;
    }
}

void ControlTask::logStatus(uint32_t nowMs)
{
    const CrsfReceiver::Counters counters  = receiver_.counters();
    const uint32_t               elapsedMs = nowMs - lastStatusMs_;
    // Log path only, so a divide is fine here.
    const uint32_t fps = elapsedMs != 0 ? (counters.rcFrames - lastStatusFrames_) * 1000U / elapsedMs : 0;
    lastStatusMs_      = nowMs;
    lastStatusFrames_  = counters.rcFrames;

    // Frames arriving while we stay in FAILSAFE means they are not trusted. Boot starts in
    // FAILSAFE, so no transition event would ever explain that; say it here instead.
    const bool untrusted = controller_.state() == DriveController::State::Failsafe && fps != 0;
    const char *hint = untrusted ? " | frames arrive but are not trusted: no fresh link statistics (see kLinkStatsStaleMs)"
                                 : "";

    // spd is the command sent to the motor driver: > 0 drives RPWM (forward), < 0 drives LPWM (reverse).
    log_i("%s steer=%u gas=%u arm=%u dir=%u spd=%d | lq=%u%% rssi=-%u dBm | %u fps ls=%u err=%u badlen=%u ovf=%u | "
          "stack %u B free%s",
          stateName(controller_.state()), static_cast<unsigned>(lastInput_.steeringUs),
          static_cast<unsigned>(lastInput_.throttleUs), static_cast<unsigned>(lastInput_.armUs),
          static_cast<unsigned>(lastInput_.directionUs), static_cast<int>(controller_.command().speedPermille),
          static_cast<unsigned>(lastFrame_.linkStats.uplinkLq), static_cast<unsigned>(lastFrame_.linkStats.uplinkRssi1),
          static_cast<unsigned>(fps), static_cast<unsigned>(counters.linkStatsFrames),
          static_cast<unsigned>(counters.parserErrors), static_cast<unsigned>(counters.badLength),
          static_cast<unsigned>(counters.overflows), static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)), hint);
}
