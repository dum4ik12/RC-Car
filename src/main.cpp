#include <Arduino.h>
#include <esp_system.h>

#include <Bts7960.h>
#include <CrsfReceiver.h>
#include <DriveController.h>
#include <ServoOutput.h>

#include "ControlTask.h"

// RC car: ExpressLRS receiver (CRSF on UART2) -> steering servo + BTS7960 motor driver.
//
//   crsfRx task  : UART2 bytes -> CRSF frames -> mailbox + notification
//   control task : frame or timeout -> DriveController (arming, failsafe, mapping) -> actuators
//
// The car boots in FAILSAFE (steering neutral, motor braked), goes DISARMED when the
// link is up, and ARMS only on an OFF->ON edge of the arm switch with the gas at
// zero. Once armed, the gas stick sets the speed and the 3-position direction switch
// selects forward / stop / reverse; a direction is accepted only with the gas at zero.
// Losing the link for kFailsafeTimeoutMs brakes the motor and centres the steering;
// arming again needs a new switch cycle.

namespace
{

    // ---- Pin map (ESP32 DevKit) -------------------------------------------------
    constexpr gpio_num_t kSteeringPin  = GPIO_NUM_25; // servo signal
    constexpr gpio_num_t kMotorRpwmPin = GPIO_NUM_26; // BTS7960 RPWM
    constexpr gpio_num_t kMotorLpwmPin = GPIO_NUM_27; // BTS7960 LPWM
    constexpr gpio_num_t kMotorREnPin  = GPIO_NUM_15; // BTS7960 R_EN
    constexpr gpio_num_t kMotorLEnPin  = GPIO_NUM_2;  // BTS7960 L_EN (10k pull-down to GND)
    constexpr gpio_num_t kCrsfRxPin    = GPIO_NUM_13; // UART2 RX <- receiver TX (D13)
    constexpr gpio_num_t kCrsfTxPin    = GPIO_NUM_17; // UART2 TX -> receiver RX (telemetry; nothing sent yet)
    constexpr uart_port_t kCrsfUart    = UART_NUM_2;

    // ---- Radio channel map (0-based: CH1 = 0) -----------------------------------
    constexpr uint8_t kSteeringChannel  = 0; // CH1: right stick, left/right
    constexpr uint8_t kThrottleChannel  = 2; // CH3: gas, left stick up/down (bottom = zero)
    constexpr uint8_t kArmChannel       = 4; // CH5 / AUX1: arm switch
    constexpr uint8_t kDirectionChannel = 5; // CH6 / AUX2: 3-position switch, forward / stop / reverse

    // ---- Control policy ---------------------------------------------------------
    constexpr uint32_t kFailsafeTimeoutMs  = 250;  // no valid frame for this long -> brake + neutral
    constexpr uint32_t kLinkStatsStaleMs   = 400;  // frames need link statistics newer than this (0 = off)
    constexpr uint16_t kThrottleZeroUs     = 1000; // gas stick at the bottom = zero speed; 2000 = full
    constexpr uint16_t kThrottleDeadbandUs = 20;   // gas counts as zero up to this far above kThrottleZeroUs
    constexpr uint16_t kArmOnUs            = 1700; // arm switch ON above this ...
    constexpr uint16_t kArmOffUs           = 1300; // ... OFF below this
    constexpr uint16_t kDirectionForwardUs = 1700; // direction switch: forward above this ...
    constexpr uint16_t kDirectionReverseUs = 1300; // ... reverse below this, stop (brake) in between
    constexpr int16_t  kMaxSpeedPermille   = 500;  // BRING-UP LIMIT (50 %). Raise to 1000 once failsafe is verified.
    constexpr bool     kSteeringReversed   = false;
    constexpr bool     kThrottleReversed   = false; // true swaps forward and reverse

    // ---- Tasks (core 1 = APP core; core 0 stays free for WiFi/BT) --------------
    constexpr BaseType_t  kAppCore           = 1;
    constexpr uint32_t    kCrsfRxStackBytes  = 4096;
    constexpr UBaseType_t kCrsfRxPriority    = 10;
    constexpr uint32_t    kControlStackBytes = 4096;
    constexpr UBaseType_t kControlPriority   = 9;
    constexpr uint32_t    kStatusLogPeriodMs = 1000; // 0 disables the periodic status line

    static_assert(kArmOffUs < kArmOnUs, "the arm switch thresholds need a dead zone between them");
    static_assert(kDirectionReverseUs < kDirectionForwardUs, "the direction switch needs a stop zone in the middle");
    static_assert(kFailsafeTimeoutMs <= 250, "a car needs a short failsafe timeout");
    static_assert(kMaxSpeedPermille > 0 && kMaxSpeedPermille <= Bts7960::kMaxSpeed, "speed limit out of range");
    static_assert(kCrsfRxPriority > kControlPriority, "the receiver must pre-empt the control loop");
    static_assert(kSteeringChannel != kThrottleChannel && kThrottleChannel != kArmChannel &&
                      kSteeringChannel != kArmChannel && kDirectionChannel != kSteeringChannel &&
                      kDirectionChannel != kThrottleChannel && kDirectionChannel != kArmChannel,
                  "channel map must not overlap");

    // ---- Driver configuration ---------------------------------------------------
    constexpr ServoOutput::Config kSteeringConfig{kSteeringPin}; // 50 Hz, 1000..2000 µs, neutral 1500 µs

    constexpr CrsfReceiver::Config crsfConfig() noexcept
    {
        CrsfReceiver::Config config{kCrsfRxPin};
        config.txPin          = kCrsfTxPin;
        config.uart           = kCrsfUart;
        config.taskStackBytes = kCrsfRxStackBytes;
        config.taskPriority   = kCrsfRxPriority;
        config.taskCore       = kAppCore;
        return config;
    }

    constexpr DriveController::Config driveConfig() noexcept
    {
        DriveController::Config config{};
        config.neutralUs          = kSteeringConfig.neutralUs;
        config.throttleZeroUs     = kThrottleZeroUs;
        config.throttleDeadbandUs = kThrottleDeadbandUs;
        config.armOnUs            = kArmOnUs;
        config.armOffUs           = kArmOffUs;
        config.directionForwardUs = kDirectionForwardUs;
        config.directionReverseUs = kDirectionReverseUs;
        config.failsafeTimeoutMs  = kFailsafeTimeoutMs;
        config.linkStatsStaleMs   = kLinkStatsStaleMs;
        config.maxSpeedPermille   = kMaxSpeedPermille;
        config.steeringMinUs      = kSteeringConfig.minUs;
        config.steeringMaxUs      = kSteeringConfig.maxUs;
        config.steeringReversed   = kSteeringReversed;
        config.throttleReversed   = kThrottleReversed;
        return config;
    }

    constexpr ControlTask::Config controlConfig() noexcept
    {
        ControlTask::Config config{};
        config.stackBytes       = kControlStackBytes;
        config.priority         = kControlPriority;
        config.core             = kAppCore;
        config.statusPeriodMs   = kStatusLogPeriodMs;
        config.steeringChannel  = kSteeringChannel;
        config.throttleChannel  = kThrottleChannel;
        config.armChannel       = kArmChannel;
        config.directionChannel = kDirectionChannel;
        return config;
    }

    // ---- Objects (constructed before setup(); no hardware is touched until begin()) ----
    ServoOutput     steering{kSteeringConfig};
    Bts7960         motor{Bts7960::Config{kMotorRpwmPin, kMotorLpwmPin, kMotorREnPin, kMotorLEnPin}};
    CrsfReceiver    receiver{crsfConfig()};
    DriveController controller{driveConfig()};
    ControlTask     control{steering, motor, receiver, controller, controlConfig()};

    const char *resetReasonName(esp_reset_reason_t reason) noexcept
    {
        switch (reason)
        {
        case ESP_RST_POWERON:
            return "power-on";
        case ESP_RST_EXT:
            return "external";
        case ESP_RST_SW:
            return "software";
        case ESP_RST_PANIC:
            return "panic";
        case ESP_RST_INT_WDT:
            return "interrupt watchdog";
        case ESP_RST_TASK_WDT:
            return "task watchdog";
        case ESP_RST_WDT:
            return "other watchdog";
        case ESP_RST_DEEPSLEEP:
            return "deep sleep";
        case ESP_RST_BROWNOUT:
            return "brownout";
        case ESP_RST_SDIO:
            return "sdio";
        default:
            return "unknown";
        }
    }

    bool beginOrLog(esp_err_t err, const char *what)
    {
        if (err != ESP_OK)
        {
            log_e("%s failed: %s", what, esp_err_to_name(err));
        }
        return err == ESP_OK;
    }

} // namespace

void setup()
{
    Serial.begin(115200);
    log_i("RC_CAR boot (reset reason: %s)", resetReasonName(esp_reset_reason()));

    // Actuators first, into their safe state: steering at neutral, motor braked.
    // If anything below fails, no task is started, so nothing can move.
    if (!beginOrLog(steering.begin(), "steering.begin"))
    {
        return;
    }
    if (!beginOrLog(motor.begin(), "motor.begin"))
    {
        return;
    }
    if (!beginOrLog(receiver.begin(), "receiver.begin"))
    {
        return;
    }
    if (!beginOrLog(control.start(), "control.start"))
    {
        return;
    }
    receiver.setConsumer(control.handle());

    log_i("RC_CAR ready: CH%u steering, CH%u gas, CH%u arm, CH%u direction, failsafe %u ms, speed limit %d permille",
          static_cast<unsigned>(kSteeringChannel + 1), static_cast<unsigned>(kThrottleChannel + 1),
          static_cast<unsigned>(kArmChannel + 1), static_cast<unsigned>(kDirectionChannel + 1),
          static_cast<unsigned>(kFailsafeTimeoutMs), static_cast<int>(kMaxSpeedPermille));
}

void loop()
{
    // All work runs in FreeRTOS tasks; drop the Arduino loop task to free its stack.
    vTaskDelete(nullptr);
}
