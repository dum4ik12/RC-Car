#pragma once

#include <cstdint>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <Bts7960.h>
#include <CrsfReceiver.h>
#include <DriveController.h>
#include <ServoOutput.h>

/**
 * @brief The radio -> actuator control loop, as a FreeRTOS task.
 *
 * Sleeps until the receiver notifies a new frame or the failsafe deadline is due,
 * runs DriveController::update() and writes the result to the steering servo and
 * the motor driver. Nothing else touches the actuators after start().
 *
 * The task subscribes to the task watchdog: if it ever hangs, the chip resets and
 * boots with the motor off. Logging happens only after the actuators are written,
 * at most a few lines per second.
 */
class ControlTask
{
public:
    struct Config
    {
        uint32_t    stackBytes      = 4096;
        UBaseType_t priority        = 9;
        BaseType_t  core            = 1;
        uint32_t    idleWaitMs      = 50;   // wake-up period while there is no link (failsafe state)
        uint32_t    maxWaitMs       = 50;   // longest sleep between watchdog resets while linked
        uint32_t    statusPeriodMs  = 1000; // periodic status line; 0 disables it
        uint8_t     steeringChannel  = 0;   // CH1
        uint8_t     throttleChannel  = 2;   // CH3: gas
        uint8_t     armChannel       = 4;   // CH5 / AUX1
        uint8_t     directionChannel = 5;   // CH6 / AUX2: forward / stop / reverse
    };

    ControlTask(ServoOutput &steering, Bts7960 &motor, CrsfReceiver &receiver, DriveController &controller,
                const Config &config) noexcept;

    ControlTask(const ControlTask &) = delete;
    ControlTask &operator=(const ControlTask &) = delete;

    /** Create the task. Call only after every driver's begin() succeeded. */
    [[nodiscard]] esp_err_t start();

    TaskHandle_t handle() const noexcept { return handle_; }

private:
    [[noreturn]] static void entry(void *arg);
    [[noreturn]] void run();

    RcInput  toInput(const CrsfReceiver::RcFrame &frame) const noexcept;
    void     apply(const DriveCommand &command);
    uint32_t nextWaitMs(uint32_t nowMs) const noexcept;
    void     logEvent(DriveController::Event event, uint32_t nowMs) const;
    void     logStatus(uint32_t nowMs);

    ServoOutput     &steering_;
    Bts7960         &motor_;
    CrsfReceiver    &receiver_;
    DriveController &controller_;
    const Config     config_;

    TaskHandle_t          handle_ = nullptr;
    CrsfReceiver::RcFrame lastFrame_{};       // newest raw frame, kept for the logs
    RcInput               lastInput_{};       // newest mapped input, kept for the logs
    uint32_t              lastStatusMs_ = 0;
    uint32_t              lastStatusFrames_ = 0;
    uint32_t              applyErrors_ = 0;
};
