#pragma once

#include <cstdint>

#include "DriveController.h"

/**
 * Compile-time scenarios for DriveController. `static_assert(drive_selftest::run())`
 * in the firmware build proves the arming and failsafe rules without a host
 * toolchain. The native Unity tests repeat them with more variations.
 */
namespace drive_selftest {

using State = DriveController::State;
using Event = DriveController::Event;

constexpr uint16_t kZeroGasUs = 1000;  // gas stick at the bottom
constexpr uint16_t kForwardUs = 2000;  // direction switch positions
constexpr uint16_t kStopUs    = 1500;
constexpr uint16_t kReverseUs = 1000;

constexpr RcInput frameAt(uint32_t ms, uint16_t steeringUs, uint16_t throttleUs, uint16_t armUs,
                          uint16_t directionUs, bool linkStats = true) noexcept {
    RcInput input{};
    input.receivedMs   = ms;
    input.steeringUs   = steeringUs;
    input.throttleUs   = throttleUs;
    input.armUs        = armUs;
    input.directionUs  = directionUs;
    input.linkStatsMs  = ms;
    input.uplinkLq     = 100;
    input.hasLinkStats = linkStats;
    return input;
}

/** Link up with the arm switch OFF, then switch ON with the gas at zero. Direction switch at forward. */
constexpr bool arm(DriveController& c, uint32_t t) noexcept {
    RcInput frame = frameAt(t, 1500, kZeroGasUs, 1000, kForwardUs);
    c.update(&frame, t);
    frame = frameAt(t + 10, 1500, kZeroGasUs, 2000, kForwardUs);
    c.update(&frame, t + 10);
    return c.state() == State::Armed && c.lastEvent() == Event::Armed;
}

constexpr bool bootIsFailsafe() noexcept {
    DriveController    c{DriveController::Config{}};
    const DriveCommand cmd = c.update(nullptr, 0);
    return c.state() == State::Failsafe && c.lastEvent() == Event::None && cmd.steeringUs == 1500 &&
           cmd.speedPermille == 0 && c.msUntilFailsafe(0) == 0;
}

constexpr bool linkUpIsDisarmedAndSteeringFollows() noexcept {
    DriveController    c{DriveController::Config{}};
    const RcInput      frame = frameAt(100, 1700, 1800, 1000, kForwardUs);  // gas applied, but disarmed
    const DriveCommand cmd   = c.update(&frame, 100);
    return c.state() == State::Disarmed && c.lastEvent() == Event::LinkUp && cmd.steeringUs == 1700 &&
           cmd.speedPermille == 0;
}

constexpr bool frameWithoutLinkStatsStaysFailsafe() noexcept {
    DriveController    c{DriveController::Config{}};
    const RcInput      frame = frameAt(100, 1700, kZeroGasUs, 1000, kForwardUs, /*linkStats=*/false);
    const DriveCommand cmd   = c.update(&frame, 100);
    return c.state() == State::Failsafe && cmd.steeringUs == 1500;
}

constexpr bool linkStatsCheckCanBeDisabled() noexcept {
    DriveController::Config config{};
    config.linkStatsStaleMs = 0;
    DriveController c{config};
    const RcInput   frame = frameAt(100, 1700, kZeroGasUs, 1000, kForwardUs, /*linkStats=*/false);
    c.update(&frame, 100);
    return c.state() == State::Disarmed;
}

constexpr bool switchAlreadyOnDoesNotArm() noexcept {
    DriveController c{DriveController::Config{}};
    RcInput         frame = frameAt(100, 1500, kZeroGasUs, 2000, kForwardUs);  // first frame with the switch ON
    c.update(&frame, 100);
    frame = frameAt(110, 1500, kZeroGasUs, 2000, kForwardUs);
    c.update(&frame, 110);
    return c.state() == State::Disarmed;
}

constexpr bool armRequiresZeroGas() noexcept {
    DriveController c{DriveController::Config{}};
    RcInput         frame = frameAt(100, 1500, kZeroGasUs, 1000, kForwardUs);  // link up, switch OFF
    c.update(&frame, 100);
    frame = frameAt(110, 1500, 1100, 2000, kForwardUs);  // switch ON with gas applied -> refused
    c.update(&frame, 110);
    if (c.state() != State::Disarmed || c.lastEvent() != Event::ArmRefusedThrottle) {
        return false;
    }
    frame                  = frameAt(120, 1500, kZeroGasUs, 2000, kForwardUs);  // gas released, switch still ON
    const DriveCommand cmd = c.update(&frame, 120);
    if (c.state() != State::Disarmed || cmd.speedPermille != 0) {
        return false;
    }
    frame = frameAt(130, 1500, kZeroGasUs, 1000, kForwardUs);  // OFF
    c.update(&frame, 130);
    frame = frameAt(140, 1500, kZeroGasUs, 2000, kForwardUs);  // ON with the gas at zero -> armed
    c.update(&frame, 140);
    return c.state() == State::Armed && c.lastEvent() == Event::Armed;
}

constexpr bool gasMapsToSpeed() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput      frame = frameAt(120, 1500, 1400, 2000, kForwardUs);
    DriveCommand cmd   = c.update(&frame, 120);
    if (cmd.speedPermille != 400) {
        return false;
    }
    frame = frameAt(130, 1500, 1015, 2000, kForwardUs);  // inside the 20 µs deadband
    cmd   = c.update(&frame, 130);
    if (cmd.speedPermille != 0) {
        return false;
    }
    frame = frameAt(140, 2011, 2011, 2000, kForwardUs);  // full stick: clamped to the servo range and 1000
    cmd   = c.update(&frame, 140);
    if (cmd.speedPermille != 1000 || cmd.steeringUs != 2000) {
        return false;
    }
    frame = frameAt(150, 988, 988, 2000, kForwardUs);  // stick at the bottom: below the zero point is still zero
    cmd   = c.update(&frame, 150);
    return cmd.speedPermille == 0 && cmd.steeringUs == 1000;
}

constexpr bool directionSwitchSetsTheSign() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput      frame = frameAt(120, 1500, kZeroGasUs, 2000, kReverseUs);  // reverse selected at zero gas
    DriveCommand cmd   = c.update(&frame, 120);
    if (cmd.speedPermille != 0 || c.lastEvent() != Event::None) {
        return false;
    }
    frame = frameAt(130, 1500, 1400, 2000, kReverseUs);
    cmd   = c.update(&frame, 130);
    if (cmd.speedPermille != -400) {
        return false;
    }
    frame = frameAt(140, 1500, 1400, 2000, kStopUs);  // middle position: brake at once, no event
    cmd   = c.update(&frame, 140);
    return cmd.speedPermille == 0 && c.state() == State::Armed && c.lastEvent() == Event::None;
}

constexpr bool directionChangeNeedsZeroGas() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput      frame = frameAt(120, 1500, 1600, 2000, kForwardUs);
    DriveCommand cmd   = c.update(&frame, 120);
    if (cmd.speedPermille != 600) {
        return false;
    }
    frame = frameAt(130, 1500, 1600, 2000, kReverseUs);  // flipped with the gas applied -> braked
    cmd   = c.update(&frame, 130);
    if (cmd.speedPermille != 0 || c.lastEvent() != Event::DirectionBlocked || c.state() != State::Armed) {
        return false;
    }
    frame = frameAt(140, 1500, 1600, 2000, kReverseUs);  // still held: stays braked, reported once
    cmd   = c.update(&frame, 140);
    if (cmd.speedPermille != 0 || c.lastEvent() != Event::None) {
        return false;
    }
    frame = frameAt(150, 1500, kZeroGasUs, 2000, kReverseUs);  // gas released: reverse accepted
    cmd   = c.update(&frame, 150);
    if (cmd.speedPermille != 0) {
        return false;
    }
    frame = frameAt(160, 1500, 1600, 2000, kReverseUs);
    cmd   = c.update(&frame, 160);
    return cmd.speedPermille == -600;
}

constexpr bool leavingStopNeedsZeroGas() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput frame = frameAt(120, 1500, 1600, 2000, kStopUs);  // stop with the gas applied
    c.update(&frame, 120);
    frame            = frameAt(130, 1500, 1600, 2000, kForwardUs);  // back to forward, gas still applied
    DriveCommand cmd = c.update(&frame, 130);
    if (cmd.speedPermille != 0 || c.lastEvent() != Event::DirectionBlocked) {
        return false;
    }
    frame = frameAt(140, 1500, kZeroGasUs, 2000, kForwardUs);
    c.update(&frame, 140);
    frame = frameAt(150, 1500, 1300, 2000, kForwardUs);
    cmd   = c.update(&frame, 150);
    return cmd.speedPermille == 300;
}

constexpr bool maxSpeedIsClamped() noexcept {
    DriveController::Config config{};
    config.maxSpeedPermille = 500;
    DriveController c{config};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput            frame = frameAt(120, 1500, 2000, 2000, kForwardUs);
    const DriveCommand cmd   = c.update(&frame, 120);
    return cmd.speedPermille == 500;
}

constexpr bool reversedFlagsInvert() noexcept {
    DriveController::Config config{};
    config.steeringReversed = true;
    config.throttleReversed = true;
    DriveController c{config};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput            frame = frameAt(120, 1600, 1400, 2000, kForwardUs);
    const DriveCommand cmd   = c.update(&frame, 120);
    return cmd.steeringUs == 1400 && cmd.speedPermille == -400;
}

constexpr bool switchOffDisarmsAndBrakes() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput frame = frameAt(120, 1500, 1800, 2000, kForwardUs);
    c.update(&frame, 120);
    frame                  = frameAt(130, 1500, 1800, 1000, kForwardUs);
    const DriveCommand cmd = c.update(&frame, 130);
    return c.state() == State::Disarmed && c.lastEvent() == Event::Disarmed && cmd.speedPermille == 0;
}

constexpr bool tickWithoutFrameKeepsLastCommand() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput frame = frameAt(120, 1650, 1500, 2000, kForwardUs);
    c.update(&frame, 120);
    const DriveCommand cmd = c.update(nullptr, 200);
    return c.state() == State::Armed && c.lastEvent() == Event::None && cmd.steeringUs == 1650 &&
           cmd.speedPermille == 500;
}

constexpr bool timeoutFiresAtExactly250ms() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput frame = frameAt(200, 1600, 1600, 2000, kForwardUs);
    c.update(&frame, 200);
    DriveCommand cmd = c.update(nullptr, 449);
    if (c.state() != State::Armed || cmd.speedPermille != 600 || cmd.steeringUs != 1600 ||
        c.msUntilFailsafe(449) != 1) {
        return false;
    }
    cmd = c.update(nullptr, 450);
    return c.state() == State::Failsafe && c.lastEvent() == Event::FailsafeTimeout && cmd.speedPermille == 0 &&
           cmd.steeringUs == 1500 && c.msUntilFailsafe(450) == 0 && c.frameAgeMs(450) == 250;
}

constexpr bool switchLeftOnDoesNotRearmAfterFailsafe() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput frame = frameAt(120, 1500, 1600, 2000, kForwardUs);  // driving forward
    c.update(&frame, 120);
    c.update(nullptr, 1000);  // -> Failsafe
    frame            = frameAt(2000, 1500, 1600, 2000, kForwardUs);  // link back, switch ON, gas still applied
    DriveCommand cmd = c.update(&frame, 2000);
    if (c.state() != State::Disarmed || c.lastEvent() != Event::LinkUp || cmd.speedPermille != 0) {
        return false;
    }
    frame = frameAt(2010, 1500, kZeroGasUs, 2000, kForwardUs);
    c.update(&frame, 2010);
    if (c.state() != State::Disarmed) {
        return false;
    }
    frame = frameAt(2020, 1500, kZeroGasUs, 1000, kForwardUs);  // OFF ...
    c.update(&frame, 2020);
    frame = frameAt(2030, 1500, kZeroGasUs, 2000, kForwardUs);  // ... ON -> armed again
    c.update(&frame, 2030);
    return c.state() == State::Armed;
}

constexpr bool staleLinkStatsOrZeroLqIsFailsafe() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 100)) {
        return false;
    }
    RcInput frame     = frameAt(1000, 1500, 1400, 2000, kForwardUs);
    frame.linkStatsMs = 1000 - 401;  // older than linkStatsStaleMs
    c.update(&frame, 1000);
    if (c.state() != State::Failsafe || c.lastEvent() != Event::FailsafeLinkStats) {
        return false;
    }

    DriveController d{DriveController::Config{}};
    if (!arm(d, 100)) {
        return false;
    }
    frame                  = frameAt(1000, 1500, 1400, 2000, kForwardUs);
    frame.uplinkLq         = 0;
    const DriveCommand cmd = d.update(&frame, 1000);
    return d.state() == State::Failsafe && cmd.speedPermille == 0 && cmd.steeringUs == 1500;
}

constexpr bool tickWrapIsHandled() noexcept {
    DriveController c{DriveController::Config{}};
    if (!arm(c, 0xFFFFFF00u)) {
        return false;
    }
    RcInput frame = frameAt(0xFFFFFFF0u, 1500, 1400, 2000, kForwardUs);
    c.update(&frame, 0xFFFFFFF0u);
    const DriveCommand cmd = c.update(nullptr, 0x10u);  // 32 ms later, past the wrap
    return c.state() == State::Armed && cmd.speedPermille == 400 && c.msUntilFailsafe(0x10u) == 250 - 32;
}

constexpr bool run() noexcept {
    return bootIsFailsafe() && linkUpIsDisarmedAndSteeringFollows() && frameWithoutLinkStatsStaysFailsafe() &&
           linkStatsCheckCanBeDisabled() && switchAlreadyOnDoesNotArm() && armRequiresZeroGas() &&
           gasMapsToSpeed() && directionSwitchSetsTheSign() && directionChangeNeedsZeroGas() &&
           leavingStopNeedsZeroGas() && maxSpeedIsClamped() && reversedFlagsInvert() &&
           switchOffDisarmsAndBrakes() && tickWithoutFrameKeepsLastCommand() && timeoutFiresAtExactly250ms() &&
           switchLeftOnDoesNotRearmAfterFailsafe() && staleLinkStatsOrZeroLqIsFailsafe() && tickWrapIsHandled();
}

}  // namespace drive_selftest
