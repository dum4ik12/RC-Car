#pragma once

#include <cstdint>

/** @brief Decoded radio input for one control step: pulse widths in µs, times in ms. */
struct RcInput {
    uint32_t receivedMs   = 0;     ///< when the CRC-valid channel frame completed
    uint16_t steeringUs   = 1500;
    uint16_t throttleUs   = 1000;  ///< gas: ~1000 = zero (stick at the bottom), ~2000 = full
    uint16_t armUs        = 1000;
    uint16_t directionUs  = 1500;  ///< 3-position switch: forward / stop / reverse
    uint32_t linkStatsMs  = 0;     ///< when the newest link-statistics frame arrived (if hasLinkStats)
    uint8_t  uplinkLq     = 0;     ///< uplink link quality in % (if hasLinkStats)
    bool     hasLinkStats = false;
};

/** @brief What the actuators must do right now. speedPermille == 0 means brake. */
struct DriveCommand {
    uint16_t steeringUs;
    int16_t  speedPermille;
};

/**
 * @brief Arming, failsafe and stick-to-actuator mapping for the car. Pure logic, no hardware.
 *
 * Call update() once per received radio frame, and again from a timer while no
 * frame arrives. It returns the command the actuators must apply. All arithmetic
 * is integer; time is an unsigned millisecond counter and only differences are
 * used, so wrap-around is harmless.
 *
 * States
 *   Failsafe  Boot state. Also entered when frames stop for failsafeTimeoutMs or
 *             when a frame is not backed by fresh link statistics. Steering neutral,
 *             motor braked. Left as soon as a trusted frame arrives -> Disarmed.
 *   Disarmed  Steering follows the stick, motor braked. Arms on an OFF->ON edge of
 *             the arm switch, but only while the gas is at zero. The edge is
 *             consumed either way, so a refused attempt needs a new switch cycle.
 *   Armed     Steering follows the stick. The gas stick sets the speed, the
 *             direction switch sets the sign. Arm switch OFF -> Disarmed.
 *
 * Every drop into Failsafe clears the switch history: a switch left ON never
 * re-arms the car by itself.
 *
 * Throttle and direction: the gas stick is one-directional (throttleZeroUs = zero
 * speed, +1 µs = +1 permille). A 3-position switch selects forward, stop or
 * reverse. Stop takes effect at once (motor braked). Forward and reverse are
 * accepted only while the gas is at zero, so the motor never starts or reverses
 * with a jump: moving the switch with the gas applied brakes the motor until
 * the stick is back at zero. Leaving Armed clears the selected direction.
 *
 * Link statistics: an ExpressLRS receiver sends them at 10 Hz while connected and
 * stops on link loss, even in the "Set/Last position" failsafe modes that keep
 * sending channel frames. Requiring fresh statistics therefore catches a link loss
 * that the frame timeout alone would miss. Set linkStatsStaleMs = 0 for a
 * receiver that doesn't send them.
 */
class DriveController {
public:
    struct Config {
        uint16_t neutralUs          = 1500;  ///< steering centre
        uint16_t throttleZeroUs     = 1000;  ///< gas stick at rest: zero speed; +1 µs = +1 permille
        uint16_t throttleDeadbandUs = 20;    ///< throttle <= zero + this counts as "gas at zero"
        uint16_t armOnUs            = 1700;  ///< arm switch is ON above this
        uint16_t armOffUs           = 1300;  ///< arm switch is OFF below this; in between: no change
        uint16_t directionForwardUs = 1700;  ///< direction switch selects forward above this
        uint16_t directionReverseUs = 1300;  ///< ... reverse below this; in between: stop (brake)
        uint32_t failsafeTimeoutMs  = 250;   ///< no valid frame for this long -> Failsafe
        uint32_t linkStatsStaleMs   = 400;   ///< frames need link statistics newer than this; 0 = don't check
        int16_t  maxSpeedPermille   = 1000;  ///< speed limit, both directions
        uint16_t steeringMinUs      = 1000;
        uint16_t steeringMaxUs      = 2000;
        bool     steeringReversed   = false;
        bool     throttleReversed   = false;  ///< swap forward and reverse
    };

    enum class State : uint8_t { Failsafe, Disarmed, Armed };

    /** What the last update() changed, for logging. */
    enum class Event : uint8_t {
        None,
        LinkUp,              ///< Failsafe -> Disarmed
        Armed,               ///< Disarmed -> Armed
        Disarmed,            ///< Armed -> Disarmed (switch off)
        ArmRefusedThrottle,  ///< switch edge with the gas not at zero; stays Disarmed
        DirectionBlocked,    ///< direction switch moved with the gas applied; braked until the gas is at zero
        FailsafeTimeout,     ///< -> Failsafe because frames stopped
        FailsafeLinkStats,   ///< -> Failsafe because link statistics are missing or stale
    };

    constexpr explicit DriveController(const Config& config) noexcept
        : config_(config), command_{config.neutralUs, 0} {}

    /**
     * Run one control step. @p frame is the new radio frame, or nullptr for a
     * timer tick without one. @p nowMs is the current time.
     */
    constexpr DriveCommand update(const RcInput* frame, uint32_t nowMs) noexcept {
        event_ = Event::None;

        if (frame != nullptr) {
            haveFrame_   = true;
            lastFrameMs_ = frame->receivedMs;
            if (!linkStatsFresh(*frame)) {
                return enterFailsafe(Event::FailsafeLinkStats);
            }
        }

        // Timeout check on every step, so the failsafe fires even if no frame ever arrives.
        if (!haveFrame_ || nowMs - lastFrameMs_ >= config_.failsafeTimeoutMs) {
            return enterFailsafe(Event::FailsafeTimeout);
        }

        if (frame == nullptr) {
            return command_;  // link alive, nothing new: keep driving as before
        }

        if (state_ == State::Failsafe) {
            state_         = State::Disarmed;
            event_         = Event::LinkUp;
            switchSeenOff_ = false;  // the switch must be cycled before the car can arm
        }

        const bool switchOn  = frame->armUs > config_.armOnUs;
        const bool switchOff = frame->armUs < config_.armOffUs;
        const bool gasAtZero = throttleAtZero(frame->throttleUs);
        if (switchOff) {
            switchSeenOff_ = true;
            if (state_ == State::Armed) {
                state_ = State::Disarmed;
                event_ = Event::Disarmed;
            }
        } else if (switchOn && switchSeenOff_ && state_ == State::Disarmed) {
            switchSeenOff_ = false;  // the edge is consumed whether or not arming succeeds
            if (gasAtZero) {
                state_ = State::Armed;
                event_ = Event::Armed;
            } else {
                event_ = Event::ArmRefusedThrottle;
            }
        }

        if (state_ == State::Armed) {
            updateDirection(switchDirection(frame->directionUs), gasAtZero);
        } else {
            clearDirection();
        }

        command_.steeringUs    = mapSteering(frame->steeringUs);
        command_.speedPermille = static_cast<int16_t>(direction_ * mapThrottle(frame->throttleUs));
        return command_;
    }

    constexpr State               state() const noexcept { return state_; }
    constexpr Event               lastEvent() const noexcept { return event_; }
    constexpr const DriveCommand& command() const noexcept { return command_; }
    constexpr const Config&       config() const noexcept { return config_; }
    constexpr bool                haveFrame() const noexcept { return haveFrame_; }

    /** Age of the last frame in ms, 0 if none arrived yet. */
    constexpr uint32_t frameAgeMs(uint32_t nowMs) const noexcept { return haveFrame_ ? nowMs - lastFrameMs_ : 0; }

    /** Time left until the frame timeout fires; 0 when already in Failsafe or overdue. */
    constexpr uint32_t msUntilFailsafe(uint32_t nowMs) const noexcept {
        if (!haveFrame_ || state_ == State::Failsafe) {
            return 0;
        }
        const uint32_t age = nowMs - lastFrameMs_;
        return age >= config_.failsafeTimeoutMs ? 0 : config_.failsafeTimeoutMs - age;
    }

private:
    static constexpr int32_t clamp(int32_t value, int32_t low, int32_t high) noexcept {
        return value < low ? low : (value > high ? high : value);
    }

    constexpr DriveCommand enterFailsafe(Event why) noexcept {
        if (state_ != State::Failsafe) {
            event_ = why;
        }
        state_         = State::Failsafe;
        switchSeenOff_ = false;
        clearDirection();
        command_ = DriveCommand{config_.neutralUs, 0};
        return command_;
    }

    constexpr bool linkStatsFresh(const RcInput& frame) const noexcept {
        if (config_.linkStatsStaleMs == 0) {
            return true;
        }
        return frame.hasLinkStats && frame.uplinkLq > 0 &&
               frame.receivedMs - frame.linkStatsMs <= config_.linkStatsStaleMs;
    }

    constexpr bool throttleAtZero(uint16_t us) const noexcept {
        return static_cast<int32_t>(us) <= static_cast<int32_t>(config_.throttleZeroUs) + config_.throttleDeadbandUs;
    }

    /** Position of the direction switch as a motor sign: +1 forward, -1 reverse, 0 stop. */
    constexpr int8_t switchDirection(uint16_t us) const noexcept {
        const int8_t forward = config_.throttleReversed ? -1 : 1;
        if (us > config_.directionForwardUs) {
            return forward;
        }
        if (us < config_.directionReverseUs) {
            return static_cast<int8_t>(-forward);
        }
        return 0;
    }

    /** Stop is taken at once; a drive direction only while the gas is at zero. Armed state only. */
    constexpr void updateDirection(int8_t wanted, bool gasAtZero) noexcept {
        if (wanted == direction_ || wanted == 0 || gasAtZero) {
            direction_        = wanted;
            directionBlocked_ = false;
            return;
        }
        direction_ = 0;  // brake until the gas is back at zero
        if (!directionBlocked_) {
            directionBlocked_ = true;
            event_            = Event::DirectionBlocked;
        }
    }

    constexpr void clearDirection() noexcept {
        direction_        = 0;
        directionBlocked_ = false;
    }

    constexpr uint16_t mapSteering(uint16_t us) const noexcept {
        int32_t value = us;
        if (config_.steeringReversed) {
            value = 2 * static_cast<int32_t>(config_.neutralUs) - value;
        }
        return static_cast<uint16_t>(clamp(value, config_.steeringMinUs, config_.steeringMaxUs));
    }

    /** Stick above throttleZeroUs -> 0..1000 permille, zero inside the deadband, limited to maxSpeedPermille. */
    constexpr int16_t mapThrottle(uint16_t us) const noexcept {
        if (throttleAtZero(us)) {
            return 0;
        }
        const int32_t gas = static_cast<int32_t>(us) - config_.throttleZeroUs;
        return static_cast<int16_t>(clamp(gas, 0, config_.maxSpeedPermille));
    }

    const Config config_;
    State        state_            = State::Failsafe;
    Event        event_            = Event::None;
    bool         haveFrame_        = false;
    bool         switchSeenOff_    = false;
    bool         directionBlocked_ = false;  // the switch asks for a direction that waits for zero gas
    int8_t       direction_        = 0;      // motor sign in use: +1, -1, or 0 = stop
    uint32_t     lastFrameMs_      = 0;
    DriveCommand command_;
};
