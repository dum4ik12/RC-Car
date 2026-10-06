// Host unit tests for lib/Control (run with `pio test -e native`).
#include <unity.h>

#include <cstdint>
#include <initializer_list>

#include <DriveController.h>
#include <DriveControllerSelfTest.h>

using State = DriveController::State;
using Event = DriveController::Event;

using drive_selftest::kForwardUs;
using drive_selftest::kReverseUs;
using drive_selftest::kStopUs;
using drive_selftest::kZeroGasUs;

namespace {

RcInput frameAt(uint32_t ms, uint16_t steeringUs, uint16_t throttleUs, uint16_t armUs,
                uint16_t directionUs = kForwardUs, bool linkStats = true) {
    return drive_selftest::frameAt(ms, steeringUs, throttleUs, armUs, directionUs, linkStats);
}

DriveController armed(const DriveController::Config& config = DriveController::Config{}) {
    DriveController controller{config};
    TEST_ASSERT_TRUE(drive_selftest::arm(controller, 100));
    return controller;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_compile_time_scenarios_pass() {
    TEST_ASSERT_TRUE(drive_selftest::run());
}

void test_boot_is_failsafe_with_no_frames() {
    DriveController c{DriveController::Config{}};
    for (uint32_t t = 0; t < 5000; t += 50) {
        const DriveCommand cmd = c.update(nullptr, t);
        TEST_ASSERT_EQUAL(State::Failsafe, c.state());
        TEST_ASSERT_EQUAL(Event::None, c.lastEvent());
        TEST_ASSERT_EQUAL_UINT16(1500, cmd.steeringUs);
        TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    }
}

void test_default_input_asks_for_no_motion() {
    const RcInput input{};
    TEST_ASSERT_EQUAL_UINT16(1500, input.steeringUs);
    TEST_ASSERT_EQUAL_UINT16(kZeroGasUs, input.throttleUs);
    TEST_ASSERT_EQUAL_UINT16(kStopUs, input.directionUs);
    TEST_ASSERT_EQUAL_UINT16(1000, input.armUs);
}

void test_link_up_disarmed_steering_follows_motor_braked() {
    DriveController c{DriveController::Config{}};
    RcInput         frame = frameAt(100, 1700, 1900, 1000);  // gas applied, but disarmed
    DriveCommand    cmd   = c.update(&frame, 100);
    TEST_ASSERT_EQUAL(State::Disarmed, c.state());
    TEST_ASSERT_EQUAL(Event::LinkUp, c.lastEvent());
    TEST_ASSERT_EQUAL_UINT16(1700, cmd.steeringUs);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);

    frame = frameAt(110, 1200, 1900, 1000, kReverseUs);
    cmd   = c.update(&frame, 110);
    TEST_ASSERT_EQUAL_UINT16(1200, cmd.steeringUs);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL(Event::None, c.lastEvent());  // the direction switch means nothing while disarmed
}

void test_frame_without_link_stats_stays_failsafe_unless_disabled() {
    DriveController strict{DriveController::Config{}};
    RcInput         frame = frameAt(100, 1700, kZeroGasUs, 1000, kForwardUs, false);
    strict.update(&frame, 100);
    TEST_ASSERT_EQUAL(State::Failsafe, strict.state());

    DriveController::Config config{};
    config.linkStatsStaleMs = 0;
    DriveController lenient{config};
    lenient.update(&frame, 100);
    TEST_ASSERT_EQUAL(State::Disarmed, lenient.state());
}

void test_arm_refused_with_gas_then_needs_new_edge() {
    DriveController c{DriveController::Config{}};
    RcInput         frame = frameAt(100, 1500, kZeroGasUs, 1000);
    c.update(&frame, 100);
    frame = frameAt(110, 1500, 1021, 2000);  // just outside the deadband
    c.update(&frame, 110);
    TEST_ASSERT_EQUAL(State::Disarmed, c.state());
    TEST_ASSERT_EQUAL(Event::ArmRefusedThrottle, c.lastEvent());

    frame = frameAt(120, 1500, kZeroGasUs, 2000);  // gas released, switch still on
    c.update(&frame, 120);
    TEST_ASSERT_EQUAL(State::Disarmed, c.state());
    TEST_ASSERT_EQUAL(Event::None, c.lastEvent());

    frame = frameAt(130, 1500, kZeroGasUs, 1000);
    c.update(&frame, 130);
    frame = frameAt(140, 1500, 1020, 2000);  // edge of the deadband still counts as zero
    c.update(&frame, 140);
    TEST_ASSERT_EQUAL(State::Armed, c.state());
    TEST_ASSERT_EQUAL(Event::Armed, c.lastEvent());
}

void test_arming_works_with_any_direction_switch_position() {
    for (const uint16_t directionUs : {kForwardUs, kStopUs, kReverseUs}) {
        DriveController c{DriveController::Config{}};
        RcInput         frame = frameAt(100, 1500, kZeroGasUs, 1000, directionUs);
        c.update(&frame, 100);
        frame                  = frameAt(110, 1500, kZeroGasUs, 2000, directionUs);
        const DriveCommand cmd = c.update(&frame, 110);
        TEST_ASSERT_EQUAL(State::Armed, c.state());
        TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    }
}

void test_gas_mapping_deadband_and_clamps() {
    DriveController c = armed();
    RcInput         frame;
    DriveCommand    cmd;

    frame = frameAt(120, 1500, 1400, 2000);
    cmd   = c.update(&frame, 120);
    TEST_ASSERT_EQUAL_INT16(400, cmd.speedPermille);

    frame = frameAt(130, 1500, 1020, 2000);  // edge of the deadband
    cmd   = c.update(&frame, 130);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);

    frame = frameAt(140, 1500, 1021, 2000);  // just outside
    cmd   = c.update(&frame, 140);
    TEST_ASSERT_EQUAL_INT16(21, cmd.speedPermille);

    frame = frameAt(150, 2011, 2011, 2000);
    cmd   = c.update(&frame, 150);
    TEST_ASSERT_EQUAL_INT16(1000, cmd.speedPermille);
    TEST_ASSERT_EQUAL_UINT16(2000, cmd.steeringUs);

    frame = frameAt(160, 988, 988, 2000);  // stick at the bottom reads below the zero point
    cmd   = c.update(&frame, 160);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL_UINT16(1000, cmd.steeringUs);
}

void test_direction_switch_thresholds() {
    DriveController c = armed();
    RcInput         frame;
    DriveCommand    cmd;

    // Select each position at zero gas, then apply the gas.
    const struct {
        uint16_t directionUs;
        int16_t  expected;
    } cases[] = {
        {2011, 400}, {1701, 400}, {1700, 0}, {1500, 0}, {1300, 0}, {1299, -400}, {988, -400},
    };
    uint32_t t = 120;
    for (const auto& item : cases) {
        frame = frameAt(t, 1500, kZeroGasUs, 2000, item.directionUs);
        c.update(&frame, t);
        frame = frameAt(t + 10, 1500, 1400, 2000, item.directionUs);
        cmd   = c.update(&frame, t + 10);
        TEST_ASSERT_EQUAL_INT16(item.expected, cmd.speedPermille);
        TEST_ASSERT_EQUAL(Event::None, c.lastEvent());
        t += 20;
    }
}

void test_direction_change_with_gas_brakes_until_gas_released() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1500, 1600, 2000, kForwardUs);
    DriveCommand    cmd   = c.update(&frame, 120);
    TEST_ASSERT_EQUAL_INT16(600, cmd.speedPermille);

    frame = frameAt(130, 1500, 1600, 2000, kReverseUs);
    cmd   = c.update(&frame, 130);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL(Event::DirectionBlocked, c.lastEvent());
    TEST_ASSERT_EQUAL(State::Armed, c.state());

    for (uint32_t t = 140; t < 200; t += 10) {  // stays braked, reported once
        frame = frameAt(t, 1500, 1600, 2000, kReverseUs);
        cmd   = c.update(&frame, t);
        TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
        TEST_ASSERT_EQUAL(Event::None, c.lastEvent());
    }

    frame = frameAt(200, 1500, kZeroGasUs, 2000, kReverseUs);
    cmd   = c.update(&frame, 200);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    frame = frameAt(210, 1500, 1600, 2000, kReverseUs);
    cmd   = c.update(&frame, 210);
    TEST_ASSERT_EQUAL_INT16(-600, cmd.speedPermille);
}

void test_flipping_back_to_the_old_direction_with_gas_stays_braked() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1500, 1600, 2000, kForwardUs);
    c.update(&frame, 120);
    frame = frameAt(130, 1500, 1600, 2000, kReverseUs);  // blocked
    c.update(&frame, 130);
    frame            = frameAt(140, 1500, 1600, 2000, kForwardUs);  // back to forward, gas never released
    DriveCommand cmd = c.update(&frame, 140);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL(Event::None, c.lastEvent());  // still the same blocked episode

    frame = frameAt(150, 1500, kZeroGasUs, 2000, kForwardUs);
    c.update(&frame, 150);
    frame = frameAt(160, 1500, 1600, 2000, kForwardUs);
    cmd   = c.update(&frame, 160);
    TEST_ASSERT_EQUAL_INT16(600, cmd.speedPermille);
}

void test_stop_position_brakes_at_once_and_leaving_it_needs_zero_gas() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1500, 1600, 2000, kForwardUs);
    c.update(&frame, 120);

    frame            = frameAt(130, 1500, 1600, 2000, kStopUs);
    DriveCommand cmd = c.update(&frame, 130);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL(Event::None, c.lastEvent());
    TEST_ASSERT_EQUAL(State::Armed, c.state());

    frame = frameAt(140, 1500, 1600, 2000, kForwardUs);
    cmd   = c.update(&frame, 140);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL(Event::DirectionBlocked, c.lastEvent());

    frame = frameAt(150, 1500, kZeroGasUs, 2000, kForwardUs);
    c.update(&frame, 150);
    frame = frameAt(160, 1500, 1300, 2000, kForwardUs);
    cmd   = c.update(&frame, 160);
    TEST_ASSERT_EQUAL_INT16(300, cmd.speedPermille);
}

void test_disarm_clears_the_direction() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1500, 1600, 2000, kForwardUs);
    c.update(&frame, 120);
    frame = frameAt(130, 1500, 1600, 1000, kForwardUs);  // disarm while driving
    c.update(&frame, 130);
    TEST_ASSERT_EQUAL(State::Disarmed, c.state());

    frame = frameAt(140, 1500, 1600, 2000, kForwardUs);  // switch ON again with the gas applied -> refused
    DriveCommand cmd = c.update(&frame, 140);
    TEST_ASSERT_EQUAL(Event::ArmRefusedThrottle, c.lastEvent());
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
}

void test_max_speed_limit() {
    DriveController::Config config{};
    config.maxSpeedPermille = 500;
    DriveController c     = armed(config);
    RcInput         frame = frameAt(120, 1500, 2000, 2000, kForwardUs);
    DriveCommand    cmd   = c.update(&frame, 120);
    TEST_ASSERT_EQUAL_INT16(500, cmd.speedPermille);

    frame = frameAt(130, 1500, kZeroGasUs, 2000, kReverseUs);
    c.update(&frame, 130);
    frame = frameAt(140, 1500, 2000, 2000, kReverseUs);
    cmd   = c.update(&frame, 140);
    TEST_ASSERT_EQUAL_INT16(-500, cmd.speedPermille);
}

void test_reversed_flags() {
    DriveController::Config config{};
    config.steeringReversed = true;
    config.throttleReversed = true;
    DriveController    c     = armed(config);
    RcInput            frame = frameAt(120, 1600, 1400, 2000, kForwardUs);
    const DriveCommand cmd   = c.update(&frame, 120);
    TEST_ASSERT_EQUAL_UINT16(1400, cmd.steeringUs);
    TEST_ASSERT_EQUAL_INT16(-400, cmd.speedPermille);
}

void test_switch_off_disarms_and_brakes() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1500, 1800, 2000);
    c.update(&frame, 120);
    frame                  = frameAt(130, 1500, 1800, 1000);
    const DriveCommand cmd = c.update(&frame, 130);
    TEST_ASSERT_EQUAL(State::Disarmed, c.state());
    TEST_ASSERT_EQUAL(Event::Disarmed, c.lastEvent());
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
}

void test_switch_mid_position_changes_nothing() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1500, 1800, 1500);  // arm switch between the thresholds
    c.update(&frame, 120);
    TEST_ASSERT_EQUAL(State::Armed, c.state());
}

void test_tick_without_frame_keeps_last_command() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1650, 1500, 2000);
    c.update(&frame, 120);
    for (uint32_t t = 121; t < 120 + 250; t += 7) {
        const DriveCommand cmd = c.update(nullptr, t);
        TEST_ASSERT_EQUAL(State::Armed, c.state());
        TEST_ASSERT_EQUAL_UINT16(1650, cmd.steeringUs);
        TEST_ASSERT_EQUAL_INT16(500, cmd.speedPermille);
    }
}

void test_timeout_at_exactly_250ms() {
    DriveController c     = armed();
    RcInput         frame = frameAt(200, 1600, 1600, 2000);
    c.update(&frame, 200);
    TEST_ASSERT_EQUAL_UINT32(250, c.msUntilFailsafe(200));
    TEST_ASSERT_EQUAL_UINT32(1, c.msUntilFailsafe(449));

    DriveCommand cmd = c.update(nullptr, 449);
    TEST_ASSERT_EQUAL(State::Armed, c.state());
    TEST_ASSERT_EQUAL_INT16(600, cmd.speedPermille);

    cmd = c.update(nullptr, 450);
    TEST_ASSERT_EQUAL(State::Failsafe, c.state());
    TEST_ASSERT_EQUAL(Event::FailsafeTimeout, c.lastEvent());
    TEST_ASSERT_EQUAL_UINT16(1500, cmd.steeringUs);
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL_UINT32(0, c.msUntilFailsafe(450));
    TEST_ASSERT_EQUAL_UINT32(250, c.frameAgeMs(450));

    // The failsafe event is reported once; later ticks stay silent.
    c.update(nullptr, 500);
    TEST_ASSERT_EQUAL(Event::None, c.lastEvent());
}

void test_switch_left_on_does_not_rearm_after_failsafe() {
    DriveController c     = armed();
    RcInput         frame = frameAt(120, 1500, 1600, 2000);  // driving forward
    c.update(&frame, 120);
    c.update(nullptr, 1000);
    TEST_ASSERT_EQUAL(State::Failsafe, c.state());

    frame            = frameAt(2000, 1500, 1600, 2000);  // link back: switch ON, gas still applied
    DriveCommand cmd = c.update(&frame, 2000);
    TEST_ASSERT_EQUAL(State::Disarmed, c.state());
    TEST_ASSERT_EQUAL(Event::LinkUp, c.lastEvent());
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    frame = frameAt(2010, 1500, kZeroGasUs, 2000);
    c.update(&frame, 2010);
    TEST_ASSERT_EQUAL(State::Disarmed, c.state());

    frame = frameAt(2020, 1500, kZeroGasUs, 1000);
    c.update(&frame, 2020);
    frame = frameAt(2030, 1500, kZeroGasUs, 2000);
    c.update(&frame, 2030);
    TEST_ASSERT_EQUAL(State::Armed, c.state());
}

void test_stale_link_stats_and_zero_lq_are_failsafe() {
    DriveController c     = armed();
    RcInput         frame = frameAt(1000, 1500, 1400, 2000);
    frame.linkStatsMs     = 1000 - 400;  // exactly at the limit: still fresh
    c.update(&frame, 1000);
    TEST_ASSERT_EQUAL(State::Armed, c.state());

    frame             = frameAt(1010, 1500, 1400, 2000);
    frame.linkStatsMs = 1010 - 401;
    c.update(&frame, 1010);
    TEST_ASSERT_EQUAL(State::Failsafe, c.state());
    TEST_ASSERT_EQUAL(Event::FailsafeLinkStats, c.lastEvent());

    DriveController d = armed();
    frame             = frameAt(1000, 1500, 1400, 2000);
    frame.uplinkLq    = 0;
    const DriveCommand cmd = d.update(&frame, 1000);
    TEST_ASSERT_EQUAL(State::Failsafe, d.state());
    TEST_ASSERT_EQUAL_INT16(0, cmd.speedPermille);
    TEST_ASSERT_EQUAL_UINT16(1500, cmd.steeringUs);
}

void test_tick_counter_wrap() {
    DriveController c{DriveController::Config{}};
    TEST_ASSERT_TRUE(drive_selftest::arm(c, 0xFFFFFF00u));
    RcInput frame = frameAt(0xFFFFFFF0u, 1500, 1400, 2000);
    c.update(&frame, 0xFFFFFFF0u);
    DriveCommand cmd = c.update(nullptr, 0x10u);  // 32 ms later
    TEST_ASSERT_EQUAL(State::Armed, c.state());
    TEST_ASSERT_EQUAL_INT16(400, cmd.speedPermille);
    TEST_ASSERT_EQUAL_UINT32(250 - 32, c.msUntilFailsafe(0x10u));
    cmd = c.update(nullptr, 0xFFFFFFF0u + 250u);  // exactly the timeout, across the wrap
    TEST_ASSERT_EQUAL(State::Failsafe, c.state());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_compile_time_scenarios_pass);
    RUN_TEST(test_boot_is_failsafe_with_no_frames);
    RUN_TEST(test_default_input_asks_for_no_motion);
    RUN_TEST(test_link_up_disarmed_steering_follows_motor_braked);
    RUN_TEST(test_frame_without_link_stats_stays_failsafe_unless_disabled);
    RUN_TEST(test_arm_refused_with_gas_then_needs_new_edge);
    RUN_TEST(test_arming_works_with_any_direction_switch_position);
    RUN_TEST(test_gas_mapping_deadband_and_clamps);
    RUN_TEST(test_direction_switch_thresholds);
    RUN_TEST(test_direction_change_with_gas_brakes_until_gas_released);
    RUN_TEST(test_flipping_back_to_the_old_direction_with_gas_stays_braked);
    RUN_TEST(test_stop_position_brakes_at_once_and_leaving_it_needs_zero_gas);
    RUN_TEST(test_disarm_clears_the_direction);
    RUN_TEST(test_max_speed_limit);
    RUN_TEST(test_reversed_flags);
    RUN_TEST(test_switch_off_disarms_and_brakes);
    RUN_TEST(test_switch_mid_position_changes_nothing);
    RUN_TEST(test_tick_without_frame_keeps_last_command);
    RUN_TEST(test_timeout_at_exactly_250ms);
    RUN_TEST(test_switch_left_on_does_not_rearm_after_failsafe);
    RUN_TEST(test_stale_link_stats_and_zero_lq_are_failsafe);
    RUN_TEST(test_tick_counter_wrap);
    return UNITY_END();
}
