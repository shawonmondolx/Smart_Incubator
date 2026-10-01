#include <unity.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "control_logic.h"

void setUp() {}
void tearDown() {}

void test_manual_settings_reject_non_finite_values() {
  TEST_ASSERT_FALSE(control_logic::validManualSettings(
      NAN, 60.0F, 21, 2, 18));
  TEST_ASSERT_FALSE(control_logic::validManualSettings(
      37.5F, INFINITY, 21, 2, 18));
}

void test_manual_settings_validate_schedule_bounds() {
  TEST_ASSERT_TRUE(control_logic::validManualSettings(37.5F, 60.0F, 21, 2, 18));
  TEST_ASSERT_FALSE(control_logic::validManualSettings(37.5F, 60.0F, 21, 0, 18));
  TEST_ASSERT_FALSE(control_logic::validManualSettings(37.5F, 60.0F, 21, 2, 22));
}

void test_calibration_requires_95_percent_stability() {
  TEST_ASSERT_TRUE(control_logic::calibrationPassed(100, 95, false));
  TEST_ASSERT_FALSE(control_logic::calibrationPassed(100, 94, false));
  TEST_ASSERT_FALSE(control_logic::calibrationPassed(0, 0, false));
  TEST_ASSERT_FALSE(control_logic::calibrationPassed(100, 100, true));
}

void test_pwm_is_clamped_without_wrapping() {
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::clampPwm(-1));
  TEST_ASSERT_EQUAL_UINT8(128, control_logic::clampPwm(128));
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::clampPwm(300));
}

void test_fan_pwm_enforces_nonzero_minimum() {
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::minimumRunningPwm(0, 128));
  TEST_ASSERT_EQUAL_UINT8(128, control_logic::minimumRunningPwm(1, 128));
  TEST_ASSERT_EQUAL_UINT8(128, control_logic::minimumRunningPwm(127, 128));
  TEST_ASSERT_EQUAL_UINT8(200, control_logic::minimumRunningPwm(200, 128));
}

void test_moisture_pwm_is_bounded_and_adaptive() {
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::moistureExhaustPwm(0, 128, 0));
  const uint8_t base = control_logic::moistureExhaustPwm(6, 128, 0);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT8(128, base);
  TEST_ASSERT_GREATER_THAN_UINT8(base,
      control_logic::moistureExhaustPwm(6, 128, 16));
  TEST_ASSERT_EQUAL_UINT8(255,
      control_logic::moistureExhaustPwm(20, 128, 64));
}

void test_absolute_difference_is_order_independent() {
  TEST_ASSERT_EQUAL_UINT32(8, control_logic::absoluteDifference(10, 18));
  TEST_ASSERT_EQUAL_UINT32(8, control_logic::absoluteDifference(18, 10));
}

void test_chicken_phase_boundaries() {
  TEST_ASSERT_EQUAL_UINT8(17, control_logic::cycleDayForElapsed(16UL * 86400UL, 21));
  TEST_ASSERT_EQUAL_UINT8(18, control_logic::cycleDayForElapsed(17UL * 86400UL, 21));
  TEST_ASSERT_TRUE(control_logic::lockdownForDay(18, 18, false));
  TEST_ASSERT_FALSE(control_logic::cycleComplete(21UL * 86400UL - 1, 21));
  TEST_ASSERT_TRUE(control_logic::cycleComplete(21UL * 86400UL, 21));
}

void test_lockdown_is_sticky() {
  TEST_ASSERT_TRUE(control_logic::lockdownForDay(16, 18, true));
  TEST_ASSERT_TRUE(control_logic::lockdownForDay(16, 0, true));
  TEST_ASSERT_FALSE(control_logic::lockdownForDay(16, 0, false));
  TEST_ASSERT_FALSE(control_logic::lockdownForDay(16, 18, false));
}

void test_heater_surface_derating() {
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::heaterSurfaceDutyLimit(50, 50, 60));
  TEST_ASSERT_INT_WITHIN(1, 127, control_logic::heaterSurfaceDutyLimit(55, 50, 60));
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::heaterSurfaceDutyLimit(60, 50, 60));
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::heaterSurfaceDutyLimit(NAN, 50, 60));
}

void test_heater_surface_derating_rejects_invalid_thresholds() {
  const float nan_val = NAN;
  const float pos_inf = static_cast<float>(INFINITY);
  const float neg_inf = static_cast<float>(-INFINITY);
  const float invalid[] = {nan_val, pos_inf, neg_inf};
  for (size_t i = 0; i < 3; ++i) {
    const float threshold = invalid[i];
    TEST_ASSERT_EQUAL_UINT8(0,
        control_logic::heaterSurfaceDutyLimit(55, threshold, 60));
    TEST_ASSERT_EQUAL_UINT8(0,
        control_logic::heaterSurfaceDutyLimit(55, 50, threshold));
  }
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::heaterSurfaceDutyLimit(45, 50, 50));
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::heaterSurfaceDutyLimit(45, 60, 50));
}

void test_heater_pwm_saturates_before_integer_conversion() {
  TEST_ASSERT_EQUAL_UINT8(255,
      control_logic::adaptiveHeaterPwm(92, true, 1.0e9F, 0, 0));
  TEST_ASSERT_EQUAL_UINT8(0,
      control_logic::adaptiveHeaterPwm(92, true, -1.0e9F, 0, 0));
  TEST_ASSERT_EQUAL_UINT8(255,
      control_logic::adaptiveHeaterPwm(92, true, 0, 0, 1.0e9F));
  TEST_ASSERT_EQUAL_UINT8(0,
      control_logic::adaptiveHeaterPwm(92, true, 0, 0, -1.0e9F));
}

void test_heater_pwm_fails_closed_on_invalid_arithmetic() {
  const float nan_val2 = NAN;
  const float pos_inf2 = static_cast<float>(INFINITY);
  const float neg_inf2 = static_cast<float>(-INFINITY);
  const float invalid[] = {nan_val2, pos_inf2, neg_inf2};
  for (size_t i = 0; i < 3; ++i) {
    const float value = invalid[i];
    TEST_ASSERT_EQUAL_UINT8(0,
        control_logic::adaptiveHeaterPwm(92, true, value, 0, 0));
    TEST_ASSERT_EQUAL_UINT8(0,
        control_logic::adaptiveHeaterPwm(92, true, 0, value, 0));
    TEST_ASSERT_EQUAL_UINT8(0,
        control_logic::adaptiveHeaterPwm(92, true, 0, 0, value));
  }
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::adaptiveHeaterPwm(
      92, true, std::numeric_limits<float>::max(), 0, 0));
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::adaptiveHeaterPwm(
      92, true, 0, 0, std::numeric_limits<float>::max()));
}

void test_adaptive_heater_uses_learned_hold_pwm_at_target() {
  TEST_ASSERT_EQUAL_UINT8(
      92, control_logic::adaptiveHeaterPwm(92, true, 0, 0, 0));
  TEST_ASSERT_GREATER_THAN_UINT8(
      92, control_logic::adaptiveHeaterPwm(92, true, 0.5F, 0, 0));
  TEST_ASSERT_LESS_THAN_UINT8(
      92, control_logic::adaptiveHeaterPwm(92, true, -0.5F, 0, 0));
}

void test_adaptive_heater_ignores_untrained_feed_forward() {
  TEST_ASSERT_EQUAL_UINT8(
      25, control_logic::adaptiveHeaterPwm(92, false, 1.0F, 0, 0));
  TEST_ASSERT_EQUAL_UINT8(
      0, control_logic::adaptiveHeaterPwm(92, false, NAN, 0, 0));
}

void test_heater_response_accepts_incubator_temperature_rise() {
  TEST_ASSERT_TRUE(control_logic::heaterResponseDetected(
      37.2F, 37.0F, NAN, NAN, false));
}

void test_heater_response_accepts_surface_probe_evidence() {
  TEST_ASSERT_TRUE(control_logic::heaterResponseDetected(
      37.0F, 37.0F, 38.0F, 37.4F, true));
  TEST_ASSERT_TRUE(control_logic::heaterResponseDetected(
      37.0F, 37.0F, 39.0F, 39.0F, true));
}

void test_heater_response_rejects_no_temperature_change() {
  TEST_ASSERT_FALSE(control_logic::heaterResponseDetected(
      37.0F, 37.0F, 37.4F, 37.0F, true));
  TEST_ASSERT_FALSE(control_logic::heaterResponseDetected(
      37.0F, 37.0F, NAN, NAN, false));
}

void test_dual_humidity_consensus() {
  float result = 0;
  bool degraded = false;
  TEST_ASSERT_TRUE(
      control_logic::humidityConsensus(58, true, 62, true, 10, result, degraded));
  TEST_ASSERT_FLOAT_WITHIN(.01, 60, result);
  TEST_ASSERT_FALSE(degraded);
  TEST_ASSERT_TRUE(
      control_logic::humidityConsensus(58, true, 0, false, 10, result, degraded));
  TEST_ASSERT_TRUE(degraded);
  TEST_ASSERT_FALSE(
      control_logic::humidityConsensus(40, true, 70, true, 10, result, degraded));
}

void test_humidity_consensus_flags_degraded_mode() {
  // The helper is sensor-agnostic: with exactly one valid sensor it reports
  // that reading and raises the degraded flag. It is the caller's job to
  // reject degraded humidity for control purposes, because the outside sensor
  // cannot measure chamber humidity (see sampleEnvironment in main.cpp).
  float result = 0;
  bool degraded = false;
  TEST_ASSERT_TRUE(control_logic::humidityConsensus(
      0, false, 25, true, 10, result, degraded));
  TEST_ASSERT_TRUE(degraded);
  TEST_ASSERT_FLOAT_WITHIN(.01, 25, result);
}

void test_temperature_exhaust_thresholds_and_hysteresis() {
  const float target = 37.5F;
  bool venting = false;
  // Values keep a deliberate margin from the 0.4 C on / 0.2 C off thresholds
  // so the result does not hinge on the host's float precision.
  const float temperatures[] = {31.0F, target, target + .3F,
                                target + .5F, target + .35F,
                                target + .1F, target + .25F, target - .5F};
  const bool expected[] = {false, false, false, true, true, false, false, false};
  for (unsigned i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i) {
    venting = control_logic::temperatureExhaustRequired(
        temperatures[i], target, venting, false, .4F, .2F);
    TEST_ASSERT_EQUAL(expected[i], venting);
  }
  TEST_ASSERT_FALSE(control_logic::temperatureExhaustRequired(
      target, target, true, false, .4F, .2F));
}

void test_temperature_exhaust_preserves_safety_override() {
  TEST_ASSERT_TRUE(control_logic::temperatureExhaustRequired(
      39.5F, 40.0F, false, true, .4F, .2F));
  TEST_ASSERT_TRUE(control_logic::temperatureExhaustRequired(
      37.0F, 37.5F, false, true, .4F, .2F));
  TEST_ASSERT_FALSE(control_logic::temperatureExhaustRequired(
      NAN, 37.5F, true, false, .4F, .2F));
}

void test_exhaust_boost_expires_and_zero_stops_immediately() {
  uint8_t previous = 0;
  uint32_t started = 0;
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::exhaustStartupPwm(
      0, 128, 0, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::exhaustStartupPwm(
      1, 128, 0, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::exhaustStartupPwm(
      1, 128, 999, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(128, control_logic::exhaustStartupPwm(
      1, 128, 1000, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(200, control_logic::exhaustStartupPwm(
      200, 128, 1001, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::exhaustStartupPwm(
      255, 128, 1002, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::exhaustStartupPwm(
      0, 128, 1003, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::exhaustStartupPwm(
      100, 128, 1004, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(0, control_logic::exhaustStartupPwm(
      0, 128, 1005, 1000, previous, started));
}

void test_exhaust_boost_handles_millis_wrap() {
  uint8_t previous = 0;
  uint32_t started = 0;
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::exhaustStartupPwm(
      150, 128, UINT32_MAX - 499U, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(255, control_logic::exhaustStartupPwm(
      150, 128, 499, 1000, previous, started));
  TEST_ASSERT_EQUAL_UINT8(150, control_logic::exhaustStartupPwm(
      150, 128, 500, 1000, previous, started));
}

void test_heater_pwm_damping_and_small_target_steps() {
  TEST_ASSERT_EQUAL_UINT8(90,
      control_logic::adaptiveHeaterPwm(92, true, -.0625F, 0, 0));
  TEST_ASSERT_EQUAL_UINT8(94,
      control_logic::adaptiveHeaterPwm(92, true, .0625F, 0, 0));
  TEST_ASSERT_EQUAL_UINT8(82,
      control_logic::adaptiveHeaterPwm(92, true, 0, 0, -.05F));
  TEST_ASSERT_EQUAL_UINT8(102,
      control_logic::adaptiveHeaterPwm(92, true, 0, 0, .05F));
  TEST_ASSERT_EQUAL_UINT8(255,
      control_logic::adaptiveHeaterPwm(92, true, 20, 80, 0));
  TEST_ASSERT_EQUAL_UINT8(0,
      control_logic::adaptiveHeaterPwm(92, true, -20, -80, 0));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_temperature_exhaust_thresholds_and_hysteresis);
  RUN_TEST(test_temperature_exhaust_preserves_safety_override);
  RUN_TEST(test_exhaust_boost_expires_and_zero_stops_immediately);
  RUN_TEST(test_exhaust_boost_handles_millis_wrap);
  RUN_TEST(test_heater_pwm_damping_and_small_target_steps);
  RUN_TEST(test_manual_settings_reject_non_finite_values);
  RUN_TEST(test_manual_settings_validate_schedule_bounds);
  RUN_TEST(test_calibration_requires_95_percent_stability);
  RUN_TEST(test_pwm_is_clamped_without_wrapping);
  RUN_TEST(test_fan_pwm_enforces_nonzero_minimum);
  RUN_TEST(test_moisture_pwm_is_bounded_and_adaptive);
  RUN_TEST(test_absolute_difference_is_order_independent);
  RUN_TEST(test_chicken_phase_boundaries);
  RUN_TEST(test_lockdown_is_sticky);
  RUN_TEST(test_heater_surface_derating);
  RUN_TEST(test_heater_surface_derating_rejects_invalid_thresholds);
  RUN_TEST(test_heater_pwm_saturates_before_integer_conversion);
  RUN_TEST(test_heater_pwm_fails_closed_on_invalid_arithmetic);
  RUN_TEST(test_adaptive_heater_uses_learned_hold_pwm_at_target);
  RUN_TEST(test_adaptive_heater_ignores_untrained_feed_forward);
  RUN_TEST(test_heater_response_accepts_incubator_temperature_rise);
  RUN_TEST(test_heater_response_accepts_surface_probe_evidence);
  RUN_TEST(test_heater_response_rejects_no_temperature_change);
  RUN_TEST(test_dual_humidity_consensus);
  RUN_TEST(test_humidity_consensus_flags_degraded_mode);
  return UNITY_END();
}
