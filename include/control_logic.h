#pragma once

#include <stdint.h>

namespace control_logic {

bool validManualSettings(float temperature, float humidity, int days,
                         int turnIntervalHours, int stopTurningDay);
bool calibrationPassed(uint32_t samples, uint32_t stableSamples,
                       bool hardwareFaultLatched);
uint8_t clampPwm(int value);
uint8_t minimumRunningPwm(int value, uint8_t minimumPwm);
bool temperatureExhaustRequired(float eggTemperature, float targetTemperature,
                                bool wasVenting, bool overheat,
                                float onDelta, float offDelta);
uint8_t exhaustStartupPwm(uint8_t requested, uint8_t minimumPwm,
                          uint32_t now, uint32_t boostMs,
                          uint8_t &previousRequest, uint32_t &startedAt);
uint8_t moistureExhaustPwm(float humidityExcess, uint8_t minimumPwm,
                           uint8_t adaptiveBoost);
uint32_t absoluteDifference(uint32_t first, uint32_t second);
uint8_t cycleDayForElapsed(uint32_t elapsedSeconds, uint8_t totalDays);
bool cycleComplete(uint32_t elapsedSeconds, uint8_t totalDays);
bool lockdownForDay(uint8_t cycleDay, uint8_t firstLockdownDay,
                    bool alreadyLockedDown);
uint8_t heaterSurfaceDutyLimit(float heaterTemperature, float derateAt,
                               float cutoffAt);
uint8_t adaptiveHeaterPwm(float learnedHoldPwm, bool modelReady,
                          float temperatureError, float integralCorrection,
                          float errorRate);
bool heaterResponseDetected(float incubatorTemperature,
                            float baselineIncubatorTemperature,
                            float heaterSurfaceTemperature,
                            float baselineHeaterSurfaceTemperature,
                            bool heaterSurfaceValid);
bool humidityConsensus(float first, bool firstValid, float second,
                       bool secondValid, float maximumSpread, float &result,
                       bool &degraded);

}  // namespace control_logic
