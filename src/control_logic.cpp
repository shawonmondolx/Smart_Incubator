#include "control_logic.h"

#include <cmath>

namespace control_logic {

bool validManualSettings(float temperature, float humidity, int days,
                         int turnIntervalHours, int stopTurningDay) {
  return std::isfinite(temperature) && std::isfinite(humidity) &&
         temperature >= 35.0F && temperature <= 40.0F && humidity >= 35.0F &&
         humidity <= 80.0F && days >= 1 && days <= 35 &&
         turnIntervalHours >= 1 && turnIntervalHours <= 24 &&
         stopTurningDay >= 1 && stopTurningDay <= days;
}

bool calibrationPassed(uint32_t samples, uint32_t stableSamples,
                       bool hardwareFaultLatched) {
  if (samples == 0 || stableSamples > samples || hardwareFaultLatched) return false;
  return stableSamples / static_cast<float>(samples) >= 0.95F;
}

uint8_t clampPwm(int value) {
  if (value <= 0) return 0;
  if (value >= 255) return 255;
  return static_cast<uint8_t>(value);
}

uint8_t minimumRunningPwm(int value, uint8_t minimumPwm) {
  const uint8_t requested = clampPwm(value);
  if (requested == 0) return 0;
  return requested < minimumPwm ? minimumPwm : requested;
}

uint8_t moistureExhaustPwm(float humidityExcess, uint8_t minimumPwm,
                           uint8_t adaptiveBoost) {
  if (!std::isfinite(humidityExcess) || humidityExcess <= 0.0F) return 0;
  const float proportional = (1.0F < humidityExcess / 12.0F) ? 1.0F : (humidityExcess / 12.0F);
  const int duty = static_cast<int>(minimumPwm +
                                    proportional * (255 - minimumPwm)) +
                   adaptiveBoost;
  return clampPwm(duty);
}

bool temperatureExhaustRequired(float eggTemperature, float targetTemperature,
                                bool wasVenting, bool overheat,
                                float onDelta, float offDelta) {
  if (overheat) return true;
  if (!std::isfinite(eggTemperature) || !std::isfinite(targetTemperature)) {
    return false;
  }
  return eggTemperature >= targetTemperature + onDelta ||
         (wasVenting && eggTemperature > targetTemperature + offDelta);
}

uint8_t exhaustStartupPwm(uint8_t requested, uint8_t minimumPwm,
                          uint32_t now, uint32_t boostMs,
                          uint8_t &previousRequest, uint32_t &startedAt) {
  if (requested == 0) {
    previousRequest = 0;
    startedAt = 0;
    return 0;
  }
  if (previousRequest == 0) startedAt = now;
  previousRequest = requested;
  return now - startedAt < boostMs
             ? 255
             : minimumRunningPwm(requested, minimumPwm);
}

uint32_t absoluteDifference(uint32_t first, uint32_t second) {
  return first > second ? first - second : second - first;
}

uint8_t cycleDayForElapsed(uint32_t elapsedSeconds, uint8_t totalDays) {
  if (totalDays == 0) return 0;
  const uint32_t day = elapsedSeconds / 86400UL + 1UL;
  return static_cast<uint8_t>(day > totalDays ? totalDays : day);
}

bool cycleComplete(uint32_t elapsedSeconds, uint8_t totalDays) {
  return totalDays > 0 && elapsedSeconds >= totalDays * 86400UL;
}

bool lockdownForDay(uint8_t cycleDay, uint8_t firstLockdownDay,
                    bool alreadyLockedDown) {
  return alreadyLockedDown ||
         (firstLockdownDay > 0 && cycleDay >= firstLockdownDay);
}

uint8_t heaterSurfaceDutyLimit(float heaterTemperature, float derateAt,
                               float cutoffAt) {
  if (!std::isfinite(heaterTemperature) || !std::isfinite(derateAt) ||
      !std::isfinite(cutoffAt) || cutoffAt <= derateAt) return 0;
  if (heaterTemperature <= derateAt) return 255;
  if (heaterTemperature >= cutoffAt) return 0;
  return static_cast<uint8_t>(
      255.0F * (cutoffAt - heaterTemperature) / (cutoffAt - derateAt));
}

uint8_t adaptiveHeaterPwm(float learnedHoldPwm, bool modelReady,
                          float temperatureError, float integralCorrection,
                          float errorRate) {
  if (!std::isfinite(temperatureError) ||
      !std::isfinite(integralCorrection) || !std::isfinite(errorRate)) {
    return 0;
  }
  const float feedForward =
      modelReady && std::isfinite(learnedHoldPwm) ? learnedHoldPwm : 0.0F;
  const float KP = 25.0F;
  const float KD = 200.0F;
  const float duty =
      feedForward + KP * temperatureError + integralCorrection +
      KD * errorRate;
  // Fail closed on arithmetic overflow; saturate before integer conversion.
  if (!std::isfinite(duty) || duty <= 0.0F) return 0;
  if (duty >= 255.0F) return 255;
  return static_cast<uint8_t>(static_cast<long>(duty + 0.5F));
}

bool heaterResponseDetected(float incubatorTemperature,
                            float baselineIncubatorTemperature,
                            float heaterSurfaceTemperature,
                            float baselineHeaterSurfaceTemperature,
                            bool heaterSurfaceValid) {
  if (!std::isfinite(incubatorTemperature) ||
      !std::isfinite(baselineIncubatorTemperature)) {
    return false;
  }
  if (incubatorTemperature - baselineIncubatorTemperature >= 0.2F) return true;
  if (!heaterSurfaceValid || !std::isfinite(heaterSurfaceTemperature)) {
    return false;
  }
  const bool surfaceWarmed = std::isfinite(baselineHeaterSurfaceTemperature) &&
                             heaterSurfaceTemperature -
                                     baselineHeaterSurfaceTemperature >=
                                 0.5F;
  const bool surfaceIsHeating =
      heaterSurfaceTemperature - incubatorTemperature >= 1.0F;
  return surfaceWarmed || surfaceIsHeating;
}

bool humidityConsensus(float first, bool firstValid, float second,
                       bool secondValid, float maximumSpread, float &result,
                       bool &degraded) {
  degraded = firstValid != secondValid;
  if (firstValid && secondValid) {
    if (!std::isfinite(first) || !std::isfinite(second) ||
        std::fabs(first - second) > maximumSpread) {
      return false;
    }
    result = (first + second) / 2.0F;
    return true;
  }
  if (degraded) {
    result = firstValid ? first : second;
    return std::isfinite(result);
  }
  return false;
}

}  // namespace control_logic
