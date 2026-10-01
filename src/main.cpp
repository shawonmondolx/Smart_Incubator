#include <Arduino.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <atomic>
#include <mbedtls/md.h>
#include <Preferences.h>
#include <RTClib.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>

#include "secrets.h"
#include "firmware_config.h"
#include "control_logic.h"
#include "firebase_ca.h"

using namespace firmware;

constexpr unsigned long STATUS_INTERVAL_MS = 10000;
constexpr unsigned long PRODUCTION_TELEMETRY_INTERVAL_MS = 2000;
constexpr unsigned long SENSOR_INTERVAL_MS = 2000;
constexpr unsigned long DHT_SENSOR_INTERVAL_MS = 4000;
// 11-bit conversion needs 375 ms. The loop also runs blocking HTTPS/TLS,
// NimBLE and I2C RTC work, so it can be delayed past the nominal interval.
// 600 ms gives real margin so getTempCByIndex() never reads a half-converted
// scratchpad (the source of the intermittent shifted/wrong readings).
constexpr unsigned long DS18B20_CONVERSION_MS = 600;
// Reject a single-sample jump larger than this: physical egg/heater
// temperature cannot change this fast, so a bigger step is a read glitch.
constexpr float TEMP_SPIKE_REJECT_C = 3.0F;
constexpr unsigned long SENSOR_STALE_MS = 10000;
constexpr unsigned long DHT_SENSOR_STALE_MS = 20000;
constexpr uint16_t DHT_RECOVERY_FAILURE_THRESHOLD = 3;
constexpr unsigned long DHT_RECOVERY_COOLDOWN_MS = 10000;
constexpr unsigned long DHT_POWER_OFF_MS = 250;
constexpr unsigned long DHT_POWER_STABILIZE_MS = 2000;
constexpr uint32_t INCUBATION_LOG_INTERVAL_SECONDS = 3600UL;
constexpr uint8_t INCUBATION_EVENT_QUEUE_SIZE = 8;
// Big incubators have slow thermal mass: every actuator-response window is
// sized for a full temperature/RH swing before a no-response flag is raised.
constexpr unsigned long HEATER_RESPONSE_TIMEOUT_MS = 35UL * 60UL * 1000UL;
constexpr unsigned long HUMIDIFIER_RESPONSE_TIMEOUT_MS = 30UL * 60UL * 1000UL;
constexpr unsigned long EXHAUST_RESPONSE_TIMEOUT_MS = 30UL * 60UL * 1000UL;
// Circulation fans driven via simple digitalWrite (full DC).
// PWM was tried but caused speed issues due to PSU voltage sag from the heater.
// Full ON gives maximum voltage to the fans at all times.
// Equipment rest cycle constants kept for telemetry compatibility; rest cycle is DISABLED.
// Humidifier fault detection (Feature)
constexpr unsigned long HUMIDIFIER_FAULT_PROBE_MS = 60UL * 1000UL;  // 1 min with no RH rise
constexpr float HUMIDIFIER_FAULT_MIN_RISE_RH = 1.0F;                // expected RH gain
constexpr float SENSOR_MAX_TEMPERATURE_SPREAD_C = 1.0F;
constexpr float SENSOR_MAX_HUMIDITY_SPREAD = 10.0F;
constexpr float EGG_TEMPERATURE_DISAGREEMENT_C = 2.0F;
constexpr float EGG_TEMPERATURE_RECOVERY_C = 1.0F;
constexpr uint8_t EGG_TEMPERATURE_CONFIRM_SAMPLES = 60;
constexpr uint8_t EGG_TEMPERATURE_RECOVERY_SAMPLES = 15;
constexpr float EGG_TEMPERATURE_STABLE_RATE_C = 0.15F;
constexpr float SENSOR_MIN_INCUBATOR_TEMPERATURE_C = 5.0F;
constexpr float SENSOR_MAX_INCUBATOR_TEMPERATURE_C = 55.0F;
constexpr float SENSOR_MIN_HUMIDITY = 1.0F;
constexpr float SENSOR_MAX_HUMIDITY = 99.9F;
constexpr float HUMIDIFIER_MIN_RESPONSE_RH = 2.0F;
constexpr float EXHAUST_MIN_TEMPERATURE_DROP_C = 0.2F;
constexpr unsigned long TEST_INTERVAL_MS = 2000;
constexpr unsigned long TEST_COMMAND_INTERVAL_MS = 1000;
constexpr unsigned long TEST_COMMAND_LEASE_MS = 5UL * 60UL * 1000UL;
constexpr char FIRMWARE_BUILD_ID[] = "2026-09-20-ds18b20-crc-spikefix-v50";
constexpr unsigned long COMMAND_INTERVAL_MS = 3000;
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
constexpr unsigned long WIFI_PROVISION_TIMEOUT_MS = 30000;
// Wi-Fi transmit power. An earlier build forced this down to the 2 dBm minimum
// because the heater was believed to share the ESP32's 5 V rail, so TLS bursts
// plus heater current were thought to brown out the radio. The heater and fans
// actually run from a dedicated 12 V PSU, so the radio no longer competes with
// them and normal power is restored. Only drop this back to WIFI_POWER_2dBm if
// the ESP32 itself is ever moved onto the heater's 12 V rail.
constexpr wifi_power_t WIFI_TX_POWER = WIFI_POWER_19_5dBm;
constexpr unsigned long FACTORY_RESET_HOLD_MS = 8000;
// Ignore the BOOT button for this long after power-up, so a strapping-pin
// glitch during the boot transient cannot start the 8-second hold timer.
constexpr unsigned long FACTORY_RESET_BOOT_GRACE_MS = 5000;
constexpr unsigned long TOKEN_REFRESH_MARGIN_MS = 5UL * 60UL * 1000UL;
constexpr unsigned long RTC_SYNC_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
constexpr unsigned long RTC_SYNC_RETRY_MS = 60UL * 1000UL;
// A DS1307 on this build sits on a bus that the heater, fans and servos share
// electrically, so transient I2C glitches are expected. Losing RTC presence on
// the first glitch used to kill the clock until a reboot, which latched
// rtcFault, paused the running cycle and disabled tray turning. Presence is now
// only given up after repeated failures, and it is re-probed on a backoff so
// the clock self-heals as soon as the bus is usable again.
constexpr unsigned long RTC_REDETECT_INTERVAL_MS = 30UL * 1000UL;
constexpr uint8_t RTC_RECOVERY_STRIKES_BEFORE_LOSS = 3;
constexpr uint32_t RTC_MAX_DRIFT_SECONDS = 5;
constexpr uint32_t RTC_MIN_VALID_EPOCH = 1704067200UL;  // 2024-01-01 UTC
constexpr uint32_t RTC_MAX_VALID_EPOCH = 4102444799UL;  // 2099-12-31 UTC

// ── Servo calibration persistence ───────────────────────────────────────────
// The vent and circulation servo open/closed angles are user calibration, not
// runtime state, so they have to survive a brownout reset, a watchdog reboot
// and an interrupted flash write. A single NVS key is not enough: this device
// browns out while the heater switches (see the ESP_RST_BROWNOUT note in
// setup()), and a reset that lands in the middle of an NVS page write loses
// the entry, after which the firmware silently falls back to the compile-time
// defaults. Every save therefore writes TWO independent copies — the primary
// ("servo" / "circServo") and a checksummed shadow ("servoCfg") — and boot
// repairs whichever copy is missing or corrupt.
constexpr char SERVO_CFG_SHADOW_NS[] = "servoCfg";
constexpr uint16_t SERVO_CFG_MAGIC = 0x5343;  // 'S','C'
constexpr uint8_t SERVO_CFG_VERSION = 1;
// After a reset the companion app reconnects and can re-send its own factory
// defaults before it has read our telemetry. A default payload arriving inside
// this window is treated as that stale echo and is not allowed to overwrite a
// customised calibration. The app can force the write at any time by sending
// servoConfigSave / circServoConfigSave = true.
constexpr unsigned long SERVO_DEFAULT_ECHO_GUARD_MS = 120UL * 1000UL;
constexpr char BLE_SERVICE_UUID[] = "7f510001-6a7e-4bc4-9f2d-21a4d4671000";
constexpr char BLE_COMMAND_UUID[] = "7f510002-6a7e-4bc4-9f2d-21a4d4671000";
constexpr char BLE_STATUS_UUID[] = "7f510003-6a7e-4bc4-9f2d-21a4d4671000";
constexpr char BLE_MONITOR_UUID[] = "7f510004-6a7e-4bc4-9f2d-21a4d4671000";
constexpr char BLE_AUTH_UUID[] = "7f510005-6a7e-4bc4-9f2d-21a4d4671000";

constexpr float MIN_TARGET_TEMPERATURE = 35.0F;
constexpr float MAX_TARGET_TEMPERATURE = 40.0F;
constexpr float MIN_TARGET_HUMIDITY = 35.0F;
constexpr float MAX_TARGET_HUMIDITY = 80.0F;
constexpr float HUMIDITY_HYSTERESIS = 2.0F;
constexpr float EXHAUST_TEMPERATURE_DELTA = 0.4F;
constexpr float EXHAUST_TEMPERATURE_OFF_DELTA = 0.2F;
constexpr float MOISTURE_REDUCTION_ON_DELTA = 3.0F;
constexpr float MOISTURE_REDUCTION_OFF_DELTA = 1.0F;
constexpr unsigned long MOISTURE_ADAPT_INTERVAL_MS = 5UL * 60UL * 1000UL;
constexpr float MOISTURE_REQUIRED_DROP_RH = 0.5F;
constexpr uint8_t MOISTURE_ADAPT_STEP_PWM = 16;
constexpr unsigned long EXHAUST_STARTUP_BOOST_MS = 1000;
constexpr unsigned long USB_CONTROL_LOG_INTERVAL_MS = 10000;
constexpr unsigned long CALIBRATION_CLOUD_LOG_INTERVAL_MS = 60000;
constexpr float OVERHEAT_DELTA = 1.0F;
constexpr float ABSOLUTE_OVERHEAT_C = 39.5F;
constexpr float HEATER_SURFACE_MAX_C = 60.0F;
constexpr float HEATER_SURFACE_DERATE_C = 50.0F;
constexpr float HEATER_FULL_POWER_MARGIN_C = 0.3F;
constexpr float HEATER_APPROACH_ERROR_C = 1.2F;
constexpr float HEATER_APPROACH_PREDICT_SECONDS = 75.0F;
constexpr float HEATER_INTEGRAL_RISE_FREEZE_RATE = 0.005F;
constexpr uint32_t TURN_PULSE_SECONDS = 10;
constexpr unsigned long LID_WARNING_MS = 30UL * 1000UL;
constexpr unsigned long LID_FAULT_MS = 5UL * 60UL * 1000UL;
constexpr uint32_t CYCLE_DAY_SECONDS = 86400UL;
// Heater + circulation/exhaust fans are powered from their OWN dedicated 12 V
// PSU, electrically separate from the ESP32's 5 V/3.3 V supply. An earlier
// build capped heater duty at 200 (78 %) on the assumption that the heater and
// the Wi-Fi radio shared one 5 V rail, so that TLS bursts plus full heater duty
// would brown out the ESP. That assumption does not hold for this hardware:
// the cap only starved the element, leaving the app reporting 100 % while the
// heater actually received 78 %. Full 255 duty is allowed again. Any reset
// while the heater switches is a wiring/ground-reference problem on the 12 V
// path, NOT PSU capacity — see the boot brownout checklist in setup().
constexpr uint8_t HEATER_PWM_NETWORK_CAP = 255;
// Servo travel time used to gate the exhaust fan so the vent is cranked open
// BEFORE the fan draws its inrush current (shared 5V rail; both at once has
// brownout-reset the ESP mid-cycle).
constexpr unsigned long VENT_SERVO_TRAVEL_MS = 900;
// A cycle survives an RTC outage (deferred clock duties) but must not run
// blind forever: after this long with no clock AND no NTP, abort safely.
constexpr unsigned long CLOCK_LOSS_ABORT_MS = 6UL * 3600UL * 1000UL;

WiFiClientSecure tlsClient;
Preferences preferences;
NimBLECharacteristic *bleStatus = nullptr;
DHT dhtPrimary(DHT_PIN, DHT22);
DHT dhtEggZone(EGG_DHT_PIN, DHT22);
RTC_DS1307 rtc;
OneWire eggOneWire(EGG_DS18B20_PIN);
OneWire heaterOneWire(HEATER_DS18B20_PIN);
DallasTemperature eggTemperatureSensor(&eggOneWire);
DallasTemperature heaterTemperatureSensor(&heaterOneWire);
Servo ventServo;
Servo circServo;

float targetTemperature = 37.5F;
float targetHumidity = 60.0F;
uint64_t commandVersion = 0;
unsigned long lastTelemetryWrite = 0;
unsigned long lastProductionTelemetryWrite = 0;
unsigned long lastSensorRead = 0;
unsigned long lastDhtRead = 0;
unsigned long ds18b20ConversionStartedAt = 0;
bool ds18b20ConversionPending = false;
unsigned long lastValidSensorRead = 0;
unsigned long lastCommandRead = 0;
unsigned long lastClaimRead = 0;
unsigned long lastOwnerCheck = 0;
unsigned long lastWifiRetry = 0;
unsigned long lastTestRead = 0;
unsigned long lastTestWrite = 0;
unsigned long tokenExpiresAt = 0;
String idToken;
String wifiSsid;
String wifiPassword;
String ownerUid;
bool provisioningActive = false;
bool wifiProvisioningPending = false;
bool wifiScanPending = false;
bool factoryResetArmed = false;
bool factoryResetInProgress = false;
bool factoryResetCloudPending = false;
unsigned long factoryResetPressedAt = 0;
unsigned long lastFactoryResetCloudAttempt = 0;
bool rtcAvailable = false;
bool rtcRunning = false;
constexpr bool AUTOMATIC_MOISTURE_REDUCER_ENABLED =
    false;  // Humidity-driven exhaust is turned off in code; the exhaust fan
            // operates only on egg-temperature demand.
bool testEnabled = false;
bool testFan1 = false;
bool testFan3 = false;
bool testHeater = false;
uint8_t testHeaterPwm = 0;
uint8_t testExhaustPwm = 0;
uint8_t appliedTestHeaterPwm = 0;
String heaterTestBlocker;
String testSessionId;
bool testHumidifier = false;
bool atmosphereTestEnabled = false;
float atmosphereTargetTemperature = 37.5F;
float atmosphereTargetHumidity = 60.0F;
float atmosphereHeaterIntegral = 0.0F;
bool atmosphereTemperatureVenting = false;
// Exhaust vent servo (GPIO 13)
bool testServoEnabled = false;
uint8_t ventClosedAngle = VENT_CLOSED_ANGLE;
uint8_t ventOpenAngle = VENT_OPEN_ANGLE;
uint8_t testServoAngle = VENT_CLOSED_ANGLE;
uint8_t appliedTestServoAngle = VENT_CLOSED_ANGLE;
uint8_t physicalServoAngle = VENT_CLOSED_ANGLE;
unsigned long ventServoBusyUntil = 0;  // vent mid-travel; gate exhaust start
unsigned long clockSuspendStartedAt = 0;  // cycle active but rtcUnixNow()==0
uint8_t cloudOwnerNullStreak = 0;  // consecutive confirmed-null ownership reads
// Circulation servo (GPIO 26) — egg-turn switch
bool testCircServoEnabled = false;
uint8_t circServoClosedAngle = CIRC_SERVO_CLOSED_ANGLE;
uint8_t circServoOpenAngle = CIRC_SERVO_OPEN_ANGLE;
uint8_t testCircServoAngle = CIRC_SERVO_CLOSED_ANGLE;
uint8_t appliedTestCircServoAngle = CIRC_SERVO_CLOSED_ANGLE;
uint8_t physicalCircServoAngle = CIRC_SERVO_CLOSED_ANGLE;
bool appliedTestFan1 = false;
uint8_t appliedTestExhaustPwm = 0;
bool appliedTestHumidifier = false;
uint8_t appliedExhaustPwm = 0;
uint8_t lastRequestedExhaustPwm = 0;
uint32_t exhaustStartupStartedAt = 0;
uint64_t lastTestCommandVersion = 0;
unsigned long lastTestCommandReceived = 0;
bool preferTestTelemetry = false;
bool testTelemetryCleanupPending = false;  // deletes stale testTelemetry after first online publish per boot
std::atomic<bool> bleClientConnected{false};
std::atomic<bool> bleAuthorized{false};
std::atomic<bool> bleMonitorSubscribed{false};
std::atomic<uint32_t> bleSession{0};
std::atomic<uint16_t> bleHandle{BLE_HS_CONN_HANDLE_NONE};
NimBLECharacteristic *bleMonitor = nullptr;
NimBLECharacteristic *bleAuth = nullptr;
uint8_t bleChallenge[16];
uint8_t bleProof[32];
size_t bleProofLength = 0;
unsigned long bleChallengeAt = 0;
unsigned long bleLastAuthAttempt = 0;
bool bleAuthAttempted = false;
struct BleRequest { uint32_t session; char json[512]; };
QueueHandle_t bleRequests = nullptr;
unsigned long lastBleSnapshotAt = 0;
constexpr unsigned long BLE_SNAPSHOT_INTERVAL_MS = 2000;
bool deviceUidConfirmed = false;  // true after first successful publishTelemetry (deviceUid in DB)
unsigned long wifiProvisioningStarted = 0;
unsigned long lastRtcSyncAttempt = 0;
bool rtcTimeTrusted = false;
bool rtcHardwareDetected = false;
uint32_t lastRtcEpoch = 0;
unsigned long lastRtcMonotonicAt = 0;
// Runtime RTC recovery state — see RTC_RECOVERY_STRIKES_BEFORE_LOSS.
uint8_t rtcRecoveryStrikes = 0;
unsigned long lastRtcRedetectAt = 0;
bool softwareClockSynced = false;
uint32_t softwareClockEpoch = 0;
unsigned long softwareClockMillisAnchor = 0;
// Servo calibration persistence diagnostics — see SERVO_CFG_SHADOW_NS.
bool servoConfigRestoredFromShadow = false;
uint32_t servoConfigSavedAt = 0;
// Where the angles in force came from: 0 = compile-time defaults (nothing
// usable in NVS), 1 = the primary NVS copy, 2 = repaired from the shadow.
uint8_t servoConfigSource = 0;
// GPIO 0 must read HIGH once at boot before a factory reset can ever be armed.
// A strapping pin that is stuck low would otherwise let noise masquerade as an
// 8-second BOOT hold and wipe every persisted namespace on the next release.
bool factoryResetPinHealthy = false;
float lastRawDhtTemperature = NAN;
float lastRawDhtHumidity = NAN;
float lastRawEggDhtTemperature = NAN;
float lastRawEggDhtHumidity = NAN;
float lastRawEggTemperature = NAN;
float lastRawHeaterTemperature = NAN;
// Previous accepted (post-filter) readings, used to reject single-sample spikes.
float lastGoodEggTemperature = NAN;
float lastGoodHeaterTemperature = NAN;

bool cycleActive = false;
bool cycleResumeRequired = false;
String cycleMode = "automatic";
String profileId;
uint32_t cycleStartedAt = 0;
uint32_t lastTurnAt = 0;
uint32_t pendingTurnStartedAt = 0;
uint32_t lastCycleEpoch = 0;
uint8_t cycleDay = 0;
uint8_t startDayOffset = 0;
uint8_t totalDays = 0;
uint8_t lockdownDay = 0;
uint8_t turnIntervalHours = 2;
float normalHumidity = 60.0F;
float hatchHumidity = 70.0F;
float measuredTemperature = NAN;
float measuredHumidity = NAN;
float primaryTemperature = NAN;
float primaryHumidity = NAN;
float eggDhtTemperature = NAN;
float eggDhtHumidity = NAN;
float eggTemperature = NAN;
float heaterSurfaceTemperature = NAN;
bool eggTemperatureValid = false;
bool heaterTemperatureValid = false;
float calibrationTemperatureOffset = 0.0F;
float calibrationHumidityOffset = 0.0F;
float previousCalibrationTemperatureOffset = 0.0F;
float previousCalibrationHumidityOffset = 0.0F;
bool previousCalibrationWasCalibrated = false;
bool calibrated = false;
bool calibrationActive = false;
uint8_t calibrationDurationHours = 24;
uint32_t calibrationStartedAt = 0;
uint32_t calibrationEndsAt = 0;
uint32_t calibrationSamples = 0;
uint32_t calibrationStableSamples = 0;
unsigned long calibrationLastSampleAt = 0;
String calibrationStatus = "not_calibrated";
bool primarySensorValid = false;
bool primaryTemperatureValid = false;
bool primaryHumidityValid = false;
bool primaryDhtUsingCachedData = false;
bool eggDhtTemperatureValid = false;
bool eggDhtHumidityValid = false;
bool eggDhtUsingCachedData = false;
bool humiditySensorsAgree = false;
bool humidityDegraded = false;
bool eggTemperatureDisagreement = false;
uint8_t eggTemperatureDisagreementSamples = 0;
uint8_t eggTemperatureRecoverySamples = 0;
float eggTemperatureSpread = NAN;
float previousEggTemperature = NAN;

struct DhtRecoveryState {
  uint16_t consecutiveFailures = 0;
  uint16_t recoveryAttempts = 0;
  unsigned long lastRecoveryAt = 0;
  int readStatus = -1;
  int dataLevel = HIGH;
  String diagnostic = "starting";
  bool outageActive = false;
};

DhtRecoveryState primaryDhtRecovery;
DhtRecoveryState eggDhtRecovery;

bool recoverI2cBus();
bool probeI2cAddress(uint8_t address);
void syncSoftwareClock(uint32_t epoch);
unsigned long primarySensorUpdatedAt = 0;
unsigned long eggSensorUpdatedAt = 0;
unsigned long heaterSensorUpdatedAt = 0;
unsigned long humiditySensorUpdatedAt = 0;
unsigned long primaryDhtTemperatureUpdatedAt = 0;
unsigned long primaryHumidityUpdatedAt = 0;
unsigned long eggDhtTemperatureUpdatedAt = 0;
unsigned long eggDhtHumidityUpdatedAt = 0;
bool sensorValid = false;
bool sensorFault = false;
String sensorFaultCode;
bool rtcFault = false;
bool controlPaused = false;
String pauseReason;
bool lockdown = false;
bool lidOpen = false;
bool lidWarning = false;
unsigned long lidOpenedAt = 0;
unsigned long lidClosedAt = 0;
unsigned long lidOpenDurationMs = 0;
bool trayChangeRequired = false;
bool trayChangeEventPending = false;
uint8_t lastTrayChangeEventDay = 0;
bool overheat = false;
bool circulationFan1 = false;
bool exhaustFan = false;
uint8_t productionExhaustPwm = 0;
bool humidifierOn = false;
bool trayTurning = false;
unsigned long trayTurnStartedAt = 0;
uint8_t heaterPwm = 0;
uint8_t ventAngle = VENT_CLOSED_ANGLE;
float heaterIntegral = 0.0F;
float lastHeaterTemperature = NAN;  // Derivative on measurement avoids setpoint kick.
float lastHeaterErrorRate = 0.0F;
unsigned long lastHeaterControlSampleAt = 0;
unsigned long lastControllerUpdate = 0;

// ── Self-improving warm-up model ─────────────────────────────────────────────
// After each successful warm-up, capture the PID integral at lock-on and the
// elapsed seconds. NVS persists these across power cycles so subsequent starts
// pre-load the integral to a learned value instead of a hard-coded constant.
constexpr float HEATER_DEFAULT_INTEGRAL_PRELOAD = 60.0F;
constexpr float HEATER_INTEGRAL_PRELOAD_MIN = 30.0F;
constexpr float HEATER_INTEGRAL_PRELOAD_MAX = 100.0F;
constexpr float HEATER_WARMUP_TARGET_MARGIN_C = 0.2F;  // °C below target = "locked on"
constexpr float HEATER_HOLD_LEARN_BAND_C = 0.15F;
constexpr unsigned long HEATER_HOLD_LEARN_MS = 2UL * 60UL * 1000UL;
constexpr unsigned long HEATER_HOLD_SAMPLE_MS = 2000UL;
constexpr uint16_t HEATER_HOLD_MIN_SAMPLES = 30;
constexpr float HEATER_HOLD_PWM_MIN = 20.0F;
constexpr float HEATER_HOLD_PWM_MAX = 220.0F;
float learnedIntegralPreload = HEATER_DEFAULT_INTEGRAL_PRELOAD;
float learnedHoldPwm = 0.0F;
uint32_t learnedWarmupSeconds = 0;
uint16_t heaterModelSamples = 0;
bool warmupLockRecorded = false;  // true once we have captured the lock-on integral
unsigned long warmupStartedAt = 0;  // millis() when cycle started
float warmupStartTemperature = NAN;
bool holdPwmRecorded = false;
unsigned long holdStableStartedAt = 0;
unsigned long lastHoldSampleAt = 0;
uint32_t holdPwmAccumulator = 0;
uint16_t holdPwmSamples = 0;

bool heaterModelReady() {
  return heaterModelSamples > 0 && isfinite(learnedHoldPwm) &&
         learnedHoldPwm >= HEATER_HOLD_PWM_MIN &&
         learnedHoldPwm <= HEATER_HOLD_PWM_MAX;
}

void saveHeaterModel() {
  if (!preferences.begin("heaterModel", false)) return;
  preferences.putFloat("integralPreload", learnedIntegralPreload);
  preferences.putFloat("holdPwm", learnedHoldPwm);
  preferences.putUInt("warmupSeconds", learnedWarmupSeconds);
  preferences.putUShort("samples", heaterModelSamples);
  preferences.end();
  Serial.printf(
      "Heater model saved: holdPwm=%.1f warmup=%lus samples=%u\n",
      learnedHoldPwm, static_cast<unsigned long>(learnedWarmupSeconds),
      heaterModelSamples);
}

void loadHeaterModel() {
  if (!preferences.begin("heaterModel", true)) return;
  const float stored = preferences.getFloat(
      "integralPreload", HEATER_DEFAULT_INTEGRAL_PRELOAD);
  const float storedHoldPwm = preferences.getFloat("holdPwm", 0.0F);
  const uint32_t storedWarmupSeconds = preferences.getUInt("warmupSeconds", 0);
  const uint16_t storedSamples = preferences.getUShort("samples", 0);
  preferences.end();
  if (isfinite(stored) && stored >= HEATER_INTEGRAL_PRELOAD_MIN &&
      stored <= HEATER_INTEGRAL_PRELOAD_MAX) {
    learnedIntegralPreload = stored;
  }
  if (storedWarmupSeconds <= 24UL * 60UL * 60UL) {
    learnedWarmupSeconds = storedWarmupSeconds;
  }
  if (isfinite(storedHoldPwm) && storedHoldPwm >= HEATER_HOLD_PWM_MIN &&
      storedHoldPwm <= HEATER_HOLD_PWM_MAX && storedSamples > 0) {
    learnedHoldPwm = storedHoldPwm;
    heaterModelSamples = storedSamples;
  }
  Serial.printf(
      "Heater model loaded: ready=%s holdPwm=%.1f warmup=%lus samples=%u\n",
      heaterModelReady() ? "yes" : "no", learnedHoldPwm,
      static_cast<unsigned long>(learnedWarmupSeconds), heaterModelSamples);
}
// Equipment rest cycle state (Bug 2)
unsigned long continuousRunStartedAt = 0;  // millis() of last rest end / cycle start
bool restCycleActive = false;              // true during the 10-sec equipment rest
unsigned long restStartedAt = 0;           // millis() when rest began
// Humidifier fault state (Feature)
unsigned long humidifierOnAt = 0;       // millis() when humidifier last turned ON
bool humidifierFaultSuspected = false;  // true after 1 min ON with no RH rise
// Power-restore event flag (Bug 1)
bool powerRestoreEventPending = false;
// Room (outside) sensor — primary DHT22 physically moved outside the incubator.
float roomTemperature = NAN;  // Ambient room temperature (informational).
float roomHumidity = NAN;     // Ambient room humidity (used for exhaust gating).
bool roomHumidityValid = false;
bool exhaustHelpsHumidity = true;  // True when room air is drier than incubator.
bool reportedStatePending = false;
unsigned long lastReportedAttempt = 0;
String commandState = "accepted";
String commandError;
bool moistureReducerEnabled = false;
bool adaptiveMoistureControlEnabled = true;
uint8_t exhaustMinimumPercent = 50;
bool moistureReducerActive = false;
uint8_t moistureAdaptiveBoostPwm = 0;
float moistureBaselineHumidity = NAN;
float humidityTrendRhPerMinute = 0.0F;
unsigned long moistureEvaluationStartedAt = 0;
unsigned long lastUsbControlLogAt = 0;
unsigned long lastCalibrationCloudLogAt = 0;
bool calibrationLogPending = false;
String calibrationLogEvent = "sample";
uint8_t cloudTaskCursor = 0;
String incubationBatchId;
String incubationBatchStatus;
uint32_t incubationBatchEndedAt = 0;
bool incubationBatchRecordPending = false;
uint32_t lastIncubationHourlyLogAt = 0;
bool loggedHumidifierState = false;

struct IncubationEvent {
  String type;
  uint32_t capturedAt = 0;
  uint16_t sequence = 0;
  uint8_t day = 0;
  float temperature = NAN;
  float humidity = NAN;
  bool humidifier = false;
};

IncubationEvent incubationEvents[INCUBATION_EVENT_QUEUE_SIZE];
uint8_t incubationEventHead = 0;
uint8_t incubationEventCount = 0;
uint16_t incubationEventSequence = 0;

void queueIncubationEvent(const char *type, uint32_t capturedAt = 0);
void queueIncubationBatchRecord(const char *status, uint32_t endedAt = 0);
void trackHumidifierTransition();
void queueHourlyIncubationLog();
void publishIncubationHistory();

struct ResponseMonitor {
  bool active = false;
  unsigned long startedAt = 0;
  float baselineTemperature = NAN;
  float baselineHeaterSurfaceTemperature = NAN;
  float baselineHumidity = NAN;
};

ResponseMonitor heaterResponse;
ResponseMonitor humidifierResponse;
ResponseMonitor exhaustResponse;
bool heaterResponseVerified = false;
bool heaterResponseSlow = false;
bool humidifierResponseVerified = false;
bool exhaustResponseVerified = false;
bool exhaustResponseFailed = false;
uint8_t manualCheckTarget = 0;  // 0 none, 1 humidifier, 2 exhaust
bool manualCheckOn = false;
unsigned long manualCheckDeadline = 0;
constexpr unsigned long MANUAL_CHECK_LEASE_MS = 30UL * 1000UL;
bool hardwareFaultLatched = false;
String hardwareFaultCode;
uint32_t hardwareFaultAt = 0;

constexpr uint8_t HEATER_PWM_SLEW_STEP = 4;  // halved: smoother ramp, lower PSU inrush
constexpr unsigned long HEATER_PWM_SLEW_INTERVAL_MS = 20;
uint8_t appliedHeaterDuty = 0;
unsigned long lastHeaterRampAt = 0;

// Soft-start the heater: ramp the duty up in small steps so the 12 V PSU sees
// a gentle turn-on current step and the switching transient it couples into
// the ESP32 stays small. Decreases are instant so every safety cutoff
// (overheat, lid open, surface derate) stays immediate.
void writeHeaterPwm(uint8_t duty) {
  // Single clamp point. The heater runs from its own 12 V PSU, so Wi-Fi
  // activity must NOT throttle it — see HEATER_PWM_NETWORK_CAP.
  if (duty > HEATER_PWM_NETWORK_CAP) {
    duty = HEATER_PWM_NETWORK_CAP;
  }
  if (duty > appliedHeaterDuty) {
    const unsigned long now = millis();
    if (lastHeaterRampAt == 0 ||
        now - lastHeaterRampAt >= HEATER_PWM_SLEW_INTERVAL_MS) {
      appliedHeaterDuty =
          appliedHeaterDuty + HEATER_PWM_SLEW_STEP > duty
              ? duty
              : static_cast<uint8_t>(appliedHeaterDuty + HEATER_PWM_SLEW_STEP);
      lastHeaterRampAt = now;
    }
  } else {
    appliedHeaterDuty = duty;
  }
  // XY-MOS boards ship in both trigger polarities. For a low-level-trigger
  // module the pin level is inverted so a 255 duty still means full heat.
  ledcWrite(HEATER_PWM_CHANNEL,
            HEATER_OUTPUT_ACTIVE_LOW
                ? static_cast<uint8_t>(255U - appliedHeaterDuty)
                : appliedHeaterDuty);
}

// Diagnostic: the app shows the duty it REQUESTED, so a stalled heater output
// looks identical to a healthy one on screen. This prints to serial whenever a
// manual heat request is outstanding but nothing is reaching GPIO 32, and names
// the cause. Skipped for the atmosphere PI loop, which is allowed to sit at 0
// once the target is met.
constexpr unsigned long HEATER_STALL_WARN_INTERVAL_MS = 30UL * 1000UL;
unsigned long lastHeaterStallWarnAt = 0;
uint8_t lastHeaterStallWarnRequest = 0;

void reportHeaterOutputMismatch() {
  if (atmosphereTestEnabled) return;
  const uint8_t requested = testEnabled ? testHeaterPwm : heaterPwm;
  if (requested == 0 || appliedHeaterDuty > 0) return;
  const unsigned long now = millis();
  if (lastHeaterStallWarnAt != 0 &&
      requested == lastHeaterStallWarnRequest &&
      now - lastHeaterStallWarnAt < HEATER_STALL_WARN_INTERVAL_MS) {
    return;
  }
  lastHeaterStallWarnAt = now;
  lastHeaterStallWarnRequest = requested;
  Serial.printf(
      "HEATER OUTPUT STALLED: requested %u/255, applied 0 on GPIO %u. "
      "Software blocker: %s\n",
      static_cast<unsigned>(requested), HEATER_PIN,
      heaterTestBlocker.length() > 0
          ? heaterTestBlocker.c_str()
          : "none - suspect 12V PSU, MOSFET gate drive or common ground");
}

// Drive the circulation fans via simple digitalWrite — full DC, no PWM.
void writeCirculationFan1(bool on) {
  digitalWrite(FAN_1_PIN, on ? OUTPUT_ON : OUTPUT_OFF);
}

uint8_t minimumExhaustPwm() {
  return static_cast<uint8_t>((exhaustMinimumPercent * 255U + 50U) / 100U);
}

uint8_t writeExhaustPwm(uint8_t duty) {
  // Kick once on an off-to-on transition, then respect the requested duty
  // with the configured stall-prevention minimum. Zero always stops at once.
  appliedExhaustPwm = control_logic::exhaustStartupPwm(
      duty, minimumExhaustPwm(), millis(), EXHAUST_STARTUP_BOOST_MS,
      lastRequestedExhaustPwm, exhaustStartupStartedAt);
  ledcWrite(EXHAUST_PWM_CHANNEL, appliedExhaustPwm);
  return appliedExhaustPwm;
}

bool validEpoch(uint32_t epoch) {
  return epoch >= RTC_MIN_VALID_EPOCH && epoch <= RTC_MAX_VALID_EPOCH;
}

void synchronizeRtcFromInternet() {
  if (WiFi.status() != WL_CONNECTED) return;
  const unsigned long interval = rtcTimeTrusted ? RTC_SYNC_INTERVAL_MS
                                                : RTC_SYNC_RETRY_MS;
  if (lastRtcSyncAttempt != 0 && millis() - lastRtcSyncAttempt < interval) return;
  lastRtcSyncAttempt = millis();
  IPAddress ntpAddress;
  if (WiFi.hostByName("pool.ntp.org", ntpAddress) != 1) {
    Serial.println("NTP DNS lookup failed.");
    return;
  }
  WiFiUDP udp;
  if (!udp.begin(0)) return;
  uint8_t packet[48] = {};
  packet[0] = 0b11100011;
  packet[1] = 0;
  packet[2] = 6;
  packet[3] = 0xEC;
  udp.beginPacket(ntpAddress, 123);
  udp.write(packet, sizeof(packet));
  udp.endPacket();
  const unsigned long startedAt = millis();
  while (udp.parsePacket() < 48 && millis() - startedAt < 1500UL) delay(10);
  if (udp.available() < 48) {
    udp.stop();
    Serial.println("NTP server did not respond.");
    return;
  }
  udp.read(packet, sizeof(packet));
  udp.stop();
  const uint32_t secondsSince1900 =
      (static_cast<uint32_t>(packet[40]) << 24) |
      (static_cast<uint32_t>(packet[41]) << 16) |
      (static_cast<uint32_t>(packet[42]) << 8) | packet[43];
  constexpr uint32_t NTP_UNIX_OFFSET = 2208988800UL;
  if (secondsSince1900 <= NTP_UNIX_OFFSET) return;
  const uint32_t networkEpoch = secondsSince1900 - NTP_UNIX_OFFSET;
  if (!validEpoch(networkEpoch)) return;
  const uint32_t rtcEpoch = rtcHardwareDetected && rtcAvailable
                                ? rtc.now().unixtime()
                                : 0;
  const uint32_t drift = control_logic::absoluteDifference(rtcEpoch, networkEpoch);
  if (rtcHardwareDetected &&
      (!rtcRunning || !validEpoch(rtcEpoch) || drift > RTC_MAX_DRIFT_SECONDS)) {
    rtc.adjust(DateTime(networkEpoch));
    rtcAvailable = true;
    rtcRunning = true;
    Serial.printf("RTC synchronized from internet; corrected %lu seconds.\n",
                  static_cast<unsigned long>(drift));
  }
  rtcTimeTrusted = true;
  lastRtcEpoch = networkEpoch;
  lastRtcMonotonicAt = millis();
  syncSoftwareClock(networkEpoch);
}

void saveCalibrationState() {
  preferences.begin("calibration", false);
  preferences.putBool("calibrated", calibrated);
  preferences.putBool("active", calibrationActive);
  preferences.putFloat("tempOffset", calibrationTemperatureOffset);
  preferences.putFloat("humidityOffset", calibrationHumidityOffset);
  preferences.putFloat("previousTemp", previousCalibrationTemperatureOffset);
  preferences.putFloat("previousRh", previousCalibrationHumidityOffset);
  preferences.putBool("previousOk", previousCalibrationWasCalibrated);
  preferences.putUChar("duration", calibrationDurationHours);
  preferences.putUInt("startedAt", calibrationStartedAt);
  preferences.putUInt("endsAt", calibrationEndsAt);
  preferences.putUInt("samples", calibrationSamples);
  preferences.putUInt("stableSamples", calibrationStableSamples);
  preferences.putString("status", calibrationStatus);
  preferences.end();
}

void loadCalibrationState() {
  if (!preferences.begin("calibration", true)) return;
  calibrated = preferences.getBool("calibrated", false);
  calibrationActive = preferences.getBool("active", false);
  calibrationTemperatureOffset = preferences.getFloat("tempOffset", 0.0F);
  calibrationHumidityOffset = preferences.getFloat("humidityOffset", 0.0F);
  previousCalibrationTemperatureOffset =
      preferences.getFloat("previousTemp", calibrationTemperatureOffset);
  previousCalibrationHumidityOffset =
      preferences.getFloat("previousRh", calibrationHumidityOffset);
  previousCalibrationWasCalibrated =
      preferences.getBool("previousOk", calibrated);
  calibrationDurationHours = preferences.getUChar("duration", 24);
  calibrationStartedAt = preferences.getUInt("startedAt", 0);
  calibrationEndsAt = preferences.getUInt("endsAt", 0);
  calibrationSamples = preferences.getUInt("samples", 0);
  calibrationStableSamples = preferences.getUInt("stableSamples", 0);
  calibrationStatus = preferences.getString(
      "status", calibrated ? "calibrated" : "not_calibrated");
  preferences.end();
  const bool offsetsValid = isfinite(calibrationTemperatureOffset) &&
                            isfinite(calibrationHumidityOffset) &&
                            fabsf(calibrationTemperatureOffset) <= 5.0F &&
                            fabsf(calibrationHumidityOffset) <= 20.0F;
  const bool activeStateValid =
      !calibrationActive ||
      ((calibrationDurationHours == 24 || calibrationDurationHours == 48) &&
       validEpoch(calibrationStartedAt) && validEpoch(calibrationEndsAt) &&
       calibrationEndsAt > calibrationStartedAt);
  const bool countersValid = calibrationSamples <= 200000 &&
                             calibrationStableSamples <= 200000 &&
                             calibrationStableSamples <= calibrationSamples;
  if (!offsetsValid || !activeStateValid || !countersValid) {
    calibrated = false;
    calibrationActive = false;
    calibrationTemperatureOffset = 0.0F;
    calibrationHumidityOffset = 0.0F;
    calibrationSamples = 0;
    calibrationStableSamples = 0;
    calibrationStartedAt = 0;
    calibrationEndsAt = 0;
    calibrationStatus = "invalid_persisted_state";
    saveCalibrationState();
    Serial.println("Boot: cleared corrupted NVS calibration state.");
  }
}

void saveCycleState() {
  preferences.begin("cycle", false);
  preferences.putBool("active", cycleActive);
  preferences.putString("mode", cycleMode);
  preferences.putString("profile", profileId);
  preferences.putFloat("temperature", targetTemperature);
  preferences.putFloat("normalRh", normalHumidity);
  preferences.putFloat("hatchRh", hatchHumidity);
  preferences.putUChar("days", totalDays);
  preferences.putUChar("lockdown", lockdownDay);
  preferences.putUChar("turnHours", turnIntervalHours);
  preferences.putUInt("startedAt", cycleStartedAt);
  preferences.putUInt("lastTurnAt", lastTurnAt);
  preferences.putUInt("lastCycleEpoch", lastCycleEpoch);
  preferences.putUChar("dayOffset", startDayOffset);
  preferences.putString("batchId", incubationBatchId);
  preferences.putString("batchStatus", incubationBatchStatus);
  preferences.putUInt("batchEnded", incubationBatchEndedAt);
  preferences.putBool("batchPending", incubationBatchRecordPending);
  preferences.putUInt("hourlyAt", lastIncubationHourlyLogAt);
  preferences.putUChar("trayAlertDay", lastTrayChangeEventDay);
  preferences.putBool("trayPending", trayChangeEventPending);
  preferences.putULong64("version", commandVersion);
  preferences.putBool("hardwareFault", hardwareFaultLatched);
  preferences.putString("faultCode", hardwareFaultCode);
  preferences.putUInt("faultAt", hardwareFaultAt);
  preferences.end();
}

void loadCycleState() {
  if (!preferences.begin("cycle", true)) return;
  cycleActive = preferences.getBool("active", false);
  cycleResumeRequired = cycleActive;
  cycleMode = preferences.getString("mode", "automatic");
  profileId = preferences.getString("profile", "");
  targetTemperature = preferences.getFloat("temperature", 37.5F);
  normalHumidity = preferences.getFloat("normalRh", 60.0F);
  hatchHumidity = preferences.getFloat("hatchRh", normalHumidity);
  totalDays = preferences.getUChar("days", 0);
  lockdownDay = preferences.getUChar("lockdown", 0);
  turnIntervalHours = preferences.getUChar("turnHours", 2);
  cycleStartedAt = preferences.getUInt("startedAt", 0);
  lastTurnAt = preferences.getUInt("lastTurnAt", cycleStartedAt);
  lastCycleEpoch = preferences.getUInt("lastCycleEpoch", cycleStartedAt);
  startDayOffset = preferences.getUChar("dayOffset", 0);
  incubationBatchId = preferences.getString("batchId", "");
  incubationBatchStatus = preferences.getString("batchStatus", "");
  incubationBatchEndedAt = preferences.getUInt("batchEnded", 0);
  incubationBatchRecordPending = preferences.getBool("batchPending", false);
  lastIncubationHourlyLogAt = preferences.getUInt("hourlyAt", cycleStartedAt);
  lastTrayChangeEventDay = preferences.getUChar("trayAlertDay", 0);
  trayChangeEventPending = preferences.getBool("trayPending", false);
  commandVersion = preferences.getULong64("version", 0);
  hardwareFaultLatched = preferences.getBool("hardwareFault", false);
  hardwareFaultCode = preferences.getString("faultCode", "");
  hardwareFaultAt = preferences.getUInt("faultAt", 0);
  preferences.end();
  // Response and lid-caused faults are not direct safety events. Re-test them
  // after reboot (the lid fault re-latches after the same timeout if the lid
  // is still wrong), while preserving hard faults such as heater
  // overtemperature.
  if (hardwareFaultLatched &&
      (hardwareFaultCode == "HUMIDIFIER_NO_HUMIDITY_RISE" ||
       hardwareFaultCode == "HEATER_NO_TEMPERATURE_RISE" ||
       hardwareFaultCode == "EXHAUST_NO_COOLING_RESPONSE" ||
       hardwareFaultCode == "LID_OPEN_TOO_LONG_DURING_SETTING" ||
       hardwareFaultCode == "LID_OPEN_TOO_LONG_DURING_HATCHING")) {
    const String clearedFault = hardwareFaultCode;
    hardwareFaultLatched = false;
    hardwareFaultCode = "";
    hardwareFaultAt = 0;
    Serial.printf("Boot: auto-cleared %s; will re-verify.\n",
                  clearedFault.c_str());
  }

  const bool targetsValid = isfinite(targetTemperature) &&
                            isfinite(normalHumidity) && isfinite(hatchHumidity) &&
                            targetTemperature >= MIN_TARGET_TEMPERATURE &&
                            targetTemperature <= MAX_TARGET_TEMPERATURE &&
                            normalHumidity >= MIN_TARGET_HUMIDITY &&
                            normalHumidity <= MAX_TARGET_HUMIDITY &&
                            hatchHumidity >= MIN_TARGET_HUMIDITY &&
                            hatchHumidity <= MAX_TARGET_HUMIDITY;
  const bool scheduleValid = totalDays >= 1 && totalDays <= 35 &&
                             lockdownDay >= 1 && lockdownDay <= totalDays &&
                             turnIntervalHours >= 1 && turnIntervalHours <= 24 &&
                             startDayOffset < totalDays &&
                             validEpoch(cycleStartedAt) &&
                             validEpoch(lastTurnAt) && lastTurnAt >= cycleStartedAt;

  const bool rtcReady = rtcAvailable && rtcRunning && rtcTimeTrusted;
  if (cycleActive && (!targetsValid || !scheduleValid)) {
    cycleActive = false;
    commandState = "rejected";
    commandError = "Persisted cycle state is invalid and could not be restored.";
    saveCycleState();
    Serial.println("Boot: discarded persisted cycle (corrupt stored state).");
  } else if (cycleActive && !rtcReady) {
    // A brownout reset (e.g. exhaust fan + vent servo inrush sagging the 5V
    // rail) frequently makes the DS1307 miss its first I2C poll even though
    // the chip is healthy. Discarding a valid cycle for that once meant a
    // running batch was silently aborted on every such reset. Keep the cycle:
    // every clock-gated duty (day counting, tray turns, logs) defers while
    // rtcUnixNow() returns 0, and the RTC re-probe in sampleEnvironment() or
    // an NTP fix in synchronizeRtcFromInternet() restores time by itself. A
    // permanently dead clock is still aborted, but only after
    // CLOCK_LOSS_ABORT_MS in updateCyclePhase().
    Serial.println(
        "Boot: RTC not ready; persisted cycle kept, clock duties deferred.");
  }
  // Bug 1 fix: auto-resume active cycles after power loss — no user confirmation required.
  // The existing NVS state + RTC elapsed time are sufficient; missed tray turns are caught
  // up automatically by the turn-timer logic in runLocalController().
  cycleResumeRequired = false;
  if (cycleActive) {
    powerRestoreEventPending = true;
    Serial.println("Power-restore: resuming active cycle automatically.");
  }
  if (cycleActive || commandState == "rejected") {
    reportedStatePending = true;
  }

  // Fix: clear a stale trayChangeEventPending that was persisted from a
  // previous boot where lockdownDay was wrong or the flag was set too early.
  // If we are genuinely before lockdown (cycleDay < lockdownDay), the tray
  // notification should never be shown regardless of what NVS stored.
  if (cycleActive && lockdownDay > 0 && cycleDay < lockdownDay) {
    if (trayChangeEventPending) {
      trayChangeEventPending = false;
      trayChangeRequired = false;
      Serial.printf(
          "Boot: cleared stale trayChangeEventPending "
          "(day %u < lockdownDay %u).\n",
          cycleDay, lockdownDay);
    }
  }
}

void setSafeOutputs(bool keepCirculation = false) {
  keepCirculation = keepCirculation || cycleActive || calibrationActive;
  heaterPwm = 0;
  humidifierOn = false;
  trayTurning = false;
  exhaustFan = false;
  productionExhaustPwm = 0;
  circulationFan1 = keepCirculation;
  ventAngle = ventClosedAngle;
  writeHeaterPwm(0);
  writeCirculationFan1(circulationFan1);
  writeExhaustPwm(0);
  digitalWrite(HUMIDIFIER_PIN, OUTPUT_OFF);
  // Circulation servo: always rest at closed angle when safe.
  physicalCircServoAngle = circServoClosedAngle;
  if (circServo.attached()) circServo.write(physicalCircServoAngle);
  physicalServoAngle = ventAngle;
  if (ventServo.attached()) ventServo.write(physicalServoAngle);
}

void setBleStatus(const String &status) {
  if (bleStatus == nullptr) return;
  bleStatus->setValue(status.c_str());
  bleStatus->notify();
}

void saveWifi(const String &ssid, const String &password) {
  preferences.begin("network", false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.end();
  wifiSsid = ssid;
  wifiPassword = password;
}

// Servo calibration persistence helpers. Defined after rtcUnixNow(), which the
// shadow writer stamps into the record for diagnostics.
uint32_t rtcUnixNow();
void saveServoAngles();
void saveCircServoAngles();
bool readServoConfigShadow(uint8_t &ventClosed, uint8_t &ventOpen,
                           uint8_t &circClosed, uint8_t &circOpen,
                           uint32_t &savedAt);
bool writeServoConfigPair(const char *ns, const char *closedKey,
                          const char *openKey, uint8_t closed, uint8_t open);
bool writeServoConfigShadow(uint8_t ventClosed, uint8_t ventOpen,
                            uint8_t circClosed, uint8_t circOpen);

void loadPersistentState() {
  if (preferences.begin("network", true)) {
    wifiSsid = preferences.getString("ssid", "");
    wifiPassword = preferences.getString("password", "");
    preferences.end();
  }
  if (preferences.begin("device", true)) {
    ownerUid = preferences.getString("ownerUid", "");
    preferences.end();
  }
  if (preferences.begin("factory", true)) {
    factoryResetCloudPending = preferences.getBool("cloudPending", false);
    preferences.end();
  }
  // ── Servo calibration ────────────────────────────────────────────────────
  bool ventServoConfigValid = false;
  bool circServoConfigValid = false;
  if (preferences.begin("servo", true)) {
    if (preferences.isKey("closed") && preferences.isKey("open")) {
      const uint8_t closed = preferences.getUChar("closed", 0xFF);
      const uint8_t open = preferences.getUChar("open", 0xFF);
      ventServoConfigValid = closed <= 180 && open <= 180;
      if (ventServoConfigValid) {
        ventClosedAngle = closed;
        ventOpenAngle = open;
      }
    }
    preferences.end();
  }
  if (preferences.begin("circServo", true)) {
    if (preferences.isKey("closed") && preferences.isKey("open")) {
      const uint8_t closed = preferences.getUChar("closed", 0xFF);
      const uint8_t open = preferences.getUChar("open", 0xFF);
      circServoConfigValid = closed <= 180 && open <= 180;
      if (circServoConfigValid) {
        circServoClosedAngle = closed;
        circServoOpenAngle = open;
      }
    }
    preferences.end();
  }
  // Shadow copy. If either primary namespace lost its keys — which is what an
  // NVS page write torn by a brownout reset looks like — the checksummed shadow
  // restores it, and the repair is written back so it is durable from here on.
  const bool repairVentPrimary = !ventServoConfigValid;
  const bool repairCircPrimary = !circServoConfigValid;
  uint8_t shadowVentClosed = 0;
  uint8_t shadowVentOpen = 0;
  uint8_t shadowCircClosed = 0;
  uint8_t shadowCircOpen = 0;
  uint32_t shadowSavedAt = 0;
  if (readServoConfigShadow(shadowVentClosed, shadowVentOpen, shadowCircClosed,
                            shadowCircOpen, shadowSavedAt)) {
    servoConfigSavedAt = shadowSavedAt;
    if (!ventServoConfigValid) {
      ventClosedAngle = shadowVentClosed;
      ventOpenAngle = shadowVentOpen;
      ventServoConfigValid = true;
      servoConfigRestoredFromShadow = true;
    }
    if (!circServoConfigValid) {
      circServoClosedAngle = shadowCircClosed;
      circServoOpenAngle = shadowCircOpen;
      circServoConfigValid = true;
      servoConfigRestoredFromShadow = true;
    }
  } else if (!ventServoConfigValid && !circServoConfigValid) {
    // Nothing recoverable: first boot after a flash erase, or both copies gone.
    Serial.println("Servo config: no stored calibration, using factory defaults.");
  }
  circServoClosedAngle = constrain(circServoClosedAngle, 0, 180);
  circServoOpenAngle   = constrain(circServoOpenAngle, 0, 180);
  if (preferences.begin("control", true)) {
    moistureReducerEnabled = preferences.getBool("moisture", false);
    adaptiveMoistureControlEnabled = preferences.getBool("adaptive", true);
    exhaustMinimumPercent = preferences.getUChar("exhaustMin", 50);
    preferences.end();
  }
  exhaustMinimumPercent = constrain(exhaustMinimumPercent, 50, 80);
  ventClosedAngle = constrain(ventClosedAngle, 0, 180);
  ventOpenAngle = constrain(ventOpenAngle, 0, 180);
  ventAngle = ventClosedAngle;
  // Record where the values actually came from, for the telemetry below.
  if (servoConfigRestoredFromShadow) {
    servoConfigSource = 2;
  } else if (ventServoConfigValid || circServoConfigValid) {
    servoConfigSource = 1;
  } else {
    servoConfigSource = 0;
  }
  // Persist immediately whenever a copy was missing, so a device that is stuck
  // in a brownout reboot loop heals its storage on the first clean boot instead
  // of losing the calibration again on the next one.
  if (servoConfigRestoredFromShadow) {
    // Repair only the damaged primaries; preserve the known-good shadow record.
    if (repairVentPrimary &&
        !writeServoConfigPair("servo", "closed", "open", ventClosedAngle,
                              ventOpenAngle)) {
      Serial.println("Servo config: vent primary repair failed; shadow retained.");
    }
    if (repairCircPrimary &&
        !writeServoConfigPair("circServo", "closed", "open", circServoClosedAngle,
                              circServoOpenAngle)) {
      Serial.println("Servo config: circulation primary repair failed; shadow retained.");
    }
  } else if (!ventServoConfigValid || !circServoConfigValid) {
    saveServoAngles();
    saveCircServoAngles();
  }
  Serial.printf("Servo config loaded: vent %u/%u, circ %u/%u%s\n",
                ventClosedAngle, ventOpenAngle, circServoClosedAngle,
                circServoOpenAngle,
                servoConfigRestoredFromShadow ? " (restored from shadow)" : "");
}

void saveServoAngles() {
  const bool primary = writeServoConfigPair("servo", "closed", "open",
                                            ventClosedAngle, ventOpenAngle);
  // The shadow carries all four angles, so either namespace can be repaired.
  const bool shadow = writeServoConfigShadow(ventClosedAngle, ventOpenAngle,
                                             circServoClosedAngle,
                                             circServoOpenAngle);
  if (primary && shadow) {
    servoConfigSavedAt = rtcUnixNow();
    servoConfigSource = 1;
    Serial.printf("Servo config saved: vent %u/%u, circ %u/%u\n",
                  ventClosedAngle, ventOpenAngle, circServoClosedAngle,
                  circServoOpenAngle);
  } else {
    // Never fail silently — a dropped save is exactly what looked like
    // "the settings reset themselves after a reboot".
    Serial.printf("Servo config SAVE FAILED (primary=%s shadow=%s)\n",
                  primary ? "ok" : "FAIL", shadow ? "ok" : "FAIL");
  }
}

void saveCircServoAngles() {
  const bool primary = writeServoConfigPair("circServo", "closed", "open",
                                            circServoClosedAngle,
                                            circServoOpenAngle);
  const bool shadow = writeServoConfigShadow(ventClosedAngle, ventOpenAngle,
                                             circServoClosedAngle,
                                             circServoOpenAngle);
  if (primary && shadow) {
    servoConfigSavedAt = rtcUnixNow();
    servoConfigSource = 1;
    Serial.printf("Servo config saved: vent %u/%u, circ %u/%u\n",
                  ventClosedAngle, ventOpenAngle, circServoClosedAngle,
                  circServoOpenAngle);
  } else {
    Serial.printf("Servo config SAVE FAILED (primary=%s shadow=%s)\n",
                  primary ? "ok" : "FAIL", shadow ? "ok" : "FAIL");
  }
}

uint16_t servoConfigChecksum(uint8_t ventClosed, uint8_t ventOpen,
                             uint8_t circClosed, uint8_t circOpen) {
  uint32_t hash = SERVO_CFG_MAGIC;
  hash = hash * 31U + ventClosed;
  hash = hash * 31U + ventOpen;
  hash = hash * 31U + circClosed;
  hash = hash * 31U + circOpen;
  hash = hash * 31U + SERVO_CFG_VERSION;
  return static_cast<uint16_t>(hash ^ (hash >> 16));
}

// Writes one angle pair and reads it straight back. NVS reports a short write
// when the flash is full or the partition is unhappy, and that used to be
// indistinguishable from a successful save.
bool writeServoConfigPair(const char *ns, const char *closedKey,
                          const char *openKey, uint8_t closed, uint8_t open) {
  if (!preferences.begin(ns, false)) {
    Serial.printf("Servo config: cannot open NVS namespace '%s' for writing.\n", ns);
    return false;
  }
  const size_t closedWritten = preferences.putUChar(closedKey, closed);
  const size_t openWritten = preferences.putUChar(openKey, open);
  const bool verified = preferences.getUChar(closedKey, 0xFF) == closed &&
                        preferences.getUChar(openKey, 0xFF) == open;
  preferences.end();
  if (closedWritten == 0 || openWritten == 0 || !verified) {
    Serial.printf("Servo config: write to '%s' did not stick (%u/%u bytes, verify %s).\n",
                  ns, static_cast<unsigned>(closedWritten),
                  static_cast<unsigned>(openWritten), verified ? "ok" : "FAILED");
    return false;
  }
  return true;
}

bool writeServoConfigShadow(uint8_t ventClosed, uint8_t ventOpen,
                            uint8_t circClosed, uint8_t circOpen) {
  if (!preferences.begin(SERVO_CFG_SHADOW_NS, false)) return false;
  preferences.putUShort("magic", SERVO_CFG_MAGIC);
  preferences.putUChar("ver", SERVO_CFG_VERSION);
  preferences.putUChar("vc", ventClosed);
  preferences.putUChar("vo", ventOpen);
  preferences.putUChar("cc", circClosed);
  preferences.putUChar("co", circOpen);
  preferences.putUShort(
      "crc", servoConfigChecksum(ventClosed, ventOpen, circClosed, circOpen));
  preferences.putUInt("savedAt", rtcUnixNow());  // 0 when the RTC is not up yet
  const bool verified = preferences.getUChar("vc", 0xFF) == ventClosed &&
                        preferences.getUChar("vo", 0xFF) == ventOpen &&
                        preferences.getUChar("cc", 0xFF) == circClosed &&
                        preferences.getUChar("co", 0xFF) == circOpen;
  preferences.end();
  return verified;
}

bool readServoConfigShadow(uint8_t &ventClosed, uint8_t &ventOpen,
                           uint8_t &circClosed, uint8_t &circOpen,
                           uint32_t &savedAt) {
  if (!preferences.begin(SERVO_CFG_SHADOW_NS, true)) return false;
  const uint16_t magic = preferences.getUShort("magic", 0);
  const uint8_t version = preferences.getUChar("ver", 0);
  ventClosed = preferences.getUChar("vc", 0xFF);
  ventOpen = preferences.getUChar("vo", 0xFF);
  circClosed = preferences.getUChar("cc", 0xFF);
  circOpen = preferences.getUChar("co", 0xFF);
  const uint16_t storedCrc = preferences.getUShort("crc", 0);
  savedAt = preferences.getUInt("savedAt", 0);
  preferences.end();
  if (magic != SERVO_CFG_MAGIC || version != SERVO_CFG_VERSION) return false;
  if (ventClosed > 180 || ventOpen > 180 || circClosed > 180 || circOpen > 180) {
    return false;
  }
  return storedCrc ==
         servoConfigChecksum(ventClosed, ventOpen, circClosed, circOpen);
}

void saveControlSettings() {
  if (!preferences.begin("control", false)) return;
  preferences.putBool("moisture", moistureReducerEnabled);
  preferences.putBool("adaptive", adaptiveMoistureControlEnabled);
  preferences.putUChar("exhaustMin", exhaustMinimumPercent);
  preferences.end();
}

void saveOwner(const String &uid) {
  preferences.begin("device", false);
  preferences.putString("ownerUid", uid);
  preferences.end();
  ownerUid = uid;
}

void startProvisioning();
String buildBleSnapshotJson();
void fillProductionTelemetry(JsonDocument &telemetry);
int firebaseRequest(const char *method, const char *path, const String &body,
                    String &response);

void clearNamespace(const char *name) {
  if (!preferences.begin(name, false)) return;
  preferences.clear();
  preferences.end();
}

void saveFactoryResetPending(bool pending) {
  preferences.begin("factory", false);
  preferences.putBool("cloudPending", pending);
  preferences.end();
  factoryResetCloudPending = pending;
}

bool clearCloudForFactoryReset() {
  if (WiFi.status() != WL_CONNECTED) return false;
  JsonDocument reset;
  reset["ownerUid"] = nullptr;
  reset["claimRequest"] = nullptr;
  reset["desired"] = nullptr;
  reset["reported"] = nullptr;
  reset["telemetry"] = nullptr;
  reset["testCommand"] = nullptr;
  reset["testTelemetry"] = nullptr;
  reset["status"]["online"] = false;
  reset["status"]["lastSeen"][".sv"] = "timestamp";
  reset["status"]["factoryReset"] = true;
  reset["deviceUid"] = "QLY2SB9qvleL6jhw02SgsOKSLUu1";
  reset["claimCode"] = DEVICE_CLAIM_CODE;
  String body;
  serializeJson(reset, body);
  String response;
  const int status = firebaseRequest("PATCH", "", body, response);
  if (status >= 200 && status < 300) return true;
  Serial.printf("Factory-reset cloud cleanup failed (%d): %s\n", status,
                response.c_str());
  return false;
}

void performFactoryReset() {
  factoryResetInProgress = true;
  cycleActive = false;
  testEnabled = false;
  calibrationActive = false;
  bleAuthorized = false;
  ++bleSession;
  wifiProvisioningPending = false;
  wifiScanPending = false;
  setSafeOutputs();
  Serial.println("Factory reset confirmed. Outputs are OFF.");

  const bool cloudCleared = clearCloudForFactoryReset();
  saveFactoryResetPending(!cloudCleared);
  clearNamespace("cycle");
  clearNamespace("calibration");
  clearNamespace("control");
  clearNamespace("servo");
  // circServo was missing here, so a factory reset used to leave the egg-turn
  // calibration behind. The shadow must go too, otherwise loadPersistentState()
  // would restore the old angles on the very next boot.
  clearNamespace("circServo");
  clearNamespace(SERVO_CFG_SHADOW_NS);
  clearNamespace("device");
  clearNamespace("network");
  wifiSsid.clear();
  wifiPassword.clear();
  ownerUid.clear();
  profileId.clear();
  commandVersion = 0;
  calibrated = false;
  calibrationActive = false;
  calibrationTemperatureOffset = 0.0F;
  calibrationHumidityOffset = 0.0F;
  calibrationStatus = "not_calibrated";
  idToken.clear();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  startProvisioning();
  setBleStatus(
      cloudCleared
          ? "{\"state\":\"factoryReset\",\"message\":\"Ready to pair with a new owner and Wi-Fi.\"}"
          : "{\"state\":\"factoryReset\",\"message\":\"Local reset complete. Cloud cleanup needs internet after Wi-Fi pairing.\"}");
  Serial.printf("Factory reset complete (%s). BLE pairing is ready.\n",
                cloudCleared ? "cloud cleared" : "local only");
  // Reload all defaults and drop old BLE sessions, queued commands and RAM state.
  ESP.restart();
}

void completePendingFactoryReset() {
  if (!factoryResetCloudPending || WiFi.status() != WL_CONNECTED ||
      millis() - lastFactoryResetCloudAttempt < COMMAND_INTERVAL_MS) {
    return;
  }
  lastFactoryResetCloudAttempt = millis();
  if (clearCloudForFactoryReset()) {
    saveFactoryResetPending(false);
    Serial.println("Completed pending factory-reset cloud cleanup.");
    setBleStatus(
        "{\"state\":\"ready\",\"message\":\"Cloud reset complete. Ready for ownership claim.\"}");
  }
}

void handleFactoryResetButton() {
  const bool pressed = digitalRead(FACTORY_RESET_PIN) == LOW;
  if (!pressed) {
    factoryResetPressedAt = 0;
    if (factoryResetArmed && !factoryResetInProgress) performFactoryReset();
    factoryResetArmed = false;
    return;
  }
  if (factoryResetInProgress) return;
  // Refuse to arm while the pin never read HIGH at boot, or during the first
  // few seconds. GPIO 0 is a strapping pin on a board that shares its ground
  // with a 12 V heater/fan rail, and an accidental 8-second low there wipes
  // every persisted namespace — including the servo calibration.
  if (!factoryResetPinHealthy) return;
  if (millis() < FACTORY_RESET_BOOT_GRACE_MS) return;
  if (factoryResetPressedAt == 0) {
    factoryResetPressedAt = millis();
    Serial.println("BOOT held: keep holding 8 seconds for factory reset.");
    return;
  }
  if (!factoryResetArmed &&
      millis() - factoryResetPressedAt >= FACTORY_RESET_HOLD_MS) {
    factoryResetArmed = true;
    setSafeOutputs();
    Serial.println("Factory reset armed. Release BOOT to confirm.");
  }
}

void applyTestOutputs() {
  const unsigned long now = millis();
  if (testEnabled && now - lastTestCommandReceived >= TEST_COMMAND_LEASE_MS) {
    testEnabled = false;
    atmosphereTestEnabled = false;
    Serial.println("Hardware test lease expired; outputs disabled.");
  }
  const bool probesFresh = eggTemperatureValid && heaterTemperatureValid &&
                           sensorValid && eggSensorUpdatedAt > 0 &&
                           heaterSensorUpdatedAt > 0 && humiditySensorUpdatedAt > 0 &&
                           now - eggSensorUpdatedAt <= SENSOR_STALE_MS &&
                           now - heaterSensorUpdatedAt <= SENSOR_STALE_MS &&
                           now - humiditySensorUpdatedAt <= DHT_SENSOR_STALE_MS;
  const bool basicTestAllowed = testEnabled && !cycleActive && !calibrationActive;
  const bool heaterTestAllowed =
      basicTestAllowed && !lidOpen && !hardwareFaultLatched;
  uint8_t safeHeaterPwm =
      heaterTestAllowed && probesFresh && testHeater ? testHeaterPwm : 0;
  bool safeHumidifier = basicTestAllowed && testHumidifier;
  uint8_t safeExhaustPwm = basicTestAllowed ? testExhaustPwm : 0;
  bool safeFan1 = basicTestAllowed && testFan1;
  if (heaterTestAllowed && atmosphereTestEnabled && probesFresh) {
    safeFan1 = true;
    const float temperatureError =
        atmosphereTargetTemperature - measuredTemperature;
    const float dt = lastControllerUpdate == 0
                         ? 0.1F
                         : min(2.0F, (now - lastControllerUpdate) / 1000.0F);
    constexpr float TEST_KP = 90.0F;
    constexpr float TEST_KI = 2.0F;
    const float candidateIntegral = constrain(
        atmosphereHeaterIntegral + temperatureError * dt, 0.0F, 100.0F);
    const float unconstrained =
        TEST_KP * temperatureError + TEST_KI * candidateIntegral;
    if ((unconstrained > 0.0F && unconstrained < 255.0F) ||
        (unconstrained <= 0.0F && temperatureError > 0.0F) ||
        (unconstrained >= 255.0F && temperatureError < 0.0F)) {
      atmosphereHeaterIntegral = candidateIntegral;
    }
    safeHeaterPwm = static_cast<uint8_t>(constrain(
        TEST_KP * temperatureError + TEST_KI * atmosphereHeaterIntegral,
        0.0F, 255.0F));
    safeHumidifier = measuredHumidity < atmosphereTargetHumidity - 2.0F;
    atmosphereTemperatureVenting = atmosphereTemperatureVenting
                                       ? measuredTemperature >
                                             atmosphereTargetTemperature + 0.2F
                                       : measuredTemperature >=
                                             atmosphereTargetTemperature + 0.5F;
    safeExhaustPwm = atmosphereTemperatureVenting ? 255 : 0;
  } else {
    atmosphereHeaterIntegral = 0.0F;
    atmosphereTemperatureVenting = false;
  }
  if (safeExhaustPwm > 0) safeHumidifier = false;
  heaterTestBlocker = "";
  if (!testEnabled)
    heaterTestBlocker = "ENABLE_HARDWARE_TEST_MODE";
  else if (cycleActive)
    heaterTestBlocker = "INCUBATION_CYCLE_ACTIVE";
  else if (lidOpen)
    heaterTestBlocker = "CLOSE_LID_SWITCH";
  else if (hardwareFaultLatched)
    heaterTestBlocker = "ACKNOWLEDGE_HARDWARE_FAULT";
  else if (!eggTemperatureValid)
    heaterTestBlocker = "CONNECT_EGG_DS18B20_GPIO16";
  else if (!heaterTemperatureValid)
    heaterTestBlocker = "CONNECT_HEATER_DS18B20_GPIO17";
  else if (!sensorValid)
    heaterTestBlocker = sensorFaultCode;
  else if (!probesFresh)
    heaterTestBlocker = "TEMPERATURE_PROBES_STALE";
  else if (!testHeater || testHeaterPwm == 0)
    heaterTestBlocker = "SET_HEATER_SLIDER_ABOVE_ZERO";
  if (heaterTemperatureValid &&
      heaterSurfaceTemperature > HEATER_SURFACE_DERATE_C) {
    const float ratio = (HEATER_SURFACE_MAX_C - heaterSurfaceTemperature) /
                        (HEATER_SURFACE_MAX_C - HEATER_SURFACE_DERATE_C);
    safeHeaterPwm = min(
        safeHeaterPwm,
        static_cast<uint8_t>(constrain(255.0F * ratio, 0.0F, 255.0F)));
  }
  if (basicTestAllowed && isfinite(measuredTemperature) &&
      measuredTemperature >= ABSOLUTE_OVERHEAT_C) {
    safeHeaterPwm = 0;
    safeHumidifier = false;
    safeFan1 = true;
    safeExhaustPwm = 255;
    heaterTestBlocker = "EGG_OVERTEMPERATURE";
  }
  if (safeHeaterPwm > 0) safeFan1 = true;
  writeCirculationFan1(safeFan1);
  appliedTestExhaustPwm = writeExhaustPwm(safeExhaustPwm);
  appliedTestFan1 = safeFan1;
  appliedTestHumidifier = safeHumidifier;
  appliedTestHeaterPwm = safeHeaterPwm;
  writeHeaterPwm(safeHeaterPwm);
  digitalWrite(HUMIDIFIER_PIN,
               safeHumidifier ? OUTPUT_ON : OUTPUT_OFF);
  // ── Exhaust vent servo (GPIO 13) ────────────────────────────────────────
  // Bug fix: the slider is only active when testServoEnabled is explicitly ON.
  // When the toggle is OFF the servo follows exhaust fan automatically.
  const bool exhaustRunning = safeExhaustPwm > 0;
  if (ventServo.attached()) {
    appliedTestServoAngle = (testServoEnabled && basicTestAllowed &&
                             heaterTestBlocker != "EGG_OVERTEMPERATURE")
                                ? testServoAngle
                                : (exhaustRunning ? ventOpenAngle
                                                  : ventClosedAngle);
    // Write only on change (same buzz/stall-current reasoning as production).
    if (appliedTestServoAngle != physicalServoAngle) {
      physicalServoAngle = appliedTestServoAngle;
      ventServo.write(physicalServoAngle);
    }
  }
  // ── Circulation servo (GPIO 26) — egg-turn switch ───────────────────────
  // Slider only drives the servo when testCircServoEnabled is explicitly ON.
  if (circServo.attached()) {
    appliedTestCircServoAngle = (testCircServoEnabled && basicTestAllowed)
                                    ? testCircServoAngle
                                    : circServoClosedAngle;
    if (appliedTestCircServoAngle != physicalCircServoAngle) {
      physicalCircServoAngle = appliedTestCircServoAngle;
      circServo.write(physicalCircServoAngle);
    }
  }
}

const char *dhtStatusName(int status) {
  return status == 0 ? "ok" : "read_failed";
}

// Read one DS18B20 with the scratchpad CRC verified and a single-sample spike
// rejected. The library's getTempCByIndex() trusts the scratchpad even when the
// 1-Wire transfer was corrupt, and a half-converted read (see
// DS18B20_CONVERSION_MS) can pass the range filter with a shifted value.
float readDs18b20Checked(DallasTemperature &sensor, float previousGood,
                         bool &valid, const char *label) {
  valid = false;
  DeviceAddress address;
  if (!sensor.getAddress(address, 0)) {
    return DEVICE_DISCONNECTED_C;
  }
  float value = sensor.getTempC(address);
  if (value <= DEVICE_DISCONNECTED_C || value == 85.0F || !isfinite(value)) {
    return value;  // genuine disconnect / power-on reset; caller filters it
  }
  // Spike rejection: a real temperature cannot jump this fast in one sample.
  if (isfinite(previousGood) &&
      fabsf(value - previousGood) > TEMP_SPIKE_REJECT_C) {
    Serial.printf("DS18B20 spike rejected sensor=%s value=%.2f previous=%.2f\n",
                  label, value, previousGood);
    return previousGood;
  }
  valid = true;
  return value;
}

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "power_on";
    case ESP_RST_SW:
      return "software_restart";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
      return "interrupt_watchdog";
    case ESP_RST_TASK_WDT:
      return "task_watchdog";
    case ESP_RST_WDT:
      return "other_watchdog";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_DEEPSLEEP:
      return "deep_sleep_wake";
    default:
      return "unknown";
  }
}

void configureDhtSensor(DHT &sensor) {
  sensor.begin();
}

// Persistent boot health counters. A brownout that happens only when the heater
// switches is the signature of a 12 V wiring or ground-reference fault, not a
// firmware fault. Counting resets across reboots is the only way to verify the
// wiring fix: once the ground and gate drive are corrected, brownoutCount must
// stop rising.
uint32_t bootCount = 0;
uint32_t brownoutCount = 0;

void recordBootHealth(esp_reset_reason_t reason) {
  if (!preferences.begin("health", false)) return;
  bootCount = preferences.getUInt("boots", 0) + 1;
  brownoutCount = preferences.getUInt("brownouts", 0);
  if (reason == ESP_RST_BROWNOUT) brownoutCount += 1;
  preferences.putUInt("boots", bootCount);
  preferences.putUInt("brownouts", brownoutCount);
  preferences.end();
  Serial.printf("Boot health: boot #%lu, cumulative brownout resets %lu\n",
                static_cast<unsigned long>(bootCount),
                static_cast<unsigned long>(brownoutCount));
  if (brownoutCount >= 5) {
    Serial.println(
        "WARNING: repeated brownout resets. This is electrical — fix the 12 V "
        "ground reference and MOSFET gate drive before running a cycle.");
  }
}

void powerCycleDhtSensor(DHT &sensor, uint8_t dataPin, uint8_t powerPin) {
  // The external pull-up remains on 3.3 V, so hold DATA low to prevent it from
  // partially powering the sensor while its switched VCC is off.
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, LOW);
  digitalWrite(powerPin, LOW);
  delay(DHT_POWER_OFF_MS);
  digitalWrite(powerPin, HIGH);
  delay(DHT_POWER_STABILIZE_MS);
  pinMode(dataPin, INPUT_PULLUP);
  configureDhtSensor(sensor);
}

void updateDhtRecovery(DHT &sensor, uint8_t pin, const char *label,
                       int readStatus, bool temperatureValid,
                       bool humidityValid, unsigned long nowMs,
                       DhtRecoveryState &state, int powerPin = -1) {
  state.readStatus = readStatus;
  pinMode(pin, INPUT_PULLUP);
  state.dataLevel = digitalRead(pin);
  const bool usable = readStatus == 0 && (temperatureValid || humidityValid);
  if (usable) {
    if (state.outageActive) {
      Serial.printf("DHT_RECOVERED sensor=%s pin=%u attempts=%u\n", label, pin,
                    state.recoveryAttempts);
    }
    state.consecutiveFailures = 0;
    state.recoveryAttempts = 0;
    state.diagnostic = temperatureValid && humidityValid ? "working"
                                                        : "partial_read";
    state.outageActive = false;
    return;
  }

  state.outageActive = true;
  if (state.consecutiveFailures < UINT16_MAX) {
    ++state.consecutiveFailures;
  }
  if (state.dataLevel == LOW) {
    state.diagnostic = "data_line_stuck_low_check_short_or_pinout";
  } else {
    state.diagnostic = "no_response_check_power_data_and_pullup";
  }
  if (state.consecutiveFailures == DHT_RECOVERY_FAILURE_THRESHOLD) {
    Serial.printf(
        "DHT_FAILURE sensor=%s pin=%u failures=%u data=%s status=%s(%d)\n",
        label, pin, state.consecutiveFailures,
        state.dataLevel == LOW ? "LOW" : "HIGH", dhtStatusName(readStatus),
        readStatus);
  }
  if (state.consecutiveFailures < DHT_RECOVERY_FAILURE_THRESHOLD ||
      (state.lastRecoveryAt != 0 &&
       nowMs - state.lastRecoveryAt < DHT_RECOVERY_COOLDOWN_MS)) {
    return;
  }

  if (powerPin >= 0) {
    powerCycleDhtSensor(sensor, pin, static_cast<uint8_t>(powerPin));
  } else {
    pinMode(pin, INPUT_PULLUP);
    delay(5);
    configureDhtSensor(sensor);
  }
  state.lastRecoveryAt = millis();
  if (state.recoveryAttempts < UINT16_MAX) ++state.recoveryAttempts;
  Serial.printf(
      "DHT_RECOVERY sensor=%s pin=%u attempt=%u status=%s diagnostic=%s\n",
      label, pin, state.recoveryAttempts, dhtStatusName(readStatus),
      state.diagnostic.c_str());
}

void sampleEnvironment() {
  const unsigned long nowMs = millis();
  if (!ds18b20ConversionPending && nowMs - lastSensorRead >= SENSOR_INTERVAL_MS) {
    eggTemperatureSensor.requestTemperatures();
    heaterTemperatureSensor.requestTemperatures();
    ds18b20ConversionStartedAt = nowMs;
    ds18b20ConversionPending = true;
    return;
  }
  if (!ds18b20ConversionPending ||
      nowMs - ds18b20ConversionStartedAt < DS18B20_CONVERSION_MS) {
    return;
  }
  ds18b20ConversionPending = false;
  lastSensorRead = nowMs;
  // Re-probe a lost RTC on a backoff so a transient bus glitch self-heals
  // instead of killing the clock until the next reboot. rtc.begin() is required
  // here rather than a plain bus probe: setup() short-circuits it behind
  // rtcHardwareDetected, so i2c_dev can still be null when the RTC was absent at
  // boot, and rtc.now() without it would dereference a null pointer.
  if (!rtcHardwareDetected &&
      (lastRtcRedetectAt == 0 ||
       nowMs - lastRtcRedetectAt >= RTC_REDETECT_INTERVAL_MS)) {
    lastRtcRedetectAt = nowMs;
    if (recoverI2cBus() && rtc.begin()) {
      rtcHardwareDetected = true;
      rtcRecoveryStrikes = 0;
      rtcAvailable = true;
      rtcRunning = rtc.isrunning();
      // Mirror the setup-time validation: the DS1307 kept counting on its
      // backup battery, so its time is usable again if it is a valid epoch.
      rtcTimeTrusted = rtcRunning && validEpoch(rtc.now().unixtime());
      Serial.printf(
          "RTC re-detected on the I2C bus; clock restored (%s), time %s.\n",
          rtcRunning ? "running" : "stopped",
          rtcTimeTrusted ? "trusted" : "unverified");
      reportedStatePending = true;
    }
  }

  if (rtcHardwareDetected) {
    if (digitalRead(I2C_SDA_PIN) == LOW || digitalRead(I2C_SCL_PIN) == LOW) {
      const bool recovered = recoverI2cBus();
      rtcAvailable = recovered && probeI2cAddress(0x68);
      if (recovered) {
        rtcRecoveryStrikes = 0;
      } else if (rtcRecoveryStrikes < 255 &&
                 ++rtcRecoveryStrikes >= RTC_RECOVERY_STRIKES_BEFORE_LOSS) {
        // Tolerate brief glitches. Presence is only given up after repeated
        // failures, and even then the re-probe above keeps retrying, so a
        // recovered bus brings the clock back without a reboot.
        rtcHardwareDetected = false;
        lastRtcRedetectAt = 0;
        Serial.printf(
            "RTC lost after %u failed bus recoveries; re-probing every %lus.\n",
            static_cast<unsigned>(rtcRecoveryStrikes),
            static_cast<unsigned long>(RTC_REDETECT_INTERVAL_MS / 1000UL));
        reportedStatePending = true;
      }
    } else {
      rtcAvailable = probeI2cAddress(0x68);
      if (rtcAvailable) rtcRecoveryStrikes = 0;
    }
    rtcRunning = rtcAvailable && rtc.isrunning();
  } else {
    rtcAvailable = false;
    rtcRunning = false;
  }
  const bool dhtReadDue = lastDhtRead == 0 ||
                          nowMs - lastDhtRead >= DHT_SENSOR_INTERVAL_MS;
  bool primaryDhtRead = false;
  bool eggDhtRead = false;
  float primaryT = NAN;
  float primaryH = NAN;
  float eggDhtT = NAN;
  float eggDhtH = NAN;
  int primaryDhtStatus = primaryDhtRecovery.readStatus;
  int eggDhtStatus = eggDhtRecovery.readStatus;
  if (dhtReadDue) {
    lastDhtRead = nowMs;
    primaryDhtRead = dhtPrimary.read();
    primaryT = primaryDhtRead ? dhtPrimary.readTemperature() : NAN;
    primaryH = primaryDhtRead ? dhtPrimary.readHumidity() : NAN;
    primaryDhtStatus = primaryDhtRead ? 0 : -1;
    eggDhtRead = dhtEggZone.read();
    eggDhtT = eggDhtRead ? dhtEggZone.readTemperature() : NAN;
    eggDhtH = eggDhtRead ? dhtEggZone.readHumidity() : NAN;
    eggDhtStatus = eggDhtRead ? 0 : -1;
    lastRawDhtTemperature = primaryT;
    lastRawDhtHumidity = primaryH;
    lastRawEggDhtTemperature = eggDhtT;
    lastRawEggDhtHumidity = eggDhtH;
  }
  bool rawEggOk = false;
  bool rawHeaterOk = false;
  const float rawEggTemperature =
      readDs18b20Checked(eggTemperatureSensor, lastGoodEggTemperature,
                         rawEggOk, "EGG");
  const float rawHeaterTemperature =
      readDs18b20Checked(heaterTemperatureSensor, lastGoodHeaterTemperature,
                         rawHeaterOk, "HEATER");
  lastRawEggTemperature = rawEggTemperature;
  lastRawHeaterTemperature = rawHeaterTemperature;
  if (dhtReadDue) {
    Serial.printf(
        "Sensors raw: DHT top T=%.2f H=%.2f status=%s | DHT egg T=%.2f H=%.2f status=%s | DS18B20 egg=%.2f heater=%.2f\n",
        primaryT, primaryH, dhtStatusName(primaryDhtStatus), eggDhtT, eggDhtH,
        dhtStatusName(eggDhtStatus), rawEggTemperature, rawHeaterTemperature);
  }
  const bool primaryTemperatureFresh =
      dhtReadDue && !isnan(primaryT) &&
      primaryT >= SENSOR_MIN_INCUBATOR_TEMPERATURE_C &&
      primaryT <= SENSOR_MAX_INCUBATOR_TEMPERATURE_C;
  const bool primaryHumidityFresh = dhtReadDue && isfinite(primaryH) &&
                                     primaryH >= SENSOR_MIN_HUMIDITY &&
                                     primaryH <= SENSOR_MAX_HUMIDITY;
  const bool eggDhtTemperatureFresh =
      dhtReadDue && isfinite(eggDhtT) &&
      eggDhtT >= SENSOR_MIN_INCUBATOR_TEMPERATURE_C &&
      eggDhtT <= SENSOR_MAX_INCUBATOR_TEMPERATURE_C;
  const bool eggDhtHumidityFresh = dhtReadDue && isfinite(eggDhtH) &&
                                    eggDhtH >= SENSOR_MIN_HUMIDITY &&
                                    eggDhtH <= SENSOR_MAX_HUMIDITY;
  if (dhtReadDue) {
    updateDhtRecovery(dhtPrimary, DHT_PIN, "PRIMARY", primaryDhtStatus,
                      primaryTemperatureFresh, primaryHumidityFresh, nowMs,
                      primaryDhtRecovery);
    updateDhtRecovery(dhtEggZone, EGG_DHT_PIN, "EGG", eggDhtStatus,
                      eggDhtTemperatureFresh, eggDhtHumidityFresh, nowMs,
                      eggDhtRecovery, EGG_DHT_POWER_PIN);
  }
  if (primaryTemperatureFresh) {
    primaryTemperature = primaryT;
    primaryDhtTemperatureUpdatedAt = nowMs;
  }
  if (primaryHumidityFresh) {
    primaryHumidity = primaryH;
    primaryHumidityUpdatedAt = nowMs;
  }
  if (eggDhtTemperatureFresh) {
    eggDhtTemperature = eggDhtT;
    eggDhtTemperatureUpdatedAt = nowMs;
  }
  if (eggDhtHumidityFresh) {
    eggDhtHumidity = eggDhtH;
    eggDhtHumidityUpdatedAt = nowMs;
  }
  primaryTemperatureValid = primaryDhtTemperatureUpdatedAt > 0 &&
                            nowMs - primaryDhtTemperatureUpdatedAt <=
                                DHT_SENSOR_STALE_MS;
  primaryHumidityValid = primaryHumidityUpdatedAt > 0 &&
                          nowMs - primaryHumidityUpdatedAt <=
                              DHT_SENSOR_STALE_MS;
  primarySensorValid = primaryTemperatureValid && primaryHumidityValid;
  eggDhtTemperatureValid = eggDhtTemperatureUpdatedAt > 0 &&
                           nowMs - eggDhtTemperatureUpdatedAt <=
                               DHT_SENSOR_STALE_MS;
  eggDhtHumidityValid = eggDhtHumidityUpdatedAt > 0 &&
                         nowMs - eggDhtHumidityUpdatedAt <=
                             DHT_SENSOR_STALE_MS;
  primaryDhtUsingCachedData =
      (primaryTemperatureValid && !primaryTemperatureFresh) ||
      (primaryHumidityValid && !primaryHumidityFresh);
  eggDhtUsingCachedData =
      (eggDhtTemperatureValid && !eggDhtTemperatureFresh) ||
      (eggDhtHumidityValid && !eggDhtHumidityFresh);
  eggTemperatureValid = rawEggOk && isfinite(rawEggTemperature) &&
                        rawEggTemperature != DEVICE_DISCONNECTED_C &&
                        rawEggTemperature != 85.0F &&
                        rawEggTemperature >= SENSOR_MIN_INCUBATOR_TEMPERATURE_C &&
                        rawEggTemperature <= SENSOR_MAX_INCUBATOR_TEMPERATURE_C;
  heaterTemperatureValid =
      rawHeaterOk && isfinite(rawHeaterTemperature) &&
      rawHeaterTemperature != DEVICE_DISCONNECTED_C &&
      rawHeaterTemperature != 85.0F && rawHeaterTemperature >= -10.0F &&
      rawHeaterTemperature <= 125.0F;
  eggTemperature = eggTemperatureValid
                       ? rawEggTemperature + calibrationTemperatureOffset
                       : NAN;
  heaterSurfaceTemperature =
      heaterTemperatureValid ? rawHeaterTemperature : NAN;
  if (eggTemperatureValid) {
    eggSensorUpdatedAt = millis();
    lastGoodEggTemperature = rawEggTemperature;
  }
  if (heaterTemperatureValid) {
    heaterSensorUpdatedAt = millis();
    lastGoodHeaterTemperature = rawHeaterTemperature;
  }
  if (primaryTemperatureFresh || primaryHumidityFresh) {
    primarySensorUpdatedAt = nowMs;
  }
  const bool eggTemperatureStable =
      eggTemperatureValid && isfinite(previousEggTemperature) &&
      fabsf(eggTemperature - previousEggTemperature) <=
          EGG_TEMPERATURE_STABLE_RATE_C;
  if (eggTemperatureValid && eggDhtTemperatureValid) {
    eggTemperatureSpread = fabsf(eggTemperature - eggDhtTemperature);
    const bool crossCheckWindow = eggTemperatureStable && !lidOpen &&
                                  heaterPwm <= 64 && !controlPaused;
    if (!eggTemperatureDisagreement && crossCheckWindow) {
      eggTemperatureRecoverySamples = 0;
      if (eggTemperatureSpread > EGG_TEMPERATURE_DISAGREEMENT_C) {
        eggTemperatureDisagreementSamples = min<uint8_t>(
            EGG_TEMPERATURE_CONFIRM_SAMPLES,
            eggTemperatureDisagreementSamples + 1);
        if (eggTemperatureDisagreementSamples >=
            EGG_TEMPERATURE_CONFIRM_SAMPLES) {
          eggTemperatureDisagreement = true;
        }
      } else {
        eggTemperatureDisagreementSamples = 0;
      }
    } else if (eggTemperatureSpread <= EGG_TEMPERATURE_RECOVERY_C) {
      eggTemperatureRecoverySamples = min<uint8_t>(
          EGG_TEMPERATURE_RECOVERY_SAMPLES, eggTemperatureRecoverySamples + 1);
      if (eggTemperatureRecoverySamples >= EGG_TEMPERATURE_RECOVERY_SAMPLES) {
        eggTemperatureDisagreement = false;
        eggTemperatureDisagreementSamples = 0;
        eggTemperatureRecoverySamples = 0;
      }
    } else if (eggTemperatureDisagreement) {
      eggTemperatureRecoverySamples = 0;
    } else {
      eggTemperatureDisagreementSamples = 0;
    }
  } else {
    eggTemperatureSpread = NAN;
    eggTemperatureDisagreementSamples = 0;
    eggTemperatureRecoverySamples = 0;
  }
  if (eggTemperatureValid) previousEggTemperature = eggTemperature;
  // Primary DHT22 is now the ROOM (outside) sensor. Capture room conditions
  // for smart exhaust gating when egg-zone DHT is healthy.
  roomTemperature = primaryTemperatureValid ? primaryTemperature : NAN;
  roomHumidity = primaryHumidityValid ? primaryHumidity : NAN;
  roomHumidityValid = primaryHumidityValid;

  // Humidity consensus:
  //   NORMAL:   egg-zone DHT22 (GPIO 5) drives incubator control;
  //             primary DHT22 (GPIO 4) provides room context for exhaust gate.
  //   DEGRADED: if egg-zone DHT fails, fall back to primary DHT22 to keep
  //             the system running — this is better than a full shutdown.
  //             In degraded mode, smart exhaust gating is disabled because
  //             we cannot compare room vs incubator humidity accurately.
  float rawHumidityConsensus = NAN;
  bool humidityValid = false;
  if (eggDhtHumidityValid) {
    rawHumidityConsensus = eggDhtHumidity;
    humidityValid = true;
    // humiditySensorsAgree: room sensor available → smart exhaust gate active.
    humiditySensorsAgree = roomHumidityValid;
    humidityDegraded = false;
  } else if (primaryHumidityValid) {
    humiditySensorsAgree = false;
    humidityDegraded = true;
    humidityValid = false;
    rawHumidityConsensus = NAN;
  } else {
    humiditySensorsAgree = false;
    humidityDegraded = false;
  }
  measuredHumidity = humidityValid
                         ? rawHumidityConsensus + calibrationHumidityOffset
                         : NAN;
  if (humidityValid) {
    humiditySensorUpdatedAt = eggDhtHumidityValid
                                  ? eggDhtHumidityUpdatedAt
                                  : primaryHumidityUpdatedAt;
  }
  measuredTemperature = eggTemperatureValid ? eggTemperature : NAN;
  sensorValid = eggTemperatureValid && humidityValid;
  if (humidityValid) {
    measuredHumidity = constrain(measuredHumidity, 0.0F, 100.0F);
  }
  if (sensorValid) {
    lastValidSensorRead = min(eggSensorUpdatedAt, humiditySensorUpdatedAt);
  }
  sensorFaultCode = "";
  if (!eggTemperatureValid) {
    sensorFaultCode = "EGG_DS18B20_NOT_CONNECTED";
  }
  if (!eggDhtHumidityValid && !primaryHumidityValid) {
    sensorFaultCode = sensorFaultCode.isEmpty()
                          ? "NO_VALID_DHT22_HUMIDITY"
                          : sensorFaultCode + ";NO_VALID_DHT22_HUMIDITY";
  } else if (humidityDegraded) {
    // Egg DHT failed; running on room sensor fallback.
    sensorFaultCode = sensorFaultCode.isEmpty()
                          ? "EGG_DHT22_FAILED_USING_ROOM_FALLBACK"
                          : sensorFaultCode + ";EGG_DHT22_FAILED_USING_ROOM_FALLBACK";
  }
  if (!heaterTemperatureValid) {
    sensorFaultCode = sensorFaultCode.isEmpty()
                          ? "HEATER_DS18B20_NOT_CONNECTED"
                          : sensorFaultCode + ";HEATER_DS18B20_NOT_CONNECTED";
  }
  if (eggTemperatureDisagreement) {
    sensorValid = false;
    sensorFaultCode = sensorFaultCode.isEmpty()
                          ? "EGG_TEMPERATURE_SENSOR_DISAGREEMENT"
                          : sensorFaultCode +
                                ";EGG_TEMPERATURE_SENSOR_DISAGREEMENT";
  }
  if (!sensorFaultCode.isEmpty()) {
    Serial.printf("Environmental sensor fault: %s\n", sensorFaultCode.c_str());
  }
}

void syncSoftwareClock(uint32_t epoch) {
  if (!validEpoch(epoch)) return;
  // mbedTLS validates certificate dates against the system clock, not our RTC.
  const timeval systemTime = {static_cast<time_t>(epoch), 0};
  if (settimeofday(&systemTime, nullptr) != 0) {
    softwareClockSynced = false;
    return;
  }
  softwareClockEpoch = epoch;
  softwareClockMillisAnchor = millis();
  softwareClockSynced = true;
}

uint32_t rtcUnixNow() {
  uint32_t trustedSourceEpoch = 0;
  if (rtcHardwareDetected && rtcAvailable && rtcRunning) {
    const uint32_t rtcEpoch = rtc.now().unixtime();
    if (validEpoch(rtcEpoch)) {
      if (lastRtcEpoch > 0 && lastRtcMonotonicAt > 0) {
        const uint32_t expected =
            lastRtcEpoch + (millis() - lastRtcMonotonicAt) / 1000UL;
        if (control_logic::absoluteDifference(rtcEpoch, expected) <= 60UL) {
          trustedSourceEpoch = rtcEpoch;
        } else {
          rtcTimeTrusted = false;
        }
      } else {
        trustedSourceEpoch = rtcEpoch;
      }
      if (trustedSourceEpoch > 0) {
        lastRtcEpoch = trustedSourceEpoch;
        lastRtcMonotonicAt = millis();
      }
    }
  }
  if (trustedSourceEpoch > 0) {
    if (!softwareClockSynced ||
        control_logic::absoluteDifference(softwareClockEpoch + (millis() - softwareClockMillisAnchor) / 1000UL, trustedSourceEpoch) > 5UL) {
      syncSoftwareClock(trustedSourceEpoch);
    }
  }
  if (!softwareClockSynced) return 0;
  const uint32_t now =
      softwareClockEpoch + (millis() - softwareClockMillisAnchor) / 1000UL;
  if (!validEpoch(now)) return 0;
  return now;
}

void updateCyclePhase(uint32_t now) {
  rtcFault = cycleActive && now == 0;
  if (!cycleActive) {
    clockSuspendStartedAt = 0;
    return;
  }
  if (now == 0) {
    if (clockSuspendStartedAt == 0) {
      clockSuspendStartedAt = millis();
    } else if (millis() - clockSuspendStartedAt >= CLOCK_LOSS_ABORT_MS) {
      // Offline AND no RTC for the full grace window: heating must not run
      // without any real day tracking (eggs could be held past hatch).
      Serial.println(
          "Clock lost beyond grace window (offline with dead RTC); aborting cycle safely.");
      queueIncubationEvent("batch_aborted", 0);
      queueIncubationBatchRecord("aborted", 0);
      cycleActive = false;
      clockSuspendStartedAt = 0;
      lockdown = false;
      trayChangeRequired = false;
      trayChangeEventPending = false;
      controlPaused = false;
      pauseReason = "";
      trayTurning = false;
      pendingTurnStartedAt = 0;
      commandState = "rejected";
      commandError = "RTC_TIME_UNAVAILABLE_DURING_CYCLE";
      saveCycleState();
      reportedStatePending = true;
      setSafeOutputs();
    }
    return;
  }
  clockSuspendStartedAt = 0;
  if (cycleStartedAt == 0) return;
  if (now < cycleStartedAt || (lastTurnAt > 0 && now < lastTurnAt) ||
      (lastCycleEpoch > 0 && now + 60UL < lastCycleEpoch)) {
    rtcFault = true;
    trayTurning = false;
    return;
  }
  const uint32_t elapsed = now - cycleStartedAt;
  const uint32_t incubationElapsed =
      elapsed + static_cast<uint32_t>(startDayOffset) * CYCLE_DAY_SECONDS;
  lastCycleEpoch = max(lastCycleEpoch, now);
  const uint32_t calculatedDay =
      control_logic::cycleDayForElapsed(incubationElapsed, totalDays);
  if (control_logic::cycleComplete(incubationElapsed, totalDays)) {
    cycleDay = totalDays;
    queueIncubationEvent("batch_completed", now);
    queueIncubationBatchRecord("completed", now);
    cycleActive = false;
    lockdown = false;
    trayChangeRequired = false;
    trayChangeEventPending = false;
    controlPaused = false;
    pauseReason = "";
    trayTurning = false;
    pendingTurnStartedAt = 0;
    commandState = "completed";
    commandError = "";
    saveCycleState();
    reportedStatePending = true;
    setSafeOutputs();
    Serial.println("Incubation cycle completed.");
    return;
  }
  cycleDay = static_cast<uint8_t>(calculatedDay);
  lockdown = control_logic::lockdownForDay(cycleDay, lockdownDay, lockdown);
  trayChangeRequired = lockdown;
  if (lockdown && lastTrayChangeEventDay != lockdownDay) {
    lastTrayChangeEventDay = lockdownDay;
    trayChangeEventPending = true;
    reportedStatePending = true;
    saveCycleState();
  }
  if (lockdown) trayTurning = false;
  targetHumidity = lockdown ? hatchHumidity : normalHumidity;
}

void applyProductionOutputs() {
  if (manualCheckTarget && static_cast<int32_t>(millis() - manualCheckDeadline) >= 0) {
    manualCheckTarget = 0;
    Serial.println("MANUAL_CHECK: lease expired; production control resumed.");
    reportedStatePending = true;
  }
  if (cycleActive || calibrationActive) {
    circulationFan1 = true;
  }
  // Route checks through normal vent sequencing and environmental interlocks.
  if (manualCheckTarget == 1 && manualCheckOn && !lidOpen && !overheat &&
      sensorValid && !controlPaused && !exhaustFan) {
    humidifierOn = true;
  } else if (manualCheckTarget == 2 && manualCheckOn) {
    exhaustFan = true;
    productionExhaustPwm = max(productionExhaustPwm, minimumExhaustPwm());
    ventAngle = ventOpenAngle;
    humidifierOn = false;
  }
  writeHeaterPwm(heaterPwm);
  writeCirculationFan1(circulationFan1);
  digitalWrite(HUMIDIFIER_PIN, humidifierOn ? OUTPUT_ON : OUTPUT_OFF);
  trackHumidifierTransition();
  // Egg-tray servo replaces the old egg-turner relay (GPIO 26).
  // ON angle = tray-turning position; OFF angle = resting position.
  // Write only on change: re-writing the same pulse every tick keeps the
  // output stage excited and makes stalled servos buzz and draw current.
  const uint8_t desiredCircAngle = trayTurning ? circServoOpenAngle : circServoClosedAngle;
  if (desiredCircAngle != physicalCircServoAngle) {
    physicalCircServoAngle = desiredCircAngle;
    if (circServo.attached()) circServo.write(physicalCircServoAngle);
  }
  // ── Vent servo (GPIO 13) sequenced with the exhaust fan ──────────────────
  // Opening an overheat vent used to energize the servo and the 100%-duty
  // fan at the same instant; the combined inrush sagged the shared 5V rail,
  // brownout-reset the ESP (killing running cycles), and the servo only
  // buzzed against live airflow. The servo now cranks the vent open FIRST
  // and the fan is held off until it has finished travelling; on close the
  // fan is stopped first so the flap is never forced against pressure.
  const uint8_t exhaustRequest =
      exhaustFan
          ? static_cast<uint8_t>(productionExhaustPwm == 0 ? 255
                                                           : productionExhaustPwm)
          : 0;
  if (ventAngle != physicalServoAngle) {
    if (ventAngle == ventOpenAngle) {
      physicalServoAngle = ventOpenAngle;
      if (ventServo.attached()) ventServo.write(physicalServoAngle);
      ventServoBusyUntil = millis() + VENT_SERVO_TRAVEL_MS;
    } else if (exhaustRequest == 0 && appliedExhaustPwm == 0) {
      physicalServoAngle = ventClosedAngle;
      if (ventServo.attached()) ventServo.write(physicalServoAngle);
    }
  }
  const bool ventStillTraveling =
      ventAngle == ventOpenAngle && static_cast<int32_t>(millis() - ventServoBusyUntil) < 0;
  writeExhaustPwm(ventStillTraveling ? 0 : exhaustRequest);
}

// Starts (or releases) a 30-second manual actuator lease, or fires a one-shot
// egg-turn pulse. Returns nullptr on success, otherwise a refusal message.
// Both the authorized BLE command channel and the cloud desired command call
// this single core, and overheat protection always outranks the check.
const char *beginManualCheck(const String &target, bool on) {
  const unsigned long now = millis();
  if (on && !cycleActive && !calibrationActive) {
    return "Use hardware test mode while no cycle or calibration is active.";
  }
  if (target == "eggTurner") {
    if (trayTurning) return "A tray turn is already running.";
    if (lidOpen) return "Close the lid before moving the egg tray.";
    if (lockdown) return "Egg turning is paused during lockdown.";
    trayTurning = true;
    trayTurnStartedAt = now;
    pendingTurnStartedAt = rtcUnixNow();
    Serial.println("MANUAL_CHECK: egg-turner pulse started.");
    return nullptr;
  }
  if (target == "humidifier") {
    if (!on) {
      if (manualCheckTarget == 1) manualCheckTarget = 0;
      Serial.println("MANUAL_CHECK: humidifier check ended; production control resumed.");
      return nullptr;
    }
    manualCheckTarget = 1;
    manualCheckOn = true;
    manualCheckDeadline = now + MANUAL_CHECK_LEASE_MS;
    Serial.printf("MANUAL_CHECK: humidifier ON for %lu s\n",
                  MANUAL_CHECK_LEASE_MS / 1000UL);
    return nullptr;
  }
  if (target == "exhaust") {
    if (!on) {
      if (manualCheckTarget == 2) manualCheckTarget = 0;
      Serial.println("MANUAL_CHECK: exhaust check ended; production control resumed.");
      return nullptr;
    }
    manualCheckTarget = 2;
    manualCheckOn = true;
    manualCheckDeadline = now + MANUAL_CHECK_LEASE_MS;
    Serial.printf("MANUAL_CHECK: exhaust ON for %lu s\n",
                  MANUAL_CHECK_LEASE_MS / 1000UL);
    return nullptr;
  }
  return "Unknown check target.";
}

void resetResponseMonitor(ResponseMonitor &monitor) {
  monitor.active = false;
  monitor.startedAt = 0;
  monitor.baselineTemperature = NAN;
  monitor.baselineHeaterSurfaceTemperature = NAN;
  monitor.baselineHumidity = NAN;
}

void resetAllResponseMonitors() {
  resetResponseMonitor(heaterResponse);
  resetResponseMonitor(humidifierResponse);
  resetResponseMonitor(exhaustResponse);
}

void latchHardwareFault(const String &code) {
  if (hardwareFaultLatched) return;
  hardwareFaultLatched = true;
  hardwareFaultCode = code;
  hardwareFaultAt = rtcUnixNow();
  resetAllResponseMonitors();
  saveCycleState();
  reportedStatePending = true;
  Serial.printf("Latched actuator response fault: %s\n", code.c_str());
}

void clearHardwareFault() {
  hardwareFaultLatched = false;
  hardwareFaultCode = "";
  hardwareFaultAt = 0;
  heaterResponseVerified = false;
  heaterResponseSlow = false;
  humidifierResponseVerified = false;
  exhaustResponseVerified = false;
  exhaustResponseFailed = false;
  resetAllResponseMonitors();
  saveCycleState();
}

void beginResponseMonitor(ResponseMonitor &monitor, unsigned long now) {
  monitor.active = true;
  monitor.startedAt = now;
  monitor.baselineTemperature = measuredTemperature;
  monitor.baselineHeaterSurfaceTemperature = heaterSurfaceTemperature;
  monitor.baselineHumidity = measuredHumidity;
}

void updateActuatorResponseChecks(unsigned long now) {
  if (!cycleActive || !eggTemperatureValid || !isfinite(measuredTemperature) ||
      hardwareFaultLatched) {
    resetAllResponseMonitors();
    return;
  }

  const bool heaterTestable = heaterPwm >= 128 && !exhaustFan &&
                              measuredTemperature <= targetTemperature - 0.3F;
  if (heaterTestable) {
    if (!heaterResponse.active) beginResponseMonitor(heaterResponse, now);
    if (control_logic::heaterResponseDetected(
            measuredTemperature, heaterResponse.baselineTemperature,
            heaterSurfaceTemperature,
            heaterResponse.baselineHeaterSurfaceTemperature,
            heaterTemperatureValid)) {
      heaterResponseVerified = true;
      heaterResponseSlow = false;
      resetResponseMonitor(heaterResponse);
    } else if (now - heaterResponse.startedAt >= HEATER_RESPONSE_TIMEOUT_MS) {
      // A small heater in a large chamber may warm very slowly. Keep heating
      // and expose a warning instead of latching a shutdown. Independent egg-
      // zone and heater-surface overtemperature protections remain active.
      heaterResponseSlow = true;
      resetResponseMonitor(heaterResponse);
      Serial.println(
          "HEATER_SLOW_TEMPERATURE_RISE: warning only; heating continues.");
    }
  } else {
    resetResponseMonitor(heaterResponse);
  }

  const bool humidifierTestable = humidifierOn && !exhaustFan &&
                                  measuredHumidity <= targetHumidity - 2.0F;
  if (humidifierTestable) {
    if (!humidifierResponse.active) beginResponseMonitor(humidifierResponse, now);
    if (measuredHumidity - humidifierResponse.baselineHumidity >=
        HUMIDIFIER_MIN_RESPONSE_RH) {
      humidifierResponseVerified = true;
      resetResponseMonitor(humidifierResponse);
    } else if (now - humidifierResponse.startedAt >=
               HUMIDIFIER_RESPONSE_TIMEOUT_MS) {
      // Soft fault only — a non-responding humidifier is non-critical.
      // Do NOT halt the cycle with a hard latch; just flag it and reset the
      // monitor so telemetry shows "suspected" without blocking incubation.
      humidifierFaultSuspected = true;
      resetResponseMonitor(humidifierResponse);
      Serial.println("HUMIDIFIER_NO_HUMIDITY_RISE: soft fault flagged, cycle continues.");
    }
  } else {
    resetResponseMonitor(humidifierResponse);
  }

  const bool exhaustTestable =
      exhaustFan && measuredTemperature >=
                        targetTemperature + EXHAUST_TEMPERATURE_DELTA;
  if (exhaustTestable) {
    if (!exhaustResponse.active) beginResponseMonitor(exhaustResponse, now);
    const bool cooled =
        exhaustResponse.baselineTemperature - measuredTemperature >=
        EXHAUST_MIN_TEMPERATURE_DROP_C;
    if (cooled) {
      exhaustResponseVerified = true;
      exhaustResponseFailed = false;
      resetResponseMonitor(exhaustResponse);
    } else if (now - exhaustResponse.startedAt >=
               EXHAUST_RESPONSE_TIMEOUT_MS) {
      // Soft fault: an inoperative exhaust must never freeze the whole
      // incubator (temperature safety is covered by the overheat clamp).
      // Health is reported as "fault"; the next vent attempt re-verifies.
      exhaustResponseFailed = true;
      resetResponseMonitor(exhaustResponse);
      Serial.println("EXHAUST_NO_COOLING_RESPONSE (soft) — heating continues");
    }
  } else {
    resetResponseMonitor(exhaustResponse);
  }
}

const char *actuatorHealth(const String &faultPrefix, bool verified,
                           const ResponseMonitor &monitor) {
  if (hardwareFaultLatched && hardwareFaultCode.startsWith(faultPrefix)) {
    return "fault";
  }
  if (monitor.active) return "verifying";
  if (verified) return "responsive";
  return "unverified";
}

uint8_t updateMoistureReducer(unsigned long now, float humidityTarget) {
  const float excess = measuredHumidity - humidityTarget;
  const bool shouldRun = AUTOMATIC_MOISTURE_REDUCER_ENABLED &&
                         moistureReducerEnabled && isfinite(measuredHumidity) &&
                         (moistureReducerActive
                              ? excess > MOISTURE_REDUCTION_OFF_DELTA
                              : excess >= MOISTURE_REDUCTION_ON_DELTA);
  if (!shouldRun) {
    moistureReducerActive = false;
    moistureBaselineHumidity = NAN;
    moistureEvaluationStartedAt = 0;
    humidityTrendRhPerMinute = 0.0F;
    moistureAdaptiveBoostPwm = 0;
    return 0;
  }

  moistureReducerActive = true;
  if (!isfinite(moistureBaselineHumidity) || moistureEvaluationStartedAt == 0) {
    moistureBaselineHumidity = measuredHumidity;
    moistureEvaluationStartedAt = now;
  } else if (now - moistureEvaluationStartedAt >= MOISTURE_ADAPT_INTERVAL_MS) {
    const unsigned long elapsed = now - moistureEvaluationStartedAt;
    const float drop = moistureBaselineHumidity - measuredHumidity;
    humidityTrendRhPerMinute =
        (measuredHumidity - moistureBaselineHumidity) * 60000.0F / elapsed;
    if (adaptiveMoistureControlEnabled) {
      if (drop < MOISTURE_REQUIRED_DROP_RH) {
        moistureAdaptiveBoostPwm = static_cast<uint8_t>(
            min(96, moistureAdaptiveBoostPwm + MOISTURE_ADAPT_STEP_PWM));
      } else if (drop >= MOISTURE_REQUIRED_DROP_RH * 3.0F) {
        moistureAdaptiveBoostPwm =
            moistureAdaptiveBoostPwm > MOISTURE_ADAPT_STEP_PWM / 2
                ? moistureAdaptiveBoostPwm - MOISTURE_ADAPT_STEP_PWM / 2
                : 0;
      }
    }
    Serial.printf(
        "MOISTURE_ADAPT baseline=%.1f current=%.1f trend=%.3fRH/min boost=%u\n",
        moistureBaselineHumidity, measuredHumidity, humidityTrendRhPerMinute,
        moistureAdaptiveBoostPwm);
    moistureBaselineHumidity = measuredHumidity;
    moistureEvaluationStartedAt = now;
  }

  return control_logic::moistureExhaustPwm(
      excess, minimumExhaustPwm(), moistureAdaptiveBoostPwm);
}

void populateControlLog(JsonDocument &log, const String &event) {
  const uint32_t now = rtcUnixNow();
  log["event"] = event;
  log["firmware"] = FIRMWARE_BUILD_ID;
  log["capturedAt"] = now == 0 ? 0 : now * 1000ULL;
  log["uptimeMs"] = millis();
  log["task"] = calibrationActive ? "calibration"
                                    : (cycleActive ? "incubation" : "idle");
  log["calibrationSession"] = calibrationStartedAt;
  log["calibrationStatus"] = calibrationStatus;
  log["calibrationSamples"] = calibrationSamples;
  log["calibrationStableSamples"] = calibrationStableSamples;
  log["cycleMode"] = cycleMode;
  log["profileId"] = profileId;
  log["cycleDay"] = cycleDay;
  log["phase"] = lockdown ? "hatching" : "setting";
  log["temperature"] = measuredTemperature;
  log["humidity"] = measuredHumidity;
  log["targetTemperature"] = calibrationActive ? 37.5F : targetTemperature;
  log["targetHumidity"] = calibrationActive ? 60.0F : targetHumidity;
  log["rawPrimaryTemperature"] = lastRawDhtTemperature;
  log["rawPrimaryHumidity"] = lastRawDhtHumidity;
  log["rawEggDhtTemperature"] = lastRawEggDhtTemperature;
  log["rawEggDhtHumidity"] = lastRawEggDhtHumidity;
  log["rawEggTemperature"] = lastRawEggTemperature;
  log["rawHeaterTemperature"] = lastRawHeaterTemperature;
  log["primaryDhtDiagnostic"] = primaryDhtRecovery.diagnostic;
  log["primaryDhtReadStatus"] = primaryDhtRecovery.readStatus;
  log["primaryDhtReadError"] = dhtStatusName(primaryDhtRecovery.readStatus);
  log["primaryDhtUsingCachedData"] = primaryDhtUsingCachedData;
  log["primaryDhtFailures"] = primaryDhtRecovery.consecutiveFailures;
  log["primaryDhtRecoveryAttempts"] = primaryDhtRecovery.recoveryAttempts;
  log["primaryDhtDataLevel"] = primaryDhtRecovery.dataLevel;
  log["eggDhtDiagnostic"] = eggDhtRecovery.diagnostic;
  log["eggDhtReadStatus"] = eggDhtRecovery.readStatus;
  log["eggDhtReadError"] = dhtStatusName(eggDhtRecovery.readStatus);
  log["eggDhtUsingCachedData"] = eggDhtUsingCachedData;
  log["eggDhtFailures"] = eggDhtRecovery.consecutiveFailures;
  log["eggDhtRecoveryAttempts"] = eggDhtRecovery.recoveryAttempts;
  log["eggDhtDataLevel"] = eggDhtRecovery.dataLevel;
  log["temperatureOffset"] = calibrationTemperatureOffset;
  log["humidityOffset"] = calibrationHumidityOffset;
  log["sensorValid"] = sensorValid;
  log["humiditySensorsAgree"] = humiditySensorsAgree;
  log["heaterPwm"] = heaterPwm;
  // `heaterPwm` is the duty the controller ASKED for; `heaterPwmApplied` is what
  // actually reached GPIO 32 after the slew limiter and clamp. The app slider
  // shows the request, so this pair is the only ground truth for diagnosing a
  // heater that reports 100 % but stays cold.
  log["heaterPwmApplied"] = appliedHeaterDuty;
  log["heaterBlocked"] = heaterTestBlocker;
  log["circulationFan1"] = circulationFan1;
  log["circServoOpen"] = (physicalCircServoAngle != circServoClosedAngle);
  log["circServoAngle"] = physicalCircServoAngle;
  log["exhaustPwm"] = appliedExhaustPwm;
  log["humidifier"] = humidifierOn;
  log["ventAngle"] = physicalServoAngle;
  log["moistureReducerActive"] = moistureReducerActive;
  log["moistureAdaptiveBoostPwm"] = moistureAdaptiveBoostPwm;
  log["humidityTrendRhPerMinute"] = humidityTrendRhPerMinute;
  log["lidOpen"] = lidOpen;
  log["controlPaused"] = controlPaused;
  log["pauseReason"] = pauseReason;
  log["hardwareFault"] = hardwareFaultCode;
  // Room sensor (primary DHT22 moved outside).
  log["roomTemperature"] = roomTemperature;
  log["roomHumidity"] = roomHumidity;
  log["roomHumidityValid"] = roomHumidityValid;
  log["exhaustHelpsHumidity"] = exhaustHelpsHumidity;
}

void emitUsbControlLog(const String &event) {
  JsonDocument log;
  populateControlLog(log, event);
  Serial.print("INCUBATOR_LOG ");
  serializeJson(log, Serial);
  Serial.println();
}

void queueCalibrationLog(const String &event) {
  calibrationLogEvent = event;
  calibrationLogPending = true;
  emitUsbControlLog(event);
}

void serviceUsbControlLogging() {
  if (!cycleActive && !calibrationActive) return;
  const unsigned long now = millis();
  if (lastUsbControlLogAt != 0 &&
      now - lastUsbControlLogAt < USB_CONTROL_LOG_INTERVAL_MS) {
    return;
  }
  lastUsbControlLogAt = now;
  emitUsbControlLog("sample");
}

void runLocalController() {
  sampleEnvironment();
  const unsigned long monotonicNow = millis();
  const uint32_t clockNow = rtcUnixNow();
  updateCyclePhase(clockNow);
  const bool lidOpenNow = digitalRead(LID_SWITCH_PIN) == HIGH;
  if (lidOpenNow && !lidOpen) {
    lidOpenedAt = monotonicNow;
    lidClosedAt = 0;
    reportedStatePending = true;
  } else if (!lidOpenNow && lidOpen) {
    lidClosedAt = monotonicNow;
    reportedStatePending = true;
  }
  const bool lidOpenWas = lidOpen;  // captured before overwrite for edge detection
  lidOpen = lidOpenNow;
  lidOpenDurationMs = lidOpen && lidOpenedAt > 0 ? monotonicNow - lidOpenedAt : 0;
  lidWarning = lidOpenDurationMs >= LID_WARNING_MS;

  // Auto-clear any lid-caused fault the moment the lid is closed so the
  // incubation cycle resumes automatically without requiring the user to
  // acknowledge via the app.
  if (!lidOpenNow && lidOpenWas) {  // transition: open → closed
    if (hardwareFaultLatched &&
        (hardwareFaultCode == "LID_OPEN_TOO_LONG_DURING_SETTING" ||
         hardwareFaultCode == "LID_OPEN_TOO_LONG_DURING_HATCHING")) {
      clearHardwareFault();
      Serial.println("Lid closed: auto-cleared lid-open fault; resuming cycle.");
    }
    resetAllResponseMonitors();  // discard any stale actuator-response timers
  }
  controlPaused = false;
  pauseReason = "";

  // Manual hardware testing has priority while the incubator is otherwise
  // idle. Per-output safety is enforced inside applyTestOutputs(). Without
  // this early return, the lid handler below repeatedly forced GPIOs LOW and
  // caused visible blinking between Firebase polls.
  if (!cycleActive && !calibrationActive) {
    if (testEnabled)
      applyTestOutputs();
    else
      setSafeOutputs();
    lastControllerUpdate = monotonicNow;
    return;
  }

  if (lidOpen) {
    heaterIntegral = 0.0F;
    lastHeaterTemperature = NAN;
    lastHeaterErrorRate = 0.0F;
    lastHeaterControlSampleAt = 0;
    setSafeOutputs();
    circulationFan1 = cycleActive || calibrationActive;
    productionExhaustPwm = 0;
    resetAllResponseMonitors();  // prevent spurious heater-no-response fault
    applyProductionOutputs();
    controlPaused = true;
    pauseReason = "INCUBATOR_LID_OPEN";
    if (cycleActive && lidOpenDurationMs >= LID_FAULT_MS &&
        !hardwareFaultLatched) {
      latchHardwareFault(lockdown ? "LID_OPEN_TOO_LONG_DURING_HATCHING"
                                  : "LID_OPEN_TOO_LONG_DURING_SETTING");
    }
    return;
  }

  if (calibrationActive) {
    cycleActive = false;
    testEnabled = false;
    const bool requiredSensorsFresh =
        eggTemperatureValid && heaterTemperatureValid && sensorValid &&
        monotonicNow - eggSensorUpdatedAt <= SENSOR_STALE_MS &&
        monotonicNow - heaterSensorUpdatedAt <= SENSOR_STALE_MS &&
        monotonicNow - humiditySensorUpdatedAt <= DHT_SENSOR_STALE_MS;
    if (!requiredSensorsFresh || !isfinite(measuredTemperature) ||
        !isfinite(measuredHumidity) || clockNow == 0) {
      calibrationActive = false;
      calibrated = previousCalibrationWasCalibrated;
      calibrationTemperatureOffset = previousCalibrationTemperatureOffset;
      calibrationHumidityOffset = previousCalibrationHumidityOffset;
      calibrationStatus = "failed_sensor_or_rtc";
      queueCalibrationLog("failed_sensor_or_rtc");
      setSafeOutputs();
      saveCalibrationState();
      reportedStatePending = true;
    } else if (lastValidSensorRead != calibrationLastSampleAt) {
      calibrationLastSampleAt = lastValidSensorRead;
      ++calibrationSamples;
      const bool stable = fabsf(measuredTemperature - 37.5F) <= 0.5F &&
                          fabsf(measuredHumidity - 60.0F) <= 5.0F;
      if (stable) ++calibrationStableSamples;
      if (clockNow >= calibrationEndsAt) {
        calibrationActive = false;
        calibrated = control_logic::calibrationPassed(
            calibrationSamples, calibrationStableSamples, hardwareFaultLatched);
        calibrationStatus = calibrated ? "calibrated" : "failed_stability";
        if (!calibrated) {
          calibrationTemperatureOffset = previousCalibrationTemperatureOffset;
          calibrationHumidityOffset = previousCalibrationHumidityOffset;
          calibrated = previousCalibrationWasCalibrated;
        } else {
          previousCalibrationTemperatureOffset = calibrationTemperatureOffset;
          previousCalibrationHumidityOffset = calibrationHumidityOffset;
          previousCalibrationWasCalibrated = true;
        }
        queueCalibrationLog(calibrationStatus == "calibrated"
                                ? "completed"
                                : "failed_stability");
        saveCalibrationState();
        reportedStatePending = true;
      } else if (calibrationSamples % 30 == 0) {
        saveCalibrationState();
        reportedStatePending = true;
      }
    }
    if (calibrationActive) {
      circulationFan1 = true;
      const float temperatureError = 37.5F - measuredTemperature;
      heaterPwm = temperatureError <= 0.0F
                      ? 0
                      : static_cast<uint8_t>(constrain(90.0F * temperatureError,
                                                        0.0F, 255.0F));
      const bool temperatureVenting = measuredTemperature > 37.9F;
      exhaustFan = temperatureVenting;
      productionExhaustPwm = temperatureVenting ? 255 : 0;
      humidifierOn = !exhaustFan && measuredHumidity < 58.0F;
      if (measuredTemperature >= ABSOLUTE_OVERHEAT_C) {
        heaterPwm = 0;
        humidifierOn = false;
        exhaustFan = true;
        productionExhaustPwm = 255;
      }
      if (heaterSurfaceTemperature >= HEATER_SURFACE_MAX_C) {
        heaterPwm = 0;
        calibrationActive = false;
        calibrated = previousCalibrationWasCalibrated;
        calibrationStatus = "failed_heater_overtemperature";
        queueCalibrationLog("failed_heater_overtemperature");
        latchHardwareFault("HEATER_SURFACE_OVERTEMPERATURE");
        saveCalibrationState();
      } else if (heaterSurfaceTemperature > HEATER_SURFACE_DERATE_C) {
        const float ratio = (HEATER_SURFACE_MAX_C - heaterSurfaceTemperature) /
                            (HEATER_SURFACE_MAX_C - HEATER_SURFACE_DERATE_C);
        heaterPwm = min(
            heaterPwm,
            static_cast<uint8_t>(constrain(255.0F * ratio, 0.0F, 255.0F)));
      }
      ventAngle = exhaustFan ? ventOpenAngle : ventClosedAngle;
      trayTurning = false;
      applyProductionOutputs();
      return;
    }
  }

  if (!cycleActive) {
    setSafeOutputs();
    lastControllerUpdate = monotonicNow;
    return;
  }

  // Recover after lid closure through the normal warm-up controller below,
  // including fresh-sensor, fault, and overtemperature checks.

  testEnabled = false;
  const bool eggTemperatureFresh =
      eggTemperatureValid && eggSensorUpdatedAt > 0 &&
      monotonicNow - eggSensorUpdatedAt <= SENSOR_STALE_MS &&
      isfinite(measuredTemperature);
  const bool heaterTemperatureFresh =
      heaterTemperatureValid && heaterSensorUpdatedAt > 0 &&
      monotonicNow - heaterSensorUpdatedAt <= SENSOR_STALE_MS &&
      isfinite(heaterSurfaceTemperature);
  sensorFault = !eggTemperatureFresh || !heaterTemperatureFresh ||
                eggTemperatureDisagreement;
  controlPaused = sensorFault || rtcFault || hardwareFaultLatched;
  pauseReason = sensorFault
                     ? (!heaterTemperatureFresh
                            ? "HEATER_DS18B20_NOT_CONNECTED_OR_STALE"
                            : sensorFaultCode.isEmpty()
                            ? "EGG_DS18B20_NOT_CONNECTED_OR_STALE"
                            : sensorFaultCode)
                    : (rtcFault ? "RTC_NOT_CONNECTED_OR_INVALID"
                                : hardwareFaultCode);

  if (controlPaused) {
    humidifierOn = false;
    trayTurning = false;
    circulationFan1 = true;
    exhaustFan = false;
    productionExhaustPwm = 0;
    ventAngle = ventClosedAngle;
    heaterIntegral = 0;
    lastHeaterTemperature = NAN;
    lastHeaterErrorRate = 0.0F;
    lastHeaterControlSampleAt = 0;

    // Never heat blind or reboot into another blind-heating grace period.
    // Sensor recovery remains active and automatically resumes valid control.
    heaterPwm = 0;
    if ((eggTemperatureFresh && measuredTemperature >= ABSOLUTE_OVERHEAT_C) ||
        (heaterTemperatureFresh && heaterSurfaceTemperature >= HEATER_SURFACE_MAX_C)) {
      exhaustFan = true;
      productionExhaustPwm = 255;
      ventAngle = ventOpenAngle;
    }

    applyProductionOutputs();
    lastControllerUpdate = monotonicNow;
    return;
  }

  circulationFan1 = true;

  const float dt = lastControllerUpdate == 0
                       ? 0.1F
                       : min(2.0F, (monotonicNow - lastControllerUpdate) / 1000.0F);
  lastControllerUpdate = monotonicNow;
  const float error = targetTemperature - measuredTemperature;

  // Only update derivative when the DS18B20 provides a new sample. Calculating
  // it on every fast loop iteration amplified each 2-second sensor step and
  // made PWM fight the setpoint.
  float dError = lastHeaterErrorRate;
  if (eggSensorUpdatedAt != lastHeaterControlSampleAt) {
    if (lastHeaterControlSampleAt > 0 && isfinite(lastHeaterTemperature)) {
      const float sensorDt =
          max(0.1F, (eggSensorUpdatedAt - lastHeaterControlSampleAt) / 1000.0F);
      dError = (lastHeaterTemperature - measuredTemperature) / sensorDt;
    }
    lastHeaterTemperature = measuredTemperature;
    lastHeaterErrorRate = dError;
    lastHeaterControlSampleAt = eggSensorUpdatedAt;
  }

  const bool modelReady = heaterModelReady();
  const float temperatureRiseRate = lastHeaterErrorRate;
  const bool glideInApproach =
      error > HEATER_FULL_POWER_MARGIN_C &&
      error <= HEATER_APPROACH_ERROR_C && temperatureRiseRate >= 0.008F &&
      (error / temperatureRiseRate) <= HEATER_APPROACH_PREDICT_SECONDS;
  const bool fullPowerWarmup =
      error > HEATER_FULL_POWER_MARGIN_C && !glideInApproach;
  if (fullPowerWarmup && !modelReady) {
    // The first training cycle has no learned feed-forward yet. Preload the
    // near-target controller while full power handles the bulk warm-up.
    heaterIntegral = learnedIntegralPreload;
  }
  const float integralMinimum = modelReady ? -80.0F : 0.0F;
  const float integralMaximum = modelReady ? 80.0F : 150.0F;
  const float candidateIntegral = constrain(
      heaterIntegral + error * dt, integralMinimum, integralMaximum);
  const float feedForward = modelReady ? learnedHoldPwm : 0.0F;
  constexpr float KP = 25.0F;
  constexpr float KD = 200.0F;
  const float unconstrained =
      feedForward + KP * error + candidateIntegral + KD * dError;
  const bool closingOnTarget =
      error > 0.0F && lastHeaterErrorRate >= HEATER_INTEGRAL_RISE_FREEZE_RATE;
  if (!fullPowerWarmup && !closingOnTarget &&
      ((unconstrained > 0.0F && unconstrained < 255.0F) ||
       (unconstrained <= 0.0F && error > 0.0F) ||
       (unconstrained >= 255.0F && error < 0.0F))) {
    heaterIntegral = candidateIntegral;
  }
  heaterPwm = fullPowerWarmup
                  ? 255
                  : control_logic::adaptiveHeaterPwm(
                        learnedHoldPwm, modelReady, error, heaterIntegral,
                        dError);

  // ── Dual-sensor heater feedforward ───────────────────────────────────────
  // When the heater surface (DS18B20) is hot relative to its safe ceiling, we
  // pre-emptively back off the PWM *before* the surface derate kicks in. This
  // improves energy efficiency and reduces overshoot. The feedforward is a
  // gain multiplier: 1.0 when surface is cold, tapering to ~0.5 at derate
  // threshold. The existing hard surface derate + cutoff safety remains.
  if (!fullPowerWarmup && !modelReady && heaterTemperatureValid &&
      isfinite(heaterSurfaceTemperature)) {
    // Feedforward window: between 35°C and DERATE threshold (50°C).
    constexpr float FF_COLD_C = 35.0F;
    if (heaterSurfaceTemperature > FF_COLD_C &&
        heaterSurfaceTemperature < HEATER_SURFACE_DERATE_C) {
      const float ffRatio = 1.0F - 0.5F * (heaterSurfaceTemperature - FF_COLD_C) /
                                          (HEATER_SURFACE_DERATE_C - FF_COLD_C);
      heaterPwm = static_cast<uint8_t>(constrain(
          static_cast<float>(heaterPwm) * ffRatio, 0.0F, 255.0F));
    }
  }

  overheat = measuredTemperature >= ABSOLUTE_OVERHEAT_C ||
             measuredTemperature >= targetTemperature + OVERHEAT_DELTA;
  if (overheat) {
    heaterPwm = 0;
    // Integral is NOT zeroed here: the anti-windup guard above will bleed it
    // down naturally as the error turns negative, producing a smooth recovery.
  }
  if (heaterTemperatureValid) {
    if (heaterSurfaceTemperature >= HEATER_SURFACE_MAX_C) {
      heaterPwm = 0;
      overheat = true;
      if (!hardwareFaultLatched) {
        latchHardwareFault("HEATER_SURFACE_OVERTEMPERATURE");
      }
    } else if (heaterSurfaceTemperature > HEATER_SURFACE_DERATE_C) {
      const uint8_t maximumSafeDuty = control_logic::heaterSurfaceDutyLimit(
          heaterSurfaceTemperature, HEATER_SURFACE_DERATE_C,
          HEATER_SURFACE_MAX_C);
      heaterPwm = min(heaterPwm, maximumSafeDuty);
      heaterIntegral = min(heaterIntegral, static_cast<float>(maximumSafeDuty));
    }
  }

  // Record real time-to-target. A restored cycle can still learn hold PWM, but
  // does not invent a partial warm-up duration when its start timer is absent.
  if (!warmupLockRecorded && cycleActive &&
      measuredTemperature >= targetTemperature - HEATER_WARMUP_TARGET_MARGIN_C) {
    warmupLockRecorded = true;
    if (warmupStartedAt > 0) {
      const uint32_t observedWarmupSeconds =
          (monotonicNow - warmupStartedAt) / 1000UL;
      learnedWarmupSeconds = learnedWarmupSeconds == 0
                                 ? observedWarmupSeconds
                                 : (3U * learnedWarmupSeconds +
                                    observedWarmupSeconds) /
                                       4U;
      saveHeaterModel();
      Serial.printf(
          "Heater warm-up learned: %.2fC to %.2fC in %lus\n",
          warmupStartTemperature, measuredTemperature,
          static_cast<unsigned long>(observedWarmupSeconds));
    }
  }
  const bool temperatureVenting = control_logic::temperatureExhaustRequired(
      measuredTemperature, targetTemperature, exhaustFan, overheat,
      EXHAUST_TEMPERATURE_DELTA, EXHAUST_TEMPERATURE_OFF_DELTA);

  // ── Smart exhaust gate ──────────────────────────────────────────────────
  // Humidity reduction via exhaust only works when room air is DRIER than
  // incubator air. If room is wetter, opening the vent would push humid air
  // in and make things worse. Temperature-safety venting is always allowed.
  //
  // Gate bypassed (exhaust allowed) when:
  //   - Egg DHT failed and system is in degraded mode (room sensor is the
  //     fallback — comparing room vs incubator is meaningless).
  //   - Room sensor absent (fail-open, safer than locking humidity control).
  constexpr float EXHAUST_HUMIDITY_GATE_MARGIN_RH = 3.0F;
  exhaustHelpsHumidity =
      humidityDegraded ||                               // degraded → allow
      !roomHumidityValid ||                             // unknown  → allow
      !isfinite(measuredHumidity) ||                    // no inside reading
      (roomHumidity < measuredHumidity - EXHAUST_HUMIDITY_GATE_MARGIN_RH);

  // Humidity may be monitored during temperature venting, but must never
  // start or sustain exhaust independently of egg-temperature demand.
  if (temperatureVenting && exhaustHelpsHumidity && warmupLockRecorded) {
    updateMoistureReducer(monotonicNow, targetHumidity);
  } else {
    // Reset moisture reducer so adaptive boost doesn't ramp while blocked.
    if (moistureReducerActive) {
      moistureReducerActive = false;
      moistureBaselineHumidity = NAN;
      moistureEvaluationStartedAt = 0;
      humidityTrendRhPerMinute = 0.0F;
      moistureAdaptiveBoostPwm = 0;
    }
  }

  exhaustFan = temperatureVenting;
  productionExhaustPwm = temperatureVenting ? 255 : 0;
  if (temperatureVenting) {
    heaterPwm = 0;
  }
  ventAngle = exhaustFan ? ventOpenAngle : ventClosedAngle;

  const bool humidityControlAvailable =
      (eggDhtHumidityValid || primaryHumidityValid) &&
      isfinite(measuredHumidity);
  if (!humidityControlAvailable) {
    // Temperature control, circulation, and tray turning continue from the
    // egg DS18B20, but never run humidity equipment without humidity feedback.
    humidifierOn = false;
    moistureReducerActive = false;
    moistureBaselineHumidity = NAN;
    moistureEvaluationStartedAt = 0;
    humidityTrendRhPerMinute = 0.0F;
    moistureAdaptiveBoostPwm = 0;
  } else if (measuredHumidity <= targetHumidity - HUMIDITY_HYSTERESIS) {
    humidifierOn = true;
  } else if (measuredHumidity >= targetHumidity + HUMIDITY_HYSTERESIS) {
    humidifierOn = false;
  }
  if (exhaustFan) humidifierOn = false;

  // Learn the PWM that actually holds the target after two uninterrupted
  // minutes in a tight temperature band. One observation is stored per cycle;
  // later cycles gently adapt the model rather than replacing it outright.
  const bool safeStableHold =
      cycleActive && warmupLockRecorded && !holdPwmRecorded && !exhaustFan &&
      !overheat && fabsf(error) <= HEATER_HOLD_LEARN_BAND_C &&
      (!heaterTemperatureValid ||
       heaterSurfaceTemperature < HEATER_SURFACE_DERATE_C);
  if (safeStableHold) {
    if (holdStableStartedAt == 0) holdStableStartedAt = monotonicNow;
    if (lastHoldSampleAt == 0 ||
        monotonicNow - lastHoldSampleAt >= HEATER_HOLD_SAMPLE_MS) {
      holdPwmAccumulator += heaterPwm;
      ++holdPwmSamples;
      lastHoldSampleAt = monotonicNow;
    }
    if (monotonicNow - holdStableStartedAt >= HEATER_HOLD_LEARN_MS &&
        holdPwmSamples >= HEATER_HOLD_MIN_SAMPLES) {
      const float observedHoldPwm = constrain(
          holdPwmAccumulator / static_cast<float>(holdPwmSamples),
          HEATER_HOLD_PWM_MIN, HEATER_HOLD_PWM_MAX);
      learnedHoldPwm = heaterModelReady()
                           ? 0.7F * learnedHoldPwm + 0.3F * observedHoldPwm
                           : observedHoldPwm;
      if (heaterModelSamples < UINT16_MAX) ++heaterModelSamples;
      learnedIntegralPreload = constrain(
          learnedHoldPwm, HEATER_INTEGRAL_PRELOAD_MIN,
          HEATER_INTEGRAL_PRELOAD_MAX);
      // The observed PWM already contains the training integral. Start the
      // learned feed-forward controller with zero correction to avoid a jump.
      heaterIntegral = 0.0F;
      holdPwmRecorded = true;
      saveHeaterModel();
      Serial.printf(
          "Heater hold learned: observed=%.1f PWM model=%.1f (%u samples)\n",
          observedHoldPwm, learnedHoldPwm, heaterModelSamples);
    }
  } else if (!holdPwmRecorded) {
    holdStableStartedAt = 0;
    lastHoldSampleAt = 0;
    holdPwmAccumulator = 0;
    holdPwmSamples = 0;
  }

  if (trayTurning && trayTurnStartedAt > 0 &&
      monotonicNow - trayTurnStartedAt >= TURN_PULSE_SECONDS * 1000UL) {
    trayTurning = false;
    trayTurnStartedAt = 0;
    lastTurnAt = pendingTurnStartedAt;
    pendingTurnStartedAt = 0;
    queueIncubationEvent("tray_turn_completed", lastTurnAt);
    saveCycleState();
  }
  const uint32_t turnIntervalSeconds = static_cast<uint32_t>(turnIntervalHours) * 3600UL;
  if (!lockdown && !rtcFault && !trayTurning && clockNow > 0 &&
      clockNow - lastTurnAt >= turnIntervalSeconds) {
    trayTurning = true;
    trayTurnStartedAt = monotonicNow;
    pendingTurnStartedAt = clockNow;
  }

  updateActuatorResponseChecks(monotonicNow);
  if (hardwareFaultLatched) {
    controlPaused = true;
    pauseReason = hardwareFaultCode;
    heaterPwm = 0;
    humidifierOn = false;
    trayTurning = false;
    restCycleActive = false;  // cancel any active rest
    if (hardwareFaultCode.startsWith("EXHAUST")) {
      circulationFan1 = true;
      exhaustFan = false;
      productionExhaustPwm = 0;
      ventAngle = ventOpenAngle;
    } else {
      // Keep air moving even while paused: a stalled hot box stratifies and
      // holds surface heat longer. Matches setSafeOutputs() cycle behavior.
      circulationFan1 = true;
      exhaustFan = false;
      productionExhaustPwm = 0;
      ventAngle = ventClosedAngle;
    }
  }

  // ── Equipment rest cycle DISABLED ──────────────────────────────────────
  // The 30-minute rest break has been removed. Circulation fans now run at
  // 80 % PWM (CIRCULATION_FAN_PWM_DUTY) to protect 12 V fans from the 14 V
  // PSU without needing periodic power-downs.
  // restCycleActive is kept false for telemetry compatibility.
  restCycleActive = false;

  // ── Feature: Humidifier fault detection ──────────────────────────────────
  // If humidifier has been ON for HUMIDIFIER_FAULT_PROBE_MS without the
  // expected humidity rise, flag it as suspected faulty (telemetry + serial).
  // Humidity raising is humidifier-only: the exhaust port is never used to
  // pull in outside air, so opening the vent can never destabilise the
  // egg-zone temperature during a humidity shortfall.
  if (humidifierOn) {
    if (humidifierOnAt == 0) humidifierOnAt = monotonicNow;
  } else {
    humidifierOnAt = 0;
  }
  if (humidifierResponseVerified) humidifierFaultSuspected = false;
  if (humidifierOn && humidifierOnAt > 0 &&
      monotonicNow - humidifierOnAt >= HUMIDIFIER_FAULT_PROBE_MS &&
      !humidifierResponseVerified &&
      humidifierResponse.active &&
      isfinite(humidifierResponse.baselineHumidity) &&
      isfinite(measuredHumidity) &&
      measuredHumidity - humidifierResponse.baselineHumidity <
          HUMIDIFIER_FAULT_MIN_RISE_RH) {
    humidifierFaultSuspected = true;
    Serial.println(
        "HUMIDIFIER_FAULT_SUSPECTED: no RH rise after 1 min; "
        "humidifier-only control continues (no exhaust fallback).");
  }

  applyProductionOutputs();
}

bool recoverI2cBus() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  if (digitalRead(I2C_SDA_PIN) == LOW) {
    pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
    for (uint8_t pulse = 0; pulse < 9 && digitalRead(I2C_SDA_PIN) == LOW;
         ++pulse) {
      digitalWrite(I2C_SCL_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(I2C_SCL_PIN, LOW);
      delayMicroseconds(10);
    }
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  }
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(50);
  return digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH;
}

bool probeI2cAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

bool scanI2cBus() {
  Serial.println("Checking I2C bus on SDA GPIO 21 / SCL GPIO 22.");
  if (!recoverI2cBus()) {
    Serial.printf("I2C bus stuck: SDA=%d SCL=%d. Check RTC polarity/wiring.\n",
                  digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
    return false;
  }
  const bool rtcFound = probeI2cAddress(0x68);
  if (rtcFound) Serial.println("I2C device found at 0x68 (RTC).");
  if (!rtcFound) Serial.println("No RTC found on the I2C bus.");
  return true;
}

// BLE callbacks only authenticate or enqueue bounded work. Control state and
// Wi-Fi operations remain on the Arduino loop task.
class BleAuthCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic,
               NimBLEConnInfo &connInfo) override {
    const auto raw = characteristic->getValue();
    if (!connInfo.isEncrypted() || connInfo.getConnHandle() != bleHandle.load() ||
        millis() - bleChallengeAt > 30000 || raw.size() < 2 || raw.size() > 20 ||
        static_cast<uint8_t>(raw[0]) != bleProofLength ||
        bleProofLength + raw.size() - 1 > sizeof(bleProof)) return;
    memcpy(bleProof + bleProofLength, raw.data() + 1, raw.size() - 1);
    bleProofLength += raw.size() - 1;
    if (bleProofLength != sizeof(bleProof)) return;
    if (bleAuthAttempted && millis() - bleLastAuthAttempt < 5000) {
      memset(bleProof, 0, sizeof(bleProof));
      bleProofLength = 0;
      return;
    }
    bleAuthAttempted = true;
    bleLastAuthAttempt = millis();
    uint8_t expected[32]{};
    const int result = mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
        reinterpret_cast<const uint8_t *>(DEVICE_CLAIM_CODE), strlen(DEVICE_CLAIM_CODE),
        bleChallenge, sizeof(bleChallenge), expected);
    uint8_t difference = 0;
    for (size_t i = 0; i < sizeof(expected); ++i) difference |= expected[i] ^ bleProof[i];
    bleAuthorized = result == 0 && difference == 0;
    memset(expected, 0, sizeof(expected));
    memset(bleProof, 0, sizeof(bleProof));
    const uint8_t status = bleAuthorized ? 1 : 0;
    bleProofLength = 0;
    characteristic->setValue(&status, 1);
  }
};

class ProvisioningCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic,
               NimBLEConnInfo &connInfo) override {
    const auto raw = characteristic->getValue();
    if (bleRequests == nullptr || !bleAuthorized || connInfo.getConnHandle() != bleHandle.load() ||
        !connInfo.isEncrypted() || raw.size() == 0 || raw.size() >= sizeof(BleRequest::json)) return;
    BleRequest request{};
    request.session = bleSession;
    memcpy(request.json, raw.data(), raw.size());
    xQueueSend(bleRequests, &request, 0);
  }
};

void processBleRequest() {
    if (bleRequests == nullptr) return;
    BleRequest request{};
    if (xQueueReceive(bleRequests, &request, 0) != pdTRUE) return;
    if (!bleAuthorized || request.session != bleSession.load()) return;
    JsonDocument command;
    const auto error = deserializeJson(command, request.json);
    memset(request.json, 0, sizeof(request.json));
    if (error) {
      setBleStatus("{\"state\":\"error\",\"message\":\"Invalid request\"}");
      return;
    }

    const String action = command["action"] | "";
    if (action == "info") {
      JsonDocument response;
      response["state"] = "ready";
      response["deviceId"] = FIREBASE_DEVICE_ID;
      String json;
      serializeJson(response, json);
      setBleStatus(json);
      return;
    }

    if (action == "status") {
      lastBleSnapshotAt = 0;
      return;
    }

    if (action == "wifi") {
      const String ssid = command["ssid"] | "";
      const String password = command["password"] | "";
      if (ssid.isEmpty()) {
        setBleStatus("{\"state\":\"error\",\"message\":\"SSID required\"}");
        return;
      }
      Serial.printf("BLE: Wi-Fi network changed to \"%s\" during %s.\n",
                    ssid.c_str(), cycleActive ? "active cycle" : "idle");
      saveWifi(ssid, password);
      setBleStatus("{\"state\":\"connecting\"}");
      WiFi.mode(WIFI_STA);
      WiFi.disconnect(true);
      WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
      wifiProvisioningStarted = millis();
      wifiProvisioningPending = true;
      return;
    }

    if (action == "scanWifi") {
      if (wifiProvisioningPending) {
        setBleStatus(
            "{\"state\":\"error\",\"message\":\"Wi-Fi connection is already in progress.\"}");
        return;
      }
      // Allowed during cycles: GPIO/PWM outputs latch, and the loop restores
      // control (including test/heater re-apply) right after the scan.
      Serial.println("BLE: Wi-Fi scan requested (cycle-safe).");
      wifiScanPending = true;
      setBleStatus("{\"state\":\"wifiScanStarted\"}");
      return;
    }

    if (action == "manualCheck") {
      const String target = command["target"] | "";
      const bool on = command["on"] | false;
      String responseJson;
      if (const char *refuse = beginManualCheck(target, on)) {
        JsonDocument response;
        response["state"] = "error";
        response["message"] = refuse;
        serializeJson(response, responseJson);
      } else {
        JsonDocument response;
        response["state"] = "check";
        response["target"] = target;
        response["on"] = on;
        response["leaseSeconds"] = target == "eggTurner" ? 0 : 30;
        serializeJson(response, responseJson);
      }
      lastBleSnapshotAt = 0;
      setBleStatus(responseJson);
      return;
    }

    if (action == "acknowledgeFault") {
      if (!hardwareFaultLatched) {
        setBleStatus(
            "{\"state\":\"faultAck\",\"cleared\":false,\"message\":\"No latched fault.\"}");
        return;
      }
      // A heater-surface latch may only be acknowledged offline once the
      // surface is provably cool; every other latched code is self-rearming
      // and clears immediately.
      if (hardwareFaultCode.startsWith("HEATER_SURFACE") &&
          (!heaterTemperatureValid ||
           heaterSurfaceTemperature >= HEATER_SURFACE_DERATE_C)) {
        setBleStatus(
            "{\"state\":\"error\",\"message\":\"Heater surface must be cool with a valid sensor before acknowledge.\"}");
        return;
      }
      clearHardwareFault();
      reportedStatePending = true;
      Serial.println("BLE: hardware fault acknowledged by authorized client.");
      setBleStatus("{\"state\":\"faultAck\",\"cleared\":true}");
      return;
    }

    setBleStatus("{\"state\":\"error\",\"message\":\"Unknown request\"}");
}

class BleMonitorCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &connInfo,
                   uint16_t value) override {
    if (connInfo.getConnHandle() == bleHandle.load())
      bleMonitorSubscribed = (value & 1) != 0;
  }
};

class ProvisioningServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    bleAuthorized = false;
    bleMonitorSubscribed = false;
    ++bleSession;
    bleHandle = connInfo.getConnHandle();
    bleProofLength = 0;
    memset(bleProof, 0, sizeof(bleProof));
    esp_fill_random(bleChallenge, sizeof(bleChallenge));
    bleChallengeAt = millis();
    bleAuth->setValue(bleChallenge, sizeof(bleChallenge));
    bleClientConnected = true;
    Serial.println("Bluetooth client connected.");
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo,
                    int reason) override {
    bleClientConnected = false;
    bleAuthorized = false;
    bleMonitorSubscribed = false;
    ++bleSession;
    bleHandle = BLE_HS_CONN_HANDLE_NONE;
    bleMonitor->setValue("");
    Serial.printf("Bluetooth client disconnected, reason: %d\n", reason);
    // Always re-advertise (owned or not) so the app can reconnect from
    // Settings at any time, not only during first-time provisioning.
    if (!factoryResetInProgress) server->startAdvertising();
  }
};

void startProvisioning() {
  if (provisioningActive) return;
  bleRequests = xQueueCreate(4, sizeof(BleRequest));
  if (bleRequests == nullptr) {
    Serial.println("Bluetooth provisioning unavailable: request queue allocation failed.");
    return;
  }
  provisioningActive = true;
  NimBLEDevice::init("INCUBATOR_PRO_01");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, true);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ProvisioningServerCallbacks());
  NimBLEService *service = server->createService(BLE_SERVICE_UUID);
  NimBLECharacteristic *command = service->createCharacteristic(
      BLE_COMMAND_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
  bleStatus = service->createCharacteristic(
      BLE_STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  bleMonitor = service->createCharacteristic(BLE_MONITOR_UUID, NIMBLE_PROPERTY::NOTIFY);
  bleMonitor->setCallbacks(new BleMonitorCallbacks());
  bleAuth = service->createCharacteristic(BLE_AUTH_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
      NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC);
  bleAuth->setCallbacks(new BleAuthCallbacks());
  command->setCallbacks(new ProvisioningCallbacks());
  bleStatus->setValue("{\"state\":\"ready\"}");
  server->start();  // NimBLE 2.x: must call server->start() to initialize GATT database
  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->setName("INCUBATOR_PRO_01");
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->enableScanResponse(true);
  advertising->start();
  Serial.println("Bluetooth provisioning ready: INCUBATOR_PRO_01");
}

// Same live telemetry as Firebase, without cloud timestamps or credentials.
String buildBleSnapshotJson() {
  JsonDocument doc;
  fillProductionTelemetry(doc);
  doc["state"] = "status";
  doc["firmwareBuildId"] = FIRMWARE_BUILD_ID;
  doc["deviceId"] = FIREBASE_DEVICE_ID;
  doc["updatedAt"] = rtcUnixNow();
  doc["uptimeMs"] = millis();
  const bool wifiUp = WiFi.status() == WL_CONNECTED;
  doc["wifiConnected"] = wifiUp;
  doc["ssid"] = wifiSsid;
  if (wifiUp) {
    doc["localIp"] = WiFi.localIP().toString();
    doc["wifiRssi"] = WiFi.RSSI();
  }
  doc["testSessionId"] = testSessionId;
  const unsigned long checkRemaining =
      manualCheckTarget && static_cast<int32_t>(millis() - manualCheckDeadline) < 0
          ? manualCheckDeadline - millis()
          : 0;
  doc["manualCheckTarget"] =
      manualCheckTarget == 1   ? "humidifier"
      : manualCheckTarget == 2 ? "exhaust"
                               : "";
  doc["manualCheckOn"] = checkRemaining > 0 && manualCheckOn;
  doc["manualCheckRemainingMs"] = checkRemaining;
  doc["trayTurning"] = trayTurning;
  doc["testEnabled"] = testEnabled;
  doc["turnIntervalHours"] = turnIntervalHours;
  doc["cycleStartedAtMs"] = cycleStartedAt * 1000ULL;
  doc["bleProtocolVersion"] = 31;
  String json;
  serializeJson(doc, json);
  return json;
}

void serviceBleSnapshot() {
  if (!bleClientConnected || !bleAuthorized || bleMonitor == nullptr ||
      !bleMonitorSubscribed) return;
  if (millis() - lastBleSnapshotAt < BLE_SNAPSHOT_INTERVAL_MS) return;
  const uint32_t session = bleSession;
  const uint16_t handle = bleHandle;
  const String json = buildBleSnapshotJson();
  static uint16_t sequence = 0;
  ++sequence;
  // 8-byte header: magic, version, sequence, byte offset, total (u16 LE).
  // A minimum-MTU notification is 20 bytes, leaving 12 bytes of payload.
  const size_t total = json.length();
  if (total == 0 || total > 16384) return;
  const uint16_t mtu = NimBLEDevice::getServer()->getPeerInfo(handle).getMTU();
  const size_t capacity = min(static_cast<size_t>(mtu >= 23 ? mtu - 3 : 20), size_t(244));
  uint8_t frame[244];
  const unsigned long started = millis();
  for (size_t offset = 0; offset < total;) {
    if (!bleAuthorized || !bleMonitorSubscribed || session != bleSession.load() || millis() - started > 20000) break;
    const size_t count = min(capacity - 8, total - offset);
    frame[0] = 0xB1; frame[1] = 1;
    frame[2] = sequence & 255; frame[3] = sequence >> 8;
    frame[4] = offset & 255; frame[5] = offset >> 8;
    frame[6] = total & 255; frame[7] = total >> 8;
    memcpy(frame + 8, json.c_str() + offset, count);
    if (!bleMonitor->notify(frame, count + 8, handle)) break;
    offset += count;
    // Pace the host queue without starving safety work at MTU 23.
    runLocalController();
    delay(15);
  }
  lastBleSnapshotAt = millis();
}

bool hasFirebaseConfiguration() {
  return String(FIREBASE_API_KEY) != "YOUR_FIREBASE_WEB_API_KEY" &&
         !String(FIREBASE_DATABASE_URL).startsWith("https://YOUR_PROJECT") &&
         String(FIREBASE_DEVICE_PASSWORD) != "YOUR_DEVICE_ACCOUNT_PASSWORD";
}

String firebaseUrl(const char *child) {
  String url = FIREBASE_DATABASE_URL;
  while (url.endsWith("/")) {
    url.remove(url.length() - 1);
  }
  if (child != nullptr && child[0] == '/') {
    url += child;
  } else {
    url += "/devices/";
    url += FIREBASE_DEVICE_ID;
    if (child != nullptr && child[0] != '\0') {
      url += '/';
      url += child;
    }
  }
  url += ".json?auth=";
  url += idToken;
  return url;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  WiFi.mode(WIFI_STA);
  if (wifiSsid.isEmpty()) {
    startProvisioning();
    return;
  }
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  Serial.printf("Connecting to Wi-Fi: %s\n", wifiSsid.c_str());
  const unsigned long started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 20000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi connected: %s, RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    setBleStatus("{\"state\":\"connected\",\"ip\":\"" +
                 WiFi.localIP().toString() + "\",\"deviceId\":\"" +
                 FIREBASE_DEVICE_ID + "\"}");
  } else {
    Serial.println("Wi-Fi connection failed. ESP32 requires 2.4 GHz Wi-Fi.");
  }
}

bool authenticateFirebase() {
  if (!hasFirebaseConfiguration() || WiFi.status() != WL_CONNECTED) {
    return false;
  }
  const int64_t systemEpoch = static_cast<int64_t>(time(nullptr));
  if (!softwareClockSynced || systemEpoch < RTC_MIN_VALID_EPOCH ||
      systemEpoch > RTC_MAX_VALID_EPOCH) {
    return false;
  }

  HTTPClient http;
  String url = "https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=";
  url += FIREBASE_API_KEY;
  if (!http.begin(tlsClient, url)) {
    return false;
  }
  http.setTimeout(5000);  // TLS handshake can take 2-4s on reconnect; 1500ms was too short
  http.addHeader("Content-Type", "application/json");

  JsonDocument request;
  request["email"] = FIREBASE_DEVICE_EMAIL;
  request["password"] = FIREBASE_DEVICE_PASSWORD;
  request["returnSecureToken"] = true;
  String body;
  serializeJson(request, body);

  const int status = http.POST(body);
  const String response = http.getString();
  http.end();
  tlsClient.stop();
  if (status != HTTP_CODE_OK) {
    Serial.printf("Firebase authentication failed (%d): %s\n", status,
                  response.c_str());
    return false;
  }

  JsonDocument result;
  if (deserializeJson(result, response)) {
    Serial.println("Firebase authentication response was invalid.");
    return false;
  }

  idToken = result["idToken"].as<String>();
  const String localId = result["localId"].as<String>();
  const unsigned long expiresSeconds =
      String(result["expiresIn"].as<const char *>()).toInt();
  tokenExpiresAt = millis() + expiresSeconds * 1000UL;
  Serial.printf("Firebase authentication ready. UID: %s\n", localId.c_str());
  return !idToken.isEmpty();
}

bool ensureFirebaseToken() {
  if (idToken.isEmpty()) {
    return authenticateFirebase();
  }
  const long remaining = static_cast<long>(tokenExpiresAt - millis());
  if (remaining <= static_cast<long>(TOKEN_REFRESH_MARGIN_MS)) {
    idToken.clear();
    return authenticateFirebase();
  }
  return true;
}

int firebaseRequest(const char *method, const char *path, const String &body,
                    String &response) {
  if (!ensureFirebaseToken()) {
    return -1;
  }

  HTTPClient http;
  if (!http.begin(tlsClient, firebaseUrl(path))) {
    return -1;
  }
  http.setTimeout(1500);
  // Keep-alive: reuse the TLS session across requests. A fresh handshake per
  // request allocates ~40 KB of mbedTLS heap continuously, and the resulting
  // fragmentation reboots the ESP after hours of running.
  http.setReuse(true);
  http.addHeader("Content-Type", "application/json");

  int status = -1;
  if (strcmp(method, "GET") == 0) {
    status = http.GET();
  } else if (strcmp(method, "PATCH") == 0) {
    status = http.sendRequest("PATCH", body);
  } else if (strcmp(method, "PUT") == 0) {
    status = http.PUT(body);
  } else if (strcmp(method, "POST") == 0) {
    status = http.POST(body);
  } else if (strcmp(method, "DELETE") == 0) {
    status = http.sendRequest("DELETE");
  }
  response = http.getString();
  http.end();

  if (status == HTTP_CODE_UNAUTHORIZED) {
    idToken.clear();
  }
  return status;
}

void publishCalibrationLog() {
  if (WiFi.status() != WL_CONNECTED || calibrationStartedAt == 0) return;
  const String event = calibrationLogPending ? calibrationLogEvent : "sample";
  JsonDocument log;
  populateControlLog(log, event);
  log["serverAt"][".sv"] = "timestamp";
  String body;
  serializeJson(log, body);

  const uint32_t now = rtcUnixNow();
  String path = "/calibrationLogs/";
  path += FIREBASE_DEVICE_ID;
  path += '/';
  path += String(calibrationStartedAt);
  path += '-';
  path += String(now);
  path += '-';
  path += String(millis());
  String response;
  const int status = firebaseRequest("PUT", path.c_str(), body, response);
  if (status >= 200 && status < 300) {
    calibrationLogPending = false;
    calibrationLogEvent = "sample";
    lastCalibrationCloudLogAt = millis();
  } else {
    Serial.printf("Calibration log upload failed (%d): %s\n", status,
                  response.c_str());
  }
}

void queueIncubationEvent(const char *type, uint32_t capturedAt) {
  if (incubationBatchId.isEmpty()) return;
  if (capturedAt == 0) capturedAt = rtcUnixNow();
  if (capturedAt == 0) return;

  if (incubationEventCount == INCUBATION_EVENT_QUEUE_SIZE) {
    incubationEventHead =
        (incubationEventHead + 1) % INCUBATION_EVENT_QUEUE_SIZE;
    --incubationEventCount;
  }
  const uint8_t index =
      (incubationEventHead + incubationEventCount) % INCUBATION_EVENT_QUEUE_SIZE;
  IncubationEvent &event = incubationEvents[index];
  event.type = type;
  event.capturedAt = capturedAt;
  event.sequence = ++incubationEventSequence;
  event.day = cycleDay;
  event.temperature = measuredTemperature;
  event.humidity = measuredHumidity;
  event.humidifier = humidifierOn;
  ++incubationEventCount;
}

void queueIncubationBatchRecord(const char *status, uint32_t endedAt) {
  if (incubationBatchId.isEmpty()) return;
  incubationBatchStatus = status;
  incubationBatchEndedAt = endedAt;
  incubationBatchRecordPending = true;
}

void trackHumidifierTransition() {
  if (!cycleActive || incubationBatchId.isEmpty()) {
    loggedHumidifierState = humidifierOn;
    return;
  }
  if (humidifierOn == loggedHumidifierState) return;
  loggedHumidifierState = humidifierOn;
  queueIncubationEvent(humidifierOn ? "humidifier_on" : "humidifier_off");
}

void queueHourlyIncubationLog() {
  if (!cycleActive || incubationBatchId.isEmpty()) return;
  const uint32_t now = rtcUnixNow();
  if (now == 0 || now - lastIncubationHourlyLogAt <
                      INCUBATION_LOG_INTERVAL_SECONDS) {
    return;
  }
  lastIncubationHourlyLogAt = now;
  queueIncubationEvent("hourly_reading", now);
}

void publishIncubationHistory() {
  // Guard: deviceUid must be confirmed in the Firebase DB first.
  // The incubation history write rule checks devices/{id}/deviceUid, so if
  // publishTelemetry hasn't run yet this will always fail with 401.
  if (!deviceUidConfirmed || WiFi.status() != WL_CONNECTED || incubationBatchId.isEmpty()) return;

  String path = "/incubationHistory/";
  path += FIREBASE_DEVICE_ID;
  path += '/';
  path += incubationBatchId;

  JsonDocument document;
  if (incubationBatchRecordPending) {
    document["batchId"] = incubationBatchId;
    document["firmware"] = FIRMWARE_BUILD_ID;
    document["status"] = incubationBatchStatus;
    document["mode"] = cycleMode;
    document["profileId"] = profileId;
    document["startedAt"] = cycleStartedAt * 1000ULL;
    document["startDayOffset"] = startDayOffset;
    document["totalDays"] = totalDays;
    document["lockdownDay"] = lockdownDay;
    document["turnIntervalHours"] = turnIntervalHours;
    document["targetTemperature"] = targetTemperature;
    document["normalHumidity"] = normalHumidity;
    document["hatchHumidity"] = hatchHumidity;
    document["finalCycleDay"] = cycleDay;
    if (incubationBatchEndedAt > 0) {
      document["endedAt"] = incubationBatchEndedAt * 1000ULL;
    }
    document["serverAt"][".sv"] = "timestamp";
    path += "/record";
  } else if (incubationEventCount > 0) {
    const IncubationEvent &event = incubationEvents[incubationEventHead];
    document["event"] = event.type;
    document["capturedAt"] = event.capturedAt * 1000ULL;
    document["cycleDay"] = event.day;
    if (isfinite(event.temperature))
      document["temperature"] = event.temperature;
    else
      document["temperature"] = nullptr;
    if (isfinite(event.humidity))
      document["humidity"] = event.humidity;
    else
      document["humidity"] = nullptr;
    document["humidifier"] = event.humidifier;
    document["serverAt"][".sv"] = "timestamp";
    path += "/events/";
    if (event.type == "hourly_reading") {
      path += "hour-";
      path += String(event.capturedAt / INCUBATION_LOG_INTERVAL_SECONDS);
    } else {
      path += String(event.capturedAt);
      path += '-';
      path += String(event.sequence);
    }
  } else {
    return;
  }

  String body;
  serializeJson(document, body);
  String response;
  const int status = firebaseRequest("PUT", path.c_str(), body, response);
  if (status >= 200 && status < 300) {
    if (incubationBatchRecordPending) {
      incubationBatchRecordPending = false;
      saveCycleState();
    } else {
      incubationEvents[incubationEventHead] = IncubationEvent();
      incubationEventHead =
          (incubationEventHead + 1) % INCUBATION_EVENT_QUEUE_SIZE;
      --incubationEventCount;
    }
  } else {
    Serial.printf("Incubation history upload failed (%d): %s\n", status,
                  response.c_str());
  }
}

void publishTelemetry() {
  if (millis() - lastTelemetryWrite < STATUS_INTERVAL_MS ||
      WiFi.status() != WL_CONNECTED) {
    return;
  }
  lastTelemetryWrite = millis();

  JsonDocument update;
  update["status"]["online"] = true;
  update["status"]["lastSeen"][".sv"] = "timestamp";
  // Expose the boot-random test session so the app can start/renew hardware
  // tests after an ESP reboot; testTelemetry alone cannot bootstrap it.
  update["status"]["testSessionId"] = testSessionId;
  update["status"]["firmwareVersion"] = FIRMWARE_BUILD_ID;
  update["status"]["localIp"] = WiFi.localIP().toString();
  update["status"]["wifiRssi"] = WiFi.RSSI();
  update["deviceUid"] = "QLY2SB9qvleL6jhw02SgsOKSLUu1";
  update["claimCode"] = DEVICE_CLAIM_CODE;

  String body;
  serializeJson(update, body);
  String response;
  const int status = firebaseRequest("PATCH", "", body, response);
  if (status < 200 || status >= 300) {
    // Auth failure (401) means idToken is already cleared by firebaseRequest.
    // The next call will re-authenticate. Log clearly so it's visible in serial.
    Serial.printf("[OFFLINE] Firebase status write failed (HTTP %d) — device will show offline in app until this succeeds. Response: %s\n",
                  status, response.c_str());
    lastTelemetryWrite = 0;  // retry immediately on next loop tick
  } else {
    deviceUidConfirmed = true;
    if (testTelemetryCleanupPending) {
      testTelemetryCleanupPending = false;
      String cleanupResponse;
      const int cleanupStatus =
          firebaseRequest("DELETE", "testTelemetry", "", cleanupResponse);
      if (cleanupStatus < 200 || cleanupStatus >= 300) {
        Serial.printf("Stale testTelemetry cleanup failed (%d): %s\n",
                      cleanupStatus, cleanupResponse.c_str());
      }
    }
    Serial.println("[ONLINE] Firebase status.online = true written successfully.");
  }
}

void publishReportedState() {
  lastReportedAttempt = millis();
  JsonDocument reported;
  reported["version"] = commandVersion;
  reported["commandState"] = commandState;
  reported["error"] = commandError;
  reported["cycleActive"] = cycleActive;
  reported["cycleResumeRequired"] = false;  // always false — auto-resume is enabled
  if (powerRestoreEventPending) {
    reported["powerRestoreEvent"] = true;
    powerRestoreEventPending = false;
  }
  reported["mode"] = cycleMode;
  reported["profileId"] = profileId;
  reported["targetTemperature"] = targetTemperature;
  reported["targetHumidity"] = targetHumidity;
  reported["normalHumidity"] = normalHumidity;
  reported["hatchHumidity"] = hatchHumidity;
  reported["totalDays"] = totalDays;
  reported["startDayOffset"] = startDayOffset;
  reported["batchId"] = incubationBatchId;
  reported["lockdownDay"] = lockdownDay;
  reported["phase"] = lockdown ? "hatching" : "setting";
  reported["trayChangeRequired"] = trayChangeRequired;
  reported["trayChangeEventPending"] = trayChangeEventPending;
  reported["lidOpen"] = lidOpen;
  reported["lidWarning"] = lidWarning;
  reported["lidOpenSeconds"] = lidOpenDurationMs / 1000UL;
  reported["lidMessage"] = lidOpen
                                ? (lockdown ? "Close lid now. Opening during hatching can reduce hatch success."
                                            : "Close lid to restore temperature and humidity.")
                                : "closed";
  reported["turnIntervalHours"] = turnIntervalHours;
  reported["cycleStartedAt"] = cycleStartedAt;
  reported["lastTurnAt"] = lastTurnAt == 0 ? 0 : lastTurnAt * 1000ULL;
  reported["controlPaused"] = controlPaused;
  reported["pauseReason"] = pauseReason;
  reported["hardwareFaultLatched"] = hardwareFaultLatched;
  reported["hardwareFaultCode"] = hardwareFaultCode;
  reported["calibrated"] = calibrated;
  reported["calibrationActive"] = calibrationActive;
  reported["calibrationStatus"] = calibrationStatus;
  reported["calibrationDurationHours"] = calibrationDurationHours;
  reported["calibrationStartedAt"] = calibrationStartedAt * 1000ULL;
  reported["calibrationEndsAt"] = calibrationEndsAt * 1000ULL;
  reported["calibrationSamples"] = calibrationSamples;
  reported["calibrationStableSamples"] = calibrationStableSamples;
  reported["calibrationTemperatureOffset"] = calibrationTemperatureOffset;
  reported["calibrationHumidityOffset"] = calibrationHumidityOffset;
  reported["moistureReducerEnabled"] =
      AUTOMATIC_MOISTURE_REDUCER_ENABLED && moistureReducerEnabled;
  reported["adaptiveMoistureControlEnabled"] =
      adaptiveMoistureControlEnabled;
  reported["exhaustMinimumPercent"] = exhaustMinimumPercent;
  reported["updatedAt"][".sv"] = "timestamp";

  String body;
  serializeJson(reported, body);
  String response;
  const int status = firebaseRequest("PUT", "reported", body, response);
  if (status < 200 || status >= 300) {
    Serial.printf("Firebase reported-state failed (%d): %s\n", status,
                  response.c_str());
  } else {
    reportedStatePending = false;
  }
}

void rejectCommand(uint64_t requestedVersion, const String &message) {
  commandVersion = requestedVersion;
  commandState = "rejected";
  commandError = message;
  saveCycleState();
  reportedStatePending = true;
  Serial.printf("Rejected command: %s\n", message.c_str());
}

void readDesiredState() {
  if (millis() - lastCommandRead < COMMAND_INTERVAL_MS ||
      WiFi.status() != WL_CONNECTED) {
    return;
  }
  lastCommandRead = millis();

  String response;
  const int status = firebaseRequest("GET", "desired", "", response);
  if (status == HTTP_CODE_NOT_FOUND || response == "null") {
    return;
  }
  if (status != HTTP_CODE_OK) {
    Serial.printf("Firebase command read failed (%d): %s\n", status,
                  response.c_str());
    return;
  }

  JsonDocument desired;
  if (deserializeJson(desired, response)) {
    Serial.println("Firebase command JSON was invalid.");
    return;
  }

  const uint64_t requestedVersion = desired["version"] | 0ULL;
  if (requestedVersion == 0 || requestedVersion <= commandVersion) {
    return;
  }

  const String action = desired["action"] | "";
  if (action == "updateControlSettings") {
    const JsonObject settings = desired["controlSettings"].as<JsonObject>();
    const int requestedMinimum =
        settings["exhaustMinimumPercent"] | exhaustMinimumPercent;
    if (settings.isNull() || requestedMinimum < 50 || requestedMinimum > 80) {
      rejectCommand(requestedVersion, "INVALID_CONTROL_SETTINGS");
      return;
    }
    moistureReducerEnabled =
        settings["moistureReducerEnabled"] | moistureReducerEnabled;
    adaptiveMoistureControlEnabled =
        settings["adaptiveMoistureControlEnabled"] |
        adaptiveMoistureControlEnabled;
    exhaustMinimumPercent = static_cast<uint8_t>(requestedMinimum);
    moistureReducerActive = false;
    moistureAdaptiveBoostPwm = 0;
    saveControlSettings();
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    reportedStatePending = true;
    Serial.printf(
        "Control settings saved: moisture=%s adaptive=%s exhaustMin=%u%%\n",
        moistureReducerEnabled ? "on" : "off",
        adaptiveMoistureControlEnabled ? "on" : "off",
        exhaustMinimumPercent);
    return;
  }
  if (action == "updateServoConfig") {
    // Deliberate, session-independent calibration write. The hardware-test
    // command path also accepts these fields, but it needs a matching
    // testSessionId and is refused outright while a cycle runs — so a servo
    // setting changed from the app could be silently dropped and the device
    // would come back on defaults after the next reset. This path always
    // persists, so the angles can never be "set but not saved".
    const JsonObject servoConfig = desired["servoConfig"].as<JsonObject>();
    if (servoConfig.isNull()) {
      rejectCommand(requestedVersion, "INVALID_SERVO_CONFIG");
      return;
    }
    const int ventClosed = servoConfig["ventClosedAngle"] | (int)ventClosedAngle;
    const int ventOpen = servoConfig["ventOpenAngle"] | (int)ventOpenAngle;
    const int circClosed =
        servoConfig["circServoClosedAngle"] | (int)circServoClosedAngle;
    const int circOpen =
        servoConfig["circServoOpenAngle"] | (int)circServoOpenAngle;
    if (ventClosed < 0 || ventClosed > 180 || ventOpen < 0 || ventOpen > 180 ||
        circClosed < 0 || circClosed > 180 || circOpen < 0 || circOpen > 180) {
      rejectCommand(requestedVersion, "SERVO_ANGLE_OUT_OF_RANGE");
      return;
    }
    ventClosedAngle = static_cast<uint8_t>(ventClosed);
    ventOpenAngle = static_cast<uint8_t>(ventOpen);
    circServoClosedAngle = static_cast<uint8_t>(circClosed);
    circServoOpenAngle = static_cast<uint8_t>(circOpen);
    saveServoAngles();
    saveCircServoAngles();
    // Re-seat the vent servo only when it is parked, so a calibration write
    // during a running cycle cannot fight the exhaust interlock.
    if (!exhaustFan && !trayTurning) {
      ventAngle = ventClosedAngle;
      physicalServoAngle = ventClosedAngle;
      ventServo.write(ventClosedAngle);
    }
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    reportedStatePending = true;
    Serial.printf("Servo config updated: vent %u/%u, circ %u/%u\n",
                  ventClosedAngle, ventOpenAngle, circServoClosedAngle,
                  circServoOpenAngle);
    return;
  }
  if (action == "manualCheck") {
    const String target = desired["target"] | "";
    const bool on = desired["on"] | false;
    if (const char *refuse = beginManualCheck(target, on)) {
      rejectCommand(requestedVersion, refuse);
      return;
    }
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    reportedStatePending = true;
    lastBleSnapshotAt = 0;
    return;
  }
  if (action == "startCalibration") {
    if (cycleActive) {
      rejectCommand(requestedVersion, "STOP_INCUBATION_BEFORE_CALIBRATION");
      return;
    }
    if (hardwareFaultLatched) {
      rejectCommand(requestedVersion, "ACKNOWLEDGE_HARDWARE_FAULT_FIRST");
      return;
    }
    if (calibrationActive) {
      rejectCommand(requestedVersion, "CALIBRATION_ALREADY_RUNNING");
      return;
    }
    const float referenceTemperature =
        desired["calibration"]["referenceTemperature"] | NAN;
    const float referenceHumidity =
        desired["calibration"]["referenceHumidity"] | NAN;
    const int durationHours = desired["calibration"]["durationHours"] | 0;
    const uint32_t now = rtcUnixNow();
    if (!eggTemperatureValid || !sensorValid || now == 0 ||
        isnan(referenceTemperature) ||
        isnan(referenceHumidity) || referenceTemperature < 10.0F ||
        referenceTemperature > 50.0F || referenceHumidity < 10.0F ||
        referenceHumidity > 95.0F || (durationHours != 24 && durationHours != 48)) {
      rejectCommand(requestedVersion, "INVALID_CALIBRATION_REFERENCE_OR_SENSOR");
      return;
    }
    const float candidateTemperatureOffset =
        referenceTemperature - (eggTemperature - calibrationTemperatureOffset);
    const float candidateHumidityOffset =
        referenceHumidity - (measuredHumidity - calibrationHumidityOffset);
    if (fabsf(candidateTemperatureOffset) > 5.0F ||
        fabsf(candidateHumidityOffset) > 20.0F) {
      rejectCommand(requestedVersion, "CALIBRATION_OFFSET_OUT_OF_SAFE_RANGE");
      return;
    }
    previousCalibrationTemperatureOffset = calibrationTemperatureOffset;
    previousCalibrationHumidityOffset = calibrationHumidityOffset;
    previousCalibrationWasCalibrated = calibrated;
    calibrationTemperatureOffset = candidateTemperatureOffset;
    calibrationHumidityOffset = candidateHumidityOffset;
    calibrationDurationHours = durationHours;
    calibrationStartedAt = now;
    calibrationEndsAt = now + static_cast<uint32_t>(durationHours) * 3600UL;
    calibrationSamples = 0;
    calibrationStableSamples = 0;
    calibrationLastSampleAt = 0;
    calibrated = false;
    calibrationActive = true;
    calibrationStatus = "running";
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    saveCalibrationState();
    cycleActive = false;
    saveCycleState();
    reportedStatePending = true;
    lastCalibrationCloudLogAt = 0;
    queueCalibrationLog("started");
    return;
  }
  if (action == "cancelCalibration") {
    calibrationActive = false;
    calibrationTemperatureOffset = previousCalibrationTemperatureOffset;
    calibrationHumidityOffset = previousCalibrationHumidityOffset;
    calibrated = previousCalibrationWasCalibrated;
    calibrationStatus = calibrated ? "calibrated" : "cancelled";
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    setSafeOutputs();
    saveCalibrationState();
    saveCycleState();
    reportedStatePending = true;
    queueCalibrationLog("cancelled");
    return;
  }
  if (action == "acknowledgeTrayChange") {
    trayChangeEventPending = false;
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    saveCycleState();
    reportedStatePending = true;
    return;
  }
  if (action == "acknowledgeFault") {
    const unsigned long nowMs = millis();
    const bool sensorsReady = sensorValid && heaterTemperatureValid &&
                              eggSensorUpdatedAt > 0 && heaterSensorUpdatedAt > 0 &&
                              humiditySensorUpdatedAt > 0 &&
                              nowMs - eggSensorUpdatedAt <= SENSOR_STALE_MS &&
                              nowMs - heaterSensorUpdatedAt <= SENSOR_STALE_MS &&
                              nowMs - humiditySensorUpdatedAt <=
                                  DHT_SENSOR_STALE_MS;
    if (!sensorsReady || rtcUnixNow() == 0 || lidOpen ||
        heaterSurfaceTemperature >= HEATER_SURFACE_DERATE_C) {
      // Name the real blocker. A single generic code made an RTC-caused
      // deadlock look like a sensor fault, which is misleading in the app.
      // rtcUnixNow() is read once because it refreshes the cached epoch.
      const uint32_t ackClockNow = rtcUnixNow();
      const String blocker = !sensorsReady
                                 ? "SENSORS_NOT_HEALTHY"
                                 : (ackClockNow == 0
                                        ? "RTC_UNAVAILABLE"
                                        : (lidOpen ? "LID_OPEN"
                                                   : "HEATER_SURFACE_NOT_COOL"));
      if (ackClockNow == 0) {
        // The app path cannot clear any latched fault while the clock is
        // unavailable, even though the clock has no bearing on whether it is
        // safe to clear. The BLE acknowledge path does not require it.
        Serial.println(
            "Fault acknowledge blocked by RTC availability. Use the BLE "
            "acknowledge path, or restore the RTC first.");
      }
      rejectCommand(requestedVersion, blocker);
      return;
    }
    clearHardwareFault();
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    reportedStatePending = true;
    Serial.println("Hardware fault acknowledged; response verification restarted.");
    return;
  }
  if (!action.isEmpty()) {
    rejectCommand(requestedVersion, "UNKNOWN_COMMAND_ACTION");
    return;
  }

  const bool requestedActive = desired["cycleActive"] | false;
  if (!requestedActive) {
    const bool wasActive = cycleActive;
    const uint32_t stoppedAt = rtcUnixNow();
    if (wasActive) {
      queueIncubationEvent("batch_aborted", stoppedAt);
      queueIncubationBatchRecord("aborted", stoppedAt);
    }
    commandVersion = requestedVersion;
    commandState = "accepted";
    commandError = "";
    cycleActive = false;
    lockdown = false;
    trayChangeRequired = false;
    trayChangeEventPending = false;
    controlPaused = false;
    pauseReason = "";
    trayTurning = false;
    pendingTurnStartedAt = 0;
    testEnabled = false;
    saveCycleState();
    setSafeOutputs();
    reportedStatePending = true;
    Serial.println("Incubation cycle stopped by cloud command.");
    emitUsbControlLog("incubation_stopped");
    return;
  }

  if (calibrationActive) {
    rejectCommand(requestedVersion, "CANCEL_CALIBRATION_BEFORE_INCUBATION");
    return;
  }

  const unsigned long nowMs = millis();
  // Only the egg-zone DS18B20 is mandatory to start a cycle. The heater surface
  // sensor provides PWM derate/cutoff safety but its absence does NOT block
  // incubation — identical to mid-cycle behaviour when that sensor goes absent.
  const bool sensorReady = eggTemperatureValid &&
                            eggSensorUpdatedAt > 0 &&
                            nowMs - eggSensorUpdatedAt <= SENSOR_STALE_MS &&
                            isfinite(measuredTemperature);
  if (rtcUnixNow() == 0) {
    rejectCommand(requestedVersion, "RTC_NOT_CONNECTED_OR_INVALID");
    return;
  }
  if (lidOpen) {
    rejectCommand(requestedVersion, "CLOSE_INCUBATOR_LID_BEFORE_START");
    return;
  }
  if (!sensorReady) {
    rejectCommand(requestedVersion, "EGG_DS18B20_NOT_CONNECTED_OR_STALE");
    return;
  }
  if (hardwareFaultLatched) {
    rejectCommand(requestedVersion, "HARDWARE_FAULT_ACK_REQUIRED");
    return;
  }
  const String requestedMode = desired["mode"] | "";
  String nextProfileId;
  float nextTemperature = 0.0F;
  float nextNormalHumidity = 0.0F;
  float nextHatchHumidity = 0.0F;
  int nextTotalDays = 0;
  int nextLockdownDay = 0;
  int nextTurnIntervalHours = 0;
  if (requestedMode == "automatic") {
    const String requestedProfileId = desired["profileId"] | "";
    const IncubationProfile *profile = findProfile(requestedProfileId);
    if (profile == nullptr) {
      rejectCommand(requestedVersion, "UNKNOWN_PROFILE");
      return;
    }
    nextProfileId = profile->id;
    nextTemperature = profile->temperature;
    nextNormalHumidity = profile->normalHumidity;
    nextHatchHumidity = profile->hatchHumidity;
    nextTotalDays = profile->totalDays;
    nextLockdownDay = profile->lockdownDay;
    nextTurnIntervalHours = profile->turnIntervalHours;
  } else if (requestedMode == "manual") {
    const float requestedTemperature = desired["manual"]["temperature"] | 0.0F;
    const float requestedHumidity = desired["manual"]["humidity"] | 0.0F;
    const int requestedDays = desired["manual"]["days"] | 0;
    const int requestedTurnHours = desired["manual"]["turnIntervalHours"] | 0;
    const int requestedLockdown = desired["manual"]["stopTurningDay"] | 0;
    if (!control_logic::validManualSettings(
            requestedTemperature, requestedHumidity, requestedDays,
            requestedTurnHours, requestedLockdown)) {
      rejectCommand(requestedVersion, "INVALID_MANUAL_SETTINGS");
      return;
    }
    nextProfileId = "manual";
    nextTemperature = requestedTemperature;
    nextNormalHumidity = requestedHumidity;
    nextHatchHumidity = requestedHumidity;
    nextTotalDays = requestedDays;
    nextLockdownDay = requestedLockdown;
    nextTurnIntervalHours = requestedTurnHours;
  } else {
    rejectCommand(requestedVersion, "INVALID_MODE");
    return;
  }

  const int requestedStartDayOffset = desired["startDayOffset"] | 0;
  if (requestedStartDayOffset < 0 || requestedStartDayOffset >= nextTotalDays) {
    rejectCommand(requestedVersion, "INVALID_START_DAY_OFFSET");
    return;
  }
  const uint32_t now = rtcUnixNow();
  if (now == 0) {
    rejectCommand(requestedVersion, "RTC_REQUIRED");
    return;
  }
  // Commit cycle settings only after every rejection check has passed.
  cycleMode = requestedMode;
  profileId = nextProfileId;
  targetTemperature = nextTemperature;
  normalHumidity = nextNormalHumidity;
  hatchHumidity = nextHatchHumidity;
  totalDays = nextTotalDays;
  lockdownDay = nextLockdownDay;
  turnIntervalHours = nextTurnIntervalHours;
  startDayOffset = static_cast<uint8_t>(requestedStartDayOffset);
  cycleStartedAt = now;
  lastTurnAt = now;
  lastCycleEpoch = now;
  trayTurning = false;
  cycleDay = startDayOffset + 1;
  lockdown = cycleDay >= lockdownDay;
  trayChangeRequired = lockdown;
  trayChangeEventPending = lockdown;
  lastTrayChangeEventDay = 0;
  targetHumidity = lockdown ? hatchHumidity : normalHumidity;
  cycleActive = true;
  testEnabled = false;
  commandState = "accepted";
  commandError = "";
  commandVersion = requestedVersion;
  heaterIntegral = 0;
  lastHeaterTemperature = NAN;
  lastHeaterErrorRate = 0.0F;
  lastHeaterControlSampleAt = 0;
  warmupLockRecorded = false;
  warmupStartedAt = millis();
  warmupStartTemperature = measuredTemperature;
  holdPwmRecorded = false;
  holdStableStartedAt = 0;
  lastHoldSampleAt = 0;
  holdPwmAccumulator = 0;
  holdPwmSamples = 0;
  heaterResponseSlow = false;
  // Bug 2: reset rest-cycle timer when a new cycle starts.
  continuousRunStartedAt = millis();
  restCycleActive = false;
  restStartedAt = 0;
  // Feature: reset humidifier fault tracking.
  humidifierOnAt = 0;
  humidifierFaultSuspected = false;
  powerRestoreEventPending = false;
  incubationBatchId = String(now);
  incubationBatchId += '-';
  incubationBatchId += String(
      static_cast<uint32_t>(requestedVersion % 1000000ULL));
  incubationBatchEndedAt = 0;
  incubationEventHead = 0;
  incubationEventCount = 0;
  incubationEventSequence = 0;
  lastIncubationHourlyLogAt = now;
  loggedHumidifierState = false;
  queueIncubationBatchRecord("active");
  queueIncubationEvent("batch_started", now);
  saveCycleState();
  reportedStatePending = true;
  Serial.printf("Started %s cycle: %s at offset day %u, %.1f C, %.0f %% RH\n",
                cycleMode.c_str(), profileId.c_str(), startDayOffset,
                targetTemperature, targetHumidity);
  emitUsbControlLog("incubation_started");
}

void fillProductionTelemetry(JsonDocument &telemetry) {
  const uint32_t now = rtcUnixNow();
  const uint32_t interval = static_cast<uint32_t>(turnIntervalHours) * 3600UL;
  const bool canTurn = cycleActive && !lockdown && !sensorFault && !rtcFault;

  telemetry["updatedAt"][".sv"] = "timestamp";
  telemetry["firmwareBuildId"] = FIRMWARE_BUILD_ID;
  const esp_reset_reason_t lastResetReason = esp_reset_reason();
  telemetry["resetReason"] = static_cast<int>(lastResetReason);
  telemetry["resetReasonName"] = resetReasonName(lastResetReason);
  // Cumulative counters: if brownoutCount keeps climbing while the heater is
  // active, the 12 V ground reference / MOSFET gate drive still needs work.
  telemetry["bootCount"] = bootCount;
  telemetry["brownoutCount"] = brownoutCount;
  telemetry["heaterPwmApplied"] = appliedHeaterDuty;
  telemetry["heaterBlocked"] = heaterTestBlocker;
  telemetry["heaterOutputActiveLow"] = HEATER_OUTPUT_ACTIVE_LOW;
  if (isfinite(measuredTemperature) && (eggTemperatureValid || primaryTemperatureValid))
    telemetry["temperature"] = measuredTemperature;
  else telemetry["temperature"] = nullptr;
  if (isfinite(measuredHumidity) && (eggDhtHumidityValid || primaryHumidityValid))
    telemetry["humidity"] = measuredHumidity;
  else telemetry["humidity"] = nullptr;
  telemetry["sensorValid"] = !sensorFault && !isnan(measuredTemperature) &&
                             !isnan(measuredHumidity);
  if (primaryTemperatureValid) {
    telemetry["primaryTemperature"] = primaryTemperature;
  } else {
    telemetry["primaryTemperature"] = nullptr;
  }
  if (primaryHumidityValid) {
    telemetry["primaryHumidity"] = primaryHumidity;
  } else {
    telemetry["primaryHumidity"] = nullptr;
  }
  telemetry["primarySensorValid"] = primarySensorValid;
  if (eggDhtTemperatureValid)
    telemetry["eggDhtTemperature"] = eggDhtTemperature;
  else
    telemetry["eggDhtTemperature"] = nullptr;
  if (eggDhtHumidityValid)
    telemetry["eggDhtHumidity"] = eggDhtHumidity;
  else
    telemetry["eggDhtHumidity"] = nullptr;
  telemetry["eggDhtTemperatureValid"] = eggDhtTemperatureValid;
  telemetry["eggDhtHumidityValid"] = eggDhtHumidityValid;
  telemetry["humiditySensorsAgree"] = humiditySensorsAgree;
  telemetry["humidityDegraded"] = humidityDegraded;
  telemetry["eggTemperatureDisagreement"] = eggTemperatureDisagreement;
  telemetry["eggTemperatureDisagreementSamples"] =
      eggTemperatureDisagreementSamples;
  if (isfinite(eggTemperatureSpread))
    telemetry["eggTemperatureSpread"] = eggTemperatureSpread;
  else
    telemetry["eggTemperatureSpread"] = nullptr;
  telemetry["dhtDiagnostic"] = primaryDhtRecovery.diagnostic;
  telemetry["dhtDataLevel"] = primaryDhtRecovery.dataLevel;
  telemetry["dhtConsecutiveFailures"] =
      primaryDhtRecovery.consecutiveFailures;
  telemetry["primaryDhtDiagnostic"] = primaryDhtRecovery.diagnostic;
  telemetry["primaryDhtReadStatus"] = primaryDhtRecovery.readStatus;
  telemetry["primaryDhtReadError"] =
      dhtStatusName(primaryDhtRecovery.readStatus);
  telemetry["primaryDhtUsingCachedData"] = primaryDhtUsingCachedData;
  telemetry["primaryDhtFailures"] = primaryDhtRecovery.consecutiveFailures;
  telemetry["primaryDhtRecoveryAttempts"] =
      primaryDhtRecovery.recoveryAttempts;
  telemetry["primaryDhtDataLevel"] = primaryDhtRecovery.dataLevel;
  telemetry["eggDhtDiagnostic"] = eggDhtRecovery.diagnostic;
  telemetry["eggDhtReadStatus"] = eggDhtRecovery.readStatus;
  telemetry["eggDhtReadError"] = dhtStatusName(eggDhtRecovery.readStatus);
  telemetry["eggDhtUsingCachedData"] = eggDhtUsingCachedData;
  telemetry["eggDhtFailures"] = eggDhtRecovery.consecutiveFailures;
  telemetry["eggDhtRecoveryAttempts"] = eggDhtRecovery.recoveryAttempts;
  telemetry["eggDhtDataLevel"] = eggDhtRecovery.dataLevel;
  telemetry["rawDhtTemperature"] = lastRawDhtTemperature;
  telemetry["rawDhtHumidity"] = lastRawDhtHumidity;
  telemetry["rawEggTemperature"] = lastRawEggTemperature;
  telemetry["rawHeaterTemperature"] = lastRawHeaterTemperature;
  telemetry["primaryTemperatureValid"] = primaryTemperatureValid;
  telemetry["primaryHumidityValid"] = primaryHumidityValid;
  telemetry["primarySensorUpdatedAt"] =
      (primaryTemperatureValid || primaryHumidityValid) ? primarySensorUpdatedAt : 0;
  if (eggTemperatureValid) {
    telemetry["eggTemperature"] = eggTemperature;
    telemetry["secondaryTemperature"] = eggTemperature;
  } else {
    telemetry["eggTemperature"] = nullptr;
    telemetry["secondaryTemperature"] = nullptr;
  }
  if (heaterTemperatureValid)
    telemetry["heaterTemperature"] = heaterSurfaceTemperature;
  else
    telemetry["heaterTemperature"] = nullptr;
  if (primaryHumidityValid)
    telemetry["secondaryHumidity"] = primaryHumidity;
  else
    telemetry["secondaryHumidity"] = nullptr;
  telemetry["eggTemperatureValid"] = eggTemperatureValid;
  telemetry["heaterTemperatureValid"] = heaterTemperatureValid;
  telemetry["secondarySensorValid"] = eggTemperatureValid;
  telemetry["sensorSource"] = eggTemperatureValid ? "ds18b20_egg" : "none";
  telemetry["sensorDegraded"] = !eggTemperatureValid && primarySensorValid;
  telemetry["sensorFaultCode"] = sensorFaultCode;
  telemetry["calibrated"] = calibrated;
  telemetry["calibrationActive"] = calibrationActive;
  telemetry["calibrationStatus"] = calibrationStatus;
  if (primaryTemperatureValid && eggTemperatureValid) {
    telemetry["temperatureSpread"] =
        fabsf(primaryTemperature - eggTemperature);
  } else {
    telemetry["temperatureSpread"] = nullptr;
  }
  if (primaryHumidityValid && eggDhtHumidityValid) {
    telemetry["humiditySpread"] =
        fabsf(primaryHumidity - eggDhtHumidity);
  } else {
    telemetry["humiditySpread"] = nullptr;
  }
  telemetry["cycleActive"] = cycleActive;
  telemetry["mode"] = cycleMode;
  telemetry["profileId"] = profileId;
  telemetry["cycleDay"] = cycleDay;
  telemetry["startDayOffset"] = startDayOffset;
  telemetry["batchId"] = incubationBatchId;
  telemetry["totalDays"] = totalDays;
  telemetry["lockdown"] = lockdown;
  telemetry["phase"] = lockdown ? "hatching" : "setting";
  telemetry["trayChangeRequired"] = trayChangeRequired;
  telemetry["trayChangeEventPending"] = trayChangeEventPending;
  telemetry["lidOpen"] = lidOpen;
  telemetry["lidWarning"] = lidWarning;
  telemetry["lidOpenSeconds"] = lidOpenDurationMs / 1000UL;
  telemetry["gpio25Level"] = digitalRead(FAN_1_PIN);
  telemetry["gpio26CircServoAngle"] = physicalCircServoAngle;
  telemetry["gpio2Level"] = digitalRead(HUMIDIFIER_PIN);
  telemetry["gpio27Duty"] = appliedExhaustPwm;
  telemetry["gpio32Duty"] = heaterPwm;
  telemetry["lockdownDay"] = lockdownDay;
  telemetry["targetTemperature"] = targetTemperature;
  telemetry["targetHumidity"] = targetHumidity;
  telemetry["normalHumidity"] = normalHumidity;
  telemetry["hatchHumidity"] = hatchHumidity;
  telemetry["heaterPwm"] = heaterPwm;
  telemetry["circulationFan1"] = circulationFan1;
  telemetry["circServoAngle"] = physicalCircServoAngle;
  telemetry["circServoOpen"] = physicalCircServoAngle == circServoOpenAngle;
  telemetry["circServoClosedAngle"] = circServoClosedAngle;
  telemetry["circServoOpenAngle"] = circServoOpenAngle;
  telemetry["eggTurnerEnabled"] = cycleActive && !lockdown;
  telemetry["eggTurnerPulseSeconds"] = TURN_PULSE_SECONDS;
  telemetry["exhaustFan"] = exhaustFan;
  telemetry["exhaustPwm"] = appliedExhaustPwm;
  telemetry["ventOpen"] = ventAngle == ventOpenAngle;
  telemetry["ventAngle"] = ventAngle;
  // The vent servo's calibration was never published outside the hardware-test
  // telemetry, which only uploads while a test session is running. The app's
  // global/config view therefore had nothing to read and fell back to its own
  // defaults — looking exactly like "the servo setting reset itself". Both
  // servo pairs are now reported here, and once, together, under servoConfig.
  telemetry["servoClosedAngle"] = ventClosedAngle;
  telemetry["servoOpenAngle"] = ventOpenAngle;
  {
    JsonObject servoConfig = telemetry["servoConfig"].to<JsonObject>();
    servoConfig["ventClosedAngle"] = ventClosedAngle;
    servoConfig["ventOpenAngle"] = ventOpenAngle;
    servoConfig["circServoClosedAngle"] = circServoClosedAngle;
    servoConfig["circServoOpenAngle"] = circServoOpenAngle;
    servoConfig["savedAt"] = servoConfigSavedAt;
    // "default" means NVS held no usable calibration and the compile-time
    // defaults are in force; "shadow" means the primary copy was missing and
    // was repaired. Either way the app can now tell "never configured" from
    // "configured" instead of silently showing its own defaults.
    servoConfig["source"] = servoConfigSource == 2   ? "shadow"
                             : servoConfigSource == 1 ? "nvram"
                                                      : "default";
    servoConfig["persisted"] = servoConfigSource != 0;
  }
  telemetry["humidifier"] = humidifierOn;
  telemetry["moistureReducerEnabled"] =
      AUTOMATIC_MOISTURE_REDUCER_ENABLED && moistureReducerEnabled;
  telemetry["adaptiveMoistureControlEnabled"] =
      adaptiveMoistureControlEnabled;
  telemetry["moistureReducerActive"] = moistureReducerActive;
  telemetry["exhaustMinimumPercent"] = exhaustMinimumPercent;
  telemetry["moistureAdaptiveBoostPwm"] = moistureAdaptiveBoostPwm;
  telemetry["humidityTrendRhPerMinute"] = humidityTrendRhPerMinute;
  telemetry["eggTurner"] =
      physicalCircServoAngle == circServoOpenAngle;
  telemetry["lastTurnAt"] = lastTurnAt == 0 ? 0 : lastTurnAt * 1000ULL;
  telemetry["nextTurnAt"] = canTurn && lastTurnAt > 0
                                ? (lastTurnAt + interval) * 1000ULL
                                : 0;
  telemetry["rtcAvailable"] = rtcAvailable;
  telemetry["rtcRunning"] = rtcRunning;
  // Consecutive failed I2C bus recoveries. Rising while rtcAvailable is still
  // true means the bus is glitching and the RTC is about to drop out - the
  // warning sign that the DS1307 wiring or level shifting needs attention.
  telemetry["rtcRecoveryStrikes"] = rtcRecoveryStrikes;
  telemetry["rtcUnixTime"] = now;
  telemetry["sensorFault"] = sensorFault;
  telemetry["rtcFault"] = rtcFault;
  telemetry["controlPaused"] = controlPaused;
  telemetry["pauseReason"] = pauseReason;
  telemetry["actuatorFeedbackAvailable"] = false;
  telemetry["responseMonitoringAvailable"] = true;
  telemetry["feedbackType"] = "environmental_response";
  telemetry["heaterHealth"] =
      heaterResponseSlow
          ? "suspected"
          : actuatorHealth("HEATER", heaterResponseVerified, heaterResponse);
  telemetry["heaterResponseSlow"] = heaterResponseSlow;
  telemetry["circulationFan1Health"] = "unverified";
  telemetry["circulationFan2Health"] = "unverified";
  telemetry["exhaustFanHealth"] =
      exhaustResponseFailed && !hardwareFaultLatched
          ? "fault"
          : actuatorHealth("EXHAUST", exhaustResponseVerified, exhaustResponse);
  telemetry["ventServoHealth"] = "unverified";
  telemetry["humidifierHealth"] = actuatorHealth(
      "HUMIDIFIER", humidifierResponseVerified, humidifierResponse);
  telemetry["eggTurnerHealth"] = circServo.attached() ? "available" : "fault";
  telemetry["overheat"] = overheat;
  const unsigned long telemetryCheckRemaining =
      manualCheckTarget && static_cast<int32_t>(millis() - manualCheckDeadline) < 0
          ? manualCheckDeadline - millis()
          : 0;
  telemetry["manualCheckTarget"] =
      manualCheckTarget == 1   ? "humidifier"
      : manualCheckTarget == 2 ? "exhaust"
                               : "";
  telemetry["manualCheckOn"] = telemetryCheckRemaining > 0 && manualCheckOn;
  telemetry["manualCheckRemainingMs"] = telemetryCheckRemaining;
  telemetry["trayTurning"] = trayTurning;
  telemetry["hardwareFaultLatched"] = hardwareFaultLatched;
  telemetry["hardwareFaultCode"] = hardwareFaultCode;
  telemetry["hardwareFaultAt"] =
      hardwareFaultAt == 0 ? 0 : hardwareFaultAt * 1000ULL;
  telemetry["fault"] = controlPaused ? pauseReason : "";
  // Adaptive heater model telemetry.
  telemetry["selfLearnIntegralPreload"] = learnedIntegralPreload;
  telemetry["selfLearnWarmupLocked"] = warmupLockRecorded;
  telemetry["heaterModelReady"] = heaterModelReady();
  telemetry["learnedHeaterHoldPwm"] = learnedHoldPwm;
  telemetry["learnedWarmupSeconds"] = learnedWarmupSeconds;
  telemetry["heaterModelSamples"] = heaterModelSamples;
  telemetry["heaterLearningStableSeconds"] =
      holdStableStartedAt == 0
          ? 0
          : (millis() - holdStableStartedAt) / 1000UL;
  // Room (outside) sensor — primary DHT22 moved outside the incubator.
  if (primaryTemperatureValid) telemetry["roomTemperature"] = roomTemperature;
  else telemetry["roomTemperature"] = nullptr;
  if (roomHumidityValid) telemetry["roomHumidity"] = roomHumidity;
  else telemetry["roomHumidity"] = nullptr;
  telemetry["roomHumidityValid"] = roomHumidityValid;
  telemetry["exhaustHelpsHumidity"] = exhaustHelpsHumidity;
  // Bug 2: equipment rest cycle status.
  telemetry["restCycleActive"] = restCycleActive;
  // Feature: humidifier fault / exhaust fallback status.
  telemetry["humidifierFaultSuspected"] = humidifierFaultSuspected;
}

void publishProductionTelemetry() {
  if (millis() - lastProductionTelemetryWrite < PRODUCTION_TELEMETRY_INTERVAL_MS ||
      WiFi.status() != WL_CONNECTED) return;
  lastProductionTelemetryWrite = millis();
  JsonDocument telemetry;
  fillProductionTelemetry(telemetry);
  String body;
  serializeJson(telemetry, body);
  String response;
  const int status = firebaseRequest("PUT", "telemetry", body, response);
  if (status < 200 || status >= 300) {
    Serial.printf("Production telemetry failed (%d): %s\n", status,
                  response.c_str());
  }
}

void readClaimRequest() {
  if (millis() - lastClaimRead < COMMAND_INTERVAL_MS ||
      WiFi.status() != WL_CONNECTED || factoryResetCloudPending) {
    return;
  }
  lastClaimRead = millis();

  String response;
  const int status = firebaseRequest("GET", "claimRequest", "", response);
  if (status != HTTP_CODE_OK || response == "null") return;

  JsonDocument request;
  if (deserializeJson(request, response)) return;
  const String claimCode = request["claimCode"] | "";
  const String requestedOwnerUid = request["ownerUid"] | "";
  if (claimCode != DEVICE_CLAIM_CODE || requestedOwnerUid.isEmpty()) {
    Serial.println("Rejected invalid device claim.");
    return;
  }

  JsonDocument assignment;
  assignment["ownerUid"] = requestedOwnerUid;
  assignment["claimRequest"] = nullptr;
  String body;
  serializeJson(assignment, body);
  String assignmentResponse;
  const int assignmentStatus =
      firebaseRequest("PATCH", "", body, assignmentResponse);
  if (assignmentStatus >= 200 && assignmentStatus < 300) {
    saveOwner(requestedOwnerUid);
    Serial.println("Device owner assigned.");
  }
}

void reconcileCloudOwner() {
  if (WiFi.status() != WL_CONNECTED ||
      millis() - lastOwnerCheck < COMMAND_INTERVAL_MS) {
    return;
  }
  lastOwnerCheck = millis();

  String response;
  const int status = firebaseRequest("GET", "ownerUid", "", response);
  if (status != HTTP_CODE_OK) return;
  if (response == "null") {
    if (ownerUid.isEmpty()) return;
    // A single transient null read (server hiccup, rule race, reconnect flap)
    // must never wipe a running batch: require several confirmed consecutive
    // null reads on separate polls before clearing the cycle.
    if (++cloudOwnerNullStreak < 3) {
      Serial.printf(
          "Cloud ownership read returned null (%u/3); awaiting confirmation.\n",
          static_cast<unsigned>(cloudOwnerNullStreak));
      return;
    }
    cloudOwnerNullStreak = 0;
    cycleActive = false;
    clockSuspendStartedAt = 0;
    testEnabled = false;
    setSafeOutputs();
    clearNamespace("cycle");
    clearNamespace("device");
    ownerUid.clear();
    profileId.clear();
    commandVersion = 0;
    commandState = "accepted";
    commandError = "";
    Serial.println(
        "Cloud ownership was removed; cleared stale local owner and cycle. BLE Wi-Fi scanning is enabled.");
    setBleStatus(
        "{\"state\":\"ready\",\"message\":\"Previous owner removed. Ready for Wi-Fi setup.\"}");
    return;
  }
  cloudOwnerNullStreak = 0;

  JsonDocument owner;
  if (deserializeJson(owner, response)) return;
  const String cloudOwner = owner.as<String>();
  if (!cloudOwner.isEmpty() && ownerUid != cloudOwner) {
    saveOwner(cloudOwner);
    Serial.println("Synchronized cloud owner to local storage.");
  }
}

void readTestCommand() {
  // Poll aggressively only while a hardware test session is running; the
  // always-on 1 s poll added a TLS request every second around the clock.
  const unsigned long pollInterval =
      testEnabled ? TEST_COMMAND_INTERVAL_MS : COMMAND_INTERVAL_MS;
  if (millis() - lastTestRead < pollInterval ||
      WiFi.status() != WL_CONNECTED) {
    return;
  }
  lastTestRead = millis();

  String response;
  const int status = firebaseRequest("GET", "testCommand", "", response);
  if (status != HTTP_CODE_OK || response == "null") return;

  JsonDocument command;
  if (deserializeJson(command, response)) {
    Serial.println("Hardware test command JSON was invalid.");
    return;
  }
  if (cycleActive) {
    testEnabled = false;
    return;
  }
  const uint64_t requestedVersion = command["version"] | 0ULL;
  if (requestedVersion == 0) return;
  const String requestedSession = command["testSessionId"] | "";
  if (requestedSession != testSessionId) {
    // Do NOT consume the command version here. testSessionId is regenerated on
    // every boot, so the companion app always has one stale round-trip after a
    // reset: it sends the user's settings against the previous session id, gets
    // dropped, then re-sends the same version against the new id. Advancing
    // lastTestCommandVersion on the dropped attempt made the retry fail the
    // "version must be newer" test, so the settings were silently discarded and
    // the device came back up on defaults.
    Serial.printf("Test command ignored: session %s does not match current %s "
                  "(version %llu left unconsumed for retry).\n",
                  requestedSession.c_str(), testSessionId.c_str(),
                  static_cast<unsigned long long>(requestedVersion));
    return;
  }
  if (requestedVersion <= lastTestCommandVersion) return;
  lastTestCommandVersion = requestedVersion;
  lastTestCommandReceived = millis();
  testEnabled = command["enabled"] | false;
  testFan1 = command["fan1"] | false;
  testFan3 = command["fan3"] | false;
  testHeater = command["heater"] | false;
  const int requestedHeaterPwm =
      command["heaterPwm"] | (testHeater ? 255 : 0);
  const int requestedExhaustPwm =
      command["exhaustPwm"] | (testFan3 ? 255 : 0);
  testHeaterPwm = control_logic::clampPwm(requestedHeaterPwm);
  testExhaustPwm = control_logic::clampPwm(requestedExhaustPwm);
  testHumidifier = command["humidifier"] | false;
  atmosphereTestEnabled = command["atmosphereTestEnabled"] | false;
  atmosphereTargetTemperature =
      constrain(command["atmosphereTargetTemperature"] | 37.5F, 35.0F, 40.0F);
  atmosphereTargetHumidity =
      constrain(command["atmosphereTargetHumidity"] | 60.0F, 35.0F, 80.0F);
  // ── Exhaust vent servo (GPIO 13) ─────────────────────────────────────────
  // Bug fix: only update testServoAngle when the toggle is explicitly ON.
  // This prevents the slider from auto-activating the servo test mode.
  testServoEnabled = command["servoTestEnabled"] | false;
  // servoConfigSave lets the app state explicitly that the angles below are a
  // deliberate calibration write rather than an echoed default.
  const bool ventConfigSaveRequested = command["servoConfigSave"] | false;
  const uint8_t requestedVentClosedAngle =
      constrain(command["servoClosedAngle"] | (int)ventClosedAngle, 0, 180);
  const uint8_t requestedVentOpenAngle =
      constrain(command["servoOpenAngle"] | (int)ventOpenAngle, 0, 180);
  if (requestedVentClosedAngle != ventClosedAngle ||
      requestedVentOpenAngle != ventOpenAngle) {
    // Right after a reset the app reconnects and can push its own factory
    // defaults before it has read our telemetry. Letting that through wiped the
    // saved calibration and looked exactly like "settings reset themselves".
    const bool ventDefaultEcho =
        !ventConfigSaveRequested && servoConfigSource != 0 &&
        millis() < SERVO_DEFAULT_ECHO_GUARD_MS &&
        requestedVentClosedAngle == VENT_CLOSED_ANGLE &&
        requestedVentOpenAngle == VENT_OPEN_ANGLE &&
        (ventClosedAngle != VENT_CLOSED_ANGLE ||
         ventOpenAngle != VENT_OPEN_ANGLE);
    if (ventDefaultEcho) {
      Serial.printf(
          "Servo config: ignoring post-boot default echo (%u/%u), keeping saved "
          "%u/%u. Send servoConfigSave:true to force.\n",
          requestedVentClosedAngle, requestedVentOpenAngle, ventClosedAngle,
          ventOpenAngle);
    } else {
      ventClosedAngle = requestedVentClosedAngle;
      ventOpenAngle = requestedVentOpenAngle;
      saveServoAngles();
    }
  }
  if (testServoEnabled) {
    const String servoAction = command["servoAction"] | "";
    if (servoAction == "open")
      testServoAngle = ventOpenAngle;
    else if (servoAction == "close")
      testServoAngle = ventClosedAngle;
    else
      testServoAngle = constrain(command["servoAngle"] | (int)ventClosedAngle, 0, 180);
  }
  // ── Circulation servo (GPIO 26) ── egg-turn switch ───────────────────────
  // Same pattern: slider only active when circServo test toggle is ON.
  testCircServoEnabled = command["circServoTestEnabled"] | false;
  const bool circConfigSaveRequested = command["circServoConfigSave"] | false;
  const uint8_t requestedCircClosedAngle =
      constrain(command["circServoClosedAngle"] | (int)circServoClosedAngle, 0, 180);
  const uint8_t requestedCircOpenAngle =
      constrain(command["circServoOpenAngle"] | (int)circServoOpenAngle, 0, 180);
  if (requestedCircClosedAngle != circServoClosedAngle ||
      requestedCircOpenAngle != circServoOpenAngle) {
    const bool circDefaultEcho =
        !circConfigSaveRequested && servoConfigSource != 0 &&
        millis() < SERVO_DEFAULT_ECHO_GUARD_MS &&
        requestedCircClosedAngle == CIRC_SERVO_CLOSED_ANGLE &&
        requestedCircOpenAngle == CIRC_SERVO_OPEN_ANGLE &&
        (circServoClosedAngle != CIRC_SERVO_CLOSED_ANGLE ||
         circServoOpenAngle != CIRC_SERVO_OPEN_ANGLE);
    if (circDefaultEcho) {
      Serial.printf(
          "Servo config: ignoring post-boot default echo (%u/%u), keeping saved "
          "%u/%u. Send circServoConfigSave:true to force.\n",
          requestedCircClosedAngle, requestedCircOpenAngle,
          circServoClosedAngle, circServoOpenAngle);
    } else {
      circServoClosedAngle = requestedCircClosedAngle;
      circServoOpenAngle   = requestedCircOpenAngle;
      saveCircServoAngles();
    }
  }
  if (testCircServoEnabled) {
    const String circServoAction = command["circServoAction"] | "";
    if (circServoAction == "open")
      testCircServoAngle = circServoOpenAngle;
    else if (circServoAction == "close")
      testCircServoAngle = circServoClosedAngle;
    else
      testCircServoAngle = constrain(
          command["circServoAngle"] | (int)circServoClosedAngle, 0, 180);
  }
  const bool environmentSafe = sensorValid && lastValidSensorRead > 0 &&
                               millis() - lastValidSensorRead <=
                                   DHT_SENSOR_STALE_MS &&
                               measuredTemperature < ABSOLUTE_OVERHEAT_C &&
                               heaterTemperatureValid &&
                               heaterSurfaceTemperature < HEATER_SURFACE_MAX_C;
  if (!environmentSafe) {
    testHeater = false;
    testHeaterPwm = 0;
  }
  if (heaterTemperatureValid &&
      heaterSurfaceTemperature > HEATER_SURFACE_DERATE_C) {
    const float remainingRange =
        HEATER_SURFACE_MAX_C - heaterSurfaceTemperature;
    const float derateRange = HEATER_SURFACE_MAX_C - HEATER_SURFACE_DERATE_C;
    const uint8_t maximumSafeDuty = static_cast<uint8_t>(
        constrain(255.0F * remainingRange / derateRange, 0.0F, 255.0F));
    testHeaterPwm = min(testHeaterPwm, maximumSafeDuty);
  }
  applyTestOutputs();
}

void publishTestTelemetry() {
  if (millis() - lastTestWrite < TEST_INTERVAL_MS ||
      WiFi.status() != WL_CONNECTED) {
    return;
  }
  // Only upload while a hardware test session is actually running; uploading
  // test telemetry around the clock doubled the HTTPS request rate.
  if (!testEnabled) {
    return;
  }
  lastTestWrite = millis();

  JsonDocument telemetry;
  telemetry["online"] = true;
  telemetry["firmwareBuildId"] = FIRMWARE_BUILD_ID;
  telemetry["testSessionId"] = testSessionId;
  telemetry["gpio25Level"] = digitalRead(FAN_1_PIN);
  telemetry["gpio26Level"] = digitalRead(CIRC_SERVO_PIN);
  telemetry["testLeaseAgeMs"] =
      lastTestCommandReceived > 0 ? millis() - lastTestCommandReceived : 0;
  telemetry["testLeaseRemainingMs"] =
      testEnabled && lastTestCommandReceived > 0
          ? static_cast<int32_t>(TEST_COMMAND_LEASE_MS - (millis() - lastTestCommandReceived))
          : 0;
  telemetry["resetReason"] = static_cast<int>(esp_reset_reason());
  telemetry["bootCount"] = bootCount;
  telemetry["brownoutCount"] = brownoutCount;
  telemetry["heaterPwmApplied"] = appliedHeaterDuty;
  telemetry["heaterBlocked"] = heaterTestBlocker;
  telemetry["heaterOutputActiveLow"] = HEATER_OUTPUT_ACTIVE_LOW;
  telemetry["updatedAt"][".sv"] = "timestamp";
  telemetry["sensorValid"] = sensorValid;
  telemetry["primarySensorValid"] = primarySensorValid;
  if (eggDhtTemperatureValid)
    telemetry["eggDhtTemperature"] = eggDhtTemperature;
  else
    telemetry["eggDhtTemperature"] = nullptr;
  if (eggDhtHumidityValid)
    telemetry["eggDhtHumidity"] = eggDhtHumidity;
  else
    telemetry["eggDhtHumidity"] = nullptr;
  telemetry["eggDhtTemperatureValid"] = eggDhtTemperatureValid;
  telemetry["eggDhtHumidityValid"] = eggDhtHumidityValid;
  telemetry["humiditySensorsAgree"] = humiditySensorsAgree;
  telemetry["humidityDegraded"] = humidityDegraded;
  telemetry["eggTemperatureDisagreement"] = eggTemperatureDisagreement;
  telemetry["eggTemperatureDisagreementSamples"] =
      eggTemperatureDisagreementSamples;
  if (isfinite(eggTemperatureSpread))
    telemetry["eggTemperatureSpread"] = eggTemperatureSpread;
  else
    telemetry["eggTemperatureSpread"] = nullptr;
  telemetry["dhtDiagnostic"] = primaryDhtRecovery.diagnostic;
  telemetry["dhtDataLevel"] = primaryDhtRecovery.dataLevel;
  telemetry["dhtConsecutiveFailures"] =
      primaryDhtRecovery.consecutiveFailures;
  telemetry["primaryDhtDiagnostic"] = primaryDhtRecovery.diagnostic;
  telemetry["primaryDhtReadStatus"] = primaryDhtRecovery.readStatus;
  telemetry["primaryDhtReadError"] =
      dhtStatusName(primaryDhtRecovery.readStatus);
  telemetry["primaryDhtUsingCachedData"] = primaryDhtUsingCachedData;
  telemetry["primaryDhtFailures"] = primaryDhtRecovery.consecutiveFailures;
  telemetry["primaryDhtRecoveryAttempts"] =
      primaryDhtRecovery.recoveryAttempts;
  telemetry["primaryDhtDataLevel"] = primaryDhtRecovery.dataLevel;
  telemetry["eggDhtDiagnostic"] = eggDhtRecovery.diagnostic;
  telemetry["eggDhtReadStatus"] = eggDhtRecovery.readStatus;
  telemetry["eggDhtReadError"] = dhtStatusName(eggDhtRecovery.readStatus);
  telemetry["eggDhtUsingCachedData"] = eggDhtUsingCachedData;
  telemetry["eggDhtFailures"] = eggDhtRecovery.consecutiveFailures;
  telemetry["eggDhtRecoveryAttempts"] = eggDhtRecovery.recoveryAttempts;
  telemetry["eggDhtDataLevel"] = eggDhtRecovery.dataLevel;
  telemetry["primaryTemperatureValid"] = primaryTemperatureValid;
  telemetry["primaryHumidityValid"] = primaryHumidityValid;
  if (primaryTemperatureValid) {
    telemetry["primaryTemperature"] = primaryTemperature;
  } else {
    telemetry["primaryTemperature"] = nullptr;
  }
  if (primaryHumidityValid) {
    telemetry["primaryHumidity"] = primaryHumidity;
  } else {
    telemetry["primaryHumidity"] = nullptr;
  }
  if (eggTemperatureValid) {
    telemetry["eggTemperature"] = eggTemperature;
    telemetry["secondaryTemperature"] = eggTemperature;
  } else {
    telemetry["eggTemperature"] = nullptr;
    telemetry["secondaryTemperature"] = nullptr;
  }
  if (heaterTemperatureValid)
    telemetry["heaterTemperature"] = heaterSurfaceTemperature;
  else
    telemetry["heaterTemperature"] = nullptr;
  if (primaryHumidityValid)
    telemetry["secondaryHumidity"] = primaryHumidity;
  else
    telemetry["secondaryHumidity"] = nullptr;
  telemetry["eggTemperatureValid"] = eggTemperatureValid;
  telemetry["heaterTemperatureValid"] = heaterTemperatureValid;
  telemetry["secondarySensorValid"] = eggTemperatureValid;
  telemetry["sensorSource"] = eggTemperatureValid ? "ds18b20_egg" : "none";
  telemetry["sensorDegraded"] = !eggTemperatureValid && primarySensorValid;
  telemetry["sensorFaultCode"] = sensorFaultCode;
  if (!isnan(measuredTemperature) && !isnan(measuredHumidity)) {
    telemetry["temperature"] = measuredTemperature;
    telemetry["humidity"] = measuredHumidity;
  }
  telemetry["rtcAvailable"] = rtcAvailable;
  telemetry["rtcRunning"] = rtcRunning;
  // Consecutive failed I2C bus recoveries. Rising while rtcAvailable is still
  // true means the bus is glitching and the RTC is about to drop out - the
  // warning sign that the DS1307 wiring or level shifting needs attention.
  telemetry["rtcRecoveryStrikes"] = rtcRecoveryStrikes;
  telemetry["rtcStatus"] = rtcAvailable
                               ? (rtcRunning ? "working" : "clock stopped")
                               : "not detected at I2C address 0x68";
  if (rtcAvailable) {
    const DateTime now = rtc.now();
    char dateTime[20];
    snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(),
             now.second());
    telemetry["rtcDateTime"] = dateTime;
  }
  const bool testAllowed = testEnabled && !cycleActive;
  telemetry["commandVersion"] = lastTestCommandVersion;
  telemetry["testEnabled"] = testAllowed;
  telemetry["fan1"] = digitalRead(FAN_1_PIN) == OUTPUT_ON;
  telemetry["fan3"] = appliedTestExhaustPwm > 0;
  telemetry["heater"] = appliedTestHeaterPwm > 0;
  telemetry["heaterPwm"] = appliedTestHeaterPwm;
  telemetry["heaterRequestedPwm"] = testAllowed ? testHeaterPwm : 0;
  telemetry["heaterTestBlocker"] = heaterTestBlocker;
  telemetry["atmosphereTestEnabled"] = testAllowed && atmosphereTestEnabled;
  telemetry["atmosphereTargetTemperature"] = atmosphereTargetTemperature;
  telemetry["atmosphereTargetHumidity"] = atmosphereTargetHumidity;
  telemetry["exhaustPwm"] = appliedTestExhaustPwm;
  telemetry["exhaustMinimumPercent"] = exhaustMinimumPercent;
  // Exhaust vent servo (GPIO 13)
  telemetry["ventAngle"] = physicalServoAngle;
  telemetry["ventOpen"] = physicalServoAngle == ventOpenAngle;
  telemetry["servoTestEnabled"] = testAllowed && testServoEnabled;
  telemetry["servoClosedAngle"] = ventClosedAngle;
  telemetry["servoOpenAngle"] = ventOpenAngle;
  // Circulation servo (GPIO 26)
  telemetry["circServoAngle"] = physicalCircServoAngle;
  telemetry["circServoOpen"] = physicalCircServoAngle == circServoOpenAngle;
  telemetry["circServoTestEnabled"] = testAllowed && testCircServoEnabled;
  telemetry["circServoClosedAngle"] = circServoClosedAngle;
  telemetry["circServoOpenAngle"] = circServoOpenAngle;
  telemetry["humidifier"] = appliedTestHumidifier;
  telemetry["eggTurner"] = physicalCircServoAngle == circServoOpenAngle;  // servo-based
  telemetry["lidOpen"] = lidOpen;
  telemetry["lidWarning"] = lidWarning;
  telemetry["lidOpenSeconds"] = lidOpenDurationMs / 1000UL;

  String body;
  serializeJson(telemetry, body);
  String response;
  const int status = firebaseRequest("PUT", "testTelemetry", body, response);
  if (status < 200 || status >= 300) {
    Serial.printf("Hardware test telemetry failed (%d): %s\n", status,
                  response.c_str());
  }
}

void maintainWifi() {
  if (wifiProvisioningPending) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiProvisioningPending = false;
      idToken.clear();
      lastTelemetryWrite = 0;  // publish online=true immediately after provisioning
      const String ip = WiFi.localIP().toString();
      Serial.printf("Wi-Fi provisioned: %s, RSSI %d dBm\n", ip.c_str(),
                    WiFi.RSSI());
      setBleStatus("{\"state\":\"connected\",\"ip\":\"" + ip +
                   "\",\"deviceId\":\"" + FIREBASE_DEVICE_ID + "\"}");
      factoryResetInProgress = false;
      synchronizeRtcFromInternet();
      return;
    }

    if (millis() - wifiProvisioningStarted >= WIFI_PROVISION_TIMEOUT_MS) {
      wifiProvisioningPending = false;
      Serial.printf("Wi-Fi provisioning failed, status: %d\n", WiFi.status());
      setBleStatus(
          "{\"state\":\"error\",\"message\":\"Could not join Wi-Fi. Check that it is 2.4 GHz and verify the password.\"}");
    }
    return;
  }

  if (wifiSsid.isEmpty()) return;
  if (WiFi.status() == WL_CONNECTED) {
    synchronizeRtcFromInternet();
    return;
  }
  if (millis() - lastWifiRetry < WIFI_RETRY_INTERVAL_MS) {
    return;
  }
  lastWifiRetry = millis();
  idToken.clear();
  lastTelemetryWrite = 0;  // publish online=true immediately on reconnect
  Serial.println("Wi-Fi disconnected; reconnecting.");
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_TX_POWER);
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
}

void processWifiScan() {
  if (!wifiScanPending) return;
  wifiScanPending = false;

  WiFi.mode(WIFI_STA);
  Serial.println("Scanning nearby Wi-Fi networks.");
  const int count = WiFi.scanNetworks(false, true);
  if (count < 0) {
    setBleStatus(
        "{\"state\":\"error\",\"message\":\"Wi-Fi scan failed. Try again.\"}");
    return;
  }

  for (int i = 0; i < count; ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) continue;
    JsonDocument network;
    network["state"] = "wifiNetwork";
    network["ssid"] = ssid;
    network["rssi"] = WiFi.RSSI(i);
    network["secure"] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    String payload;
    serializeJson(network, payload);
    setBleStatus(payload);
    delay(35);
  }
  WiFi.scanDelete();
  setBleStatus("{\"state\":\"wifiScanComplete\"}");
  Serial.printf("Wi-Fi scan complete: %d networks\n", count);
}

void setup() {
  testEnabled = false;
  atmosphereTestEnabled = false;
  calibrationActive = false;
  circulationFan1 = false;
  exhaustFan = false;
  pinMode(FACTORY_RESET_PIN, INPUT_PULLUP);
  // GPIO 0 is a strapping pin, and this board shares ground with a 12 V heater
  // rail. If it does not read HIGH here, something is holding it down, so the
  // factory-reset gesture is disabled for this boot rather than risking an
  // accidental 8-second hold that wipes every persisted namespace.
  delay(5);
  factoryResetPinHealthy = digitalRead(FACTORY_RESET_PIN) == HIGH;
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_3_PIN, OUTPUT);
  pinMode(FAN_1_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(LID_SWITCH_PIN, INPUT_PULLUP);
  digitalWrite(HEATER_PIN, HEATER_OUTPUT_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(FAN_3_PIN, OUTPUT_OFF);
  digitalWrite(FAN_1_PIN, OUTPUT_OFF);
  digitalWrite(HUMIDIFIER_PIN, OUTPUT_OFF);
  // Servos are attached AFTER loadPersistentState() below so the NVS-saved
  // closed angles are used for the very first pulse (see post-load block).
  Serial.begin(115200);
  if (!factoryResetPinHealthy) {
    Serial.println(
        "WARNING: GPIO 0 reads LOW at boot — factory reset disabled for this "
        "boot to protect saved settings. Check the BOOT button / strapping pin.");
  }
  const esp_reset_reason_t bootResetReason = esp_reset_reason();
  Serial.printf("Boot reset reason: %s (%d)\n",
                resetReasonName(bootResetReason),
                static_cast<int>(bootResetReason));
  recordBootHealth(bootResetReason);
  if (bootResetReason == ESP_RST_BROWNOUT) {
    // The heater/fan 12 V PSU is separate from the ESP32 supply, so this is
    // NOT supply capacity. It is the 12 V switching path corrupting the ESP's
    // ground reference or injecting noise through the MOSFET gate. Check, in
    // this order:
    //   1. Common ground — 12 V PSU GND tied to ESP32 GND with a short, thick
    //      wire, single point, not through the USB cable.
    //   2. MOSFET gate — 100-220 ohm series resistor from GPIO 32 plus a 10k
    //      pull-down gate-to-source. Without these the 12 V drain transient
    //      couples through Cgd straight into GPIO 32.
    //   3. Flyback/snubber — diode across any relay coil, RC snubber across
    //      the heater and fan terminals.
    //   4. Decoupling — 470 uF electrolytic + 100 nF ceramic right at the
    //      ESP32 3V3/GND pins.
    Serial.println(
        "Brownout reset: heater/fan PSU is separate, so suspect ground "
        "reference or MOSFET gate coupling. See wiring checklist above.");
  }
  loadPersistentState();
  // Attach servos NOW — after NVS angles are loaded — so the first PWM
  // pulse the hardware sees is already at the correct closed position.
  // Also sync physicalServoAngle/physicalCircServoAngle so the main loop
  // sees no mismatch on its first tick and never issues a spurious open.
  ventAngle = ventClosedAngle;
  physicalServoAngle = ventClosedAngle;
  ventServo.setPeriodHertz(50);
  ventServo.attach(VENT_SERVO_PIN, 500, 2400);
  ventServo.write(ventClosedAngle);
  physicalCircServoAngle = circServoClosedAngle;
  circServo.setPeriodHertz(50);
  circServo.attach(CIRC_SERVO_PIN, 500, 2400);
  circServo.write(circServoClosedAngle);
  Serial.printf("Servos initialised: vent closed=%u° circ closed=%u°\n",
                ventClosedAngle, circServoClosedAngle);
  testSessionId = String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX) + "-" +
                  String(esp_random(), HEX);
  testTelemetryCleanupPending = true;
  Serial.printf("Hardware test session: %s\n", testSessionId.c_str());
  Serial.println("Boot safety: fan GPIO 25/27 and servos GPIO 13/26 forced OFF.");
  // Circulation fan: simple digitalWrite, no PWM needed.
  pinMode(FAN_1_PIN, OUTPUT);
  writeCirculationFan1(false);
  ledcSetup(HEATER_PWM_CHANNEL, HEATER_PWM_FREQUENCY, HEATER_PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, HEATER_PWM_CHANNEL);
  writeHeaterPwm(0);
  // XY-MOS trigger polarity must match the fitted board, otherwise a 100 % duty
  // command switches the heater off instead of on.
  Serial.printf("Heater XY-MOS trigger polarity: %s\n",
                HEATER_OUTPUT_ACTIVE_LOW ? "ACTIVE_LOW (inverted)"
                                         : "ACTIVE_HIGH");
  ledcSetup(EXHAUST_PWM_CHANNEL, EXHAUST_PWM_FREQUENCY,
            HEATER_PWM_RESOLUTION);
  ledcAttachPin(FAN_3_PIN, EXHAUST_PWM_CHANNEL);
  writeExhaustPwm(0);
  pinMode(DHT_PIN, INPUT_PULLUP);
  pinMode(EGG_DHT_PIN, OUTPUT);
  digitalWrite(EGG_DHT_PIN, LOW);
  pinMode(EGG_DHT_POWER_PIN, OUTPUT);
  digitalWrite(EGG_DHT_POWER_PIN, LOW);
  delay(DHT_POWER_OFF_MS);
  digitalWrite(EGG_DHT_POWER_PIN, HIGH);
  delay(DHT_POWER_STABILIZE_MS);
  pinMode(EGG_DHT_PIN, INPUT_PULLUP);
  configureDhtSensor(dhtPrimary);
  configureDhtSensor(dhtEggZone);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(50);
  const bool i2cBusReady = scanI2cBus();
  rtcHardwareDetected = i2cBusReady && probeI2cAddress(0x68);
  rtcAvailable = rtcHardwareDetected && rtc.begin();
  if (rtcAvailable) {
    rtcRunning = rtc.isrunning();
    const DateTime rtcNow = rtc.now();
    const bool rtcTimeValid = rtcNow.year() >= 2024 && rtcNow.year() <= 2099;
    rtcTimeTrusted = rtcRunning && rtcTimeValid;
    if (rtcTimeTrusted) {
      lastRtcEpoch = rtcNow.unixtime();
      lastRtcMonotonicAt = millis();
      syncSoftwareClock(lastRtcEpoch);
      Serial.println("DS1307 RTC ready; internet verification pending.");
    } else {
      rtcRunning = false;
      Serial.println("DS1307 time invalid; waiting for internet synchronization.");
    }
  } else {
    Serial.println("DS1307 RTC not detected at I2C address 0x68.");
  }
  eggTemperatureSensor.begin();
  heaterTemperatureSensor.begin();
  eggTemperatureSensor.setResolution(11);
  heaterTemperatureSensor.setResolution(11);
  eggTemperatureSensor.setWaitForConversion(false);
  heaterTemperatureSensor.setWaitForConversion(false);
  Serial.printf("DS18B20 probes: egg GPIO %u count=%u, heater GPIO %u count=%u.\n",
                EGG_DS18B20_PIN, eggTemperatureSensor.getDeviceCount(),
                HEATER_DS18B20_PIN, heaterTemperatureSensor.getDeviceCount());
  tlsClient.setCACert(FIREBASE_ROOT_CA_PEM);
  loadCalibrationState();
  loadCycleState();
  loadHeaterModel();
  if (cycleActive) {
    Serial.printf("Restored active %s cycle (%s).\n", cycleMode.c_str(),
                  profileId.c_str());
  }
  startProvisioning();
  if (wifiSsid.isEmpty()) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    Serial.println("Waiting for Wi-Fi credentials from the app.");
  } else {
    Serial.printf("Restoring saved Wi-Fi: %s\n", wifiSsid.c_str());
    WiFi.setTxPower(WIFI_TX_POWER);
    connectWifi();
    if (WiFi.status() == WL_CONNECTED) {
      // Allow the radio and PSU to stabilise before the first HTTPS call.
      delay(300);
      synchronizeRtcFromInternet();
      authenticateFirebase();
    }
  }
  if (!ownerUid.isEmpty()) {
    Serial.printf("Restored owner: %s\n", ownerUid.c_str());
  }
}

void loop() {
  handleFactoryResetButton();
  static unsigned long lastServoPrintAt = 0;
  const unsigned long nowMs = millis();
  if (nowMs - lastServoPrintAt >= 10000UL) {
    lastServoPrintAt = nowMs;
    Serial.printf("[SERVO] vent cfg=%u/%u applied=%u | circ cfg=%u/%u applied=%u\n",
                  ventClosedAngle, ventOpenAngle, physicalServoAngle,
                  circServoClosedAngle, circServoOpenAngle, physicalCircServoAngle);
  }
  // Safety-critical local work always runs before any blocking network call.
  runLocalController();
  reportHeaterOutputMismatch();
  queueHourlyIncubationLog();
  serviceUsbControlLogging();
  processBleRequest();
  processWifiScan();
  maintainWifi();
  const unsigned long now = millis();
  const bool productionTelemetryDue =
      now - lastProductionTelemetryWrite >= PRODUCTION_TELEMETRY_INTERVAL_MS;
  const bool testCommandDue = now - lastTestRead >= TEST_COMMAND_INTERVAL_MS;
  const bool testTelemetryDue = now - lastTestWrite >= TEST_INTERVAL_MS;
  const bool calibrationCloudLogDue =
      calibrationLogPending ||
      (calibrationActive &&
       (lastCalibrationCloudLogAt == 0 ||
        now - lastCalibrationCloudLogAt >= CALIBRATION_CLOUD_LOG_INTERVAL_MS));

  // Slow HTTPS calls can take longer than the polling intervals. Rotate among
  // due tasks so a perpetually due task cannot starve commands or acknowledgements.
  if (WiFi.status() == WL_CONNECTED) {
    constexpr uint8_t CLOUD_TASK_COUNT = 10;
    for (uint8_t offset = 0; offset < CLOUD_TASK_COUNT; ++offset) {
      const uint8_t task = (cloudTaskCursor + offset) % CLOUD_TASK_COUNT;
      bool serviced = true;
      switch (task) {
        case 0:
          if (now - lastTelemetryWrite >= STATUS_INTERVAL_MS)
            publishTelemetry();
          else
            serviced = false;
          break;
        case 1:
          if (factoryResetCloudPending)
            completePendingFactoryReset();
          else
            serviced = false;
          break;
        case 2:
          if (now - lastOwnerCheck >= COMMAND_INTERVAL_MS)
            reconcileCloudOwner();
          else
            serviced = false;
          break;
        case 3:
          if (now - lastClaimRead >= COMMAND_INTERVAL_MS)
            readClaimRequest();
          else
            serviced = false;
          break;
        case 4:
          if (now - lastCommandRead >= COMMAND_INTERVAL_MS)
            readDesiredState();
          else
            serviced = false;
          break;
        case 5:
          if (reportedStatePending &&
              now - lastReportedAttempt >= COMMAND_INTERVAL_MS)
            publishReportedState();
          else
            serviced = false;
          break;
        case 6:
          if (calibrationCloudLogDue)
            publishCalibrationLog();
          else
            serviced = false;
          break;
        case 7:
          if (productionTelemetryDue)
            publishProductionTelemetry();
          else
            serviced = false;
          break;
        case 8:
          if (testTelemetryDue && (!testCommandDue || preferTestTelemetry)) {
            publishTestTelemetry();
            preferTestTelemetry = false;
          } else if (testCommandDue) {
            readTestCommand();
            preferTestTelemetry = true;
          } else {
            serviced = false;
          }
          break;
        case 9:
          if (incubationBatchRecordPending || incubationEventCount > 0)
            publishIncubationHistory();
          else
            serviced = false;
          break;
      }
      if (serviced) {
        cloudTaskCursor = (task + 1) % CLOUD_TASK_COUNT;
        break;
      }
    }
  }
  // Synchronous HTTPS operations can take seconds. Restore active test outputs
  // after every network operation so no stale safe-output write can win.
  if (testEnabled && !cycleActive && !calibrationActive) applyTestOutputs();
  serviceBleSnapshot();
  delay(5);
}
