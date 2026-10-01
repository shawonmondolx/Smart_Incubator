// Standalone XY-MOS heater output test.
//
// Deliberately minimal: no Wi-Fi, no Firebase, no sensors, no control loop, no
// safety gates. It exists to answer the two questions behind "the app shows
// 100 % but the heater is cold and the ESP32 restarts", with every other
// subsystem removed as a variable:
//
//   Fault A — does the heater heat at all when GPIO 32 is driven?
//   Fault B — does the ESP32 reset when the heater switches?
//
// Build and flash this INSTEAD of the main firmware:
//     pio run -e heater-test -t upload
// Then restore the real firmware with:
//     pio run -e nodemcu-32s -t upload
//
// Serial monitor at 115200 baud. Commands:
//     0-255      set duty directly      e.g. 255 = full heat
//     <n>%       set duty as a percentage, e.g. 100%
//     a          auto-cycle 0 -> 64 -> 128 -> 192 -> 255, 20 s per step
//     s          stop (duty 0)
//     i          toggle output polarity, active HIGH <-> active LOW
//     r          report current state
//
// Reading the results:
//   - Set 100 % duty, wait 60 s.
//       Element gets hot                       -> active HIGH is correct
//       Element stays cold but 0 % heats it    -> low-level trigger module,
//                                                 leave polarity on ACTIVE_LOW
//       Element never heats at any setting     -> 12 V rail, fuse, XY-MOS power
//                                                 wiring or the element itself
//   - Watch the serial output. If it restarts, the boot banner prints the reset
//     reason and the cycle count, so a heater-triggered reset is unmistakable.
//
// SAFETY: this sketch drives the heater with no over-temperature protection and
// no sensor interlock. Run it with the chamber empty, attended, and never
// unattended with eggs loaded.

#include <Arduino.h>

constexpr uint8_t HEATER_PIN = 32;
constexpr uint8_t HEATER_PWM_CHANNEL = 2;
constexpr uint16_t HEATER_PWM_FREQUENCY = 1000;
constexpr uint8_t HEATER_PWM_RESOLUTION = 8;
constexpr unsigned long HEARTBEAT_MS = 5000;

// Start with the firmware's assumption. Toggle at runtime with 'i'.
bool activeLow = false;
uint8_t duty = 0;
bool autoMode = false;
uint8_t autoIndex = 0;
unsigned long lastAutoStepAt = 0;
unsigned long lastHeartbeatAt = 0;
uint32_t bootCounter = 0;

constexpr uint8_t AUTO_STEPS[] = {0, 64, 128, 192, 255};
constexpr size_t AUTO_STEP_COUNT = sizeof(AUTO_STEPS) / sizeof(AUTO_STEPS[0]);
constexpr unsigned long AUTO_STEP_MS = 20000;

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
      return "BROWNOUT";
    case ESP_RST_DEEPSLEEP:
      return "deep_sleep_wake";
    default:
      return "unknown";
  }
}

// The only place the pin is written. `duty` is always the logical heat level,
// so 255 means full heat regardless of which trigger polarity is selected.
void applyDuty() {
  const uint8_t level =
      activeLow ? static_cast<uint8_t>(255 - duty) : duty;
  ledcWrite(HEATER_PWM_CHANNEL, level);
}

void reportState() {
  const uint8_t level =
      activeLow ? static_cast<uint8_t>(255 - duty) : duty;
  Serial.printf(
      "STATE duty=%u/255 (%u%%) pinLevel=%u polarity=%s auto=%s\n",
      static_cast<unsigned>(duty),
      static_cast<unsigned>((duty * 100U + 127U) / 255U),
      static_cast<unsigned>(level), activeLow ? "ACTIVE_LOW" : "ACTIVE_HIGH",
      autoMode ? "on" : "off");
}

void handleCommand(String line) {
  line.trim();
  if (line.isEmpty()) return;

  if (line.equalsIgnoreCase("a")) {
    autoMode = true;
    autoIndex = 0;
    duty = AUTO_STEPS[0];
    lastAutoStepAt = millis();
    applyDuty();
    Serial.println("Auto cycle started: 0, 64, 128, 192, 255 every 20 s.");
    reportState();
    return;
  }
  if (line.equalsIgnoreCase("s")) {
    autoMode = false;
    duty = 0;
    applyDuty();
    Serial.println("Stopped.");
    reportState();
    return;
  }
  if (line.equalsIgnoreCase("i")) {
    activeLow = !activeLow;
    applyDuty();
    Serial.printf("Polarity toggled to %s\n",
                  activeLow ? "ACTIVE_LOW" : "ACTIVE_HIGH");
    reportState();
    return;
  }
  if (line.equalsIgnoreCase("r")) {
    reportState();
    return;
  }

  // Numeric: "100%" or a raw 0-255 duty.
  int requested = 0;
  if (line.endsWith("%")) {
    const int percent = line.substring(0, line.length() - 1).toInt();
    requested = constrain(percent, 0, 100) * 255 / 100;
  } else {
    requested = line.toInt();
  }
  autoMode = false;
  duty = static_cast<uint8_t>(constrain(requested, 0, 255));
  applyDuty();
  reportState();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  const esp_reset_reason_t reason = esp_reset_reason();
  bootCounter++;
  Serial.println();
  Serial.println("=== XY-MOS HEATER OUTPUT TEST ===");
  Serial.printf("Boot reset reason: %s (%d)\n", resetReasonName(reason),
                static_cast<int>(reason));
  if (reason == ESP_RST_BROWNOUT) {
    Serial.println(
        "*** BROWNOUT at boot. If this repeats while the heater is driven, the "
        "12 V switching path is resetting the ESP32. Check the common ground "
        "and the 10 kOhm pull-down on the XY-MOS control input first. ***");
  }
  Serial.printf("Heater on GPIO %u, PWM %u Hz, 8-bit.\n",
                static_cast<unsigned>(HEATER_PIN),
                static_cast<unsigned>(HEATER_PWM_FREQUENCY));

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  ledcSetup(HEATER_PWM_CHANNEL, HEATER_PWM_FREQUENCY, HEATER_PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, HEATER_PWM_CHANNEL);
  duty = 0;
  applyDuty();

  Serial.println("Output held OFF. Commands: 0-255 | <n>% | a | s | i | r");
  Serial.println("Start with: 100%");
  reportState();
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  const unsigned long now = millis();
  if (autoMode && now - lastAutoStepAt >= AUTO_STEP_MS) {
    lastAutoStepAt = now;
    autoIndex = static_cast<uint8_t>((autoIndex + 1) % AUTO_STEP_COUNT);
    duty = AUTO_STEPS[autoIndex];
    applyDuty();
    reportState();
  }

  // Heartbeat makes a reset obvious: the counter restarts from 1.
  if (now - lastHeartbeatAt >= HEARTBEAT_MS) {
    lastHeartbeatAt = now;
    Serial.printf("alive duty=%u/255 polarity=%s uptime=%lus\n",
                  static_cast<unsigned>(duty),
                  activeLow ? "ACTIVE_LOW" : "ACTIVE_HIGH",
                  static_cast<unsigned long>(now / 1000UL));
  }
}
