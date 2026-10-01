// ─────────────────────────────────────────────────────────────────────────────
// Isolated servo hardware test — vent servo GPIO 13, egg-tray servo GPIO 26.
//
// Purpose: determine whether the servo hardware connection itself is what
// disturbs the ESP32 (and therefore what makes saved settings appear to reset).
// No Wi-Fi, no cloud, no sensors, no control loop, no safety gates, no heater.
// The servos are the only load, so anything that happens is attributable to
// them.
//
// SAFETY: this sketch has no over-temperature protection of any kind. Leave the
// heater disconnected (or powered down) while running it.
//
// Flash with:  pio run -e servo-test -t upload
// Monitor with: pio device monitor -e servo-test
//
// Serial commands (115200 baud, newline terminated):
//   v <0-180>      move the vent servo (GPIO 13) to an angle
//   c <0-180>      move the egg-tray servo (GPIO 26) to an angle
//   o / x          vent servo to its open (90) / closed (10) position
//   1 / 2          sweep that servo slowly across 0-180 and back
//   b              sweep BOTH servos together — the worst-case current test
//   t <sec>        hold both servos at their current angles for <sec> seconds
//                  (stall-current test; watch the heartbeat for resets)
//   d              detach both servos (relax — removes holding torque)
//   a             re-attach both servos
//   r              report reset reason, brownout count and current state
//   h              help
//
// How to read the result:
//   The board prints a heartbeat every 5 s with an incrementing tick and the
//   cumulative brownout count. If a tick goes missing, or the counter climbs,
//   or "Boot reset reason:" prints again while you are driving a servo, then
//   moving that servo is resetting the ESP32 — fix the servo power/ground
//   before touching firmware again.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <esp_system.h>

// Must match include/firmware_config.h — duplicated deliberately so this test
// has no dependency on the main firmware's headers.
constexpr uint8_t VENT_SERVO_PIN = 13;
constexpr uint8_t CIRC_SERVO_PIN = 26;
constexpr uint8_t VENT_CLOSED_ANGLE = 10;
constexpr uint8_t VENT_OPEN_ANGLE = 90;
constexpr uint8_t CIRC_CLOSED_ANGLE = 10;
constexpr uint8_t CIRC_OPEN_ANGLE = 90;

Servo ventServo;
Servo circServo;

bool ventAttached = false;
bool circAttached = false;
uint8_t ventAngle = VENT_CLOSED_ANGLE;
uint8_t circAngle = CIRC_CLOSED_ANGLE;

unsigned long lastHeartbeat = 0;
uint32_t tick = 0;
uint32_t moveCount = 0;

Preferences preferences;

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:  return "power_on";
    case ESP_RST_EXT:      return "external_pin";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "PANIC (crash)";
    case ESP_RST_INT_WDT:  return "INT_WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK_WATCHDOG";
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_DEEPSLEEP:return "deep_sleep_wake";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply sagged)";
    case ESP_RST_SDIO:     return "sdio";
    default:               return "unknown";
  }
}

// The main firmware records these in the same "health" namespace, so the
// counters continue across this test and across re-flashes. A rising brownout
// count is the definitive evidence that the load is sagging the supply.
void reportHealth() {
  uint32_t boots = 0;
  uint32_t brownouts = 0;
  if (preferences.begin("health", true)) {
    boots = preferences.getUInt("boots", 0);
    brownouts = preferences.getUInt("brownouts", 0);
    preferences.end();
  }
  const esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("Reset reason: %s (%d)\n", resetReasonName(reason),
                static_cast<int>(reason));
  Serial.printf("Health NVS: boot #%u, cumulative brownout resets %u\n",
                static_cast<unsigned>(boots), static_cast<unsigned>(brownouts));
  if (reason == ESP_RST_BROWNOUT) {
    Serial.println(
        ">>> The last reset was a BROWNOUT. The servo supply is sagging the "
        "ESP32 rail. Give the servos their own regulated 5 V supply with "
        "470-1000 uF bulk capacitance, and bond all grounds at one point.");
  }
}

void attachServos() {
  if (!ventAttached) {
    ventServo.setPeriodHertz(50);
    ventServo.attach(VENT_SERVO_PIN, 500, 2400);
    ventAttached = true;
    ventServo.write(ventAngle);
    Serial.printf("Vent servo attached on GPIO %u at %u deg.\n", VENT_SERVO_PIN,
                  ventAngle);
  }
  if (!circAttached) {
    circServo.setPeriodHertz(50);
    circServo.attach(CIRC_SERVO_PIN, 500, 2400);
    circAttached = true;
    circServo.write(circAngle);
    Serial.printf("Egg-tray servo attached on GPIO %u at %u deg.\n",
                  CIRC_SERVO_PIN, circAngle);
  }
}

void detachServos() {
  if (ventAttached) {
    ventServo.detach();
    ventAttached = false;
  }
  if (circAttached) {
    circServo.detach();
    circAttached = false;
  }
  Serial.println("Both servos detached — no holding torque, no current draw.");
}

void writeVent(uint8_t angle) {
  attachServos();
  ventAngle = angle;
  ventServo.write(ventAngle);
  ++moveCount;
  Serial.printf("[move %lu] vent -> %u deg\n", static_cast<unsigned long>(moveCount),
                ventAngle);
}

void writeCirc(uint8_t angle) {
  attachServos();
  circAngle = angle;
  circServo.write(circAngle);
  ++moveCount;
  Serial.printf("[move %lu] egg-tray -> %u deg\n",
                static_cast<unsigned long>(moveCount), circAngle);
}

// Slow sweep so the servo is actually driving, not just being told a number.
void sweepServo(Servo &servo, const char *label) {
  attachServos();
  Serial.printf("Sweeping %s 0 -> 180 -> 0. Watch the heartbeat.\n", label);
  for (int angle = 0; angle <= 180; angle += 5) {
    servo.write(angle);
    delay(60);
  }
  for (int angle = 180; angle >= 0; angle -= 5) {
    servo.write(angle);
    delay(60);
  }
  Serial.printf("Sweep of %s finished with no reset.\n", label);
}

// Both servos driven at once: peak simultaneous current, the case most likely
// to expose an inadequate servo supply or a missing common ground.
void sweepBoth() {
  attachServos();
  Serial.println(
      "Sweeping BOTH servos together — worst-case current. Watch the heartbeat.");
  for (int angle = 0; angle <= 180; angle += 5) {
    ventServo.write(angle);
    circServo.write(angle);
    delay(60);
  }
  for (int angle = 180; angle >= 0; angle -= 5) {
    ventServo.write(angle);
    circServo.write(angle);
    delay(60);
  }
  Serial.println("Simultaneous sweep finished with no reset.");
}

void holdBoth(unsigned long seconds) {
  attachServos();
  Serial.printf(
      "Holding both servos at vent=%u, egg-tray=%u for %lu s. If a mechanism "
      "is mechanically stalled this draws stall current continuously.\n",
      ventAngle, circAngle, seconds);
  const unsigned long started = millis();
  while (millis() - started < seconds * 1000UL) {
    // Keep re-issuing the pulse: a stalled servo that gets no signal relaxes.
    ventServo.write(ventAngle);
    circServo.write(circAngle);
    delay(200);
  }
  Serial.println("Hold finished with no reset.");
}

void printHelp() {
  Serial.println(
      "\nCommands:\n"
      "  v <0-180>   vent servo (GPIO 13) to angle\n"
      "  c <0-180>   egg-tray servo (GPIO 26) to angle\n"
      "  o / x       vent servo open (90) / closed (10)\n"
      "  1 / 2       sweep that servo 0-180-0\n"
      "  b           sweep BOTH servos together (worst-case current)\n"
      "  t <sec>     hold both servos for <sec> seconds (stall test)\n"
      "  d / a       detach / re-attach both servos\n"
      "  r           report reset reason, brownout count, state\n"
      "  h           this help\n");
}

void handleCommand(String line) {
  line.trim();
  if (line.isEmpty()) return;
  const char cmd = tolower(line[0]);
  const int value = line.length() > 1 ? line.substring(1).toInt() : -1;

  switch (cmd) {
    case 'v':
      if (value < 0 || value > 180) { Serial.println("Angle must be 0-180."); return; }
      writeVent(static_cast<uint8_t>(value));
      return;
    case 'c':
      if (value < 0 || value > 180) { Serial.println("Angle must be 0-180."); return; }
      writeCirc(static_cast<uint8_t>(value));
      return;
    case 'o': writeVent(VENT_OPEN_ANGLE); return;
    case 'x': writeVent(VENT_CLOSED_ANGLE); return;
    case '1': sweepServo(ventServo, "vent servo (GPIO 13)"); return;
    case '2': sweepServo(circServo, "egg-tray servo (GPIO 26)"); return;
    case 'b': sweepBoth(); return;
    case 't':
      if (value <= 0 || value > 600) { Serial.println("Hold 1-600 seconds."); return; }
      holdBoth(static_cast<unsigned long>(value));
      return;
    case 'd': detachServos(); return;
    case 'a': attachServos(); return;
    case 'r':
      reportHealth();
      Serial.printf("tick=%lu moves=%lu vent=%u%s egg-tray=%u%s\n",
                    static_cast<unsigned long>(tick),
                    static_cast<unsigned long>(moveCount), ventAngle,
                    ventAttached ? " (attached)" : " (detached)", circAngle,
                    circAttached ? " (attached)" : " (detached)");
      return;
    case 'h':
    default:
      printHelp();
      return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(
      "\n=== Smart Incubator — isolated servo hardware test ===");
  reportHealth();
  Serial.println(
      "No heater, no Wi-Fi, no sensors. The servos are the only load.");
  printHelp();

  // Park both servos at their calibrated closed position on boot.
  attachServos();
  writeVent(VENT_CLOSED_ANGLE);
  writeCirc(CIRC_CLOSED_ANGLE);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  // Heartbeat: a missing tick, or the counter restarting, means the board reset
  // while you were driving a servo.
  const unsigned long now = millis();
  if (now - lastHeartbeat >= 5000UL) {
    lastHeartbeat = now;
    ++tick;
    Serial.printf("[heartbeat] tick=%lu uptime=%lus moves=%lu\n",
                  static_cast<unsigned long>(tick),
                  static_cast<unsigned long>(now / 1000UL),
                  static_cast<unsigned long>(moveCount));
  }
  delay(5);
}
