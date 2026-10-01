#pragma once

#include <Arduino.h>

namespace firmware {

constexpr uint8_t DHT_PIN = 4;
constexpr uint8_t EGG_DHT_PIN = 5;
constexpr uint8_t EGG_DHT_POWER_PIN = 14;
constexpr uint8_t EGG_DS18B20_PIN = 16;
constexpr uint8_t HEATER_DS18B20_PIN = 17;
constexpr uint8_t LID_SWITCH_PIN = 19;
constexpr uint8_t FAN_1_PIN = 25;          // Circulation fan (only one)
constexpr uint8_t CIRC_SERVO_PIN = 26;     // Circulation servo — egg-turn switch (GPIO 26)
constexpr uint8_t FAN_3_PIN = 27;          // Exhaust fan MOSFET
constexpr uint8_t HEATER_PIN = 32;
constexpr uint8_t HUMIDIFIER_PIN = 18;  // moved from GPIO 2 (onboard LED / strapping pin)
constexpr uint8_t VENT_SERVO_PIN = 13;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t FACTORY_RESET_PIN = 0;

constexpr uint8_t OUTPUT_ON = HIGH;
constexpr uint8_t OUTPUT_OFF = LOW;
// XY-MOS heater module trigger polarity. These boards ship in two variants: an
// active-HIGH variant switches the load ON when its control input is driven
// HIGH, and a low-level-trigger variant switches the load ON when the input is
// pulled LOW. Everything else in this firmware assumes active HIGH.
//
// If a low-level-trigger module is fitted, a 255 duty drives its input HIGH and
// switches the heater OFF — the classic "app reports 100 % but the element
// stays cold" symptom, because 100 % duty means the pin is permanently HIGH.
// Set this true for a low-level-trigger module and the PWM is inverted so a
// 255 duty still means full heat.
//
// Verify before changing: disconnect GPIO32 from the module's control input and
// measure that input against the module's GND. If it sits near the module's own
// supply voltage, the module has an internal pull-up and is most likely a
// low-level trigger — and it also needs the 10 kOhm pull-down from the hardware
// guide so it cannot float during ESP32 reset.
constexpr bool HEATER_OUTPUT_ACTIVE_LOW = false;
constexpr uint8_t HEATER_PWM_CHANNEL = 2;
constexpr uint8_t EXHAUST_PWM_CHANNEL = 4;
constexpr uint16_t HEATER_PWM_FREQUENCY = 1000;
constexpr uint32_t EXHAUST_PWM_FREQUENCY = 25000;
constexpr uint8_t HEATER_PWM_RESOLUTION = 8;
// Exhaust vent servo (GPIO 13) default angles
constexpr uint8_t VENT_CLOSED_ANGLE = 10;
constexpr uint8_t VENT_OPEN_ANGLE = 90;
// Circulation servo (GPIO 26) default angles — ON = egg-turn position, OFF = rest position
constexpr uint8_t CIRC_SERVO_CLOSED_ANGLE = 10;
constexpr uint8_t CIRC_SERVO_OPEN_ANGLE = 90;

static_assert(DHT_PIN != FAN_1_PIN &&
                  DHT_PIN != CIRC_SERVO_PIN &&
                  DHT_PIN != FAN_3_PIN && DHT_PIN != HEATER_PIN &&
                  DHT_PIN != HUMIDIFIER_PIN,
              "DHT pin conflicts with an actuator pin");
static_assert(EGG_DHT_PIN != DHT_PIN && EGG_DHT_PIN != EGG_DS18B20_PIN &&
                  EGG_DHT_PIN != HEATER_DS18B20_PIN &&
                  EGG_DHT_PIN != HEATER_PIN && EGG_DHT_PIN != HUMIDIFIER_PIN,
               "Egg-zone DHT pin conflicts with another function");
static_assert(EGG_DHT_POWER_PIN != DHT_PIN &&
                  EGG_DHT_POWER_PIN != EGG_DHT_PIN &&
                  EGG_DHT_POWER_PIN != EGG_DS18B20_PIN &&
                  EGG_DHT_POWER_PIN != HEATER_DS18B20_PIN &&
                  EGG_DHT_POWER_PIN != LID_SWITCH_PIN &&
                  EGG_DHT_POWER_PIN != FAN_1_PIN &&
                  EGG_DHT_POWER_PIN != CIRC_SERVO_PIN &&
                  EGG_DHT_POWER_PIN != FAN_3_PIN &&
                  EGG_DHT_POWER_PIN != HEATER_PIN &&
                  EGG_DHT_POWER_PIN != HUMIDIFIER_PIN &&
                  EGG_DHT_POWER_PIN != VENT_SERVO_PIN &&
                  EGG_DHT_POWER_PIN != I2C_SDA_PIN &&
                  EGG_DHT_POWER_PIN != I2C_SCL_PIN &&
                  EGG_DHT_POWER_PIN != FACTORY_RESET_PIN,
              "Egg-zone DHT power pin conflicts with another function");
static_assert(EGG_DS18B20_PIN != HEATER_DS18B20_PIN &&
                  EGG_DS18B20_PIN != DHT_PIN && HEATER_DS18B20_PIN != DHT_PIN &&
                  EGG_DS18B20_PIN != HEATER_PIN &&
                  HEATER_DS18B20_PIN != HEATER_PIN,
              "DS18B20 pins conflict with another function");
static_assert(FAN_1_PIN != CIRC_SERVO_PIN && FAN_1_PIN != FAN_3_PIN &&
                  FAN_1_PIN != HEATER_PIN && CIRC_SERVO_PIN != HEATER_PIN &&
                  FAN_3_PIN != HEATER_PIN,
              "Fan/servo/heater pin assignments must be unique");
static_assert(I2C_SDA_PIN != I2C_SCL_PIN && I2C_SDA_PIN != HEATER_PIN &&
                  I2C_SCL_PIN != HEATER_PIN,
              "I2C pins conflict with another function");

struct IncubationProfile {
  const char *id;
  const char *name;
  float temperature;
  float normalHumidity;
  float hatchHumidity;
  uint8_t totalDays;
  uint8_t lockdownDay;
  uint8_t turnIntervalHours;
};

const IncubationProfile *findProfile(const String &id);

}  // namespace firmware
