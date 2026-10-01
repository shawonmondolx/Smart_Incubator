# Automatic Incubator Hardware and Commissioning

This controller is a prototype, not a certified life-safety appliance. Incubation presets require calibrated instruments. Commission the complete chamber without eggs before use.

## ESP32 Connections

| Function | ESP32 | Connection |
|---|---:|---|
| Air-intake DHT22 data | GPIO4 | DATA with a 4.7-10 kOhm pull-up to 3.3 V |
| Egg-zone DHT22 data | GPIO5 | DATA with its own 4.7-10 kOhm pull-up to 3.3 V |
| Egg-zone DS18B20 data | GPIO16 | DATA with a 4.7 kOhm pull-up to 3.3 V |
| Heater DS18B20 data | GPIO17 | DATA with a 4.7 kOhm pull-up to 3.3 V |
| DS1307 SDA | GPIO21 | SDA through a 3.3 V-safe I2C interface |
| DS1307 SCL | GPIO22 | SCL through a 3.3 V-safe I2C interface |
| XY-MOS heater input | GPIO32 | PWM control input, 1 kHz, 8-bit |
| Circulation fan 1 driver | GPIO25 | MOSFET/driver or optocoupler input |
| Egg-tray servo signal | GPIO26 | Servo signal only |
| Exhaust fan driver | GPIO27 | MOSFET/driver or optocoupler input |
| Humidifier and its fan driver | GPIO18 | Relay/MOSFET/optocoupler input; use a 10 kOhm pull-down |
| SG90 vent servo signal | GPIO13 | Servo signal only |

Never power a load from an ESP32 GPIO. Confirm each driver input polarity before connecting a load. The firmware assumes active-HIGH drivers and initializes all outputs OFF.

Fit 10 kOhm pull-down resistors at active-HIGH heater, humidifier, fan, and tray driver inputs unless the module data sheet guarantees an inactive input during ESP32 reset. Firmware cannot control GPIO levels before boot completes.

## Power and Grounding

- Use a fused 12 V supply sized for the PTC heater startup current, all 12 V fans, humidifier interface, and at least 25% margin.
- Select wire gauge, connectors, MOSFETs, relays, and fuses for measured continuous and startup current, not only label current.
- Feed the PTC heater from the fused 12 V rail through the XY-MOS power terminals. GPIO32 connects only to its control input.
- Mount the XY-MOS and heater with airflow and heat sinking appropriate to measured dissipation. Keep hot parts away from plastic, wiring, eggs, water, and combustible material.
- Power both servos from a separate regulated 5 V supply capable of their combined stall current. Add 470-1000 uF bulk capacitance near the servos plus a 0.1 uF ceramic capacitor.
- Join ESP32 ground, 12 V low-voltage ground, 5 V servo ground, and non-isolated driver grounds at a deliberate common point. Do not join isolated mains wiring to this low-voltage ground.
- A DS1307 is normally a 5 V device. Many modules pull SDA/SCL up to module VCC. Do not expose ESP32 GPIO21/22 to 5 V; use a bidirectional I2C level shifter or a verified 3.3 V-safe module. A DS3231 module designed for 3.3 V logic is preferable.

## Actuator Notes

- Both circulation fans run continuously while a cycle is active.
- Exhaust airflow is supplemental. Provide permanent passive inlet and outlet openings; the servo vent must not be the chamber's only air path.
- Keep humidifier water and condensation physically separated from mains voltage and electronics. Use an optically isolated, correctly rated switching stage when controlling a mains humidifier.
- GPIO26 controls the egg-tray servo between its calibrated rest and turn angles.
- Fit flyback protection across DC relay coils, motors, solenoids, and fans unless the selected driver module already provides it.
- Hardware-test commands use a five-second firmware lease; the open test screen refreshes it every two seconds. Loss of commands forces every test output OFF. Production-cycle control always has priority.

## Independent Safety

- Install a correctly rated fuse at the 12 V supply output and branch protection where conductor sizes change.
- Install a non-resettable thermal fuse in the heater thermal path.
- Install an independent thermostat or over-temperature cutout wired to remove heater power without software, Wi-Fi, Firebase, or ESP32 involvement.
- Use flame-retardant enclosures, strain relief, insulated terminals, protective earth where required, and separation between mains and SELV wiring.
- Mains wiring can cause fatal shock or fire. It must be enclosed and completed by a qualified person in accordance with local electrical rules.
- The software overheat response removes PWM heat, starts exhaust, and opens the vent at target +1.0 C or 39.5 C. This does not replace independent hardware protection.

## Automatic Profiles

These forced-air prototype presets are based on [Mississippi State University Extension](https://extension.msstate.edu/agriculture/livestock/poultry/important-incubation-factors) and [Oklahoma State University Extension](https://extension.okstate.edu/fact-sheets/artificial-incubation). Waterfowl cooling and spraying are not automated.

| Species | Temperature | Normal / hatch RH | Days | Lockdown / stop turning |
|---|---:|---:|---:|---:|
| Chicken | 37.5 C | 58% / 70% | 21 | Day 18 |
| Duck | 37.5 C | 60% / 70% | 28 | Day 25 |
| Muscovy duck | 37.5 C | 60% / 72% | 35 | Day 31 |
| Coturnix quail | 37.5 C | 58% / 70% | 17 | Day 15 |
| Pigeon | 37.5 C | 58% / 70% | 17 | Day 15 |
| Turkey | 37.2 C | 60% / 72% | 28 | Day 25 |
| Goose | 37.2 C | 62% / 72% | 30 | Day 25 |
| Guinea fowl | 37.5 C | 58% / 70% | 28 | Day 25 |

All profiles turn every two hours before lockdown.

## Calibration

1. Place a traceable thermometer and hygrometer at egg height, away from direct heater, mist, wall, and fan discharge.
2. Run the empty chamber for at least 24 hours at the chicken profile.
3. Compare readings at several chamber positions. Correct airflow and sensor placement before considering any software offset.
4. Verify the heater can settle without rapid full-power cycling and that humidity hysteresis does not create condensation.
5. Verify the exhaust opens the vent and reduces temperature/humidity.
6. Mark servo linkages only after confirming 10 degrees is safely closed and 90 degrees cannot stall the mechanism.
7. Confirm one tray pulse moves the mechanism the intended distance and cannot jam at an end stop.

## No-Egg Commissioning and Failure Tests

Complete and record every test before loading eggs:

- Power-on: heater, humidifier, tray, and all test outputs remain OFF until commanded.
- Start acknowledgement: the app remains on confirmation until the ESP32 accepts the profile.
- Restart: disconnect power during a cycle; verify RTC-based day, profile, and next turn restore from NVS.
- Offline operation: disconnect Wi-Fi and verify temperature, humidity, fans, vent, and turning continue locally.
- DHT fault: disconnect either DATA line; the cycle must pause and every actuator command must turn OFF.
- Redundant DHT22: place both at egg height in different chamber areas. A missing sensor or more than 1.0 C / 10% RH disagreement must pause the cycle.
- Heater response: below target with a substantial heat command, verify at least a 0.2 C rise within eight minutes or a latched fault.
- Humidifier response: below target, verify at least a 2% RH rise within ten minutes or a latched fault.
- Exhaust response: above its threshold, verify at least a 0.2 C temperature drop or 2% RH drop within six minutes or a latched fault.
- Repair and acknowledgement: response faults remain latched across reboot. Correct the hardware first, then acknowledge the fault in the app; failed hardware will latch again.
- RTC fault: disconnect I2C during a cycle; environmental safety must continue and tray turning must be suppressed.
- Overheat: simulate a safe sensor value above target +1.0 C; heater must be OFF, exhaust ON, and vent open.
- Lockdown: use a test RTC/time setup to confirm hatch RH begins and all turning stops on the configured day.
- Tray: verify each pulse lasts about 10 seconds and the next schedule survives restart.
- Abort/completion: verify all production outputs return to their safe state.
- Mode exclusion: verify hardware-test commands cannot operate outputs during an active production cycle.
- Independent cutout: test the thermostat and thermal protection by removing heater power independently of the ESP32.

Do not commission with eggs until temperature uniformity, humidity stability, airflow, tray travel, independent protection, and all injected faults have passed.
