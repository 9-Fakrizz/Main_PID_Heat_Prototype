# Main_PID_Heat_Prototype

This repository contains a temperature control prototype built on **ESP32** using **PID control**, **MAX31855 thermocouple sensors**, and **solid state relays (SSR)** to drive AC heating elements.

The project is designed for systems that heat up quickly and cool down slowly, with intentional conservative PID tuning and sensor delay compensation.

---

## Features

* Dual K-type thermocouple measurement using MAX31855 (SPI).
* Slow, delay-aware PID control suitable for AC heaters.
* Time-proportional SSR control (windowed output).
* Sensor filtering and thermal lag modeling.
* IR temperature sensing via I2C (GY-906 / MLX90614).
* OLED display (SSD1306) for UI and status.
* Serial interface for setpoint adjustment.
* Buzzer feedback for button, confirm, and alarm events.

---

## Hardware Requirements

### Core Components

* ESP32 development board
* 2 × MAX31855 thermocouple amplifier
* 2 × K-type thermocouples
* 2 × Solid State Relays (AC type, zero-cross recommended)
* AC heating elements
* SSD1306 OLED display (128×64, I2C)
* GY-906 (MLX90614) IR temperature sensor
* Piezo buzzer
* Push buttons / rotary encoder (optional UI)

---

## Build Instructions

### 1. Wiring Overview

* Power ESP32 via USB or regulated 5V/3.3V
* Thermocouples connect to MAX31855 boards
* MAX31855 boards connect via SPI
* SSRs are driven by ESP32 GPIO pins
* OLED and IR sensor share I2C bus
* Buzzer connected to GPIO with optional resistor

### 2. Install Required Libraries

Install the following libraries via Arduino Library Manager:

* Adafruit MAX31855
* PID_v1
* Adafruit SSD1306
* Adafruit GFX
* Adafruit MLX90614 (for GY-906)

---

## Pin Mapping

### SPI – MAX31855

| Signal | ESP32 Pin |
| ------ | --------- |
| SCK    | GPIO 14   |
| MISO   | GPIO 12   |
| CS1    | GPIO 18   |
| CS2    | GPIO 19   |

Each MAX31855 uses a **separate CS pin** but shares SCK and MISO.

---

### Solid State Relays

| Heater | ESP32 Pin |
| ------ | --------- |
| SSR1   | GPIO 13   |
| SSR2   | GPIO 4    |

SSR input must be **DC compatible** (3.3V logic).

---

### I2C Devices

| Device            | SDA     | SCL     |
| ----------------- | ------- | ------- |
| OLED SSD1306      | GPIO 21 | GPIO 22 |
| GY-906 (MLX90614) | GPIO 21 | GPIO 22 |

---

### Buzzer

| Function | ESP32 Pin |
| -------- | --------- |
| Buzzer   | GPIO 27   |

---

### Buttons (Optional UI)

| Button   | ESP32 Pin |
| -------- | --------- |
| Button 1 | GPIO 25   |
| Button 2 | GPIO 26   |
| Button 3 | GPIO 33   |

---

## Circuit Description

### Heater Control (AC)

* AC Live line passes through SSR → Heater → AC Neutral
* ESP32 GPIO controls SSR input (optocoupled)
* Zero-cross SSR is recommended to reduce EMI

### Thermocouple Measurement

* Thermocouple connects directly to MAX31855
* MAX31855 performs cold-junction compensation
* ESP32 reads temperature via SPI

### Sensor Filtering

Raw thermocouple data is filtered using a **first-order low-pass model** to simulate thermal delay and reduce overshoot.

---

## Software Architecture

### Control Flow

1. Read all sensors (thermocouples, IR sensor)
2. Filter sensor data
3. Apply PID control
4. Drive SSRs using time-proportional window
5. Update display and handle user input
6. Provide sound feedback if needed

---

## PID Control Strategy

* PID input: filtered thermocouple temperature
* PID output: ON-time within a fixed window
* Conservative tuning to avoid overshoot
* Designed for systems with:

  * Fast heating
  * Slow cooling
  * Sensor lag

---

## Calibration Guide

### 1. Thermocouple Calibration

1. Place thermocouple in known temperature reference:

   * Ice water (0°C)
   * Boiling water (100°C at sea level)
2. Compare raw readings:

   ```
   Raw1, Raw2
   ```
3. Apply offset correction in code if required.

---

### 2. IR Sensor Calibration (GY-906)

* IR sensor measures surface temperature, not internal heater temperature.
* Use IR sensor mainly for:

  * Safety
  * Cross-checking
  * Over-temperature alarm

If needed, apply calibration offset:

```cpp
irTemp1 = rawIrTemp + offset;
```

---

### 3. PID Tuning Procedure

1. Start with:

   * Low Kp
   * Very low Ki
   * Kd = 0
2. Increase Kp until system responds without oscillation.
3. Slowly increase Ki to remove steady-state error.
4. Do not use aggressive tuning due to sensor delay.

---

### 4. Window Size Adjustment

* Typical values: 2000–5000 ms
* Larger window:

  * Smoother control
  * Less SSR switching
* Smaller window:

  * Faster response
  * Higher switching frequency

---

## Serial Commands

```
T1=xxx     Set heater 1 target temperature
T2=xxx     Set heater 2 target temperature
STATUS     Print system status
```

Example:

```
T1=120
T2=90
STATUS
```

---

## Safety Notes

* AC mains voltage is dangerous.
* Use proper isolation, fuses, and grounding.
* Never touch heater or SSR wiring while powered.
* Add hardware thermal cutoff if possible.

---

## Future Improvements

* EEPROM storage for setpoints
* Auto-tuning PID
* Web UI via WiFi
* Data logging
* Multi-zone thermal profiles

---


บอกผมได้ ผมจะจัดเอกสารเพิ่มให้เหมือนโปรเจกต์ระดับ production controller เลย
