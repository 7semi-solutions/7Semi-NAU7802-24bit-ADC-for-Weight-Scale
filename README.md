# 7Semi NAU7802 – 24-bit Load Cell ADC Library

**7Semi_NAU7802** is an Arduino-compatible library for the **NAU7802 24-bit low-noise ADC**, commonly used in **load cell and weighing scale applications**.  

---

## Features

- 24-bit signed ADC reading (high resolution)
- I2C interface (Arduino / ESP compatible)
- Programmable Gain Amplifier (PGA 1–128)
- Internal LDO voltage configuration
- Selectable conversion rates
- Raw ADC read and averaged read
- Tare (zero) functionality
- Scale calibration using known weight
- Weight output in user-defined units
- User-friendly examples with Serial prompts
- ESP32 / ESP8266 custom SDA/SCL support

---

## Supported Platforms

- Arduino Uno / Nano / Mega
- ESP32
- ESP8266
- Any Arduino-compatible board with I2C support

---

## Installation

### Arduino Library Manager
1. Open Arduino IDE  
2. Go to **Sketch → Include Library → Manage Libraries**  
3. Search for **7Semi_NAU7802**  
4. Click **Install**

### Manual Installation
1. Download this repository as ZIP  
2. Extract into your Arduino `libraries` folder  
3. Restart Arduino IDE  

---

## Hardware Connections

### NAU7802 Module → MCU

| NAU7802 Pin | MCU Pin |
|------------|---------|
| VCC | 3.3V (recommended) |
| GND | GND |
| SDA | SDA |
| SCL | SCL |

> ⚠️ Some NAU7802 modules support 5V. Always check your module datasheet.

---

### Load Cell (4-wire) Connections

**Most common color code:**

| Load Cell Wire | NAU7802 Pin |
|---------------|------------|
| Red | E+ (Excitation +) AVDD |
| Black | E- (Excitation -) GND |
| Green | A+ (Signal +) |
| White | A- (Signal -) |

**Important Notes:**
- Wire colors are common but **not universal**
- Always verify with the load cell datasheet
- If weight is inverted, swap **A+ and A-**

---

## Basic Usage Flow

1. Initialize sensor using `begin()`
2. Configure LDO and PGA
3. Tare with empty platform
4. Calibrate using known weight
5. Read weight continuously

---
