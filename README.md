# Weather-Monitoring-System-using-RPi-PICO-Microcontroller

This repository contains the source code, schematics, and documentation for a cost-effective and compact Weather Monitoring System built around the Raspberry Pi Pico microcontroller. The system integrates several sensors to measure atmospheric pressure, temperature, humidity, and air quality, while logging data onto an SD card.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Components](#components)
- [System Architecture](#system-architecture)
- [Installation and Setup](#installation-and-setup)
- [Usage](#usage)
- [Future Enhancements](#future-enhancements)
- [References](#references)

---

## Overview

The Weather Monitoring System (WMS) is designed as a versatile embedded solution for real-time environmental sensing. Utilizing the Raspberry Pi Pico, the system gathers and logs data from:
- **BMP280** for atmospheric pressure and temperature,
- **DHT11** for temperature and humidity,
- **MQ135** for air quality (detecting gases like CO₂, NH₃, benzene, and alcohol), and
- **SD Card Module** for local data storage.

This project is ideal for applications in meteorology, agriculture, aviation, disaster management, and educational research.

---

## Features

- **Cost-Effective Design:** Total BOM cost is kept minimal.
- **Compact and Lightweight:** Small form factor (21×51 mm) suitable for portable deployments.
- **Modular Sensor Integration:** Multiple sensors integrated via UART, I²C, and SPI interfaces.
- **Data Logging:** Continuous logging of sensor data in CSV format to an SD card.
- **Low Power Consumption:** Efficient operation at 3.3 V with a low power draw.
- **Scalability:** Foundation for future IoT connectivity and advanced data analytics.

---

## Components

| Component           | Description                                        | Approx. Cost (INR) |
|---------------------|----------------------------------------------------|--------------------|
| Raspberry Pi Pico   | Microcontroller (RP2040, dual-core, 133 MHz)       | 349                |
| BMP280              | Pressure & Temperature Sensor (±0.12 hPa accuracy)  | 30                 |
| DHT11               | Temperature & Humidity Sensor (±2°C, ±5% RH)        | 48                 |
| MQ135               | Air Quality Sensor (detects CO₂, NH₃, etc.)         | 96                 |
| SD Card Module      | SPI-based Data Logging Module                      | 40                 |
| Breadboard & Jumpers| For prototyping and circuit connections            | 200                |

*Note: Prices and component specifications are based on project estimates.*

---

## System Architecture

### Circuit Diagram

The hardware setup includes:
- **Raspberry Pi Pico:** Serving as the central controller.
- **BMP280:** Connected via I²C (SDA & SCL).
- **DHT11:** Digital connection for temperature and humidity.
- **MQ135:** Analog output connected to an ADC pin.
- **SD Card Module:** Connected via SPI (MISO, MOSI, SCK, CS).

All sensors are powered by the 3.3 V output of the Pico.

### Data Flow

1. **Initialization:** Power applied and sensors are calibrated.
2. **Data Acquisition:** Sensors are polled periodically.
3. **Data Logging:** Readings are formatted as CSV and written to the SD card.
4. **Continuous Monitoring:** The process repeats as long as power is available.

A flowchart and detailed schematic are available in the repository (see the `docs/` folder).

---

## Installation and Setup

### Hardware Assembly

1. **Connect the Sensors:**
   - Wire the BMP280 to the I²C pins (SDA and SCL).
   - Connect the DHT11 data pin to a digital GPIO (e.g., GP2).
   - Attach the MQ135 analog output to an ADC pin (e.g., GP27).
   - Hook up the SD card module using the SPI interface pins.

2. **Power Supply:**
   - The system runs on 3.3 V supplied from the Pico.
   - Optionally, use a battery for portable deployment.

### Software Requirements

- **Arduino IDE:** For compiling and uploading the code.
- **Libraries:**
  - `SD.h` for SD card operations.
  - `Wire.h` for I²C communication.
  - `SPI.h` for SPI interface.
  - `DHT.h` for the DHT11 sensor.
  - `Adafruit BMP280.h` for the BMP280 sensor.
  - `MQUnifiedsensor.h` for the MQ135 sensor.
  - `SimpleKalmanFilter.h` for sensor data filtering.

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/weather-monitoring-system.git
   cd weather-monitoring-system
