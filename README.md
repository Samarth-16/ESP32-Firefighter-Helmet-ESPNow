# ESP32-Based Fire Fighter Helmet Using ESP-NOW

## Overview

This project is a smart firefighter helmet built using ESP32 and ESP-NOW
communication protocol. The system continuously monitors:

- Temperature
- Humidity
- Toxic gas concentration
- Heart rate

Two helmets communicate directly using ESP-NOW without requiring Wi-Fi
routers or internet connectivity.

The OLED display shows both local and peer firefighter information,
improving situational awareness and safety during rescue operations.

---

## Features

✅ Real-time temperature monitoring

✅ Real-time humidity monitoring

✅ Toxic gas detection using MQ-9

✅ Heart-rate monitoring

✅ ESP-NOW peer-to-peer communication

✅ OLED display interface

✅ Offline operation (No Wi-Fi router required)

✅ Low power consumption

---

## Hardware Components

- ESP32 DevKit V1
- SSD1306 OLED Display
- DHT11 Temperature & Humidity Sensor
- MQ-9 Gas Sensor
- MAX30102 Heart Rate Sensor
- 3.7V Li-Po Battery

---

## Communication

The project uses ESP-NOW protocol for:

- Low latency communication
- Direct ESP32-to-ESP32 connection
- Infrastructure-free networking

---

## System Architecture

[Insert Block Diagram]

---

## Wiring

### DHT11

GPIO 4 → DATA

### MQ-9

GPIO 34 → Analog Output

### OLED

SDA → GPIO 21

SCL → GPIO 22

### MAX30102

I2C Interface

---

## Software Requirements

- Arduino IDE
- ESP32 Board Package

### Required Libraries

- WiFi.h
- esp_now.h
- Adafruit_GFX
- Adafruit_SSD1306
- DHT Sensor Library
- MAX30105
- heartRate

---

## Installation

1. Clone repository

```bash
git clone https://github.com/yourusername/ESP32-Firefighter-Helmet-ESPNow.git
