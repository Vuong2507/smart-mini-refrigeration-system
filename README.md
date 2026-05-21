# Smart Mini Refrigeration System

An automated temperature and humidity monitoring and control system for a mini cold storage unit using the **ESP32** microcontroller. The system features a **time-proportioning PID control algorithm** to regulate the cooling relay (Fan/Compressor), a local **SSD1306 OLED display** for real-time telemetry, and remote IoT capabilities integrated via the **Blynk platform**.

---

## 🚀 Key Features
* **Smart PID Control:** Automatically calculates relay duty cycles based on the deviation between the actual storage temperature and the target setpoint.
* **Dual-Sensor Telemetry:** High-precision cold room temperature sensing via **DS18B20** combined with ambient temperature and humidity tracking via **DHT11**.
* **Non-Volatile Storage:** Uses the ESP32's `Preferences` library to flash and retain the temperature setpoint, preventing data loss during power cycles.
* **Local User Interface:** Crisp OLED display for live parameter readouts, paired with dual physical tactile buttons for direct manual setpoint adjustment.
* **Remote IoT Dashboard:** Continuous real-time data streaming and remote control via the Blynk Mobile App and Web Dashboard.

---

## 📌 Hardware Pin Mapping

| Component / Device | ESP32 GPIO | Function |
| :--- | :---: | :--- |
| **DS18B20 Sensor** | `GPIO 4` | Cold storage temperature sensor (OneWire bus) |
| **DHT11 Sensor** | `GPIO 16` | Ambient temperature & humidity sensor |
| **Relay (Cooling Fan/Block)** | `GPIO 25` | Actuator control for the cooling system |
| **Status LED** | `GPIO 26` | Visual operation status indicator |
| **Buzzer** | `GPIO 15` | Audible system alarm/alert |
| **Button UP** | `GPIO 32` | Tactile switch to increase temperature setpoint |
| **Button DOWN** | `GPIO 33` | Tactile switch to decrease temperature setpoint |
| **OLED Display (SDA)** | `GPIO 21` | I2C Data line for local display |
| **OLED Display (SCL)** | `GPIO 22` | I2C Clock line for local display |

---

## 🌐 Blynk Virtual Pins Configuration

The system exchanges data with the Blynk Cloud through the following Virtual Pins:
* **`V0`**: Target Temperature Setpoint (`Setpoint`)
* **`V1`**: Internal Cold Storage Temperature (`Cold Room Temp - DS18B20`)
* **`V2`**: External Environment Temperature (`Ambient Temp - DHT11`)
* **`V3`**: Ambient Relative Humidity (`Humidity - DHT11`)
* **`V4`**: Actuator Status (`Cooling Fan State: ON/OFF`)
* **`V5`**: Diagnostic Message (`System Status String`)
* **`V6`**: Current PID Duty Cycle Output (`PID Output %`)

---

## 🛠 Required Libraries (Dependencies)

If you are using **PlatformIO**, ensure the following dependencies are declared in your `platformio.ini`:
* `blynkkk/Blynk`
* `adafruit/Adafruit GFX Library`
* `adafruit/Adafruit SSD1306`
* `paulstoffregen/OneWire`
* `milesburton/DallasTemperature`
* `adafruit/DHT sensor library`

---

## 📖 Getting Started & Deployment

1. Open the source code and update your network credentials along with your unique Blynk Auth Token:
   ```cpp
   #define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"
   char ssid[] = "YOUR_WIFI_SSID";
   char pass[] = "YOUR_WIFI_PASSWORD";
