# Smart Mini Refrigeration System — ESP32 + Blynk IoT

An automated mini cold-storage controller built on the **ESP32**. It reads a
DS18B20 cold-chamber probe and a DHT11 ambient sensor, drives a cooling relay,
shows live telemetry on a 128×64 SSD1306 OLED, and streams data to the **Blynk
IoT** dashboard over Wi-Fi. The setpoint is adjustable by physical buttons or the
Blynk slider and survives power cycles via ESP32 NVS.

This repository ships **two firmware editions** so the same hardware can be used
both as an introductory exercise and as a full graduation-thesis project.

---

## Two editions

| | [`firmware/basic-onoff`](firmware/basic-onoff/) | [`firmware/pid-thesis`](firmware/pid-thesis/) |
|---|---|---|
| **Level** | Basic / classroom exercise | Graduation thesis |
| **Control** | ON/OFF (bang-bang) with hysteresis deadband | PID + time-proportioning relay window |
| **Regulation** | Oscillates around setpoint | Tight, low overshoot |
| **Watchdog (TWDT)** | — | 30 s hardware watchdog |
| **Blynk pins** | V0–V5 | V0–V6 (adds PID output %) |
| **Shared** | DS18B20 + DHT11 + OLED + buttons + NVS + Blynk + anti-short-cycle | ← same |

Each edition is a self-contained PlatformIO project with its own
`platformio.ini` and `README.md` (including a control flowchart). Both compile
clean with `-Wall -Wextra` on ESP32 Arduino core 2.x and 3.x.

```bash
cd firmware/pid-thesis      # or firmware/basic-onoff
pio run                     # compile
pio run -t upload           # flash over USB
pio device monitor          # serial @ 115200
```

---

## Hardware (both editions)

| Component    | Spec                                   | Pin                     |
|--------------|----------------------------------------|-------------------------|
| MCU          | ESP32 DevKit v1 (240 MHz, Wi-Fi)       | —                       |
| Cold temp    | DS18B20, 1-Wire, **4.7 kΩ pull-up**    | GPIO4                   |
| Ambient/RH   | DHT11, **10 kΩ pull-up**               | GPIO16                  |
| Display      | OLED SSD1306 I²C 128×64 (addr `0x3C`)  | SDA=GPIO21, SCL=GPIO22  |
| Relay        | 10 A / 220 V, active-HIGH              | GPIO25                  |
| Status LED   | —                                      | GPIO26                  |
| Buzzer       | —                                      | GPIO15                  |
| Button UP    | INPUT_PULLUP, active-LOW               | GPIO32                  |
| Button DOWN  | INPUT_PULLUP, active-LOW               | GPIO33                  |

> If your relay board is active-LOW, set `RELAY_ACTIVE_LOW = true` in `main.cpp`.

---

## Configuration

Before flashing, edit the top of the edition's `src/main.cpp`:

```cpp
#define BLYNK_TEMPLATE_ID   "..."
#define BLYNK_TEMPLATE_NAME "..."
#define BLYNK_AUTH_TOKEN    "..."

char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";
```

See each edition's README for the full pin table, tuning constants, Blynk
datastream setup, and safety notes.

---

Author: **Tran Thinh Vuong** ([@Vuong2507](https://github.com/Vuong2507)) ·
Faculty of Automation, UTC · 2026 · Built with PlatformIO · ESP32 Arduino · Blynk IoT
