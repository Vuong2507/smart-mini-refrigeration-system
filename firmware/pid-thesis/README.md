# Mini Cold Storage — PID Edition (Graduation Thesis)

ESP32-based automated mini cold storage controller using a **PID control loop**
with time-proportioning relay output, an OLED HMI, physical setpoint buttons,
NVS persistence, a hardware watchdog, and **Blynk IoT** cloud monitoring.

This is the full, thesis-grade firmware. For the simplified classroom version,
see [`../basic-onoff/`](../basic-onoff/).

---

## 1. What it does

The DS18B20 probe measures the cold-chamber temperature. A PID controller
computes an output of **0–100%**, which is mapped to an ON duty-cycle within a
fixed **60-second window**. This lets a slow ON/OFF compressor behave like a
modulating actuator while still protecting the relay from short-cycling.

```
error      = coldTemp - setpoint           (positive = too warm)
PID output = Kp*error + Ki*∫error + Kd*Δerror
relay ON   = (PID% / 100) * 60 s  within each 60 s window
```

Compared to ON/OFF control, PID gives tighter temperature regulation, less
overshoot, and a smoother duty-cycle — the key talking points for the thesis
defense.

### Control flowchart

```mermaid
flowchart TD
    A[Read DS18B20 every 2s] --> B{Reading valid?}
    B -- No --> C[output = 0, relay OFF]
    B -- Yes --> D{Sample interval elapsed?}
    D -- No --> J[Reuse last output]
    D -- Yes --> E[error = temp - setpoint]
    E --> F[integral += error x dt, clamp anti-windup]
    F --> G[derivative = delta error / dt]
    G --> H[output = Kp*error + Ki*integral + Kd*derivative]
    H --> I[clamp output to 0..100 percent]
    I --> J
    J --> K[Map output percent to ON-time within 60s window]
    K --> L{Now within ON-time?}
    L -- Yes --> M[Want relay ON]
    L -- No --> N[Want relay OFF]
    M --> O{Min toggle time elapsed?}
    N --> O
    O -- Yes --> P[Switch relay]
    O -- No --> Q[Hold current state]
```

---

## 2. Hardware

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

## 3. Setup guide (step by step)

Follow these steps in order. You only need to edit **one file**: `src/main.cpp`.

### Step 1 — Install the tools (once)

- Install [VS Code](https://code.visualstudio.com/), then open the Extensions
  panel and install **"PlatformIO IDE"**. That's the whole toolchain — it pulls
  the ESP32 compiler and all libraries automatically on the first build.
- (Arduino IDE alternative: copy `src/main.cpp` into a `.ino` sketch and install
  the libraries from `platformio.ini` via *Tools → Manage Libraries*.)

### Step 2 — Create your Blynk device

1. Sign up at [blynk.cloud](https://blynk.cloud) and create a new **Template**.
2. Add the datastreams from the [table in section 5](#5-blynk-virtual-pins)
   (V0–V6), then create a **Device** from that template.
3. Blynk gives you three values: **Template ID**, **Template Name**, and the
   device **Auth Token** — copy them; you paste them in Step 3.

### Step 3 — Fill in your credentials (the only required edit)

Open `src/main.cpp` and replace the placeholders at the very top.
**These are the only lines you must change to get it running:**

```cpp
// --- lines 14-16: paste the 3 values from your Blynk device ---
#define BLYNK_TEMPLATE_ID   "TMPLxxxxxxxx"      // from Blynk → Template
#define BLYNK_TEMPLATE_NAME "Cold Storage"      // from Blynk → Template
#define BLYNK_AUTH_TOKEN    "your_auth_token"   // from Blynk → Device

// --- lines 31-32: your home Wi-Fi (2.4 GHz — ESP32 does not do 5 GHz) ---
char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";
```

> 💡 Leave everything else untouched the first time. The defaults already work
> for the wiring in [section 2](#2-hardware).

### Step 4 — Wire the hardware

Connect the parts per the pin table in [section 2](#2-hardware). The two pull-up
resistors (4.7 kΩ on DS18B20, 10 kΩ on DHT11) are **not optional** — without them
the sensors read garbage or `--`.

### Step 5 — Build & flash

Plug the ESP32 into USB, then from **this folder** run:

```bash
pio run                 # 1. compile (downloads libraries on first run)
pio run -t upload       # 2. flash the board over USB
pio device monitor      # 3. open the serial log @ 115200 baud
```

In VS Code/PlatformIO you can click the **✓ (Build)**, **→ (Upload)**, and
**🔌 (Monitor)** icons in the bottom toolbar instead of typing commands.

On boot the serial log prints `WiFi OK`, the device's IP, and the restored
setpoint — that confirms it is running.

> Works on **ESP32 Arduino core 2.x and 3.x** — the watchdog init is
> version-guarded (`initWatchdog()`), so it compiles cleanly on both.

---

## 4. How to change the behaviour

Everything below is **optional**. All tunables are named `constexpr` constants
near the top of `src/main.cpp` — change the number, re-flash (Step 5), done.

| Want to…                          | Edit this constant     | Example                       |
|-----------------------------------|------------------------|-------------------------------|
| Change the default target temp    | `DEFAULT_SETPOINT_C`   | `5.0f` for a 5 °C fridge       |
| Make cooling react harder/softer  | `PID_KP`               | lower = gentler, less overshoot |
| Use an active-LOW relay board     | `RELAY_ACTIVE_LOW`     | `true` if the relay is inverted |
| Allow colder/warmer setpoints     | `MIN/MAX_SETPOINT_C`   | widen the slider range         |
| Protect the compressor more       | `MIN_RELAY_TOGGLE_MS`  | raise to `30000` (30 s)        |

### Reference — all tuning constants

| Constant              | Default   | Meaning                                  |
|-----------------------|-----------|------------------------------------------|
| `DEFAULT_SETPOINT_C`  | `8.0`     | Target temperature (°C)                  |
| `PID_KP / KI / KD`    | `20/0.03/60` | PID gains                             |
| `PID_WINDOW_MS`       | `60000`   | Duty-cycle window (ms)                   |
| `PID_SAMPLE_MS`       | `2000`    | PID compute interval (ms)                |
| `MIN_RELAY_TOGGLE_MS` | `10000`   | Anti-short-cycle guard (ms)              |
| `WDT_TIMEOUT_S`       | `30`      | Watchdog reboot timeout (s)              |

---

## 5. Blynk virtual pins

| Pin | Direction | Data                    | Suggested widget |
|-----|-----------|-------------------------|------------------|
| V0  | R/W       | Setpoint (°C)           | Slider −5…30     |
| V1  | R         | DS18B20 cold temp       | Gauge / Chart    |
| V2  | R         | DHT11 ambient temp      | Gauge            |
| V3  | R         | DHT11 humidity (%)      | Gauge            |
| V4  | R         | Relay/fan state (0/1)   | LED              |
| V5  | R         | Status text             | Labeled value    |
| V6  | R         | PID output (%)          | Chart            |

Setpoint can be changed three ways and stays in sync: physical buttons, the
Blynk slider (V0), and it survives reboot via NVS.

---

## 6. Safety & robustness

- **Fail-safe cooling off** when the DS18B20 read is invalid.
- **Anti-short-cycle**: relay never toggles faster than `MIN_RELAY_TOGGLE_MS`.
- **PID anti-windup**: the integral term is clamped to the output range.
- **Hardware watchdog** reboots the MCU if `loop()` stalls for 30 s.
- **Non-blocking WiFi/Blynk reconnect** — the control loop keeps running even
  when the cloud is unreachable.

---

## 7. Project layout

```
pid-thesis/
├── platformio.ini
├── README.md
└── src/
    └── main.cpp
```

---

Author: **Tran Thinh Vuong** · Faculty of Automation, UTC · 2026
