# Mini Cold Storage Controller — ESP32 + PID + Blynk IoT

A PID-based temperature controller for a mini refrigeration unit, built on the ESP32
platform. The system reads dual temperature sensors, drives a relay through a
time-proportioning output, and streams live telemetry to the Blynk mobile dashboard
over Wi-Fi. Setpoints survive power cycles via ESP32 non-volatile storage (NVS).
A hardware watchdog timer (TWDT) guards against Wi-Fi hangs and runaway loop stalls,
automatically restarting the device within 30 seconds of any lockup.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Software & Dependencies](#3-software--dependencies)
4. [Installation & Setup](#4-installation--setup)
5. [How It Works](#5-how-it-works)
6. [API / Function Reference](#6-api--function-reference)
7. [Configuration](#7-configuration)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Project Overview

| Item | Detail |
|------|--------|
| **Microcontroller** | ESP32-DevKit (Xtensa LX6 dual-core, 240 MHz) |
| **Framework** | Arduino via PlatformIO |
| **Control algorithm** | Discrete PID → time-proportioning relay window |
| **Remote dashboard** | Blynk IoT (HTTPS + WebSocket) |
| **Local UI** | 128×64 SSD1306 OLED + two tactile buttons |
| **Persistence** | ESP32 NVS (`Preferences` library) |
| **Fault recovery** | ESP32 Task Watchdog Timer (TWDT, 30 s) |

The controller keeps the cold chamber at a user-defined setpoint (default **8 °C**,
adjustable from −5 °C to 30 °C in 0.5 °C steps). A DS18B20 waterproof probe measures
the chamber temperature; a DHT11 monitors ambient conditions. The PID output drives a
relay that switches the compressor (or Peltier module) on and off within a 60-second
duty-cycle window.

---

## 2. Hardware Requirements

### Component List

| Component | Model / Spec | Quantity |
|-----------|-------------|----------|
| Microcontroller | ESP32-DevKit v1 (38-pin) | 1 |
| Cold-chamber probe | DS18B20 (waterproof, 1-Wire) | 1 |
| Ambient sensor | DHT11 (temperature + humidity) | 1 |
| Display | SSD1306 OLED 128×64, I2C | 1 |
| Relay module | 5 V single-channel, active-HIGH | 1 |
| Status LED | 5 mm LED + 220 Ω resistor | 1 |
| Buzzer | Active buzzer, 5 V | 1 |
| Buttons | Tactile push-button, NO | 2 |
| Pull-up resistors | 4.7 kΩ (for DS18B20 data line) | 1 |
| Power supply | 5 V / 2 A (USB or DC barrel) | 1 |

### Pin Mapping

```
ESP32 GPIO │ Peripheral          │ Notes
───────────┼─────────────────────┼──────────────────────────────
GPIO 4     │ DS18B20 Data (1-Wire)│ Requires 4.7 kΩ pull-up to 3.3 V
GPIO 16    │ DHT11 Data          │ Internal pull-up sufficient
GPIO 25    │ Relay IN            │ Active-HIGH (see RELAY_ACTIVE_LOW flag)
GPIO 26    │ Status LED          │ HIGH = compressor running
GPIO 15    │ Buzzer              │ Mirrors relay state
GPIO 32    │ Button UP           │ Active-LOW, internal pull-up enabled
GPIO 33    │ Button DOWN         │ Active-LOW, internal pull-up enabled
GPIO 21    │ OLED SDA (I2C)      │
GPIO 22    │ OLED SCL (I2C)      │
```

> **Note:** The relay module must be rated for the load voltage/current of your
> compressor or Peltier element. Mains-voltage loads require an appropriately rated
> relay and proper electrical isolation — this firmware does not handle that isolation.

---

## 3. Software & Dependencies

### Toolchain

| Tool | Version |
|------|---------|
| PlatformIO Core | ≥ 6.x |
| platform `espressif32` | ≥ 6.x (IDF 5.x backend) |
| Arduino framework | bundled with platform |

### Library Dependencies (`platformio.ini`)

| Library | Version | Purpose |
|---------|---------|---------|
| `blynkkk/Blynk` | ^1.3.2 | Wi-Fi telemetry & remote control |
| `adafruit/Adafruit SSD1306` | ^2.5.11 | OLED driver |
| `adafruit/Adafruit GFX Library` | ^1.11.10 | Graphics primitives |
| `milesburton/DallasTemperature` | ^3.11.0 | DS18B20 abstraction |
| `paulstoffregen/OneWire` | ^2.3.8 | 1-Wire bus protocol |
| `adafruit/DHT sensor library` | ^1.4.6 | DHT11/DHT22 driver |
| `adafruit/Adafruit Unified Sensor` | ^1.1.14 | Sensor abstraction layer |
| `esp_task_wdt` | built-in (ESP-IDF) | Hardware watchdog timer — no `lib_deps` entry needed |

All library dependencies are resolved automatically by PlatformIO on first build.
`esp_task_wdt.h` ships with the `espressif32` platform and requires no additional entry.

---

## 4. Installation & Setup

### 4.1 Clone & Open

```bash
git clone https://github.com/<Vuong2507>/kho-lanh-esp32.git
cd kho-lanh-esp32
```

Open the folder in **VS Code with PlatformIO IDE** extension, or use the PlatformIO
CLI directly.

### 4.2 Configure Credentials

Open `src/main.cpp` and fill in the three placeholders at the top of the file:

```cpp
#define BLYNK_TEMPLATE_ID   "TMPLxxxxxxxxxx"   // from Blynk console → Template
#define BLYNK_TEMPLATE_NAME "Kho lạnh"
#define BLYNK_AUTH_TOKEN    "your_auth_token"   // from Blynk console → Device

char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";
```

> **Security tip:** For production or shared repositories, move these values into a
> separate `secrets.h` header excluded from version control via `.gitignore`.

### 4.3 Blynk Dashboard Setup

Create a new Blynk template and add the following datastreams:

| Virtual Pin | Name | Data Type | Range |
|-------------|------|-----------|-------|
| V0 | Setpoint | Double | −5 to 30 |
| V1 | Cold Temp (DS18B20) | Double | −20 to 50 |
| V2 | Ambient Temp (DHT11) | Double | −20 to 60 |
| V3 | Humidity | Integer | 0 to 100 |
| V4 | Compressor State | Integer | 0 or 1 |
| V5 | System Status | String | — |
| V6 | PID Output | Double | 0 to 100 |

Add a **Slider** widget on V0 (range −5 to 30, step 0.5) to allow remote setpoint
changes. Add **Gauge** or **Label** widgets on V1–V6 for monitoring.

### 4.4 Build & Flash

```bash
# Build
pio run

# Upload (ensure ESP32 is connected via USB)
pio run --target upload

# Open serial monitor
pio device monitor --baud 115200
```

---

## 5. How It Works

### 5.1 Main Loop Architecture

The firmware is non-blocking: `loop()` does no `delay()` calls. Every subsystem is
gated by a elapsed-time check against a dedicated `lastXxxMs` timestamp.

```
loop()
 ├── esp_task_wdt_reset() every tick  — feed hardware watchdog (must complete within 30 s)
 ├── Blynk.run()          every tick  — keep-alive & RX handler
 ├── Blynk.connect()      every tick  — non-blocking reconnect if disconnected
 ├── readSensors()        every 2 s   — DS18B20 + DHT11 polling
 ├── processButton()      every tick  — debounce + auto-repeat for UP/DOWN
 ├── controlFan()         every tick  — PID compute + relay window apply
 ├── updateDisplay()      every 500 ms — OLED refresh
 └── pushToBlynk()        every 3 s   — telemetry push
```

### 5.2 PID Control with Time-Proportioning Relay

Because the relay is a binary actuator, continuous PID output is converted to a
duty cycle within a fixed **60-second window**:

```
error = coldTempC − setpointC

output(%) = Kp·error + Ki·∫error·dt + Kd·(Δerror/Δt)
output    = constrain(output, 0, 100)

onTime(ms) = (output / 100) × 60 000 ms

Within each window:
  if elapsed < onTime  → relay ON
  else                 → relay OFF
```

A positive error means the chamber is **warmer** than the setpoint, so output is
positive and the compressor runs longer. When the setpoint is reached, error → 0,
output → 0, and the compressor stays off for the full window.

### 5.3 Compressor Protection

Rapid relay toggling damages compressor windings. A minimum toggle guard
(`MIN_RELAY_TOGGLE_MS = 10 000 ms`) prevents any state change sooner than 10 seconds
after the previous one. The `setRelay()` function enforces this silently — the PID
calculation still proceeds; only the physical output is held.

### 5.4 Setpoint Persistence

When the user adjusts the setpoint via hardware buttons or the Blynk slider, the new
value is flagged as *dirty*. After a 3-second quiet period with no further changes,
the firmware calls `saveSetpoint()`, which commits the value to ESP32 NVS under the
namespace `"cold-store"`. On next boot, `loadSetpoint()` restores it before entering
`loop()`.

### 5.5 Watchdog Timer & Network Fault Recovery

`Blynk.begin()` is a blocking call — if the access point is unreachable at boot, it
hangs indefinitely and the device becomes unresponsive. The firmware addresses this
with a two-layer strategy:

**Boot-time:** `WiFi.begin()` is called manually with a 20-second timeout. If the
board cannot associate within that window, `ESP.restart()` fires immediately — no
watchdog needed at this stage since the timer is not yet armed.

**Runtime:** The Task Watchdog Timer (TWDT) is armed in `setup()` **after** the
initial Wi-Fi connection succeeds. The main loop task is subscribed to it. Every
`loop()` iteration starts with `esp_task_wdt_reset()` — if the loop ever stalls
for more than 30 seconds (e.g., a blocking library call, sensor deadlock, or heap
exhaustion), the TWDT fires, prints a backtrace to Serial, and performs a hard reset.

**Blynk reconnect:** `Blynk.connect(1000)` is called non-blockingly each iteration
when the connection is lost. The 1-second timeout keeps the total per-iteration cost
bounded, ensuring the watchdog is always fed within its window.

### 5.6 Dual-Sensor Design Rationale

| Sensor | Role | Placement |
|--------|------|-----------|
| DS18B20 | PID feedback (control-critical) | Inside cold chamber |
| DHT11 | Ambient monitoring (informational) | Outside, near intake |

The PID loop depends **exclusively** on the DS18B20. If that sensor goes invalid
(`dsValid = false`), the firmware immediately sets `pidOutputPercent = 0` and turns
the relay off — a fail-safe default that prevents the compressor from running
uncontrolled.

---

## 6. API / Function Reference

### Relay Control

```cpp
void writeRelayPin(bool on)
```
Unconditionally writes the relay GPIO, updates the LED and buzzer mirrors, and records
`lastRelayChangeMs`. **Do not call directly** — use `setRelay()` instead.

```cpp
bool isRelayToggleAllowed()
```
Returns `true` if at least `MIN_RELAY_TOGGLE_MS` (10 s) has elapsed since the last
relay state change.

```cpp
void setRelay(bool on)
```
Safe relay setter. Calls `writeRelayPin()` only when the requested state differs from
the current state **and** `isRelayToggleAllowed()` returns `true`. Silently discards
the request otherwise.

---

### PID

```cpp
void computePid()
```
Runs one PID iteration if the sensor is valid and `PID_SAMPLE_MS` (2 s) has elapsed.
Computes proportional, integral (with anti-windup clamp), and derivative terms.
Stores result in `pidOutputPercent` (0–100 %).

```cpp
void applyPidRelayWindow()
```
Applies the time-proportioning logic against `PID_WINDOW_MS` (60 s). Rolls the window
start forward automatically. Calls `setRelay()` with the computed on/off decision.

```cpp
void resetPid()
```
Zeros `pidIntegral`, `pidPreviousError`, and `pidOutputPercent`. Also resets both
the PID sample timer and the window start. Called automatically whenever the setpoint
changes, preventing integral wind-up from a prior operating point.

```cpp
void controlFan()
```
Top-level control dispatcher. Calls `computePid()` then `applyPidRelayWindow()` in
sequence. Called every `loop()` iteration.

---

### Setpoint Management

```cpp
void adjustSetpoint(float delta)
```
Adds `delta` to `setpointC`, clamps to [MIN_SETPOINT_C, MAX_SETPOINT_C], marks the
value dirty, resets PID, and immediately pushes the new value to Blynk V0.

```cpp
void saveSetpoint()
```
Writes `setpointC` to NVS key `"setpoint"` under namespace `"cold-store"`. Clears
the `setpointDirty` flag.

```cpp
void loadSetpoint()
```
Opens the NVS namespace and reads `"setpoint"`, falling back to `DEFAULT_SETPOINT_C`
(8.0 °C) if no stored value exists. Clamps the result to the valid range.

---

### Display

```cpp
void updateDisplay()
```
Redraws the full OLED frame. Skipped if `displayReady` is `false` (SSD1306 init
failed). Renders: title, setpoint, DS18B20 temp, DHT11 ambient temp, PID output %,
and compressor state (ON/OFF).

```cpp
void drawLine(const String &line, int16_t y, uint8_t size = 1)
```
Helper that positions the cursor at `(0, y)` and prints `line` at the given text
size. Wraps the Adafruit GFX cursor API.

```cpp
String formatFloat(float value, uint8_t decimals = 1)
```
Returns a formatted string for `value` with `decimals` places. Returns `"--"` if
`value` is `NaN` (sensor read failure). Uses `snprintf` internally to avoid
Arduino `String` float precision issues.

---

### Blynk Telemetry

```cpp
void pushToBlynk()
```
Sends the current state of all seven virtual pins to the Blynk cloud. No-ops if
`Blynk.connected()` is `false`. Falls back to `0.0f` for sensor readings when the
corresponding validity flag is `false`.

**`BLYNK_WRITE(V0)` handler** — receives remote setpoint updates from the Blynk
dashboard or automation rules and calls `adjustSetpoint()` with the new value.

---

### Watchdog

The firmware uses the ESP-IDF Task Watchdog Timer directly via `esp_task_wdt.h`.
There are no wrapper functions — the three relevant calls are made inline:

| Call | Location | Purpose |
|------|----------|---------|
| `esp_task_wdt_init(WDT_TIMEOUT_S, true)` | `setup()` | Arm WDT with 30 s timeout; `true` = panic + reset on trigger |
| `esp_task_wdt_add(NULL)` | `setup()` | Subscribe the main Arduino task (`NULL` = calling task) |
| `esp_task_wdt_reset()` | `loop()` first line | Feed the watchdog; must execute within `WDT_TIMEOUT_S` |

---

## 7. Configuration

All tuneable constants are defined as `constexpr` at the top of `src/main.cpp`.
No `#define` macros are used for numeric parameters.

### Temperature Setpoint

| Constant | Default | Description |
|----------|---------|-------------|
| `DEFAULT_SETPOINT_C` | `8.0f` | Setpoint used on first boot (no NVS value) |
| `MIN_SETPOINT_C` | `-5.0f` | Lower bound for user adjustment |
| `MAX_SETPOINT_C` | `30.0f` | Upper bound for user adjustment |
| `SETPOINT_STEP_C` | `0.5f` | Step per button press |

### PID Tuning

| Constant | Default | Description |
|----------|---------|-------------|
| `PID_KP` | `20.0f` | Proportional gain |
| `PID_KI` | `0.03f` | Integral gain |
| `PID_KD` | `60.0f` | Derivative gain |
| `PID_SAMPLE_MS` | `2000` | PID compute interval (ms) |
| `PID_WINDOW_MS` | `60000` | Time-proportioning window length (ms) |
| `PID_OUTPUT_MIN` | `0.0f` | Minimum output (no cooling-below-setpoint drive) |
| `PID_OUTPUT_MAX` | `100.0f` | Maximum output = 100 % duty cycle |

> **Tuning note:** The default gains are starting values for a small insulated chamber
> with slow thermal dynamics. If the system overshoots significantly, reduce `PID_KP`
> first. If it is slow to recover after door-open events, increase `PID_KI` cautiously
> — high `KI` with a long window will accumulate windup quickly.

### Watchdog & Network

| Constant | Default | Description |
|----------|---------|-------------|
| `WDT_TIMEOUT_S` | `30` | Seconds before TWDT fires and resets the board |
| `WIFI_CONNECT_TIMEOUT_MS` | `20000` | Max time (ms) to wait for Wi-Fi association at boot |

### Timing

| Constant | Default | Description |
|----------|---------|-------------|
| `SENSOR_READ_MS` | `2000` | Sensor polling interval |
| `OLED_UPDATE_MS` | `500` | Display refresh interval |
| `BLYNK_PUSH_MS` | `3000` | Telemetry push interval |
| `MIN_RELAY_TOGGLE_MS` | `10000` | Minimum time between relay state changes |
| `BUTTON_DEBOUNCE_MS` | `50` | Debounce window for both buttons |
| `BUTTON_REPEAT_MS` | `250` | Auto-repeat interval while button held |

### Relay Polarity

```cpp
constexpr bool RELAY_ACTIVE_LOW = false;
```

Set to `true` if your relay module triggers on a LOW signal (common for opto-isolated
modules with active-low inputs). This inverts the GPIO write in `writeRelayPin()`
without touching any other logic.

---

## 8. Troubleshooting

### DS18B20 reads `--` (NaN) on OLED

**Most likely cause:** missing or incorrect pull-up resistor on the data line.

- Verify a **4.7 kΩ** resistor between GPIO 4 and 3.3 V.
- Run a 1-Wire scan in `setup()` and confirm the device address is found
  (`ds18b20.getDeviceCount()` returns 1).
- Check that `hasProbeAddress` is `true` after `setup()` completes (add a
  `Serial.println` to confirm).
- Try a shorter data cable — parasitic capacitance on long cables can corrupt
  the 1-Wire signal.

---

### DHT11 always returns `NaN`

- DHT11 requires a minimum **1-second** warm-up after power-on before the first
  read succeeds. Ensure `setup()` does not call `dht.readTemperature()` too early.
- The DHT11 minimum sampling interval is **1 second**; the firmware polls every
  2 seconds (`SENSOR_READ_MS`), which is safe.
- Replace with a DHT22 for higher accuracy and faster response — no code change
  required beyond updating `DHT_TYPE` to `DHT22`.

---

### Blynk connection drops frequently

- Check Wi-Fi signal strength near the ESP32 (`WiFi.RSSI()`). Values below −75 dBm
  cause instability.
- The Blynk library handles reconnection internally, but if the ESP32 loses Wi-Fi
  for more than ~60 seconds, Blynk may time out and require a full reconnect cycle.
  Consider adding a watchdog timer reset or periodic `ESP.restart()` call in the
  disconnect handler.
- Verify the auth token, template ID, and template name match exactly — a mismatch
  causes a silent authentication failure with no Serial error.

---

### Relay chatters rapidly (toggles faster than expected)

`MIN_RELAY_TOGGLE_MS` (10 s) is the guard. If you see toggling faster than this,
check that `lastRelayChangeMs` is not being reset from outside `writeRelayPin()`.
The most common cause is calling `writeRelayPin()` directly instead of `setRelay()`.

---

### OLED shows nothing (blank screen)

- Confirm I2C address: most SSD1306 modules use `0x3C`. Run an I2C scanner sketch
  to detect the actual address on your module.
- Verify SDA/SCL wiring to GPIO 21/22.
- If `display.begin()` returns `false`, `displayReady` stays `false` and all
  `updateDisplay()` calls are silently skipped. Add a `Serial.println` in `setup()`
  to confirm init status.

---

### Board reboots repeatedly at startup (WDT or WiFi timeout)

If the board restarts in a loop at boot, the most common causes in order of likelihood:

- **Wrong SSID or password** — `WiFi.begin()` keeps retrying silently; the 20-second
  timeout fires and calls `ESP.restart()`. Check `ssid` and `pass` in the source.
- **AP out of range** — move the board closer to the router for initial flashing.
- **Blynk credentials wrong** — `Blynk.connect(5000)` will fail but only logs a
  warning; it does not restart. Check the auth token and template ID.
- **WDT firing in loop** — if Serial shows a "Task watchdog got triggered" panic with
  a backtrace, a blocking call is stalling the loop beyond 30 seconds. Identify the
  function in the backtrace and either remove the blocking call or increase
  `WDT_TIMEOUT_S` temporarily to diagnose.

### Setpoint not saved after power cycle

- Verify the NVS partition exists in your partition table. The default
  `esp32dev` board configuration includes NVS; custom partition tables may not.
- The save only fires after a **3-second quiet period** following a setpoint change.
  If the board loses power within those 3 seconds, the value is lost. This is
  intentional to reduce NVS write cycles.

---

*Built with PlatformIO · ESP32 Arduino · Blynk IoT*
