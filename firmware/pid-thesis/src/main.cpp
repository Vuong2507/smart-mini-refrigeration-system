// ===========================================================================
//  Automated Mini Cold Storage System — PID Edition (Graduation Thesis)
//  MCU: ESP32 DevKit v1  |  Cloud: Blynk IoT
//
//  Control strategy: PID controller with time-proportioning relay output.
//  The PID output (0–100%) is converted into an ON duty-cycle inside a fixed
//  60-second window, so a slow on/off compressor behaves like a modulating
//  actuator without short-cycling.
//
//  Author : Tran Thinh Vuong
//  Target : ESP32 Arduino core 2.x or 3.x (watchdog init is version-guarded)
// ===========================================================================

#define BLYNK_TEMPLATE_ID   "PUT_YOUR_TEMPLATE_ID_HERE"
#define BLYNK_TEMPLATE_NAME "PUT_YOUR_TEMPLATE_NAME_HERE"
#define BLYNK_AUTH_TOKEN    "PUT_YOUR_BLYNK_TOKEN_HERE"

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_ONEWIRE    = 4;
constexpr uint8_t PIN_DHT        = 16;
constexpr uint8_t PIN_RELAY      = 25;
constexpr uint8_t PIN_STATUS_LED = 26;
constexpr uint8_t PIN_BUZZER     = 15;
constexpr uint8_t PIN_BTN_UP     = 32;
constexpr uint8_t PIN_BTN_DOWN   = 33;
constexpr uint8_t OLED_SDA       = 21;
constexpr uint8_t OLED_SCL       = 22;

constexpr uint8_t SCREEN_WIDTH  = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr int     OLED_RESET    = -1;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

constexpr uint8_t DHT_TYPE = DHT11;
DHT dht(PIN_DHT, DHT_TYPE);

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);
DeviceAddress coldProbeAddress;

Preferences prefs;

// ---------------------------------------------------------------------------
// System constants
// ---------------------------------------------------------------------------
constexpr bool  RELAY_ACTIVE_LOW    = false;
constexpr float DEFAULT_SETPOINT_C  = 8.0f;
constexpr float MIN_SETPOINT_C      = -5.0f;
constexpr float MAX_SETPOINT_C      = 30.0f;
constexpr float SETPOINT_STEP_C     = 0.5f;

// PID tuning — time-proportioning relay output.
// error = current_temp - setpoint. Positive error = too warm = relay ON.
constexpr float PID_KP            = 20.0f;
constexpr float PID_KI            = 0.03f;
constexpr float PID_KD            = 60.0f;
constexpr float PID_OUTPUT_MIN    = 0.0f;
constexpr float PID_OUTPUT_MAX    = 100.0f;

constexpr unsigned long PID_SAMPLE_MS       = 2000UL;   // PID compute interval
constexpr unsigned long PID_WINDOW_MS       = 60000UL;  // relay duty-cycle window
constexpr unsigned long MIN_RELAY_TOGGLE_MS = 10000UL;  // protect relay contacts
constexpr unsigned long SENSOR_READ_MS      = 2000UL;
constexpr unsigned long OLED_UPDATE_MS      = 500UL;
constexpr unsigned long BLYNK_PUSH_MS       = 3000UL;
constexpr unsigned long BUTTON_DEBOUNCE_MS  = 50UL;
constexpr unsigned long BUTTON_REPEAT_MS    = 250UL;
constexpr unsigned long SAVE_QUIET_MS       = 3000UL;   // NVS write debounce

// Watchdog timeout — restarts ESP32 if loop() stalls
constexpr uint32_t WDT_TIMEOUT_S = 30UL;

// WiFi connect timeout at boot
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000UL;

// ---------------------------------------------------------------------------
// Blynk virtual pins
//   NOTE: BLYNK_WRITE() needs the literal Vx token (it uses ## token-pasting),
//   so the callback below is written as BLYNK_WRITE(V0), not a constexpr.
// ---------------------------------------------------------------------------
constexpr uint8_t VPIN_SETPOINT     = V0;
constexpr uint8_t VPIN_COLD_TEMP    = V1;
constexpr uint8_t VPIN_AMBIENT_TEMP = V2;
constexpr uint8_t VPIN_HUMIDITY     = V3;
constexpr uint8_t VPIN_FAN_STATE    = V4;
constexpr uint8_t VPIN_STATUS       = V5;
constexpr uint8_t VPIN_PID_OUTPUT   = V6;

// ---------------------------------------------------------------------------
// State structs
// ---------------------------------------------------------------------------
struct ButtonState {
    uint8_t  pin;
    bool     stableLevel;
    bool     lastReading;
    unsigned long lastDebounceMs;
    unsigned long lastRepeatMs;
};

struct SensorData {
    float coldTempC    = NAN;
    float ambientTempC = NAN;
    float humidity     = NAN;
    bool  dsValid      = false;
    bool  dhtValid     = false;
};

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
ButtonState btnUp   {PIN_BTN_UP,   HIGH, HIGH, 0, 0};
ButtonState btnDown {PIN_BTN_DOWN, HIGH, HIGH, 0, 0};
SensorData  sensorData;

float setpointC      = DEFAULT_SETPOINT_C;
bool  fanOn          = false;
bool  displayReady   = false;
bool  setpointDirty  = false;

float         pidIntegral       = 0.0f;
float         pidPreviousError  = 0.0f;
float         pidOutputPercent  = 0.0f;
unsigned long lastPidMs         = 0;
unsigned long pidWindowStartMs  = 0;
unsigned long lastRelayChangeMs = 0;
unsigned long lastSensorReadMs  = 0;
unsigned long lastDisplayMs     = 0;
unsigned long lastBlynkPushMs   = 0;
unsigned long lastSetpointChangeMs = 0;

// ---------------------------------------------------------------------------
// Watchdog — API differs between ESP32 Arduino core 2.x and 3.x
// ---------------------------------------------------------------------------
void initWatchdog(uint32_t timeoutSeconds) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    esp_task_wdt_config_t wdtConfig = {
        .timeout_ms     = timeoutSeconds * 1000U,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdtConfig);   // TWDT is already running on core 3.x
#else
    esp_task_wdt_init(timeoutSeconds, true);
#endif
    esp_task_wdt_add(NULL);
}

// ---------------------------------------------------------------------------
// Relay control
// ---------------------------------------------------------------------------
void writeRelayPin(bool on) {
    fanOn = on;
    digitalWrite(PIN_RELAY,      RELAY_ACTIVE_LOW ? !on : on);
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
    digitalWrite(PIN_BUZZER,     on ? HIGH : LOW);
    lastRelayChangeMs = millis();
}

bool isRelayToggleAllowed() {
    return millis() - lastRelayChangeMs >= MIN_RELAY_TOGGLE_MS;
}

// Only toggles if state changes AND minimum interval has elapsed
void setRelay(bool on) {
    if (fanOn != on && isRelayToggleAllowed()) {
        writeRelayPin(on);
    }
}

// ---------------------------------------------------------------------------
// PID helpers
// ---------------------------------------------------------------------------
void resetPid() {
    pidIntegral      = 0.0f;
    pidPreviousError = 0.0f;
    pidOutputPercent = 0.0f;
    lastPidMs        = millis();
    pidWindowStartMs = millis();
}

// ---------------------------------------------------------------------------
// Setpoint persistence (NVS via Preferences)
// ---------------------------------------------------------------------------
void saveSetpoint() {
    prefs.putFloat("setpoint", setpointC);
    setpointDirty = false;
}

void loadSetpoint() {
    prefs.begin("cold-store", false);
    setpointC = prefs.getFloat("setpoint", DEFAULT_SETPOINT_C);
    setpointC = constrain(setpointC, MIN_SETPOINT_C, MAX_SETPOINT_C);
    lastSetpointChangeMs = millis();
}

void adjustSetpoint(float delta) {
    setpointC = constrain(setpointC + delta, MIN_SETPOINT_C, MAX_SETPOINT_C);
    setpointDirty = true;
    lastSetpointChangeMs = millis();
    resetPid();
    Blynk.virtualWrite(VPIN_SETPOINT, setpointC);
}

// ---------------------------------------------------------------------------
// PID computation
// ---------------------------------------------------------------------------
void computePid() {
    if (!sensorData.dsValid) {
        pidOutputPercent = 0.0f;
        setRelay(false);
        return;
    }
    const unsigned long now       = millis();
    const unsigned long elapsedMs = now - lastPidMs;
    if (elapsedMs < PID_SAMPLE_MS) return;

    const float dt    = elapsedMs / 1000.0f;
    const float error = sensorData.coldTempC - setpointC;

    // Anti-windup: clamp integral to prevent saturation
    pidIntegral += error * dt;
    const float maxIntegral = PID_OUTPUT_MAX / PID_KI;
    pidIntegral = constrain(pidIntegral, -maxIntegral, maxIntegral);

    const float derivative = (error - pidPreviousError) / dt;
    const float output = (PID_KP * error) + (PID_KI * pidIntegral) + (PID_KD * derivative);
    pidOutputPercent = constrain(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pidPreviousError = error;
    lastPidMs        = now;
}

// Map PID output (0-100%) to relay ON time within the 60-second window
void applyPidRelayWindow() {
    const unsigned long now = millis();
    if (now - pidWindowStartMs >= PID_WINDOW_MS) {
        pidWindowStartMs += PID_WINDOW_MS;
    }
    const unsigned long onTimeMs = static_cast<unsigned long>(
        (pidOutputPercent / 100.0f) * PID_WINDOW_MS);
    const bool shouldRelayBeOn = (now - pidWindowStartMs) < onTimeMs;
    setRelay(shouldRelayBeOn);
}

void controlFan() {
    computePid();
    applyPidRelayWindow();
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
String formatFloat(float value, uint8_t decimals = 1) {
    if (isnan(value)) return "--";
    char buffer[24];
    snprintf(buffer, sizeof(buffer), "%.*f",
             static_cast<int>(decimals), static_cast<double>(value));
    return String(buffer);
}

void drawLine(const String &line, int16_t y, uint8_t size = 1) {
    display.setTextSize(size);
    display.setCursor(0, y);
    display.print(line);
}

void updateDisplay() {
    if (!displayReady) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    drawLine("KHO LANH PID",                                    0,  1);
    drawLine("SP:  " + formatFloat(setpointC)              + " C", 12, 1);
    drawLine("DS:  " + formatFloat(sensorData.coldTempC)   + " C", 24, 1);
    drawLine("DHT: " + formatFloat(sensorData.ambientTempC)+ " C", 36, 1);
    drawLine("PID: " + formatFloat(pidOutputPercent, 0)    + " %", 48, 1);
    display.setCursor(88, 0);
    display.print(fanOn ? "ON" : "OFF");
    display.display();
}

// ---------------------------------------------------------------------------
// Blynk telemetry push
// ---------------------------------------------------------------------------
void pushToBlynk() {
    if (!Blynk.connected()) return;
    Blynk.virtualWrite(VPIN_SETPOINT,     setpointC);
    Blynk.virtualWrite(VPIN_COLD_TEMP,    sensorData.dsValid  ? sensorData.coldTempC    : 0.0f);
    Blynk.virtualWrite(VPIN_AMBIENT_TEMP, sensorData.dhtValid ? sensorData.ambientTempC : 0.0f);
    Blynk.virtualWrite(VPIN_HUMIDITY,     sensorData.dhtValid ? sensorData.humidity     : 0.0f);
    Blynk.virtualWrite(VPIN_FAN_STATE,    fanOn ? 1 : 0);
    Blynk.virtualWrite(VPIN_PID_OUTPUT,   pidOutputPercent);

    String status = sensorData.dsValid ? "He thong OK | PID" : "Mat du lieu DS18B20";
    if (!sensorData.dhtValid) status += " | DHT loi";
    Blynk.virtualWrite(VPIN_STATUS, status);
}

// ---------------------------------------------------------------------------
// Sensor reading
// ---------------------------------------------------------------------------
void readSensors() {
    // DS18B20 — 1-Wire blocking conversion (~750 ms at 12-bit resolution)
    ds18b20.requestTemperatures();
    const float dsTemp = ds18b20.getTempCByIndex(0);
    if (dsTemp == DEVICE_DISCONNECTED_C || isnan(dsTemp)) {
        sensorData.dsValid   = false;
        sensorData.coldTempC = NAN;
        Serial.println("[WARN] DS18B20 read failed");
    } else {
        sensorData.coldTempC = dsTemp;
        sensorData.dsValid   = true;
    }

    // DHT11 — library handles 1s minimum sampling interval internally
    const float t = dht.readTemperature();
    const float h = dht.readHumidity();
    if (isnan(t) || isnan(h)) {
        sensorData.dhtValid     = false;
        sensorData.ambientTempC = NAN;
        sensorData.humidity     = NAN;
    } else {
        sensorData.ambientTempC = t;
        sensorData.humidity     = h;
        sensorData.dhtValid     = true;
    }
}

// ---------------------------------------------------------------------------
// Button handling — debounce + auto-repeat
// Fires adjustSetpoint(delta) on initial press, then every BUTTON_REPEAT_MS.
// ---------------------------------------------------------------------------
void processButton(ButtonState &btn, float delta) {
    const bool          reading = digitalRead(btn.pin);
    const unsigned long now     = millis();

    if (reading != btn.lastReading) {
        btn.lastDebounceMs = now;
        btn.lastReading    = reading;
    }
    if (now - btn.lastDebounceMs < BUTTON_DEBOUNCE_MS) return;

    if (btn.stableLevel != reading) {
        btn.stableLevel = reading;
        if (reading == LOW) {
            adjustSetpoint(delta);
            btn.lastRepeatMs = now;
        }
    } else if (reading == LOW && now - btn.lastRepeatMs >= BUTTON_REPEAT_MS) {
        adjustSetpoint(delta);
        btn.lastRepeatMs = now;
    }
}

// ---------------------------------------------------------------------------
// Blynk callbacks
// ---------------------------------------------------------------------------
BLYNK_CONNECTED() {
    Blynk.syncVirtual(VPIN_SETPOINT); // re-sync slider on reconnect
}

// Must use the literal token V0 (BLYNK_WRITE pastes it into the handler name).
BLYNK_WRITE(V0) {
    const float remote = param.asFloat();
    if (remote >= MIN_SETPOINT_C && remote <= MAX_SETPOINT_C) {
        setpointC            = remote;
        setpointDirty        = true;
        lastSetpointChangeMs = millis();
        resetPid();
        Serial.printf("[INFO] Remote setpoint -> %.1f C\n", setpointC);
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("[INFO] Booting kho-lanh-esp32 (PID)...");

    // Outputs — safe state before enabling any load
    pinMode(PIN_RELAY,      OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_BUZZER,     OUTPUT);
    writeRelayPin(false); // cooling OFF by default

    // Inputs — active-LOW with internal pull-ups
    pinMode(PIN_BTN_UP,   INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

    // OLED — I2C at GPIO21/22
    Wire.begin(OLED_SDA, OLED_SCL);
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        displayReady = true;
        display.clearDisplay();
        display.display();
        Serial.println("[INFO] SSD1306 OK");
    } else {
        Serial.println("[ERROR] SSD1306 init failed — check I2C address (0x3C/0x3D) and wiring");
    }

    // DHT11
    dht.begin();

    // DS18B20 — scan bus, latch first device address, set 12-bit resolution
    ds18b20.begin();
    const uint8_t dsCount = ds18b20.getDeviceCount();
    if (dsCount > 0 && ds18b20.getAddress(coldProbeAddress, 0)) {
        ds18b20.setResolution(coldProbeAddress, 12);
        Serial.printf("[INFO] DS18B20 found (%u device(s))\n", dsCount);
    } else {
        Serial.println("[WARN] No DS18B20 on 1-Wire bus — check wiring and 4.7k pull-up resistor");
    }

    // Restore setpoint from NVS; initialize PID clean
    loadSetpoint();
    resetPid();
    Serial.printf("[INFO] Setpoint restored: %.1f C\n", setpointC);

    // WiFi — manual connect with hard timeout (avoids blocking forever)
    Serial.printf("[INFO] Connecting to WiFi: %s\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    {
        const unsigned long wifiStart = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - wifiStart > WIFI_CONNECT_TIMEOUT_MS) {
                Serial.println("[ERROR] WiFi timeout — restarting");
                ESP.restart();
            }
            delay(500);
            Serial.print(".");
        }
    }
    Serial.printf("\n[INFO] WiFi OK  IP: %s\n", WiFi.localIP().toString().c_str());

    // Blynk — non-blocking config + 5s connect attempt
    Blynk.config(auth);
    if (!Blynk.connect(5000)) {
        Serial.println("[WARN] Blynk unreachable at boot — will retry in loop");
    }

    // Watchdog — armed AFTER network init so setup delays don't trip it
    initWatchdog(WDT_TIMEOUT_S);
    Serial.printf("[INFO] WDT armed (%lu s)\n", (unsigned long)WDT_TIMEOUT_S);

    Serial.println("[INFO] Setup complete — entering loop");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    esp_task_wdt_reset(); // feed watchdog

    Blynk.run(); // keep-alive + incoming write handler

    // Non-blocking reconnect attempt (1s) — does not stall the loop
    if (!Blynk.connected()) {
        Blynk.connect(1000);
    }

    const unsigned long now = millis();

    // Sensor polling
    if (now - lastSensorReadMs >= SENSOR_READ_MS) {
        lastSensorReadMs = now;
        readSensors();
    }

    // Physical buttons — UP = +0.5C, DOWN = -0.5C
    processButton(btnUp,   +SETPOINT_STEP_C);
    processButton(btnDown, -SETPOINT_STEP_C);

    // Auto-save setpoint after a quiet period
    if (setpointDirty && (now - lastSetpointChangeMs >= SAVE_QUIET_MS)) {
        saveSetpoint();
        Serial.printf("[INFO] Setpoint saved: %.1f C\n", setpointC);
    }

    // PID + relay time-proportioning
    controlFan();

    // OLED refresh
    if (now - lastDisplayMs >= OLED_UPDATE_MS) {
        lastDisplayMs = now;
        updateDisplay();
    }

    // Blynk telemetry push
    if (now - lastBlynkPushMs >= BLYNK_PUSH_MS) {
        lastBlynkPushMs = now;
        pushToBlynk();
    }
}
