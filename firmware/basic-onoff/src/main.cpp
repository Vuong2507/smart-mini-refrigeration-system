// ===========================================================================
//  Automated Mini Cold Storage System — ON/OFF Edition (Basic)
//  MCU: ESP32 DevKit v1  |  Cloud: Blynk IoT
//
//  Control strategy: classic ON/OFF (bang-bang) control with a hysteresis
//  deadband. Cooling turns ON when the temperature rises above the upper
//  threshold and OFF when it drops below the lower threshold. A minimum
//  toggle interval protects the relay/compressor from short-cycling.
//
//      upper = setpoint + HYSTERESIS/2   -> relay ON  (too warm)
//      lower = setpoint - HYSTERESIS/2   -> relay OFF (cold enough)
//      in between (deadband)             -> keep current state
//
//  Author : Tran Thinh Vuong
//  Target : ESP32 Arduino core 2.x or 3.x
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

// ON/OFF control — full deadband width (°C). Cooling switches ON/OFF at
// setpoint ± HYSTERESIS_C/2. Wider = fewer relay cycles, looser regulation.
constexpr float HYSTERESIS_C        = 1.5f;

constexpr unsigned long MIN_RELAY_TOGGLE_MS = 10000UL;  // protect relay contacts
constexpr unsigned long SENSOR_READ_MS      = 2000UL;
constexpr unsigned long OLED_UPDATE_MS      = 500UL;
constexpr unsigned long BLYNK_PUSH_MS       = 3000UL;
constexpr unsigned long BUTTON_DEBOUNCE_MS  = 50UL;
constexpr unsigned long BUTTON_REPEAT_MS    = 250UL;
constexpr unsigned long SAVE_QUIET_MS       = 3000UL;   // NVS write debounce

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

unsigned long lastRelayChangeMs    = 0;
unsigned long lastSensorReadMs     = 0;
unsigned long lastDisplayMs        = 0;
unsigned long lastBlynkPushMs      = 0;
unsigned long lastSetpointChangeMs = 0;

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
// ON/OFF (bang-bang) control with hysteresis
// ---------------------------------------------------------------------------
void controlCooling() {
    if (!sensorData.dsValid) {
        setRelay(false);   // fail-safe: no valid temperature -> stop cooling
        return;
    }
    const float upper = setpointC + (HYSTERESIS_C / 2.0f);
    const float lower = setpointC - (HYSTERESIS_C / 2.0f);

    if (sensorData.coldTempC >= upper) {
        setRelay(true);    // too warm -> cool
    } else if (sensorData.coldTempC <= lower) {
        setRelay(false);   // cold enough -> stop
    }
    // inside the deadband -> keep the current relay state
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
    Blynk.virtualWrite(VPIN_SETPOINT, setpointC);
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
    drawLine("KHO LANH ON/OFF",                                 0,  1);
    drawLine("SP:  " + formatFloat(setpointC)              + " C", 12, 1);
    drawLine("DS:  " + formatFloat(sensorData.coldTempC)   + " C", 24, 1);
    drawLine("DHT: " + formatFloat(sensorData.ambientTempC)+ " C", 36, 1);
    drawLine("RH:  " + formatFloat(sensorData.humidity, 0) + " %", 48, 1);
    display.setCursor(92, 0);
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

    String status = sensorData.dsValid ? "He thong OK | ON/OFF" : "Mat du lieu DS18B20";
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
        Serial.printf("[INFO] Remote setpoint -> %.1f C\n", setpointC);
    }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("[INFO] Booting kho-lanh-esp32 (ON/OFF)...");

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

    // Restore setpoint from NVS
    loadSetpoint();
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

    Serial.println("[INFO] Setup complete — entering loop");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
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

    // ON/OFF cooling control
    controlCooling();

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
