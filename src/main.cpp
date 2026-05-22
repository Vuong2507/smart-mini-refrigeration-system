#define BLYNK_TEMPLATE_ID "PUT_YOUR_TEMPLATE_ID_HERE"
#define BLYNK_TEMPLATE_NAME "PUT_YOUR_TEMPLATE_NAME_HERE"
#define BLYNK_AUTH_TOKEN "PUT_YOUR_BLYNK_TOKEN_HERE"
#include <Arduino.h>
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
constexpr uint8_t PIN_ONEWIRE = 4;
constexpr uint8_t PIN_DHT = 16;
constexpr uint8_t PIN_RELAY = 25;
constexpr uint8_t PIN_STATUS_LED = 26;
constexpr uint8_t PIN_BUZZER = 15;
constexpr uint8_t PIN_BTN_UP = 32;
constexpr uint8_t PIN_BTN_DOWN = 33;
constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr int OLED_RESET = -1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
constexpr uint8_t DHT_TYPE = DHT11;
DHT dht(PIN_DHT, DHT_TYPE);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);
DeviceAddress coldProbeAddress;
bool hasProbeAddress = false;
Preferences prefs;
BlynkTimer timer;
constexpr bool RELAY_ACTIVE_LOW = false;
constexpr float DEFAULT_SETPOINT_C = 8.0f;
constexpr float MIN_SETPOINT_C = -5.0f;
constexpr float MAX_SETPOINT_C = 30.0f;
constexpr float SETPOINT_STEP_C = 0.5f;
// PID tuning ban dau cho kho lanh nho, relay dieu khien theo time-proportioning.
// Error = nhiet do hien tai - setpoint. Nhiet do cang cao hon setpoint thi output cang lon.
constexpr float PID_KP = 20.0f;
constexpr float PID_KI = 0.03f;
constexpr float PID_KD = 60.0f;
constexpr float PID_OUTPUT_MIN = 0.0f;
constexpr float PID_OUTPUT_MAX = 100.0f;
constexpr unsigned long PID_SAMPLE_MS = 2000UL;
constexpr unsigned long PID_WINDOW_MS = 60000UL;
constexpr unsigned long MIN_RELAY_TOGGLE_MS = 10000UL;
constexpr unsigned long SENSOR_READ_MS = 2000UL;
constexpr unsigned long OLED_UPDATE_MS = 500UL;
constexpr unsigned long BLYNK_PUSH_MS = 3000UL;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 50UL;
constexpr unsigned long BUTTON_REPEAT_MS = 250UL;
// Watchdog: resets ESP32 if loop() stalls for longer than this
constexpr uint32_t WDT_TIMEOUT_S = 30UL;
// WiFi: restart if association takes longer than this during setup
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000UL;
constexpr uint8_t VPIN_SETPOINT = V0;
constexpr uint8_t VPIN_COLD_TEMP = V1;
constexpr uint8_t VPIN_AMBIENT_TEMP = V2;
constexpr uint8_t VPIN_HUMIDITY = V3;
constexpr uint8_t VPIN_FAN_STATE = V4;
constexpr uint8_t VPIN_STATUS = V5;
constexpr uint8_t VPIN_PID_OUTPUT = V6;
struct ButtonState {
  uint8_t pin;
  bool stableLevel;
  bool lastReading;
  unsigned long lastDebounceMs;
  unsigned long lastRepeatMs;
};
struct SensorData {
  float coldTempC = NAN;
  float ambientTempC = NAN;
  float humidity = NAN;
  bool dsValid = false;
  bool dhtValid = false;
};
ButtonState btnUp{PIN_BTN_UP, HIGH, HIGH, 0, 0};
ButtonState btnDown{PIN_BTN_DOWN, HIGH, HIGH, 0, 0};
SensorData sensorData;
float setpointC = DEFAULT_SETPOINT_C;
bool fanOn = false;
bool displayReady = false;
bool setpointDirty = false;
float pidIntegral = 0.0f;
float pidPreviousError = 0.0f;
float pidOutputPercent = 0.0f;
unsigned long lastPidMs = 0;
unsigned long pidWindowStartMs = 0;
unsigned long lastRelayChangeMs = 0;
unsigned long lastSensorReadMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastBlynkPushMs = 0;
unsigned long lastSetpointChangeMs = 0;
void writeRelayPin(bool on) {
  fanOn = on;
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? !on : on);
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  lastRelayChangeMs = millis();
}
bool isRelayToggleAllowed() {
  return millis() - lastRelayChangeMs >= MIN_RELAY_TOGGLE_MS;
}
void setRelay(bool on) {
  if (fanOn != on && isRelayToggleAllowed()) {
    writeRelayPin(on);
  }
}
void resetPid() {
  pidIntegral = 0.0f;
  pidPreviousError = 0.0f;
  pidOutputPercent = 0.0f;
  lastPidMs = millis();
  pidWindowStartMs = millis();
}
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
void computePid() {
  if (!sensorData.dsValid) {
    pidOutputPercent = 0.0f;
    setRelay(false);
    return;
  }
  const unsigned long now = millis();
  const unsigned long elapsedMs = now - lastPidMs;
  if (elapsedMs < PID_SAMPLE_MS) {
    return;
  }
  const float dt = elapsedMs / 1000.0f;
  const float error = sensorData.coldTempC - setpointC;
  pidIntegral += error * dt;
  const float maxIntegral = PID_OUTPUT_MAX / PID_KI;
  pidIntegral = constrain(pidIntegral, -maxIntegral, maxIntegral);
  const float derivative = (error - pidPreviousError) / dt;
  float output = (PID_KP * error) + (PID_KI * pidIntegral) + (PID_KD * derivative);
  pidOutputPercent = constrain(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  pidPreviousError = error;
  lastPidMs = now;
}
void applyPidRelayWindow() {
  const unsigned long now = millis();
  if (now - pidWindowStartMs >= PID_WINDOW_MS) {
    pidWindowStartMs += PID_WINDOW_MS;
  }
  const unsigned long onTimeMs = static_cast<unsigned long>((pidOutputPercent / 100.0f) * PID_WINDOW_MS);
  const bool shouldRelayBeOn = (now - pidWindowStartMs) < onTimeMs;
  setRelay(shouldRelayBeOn);
}
void controlFan() {
  computePid();
  applyPidRelayWindow();
}
String formatFloat(float value, uint8_t decimals = 1) {
  if (isnan(value)) {
    return "--";
  }
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%.*f", static_cast<int>(decimals), static_cast<double>(value));
  return String(buffer);
}
void drawLine(const String &line, int16_t y, uint8_t size = 1) {
  display.setTextSize(size);
  display.setCursor(0, y);
  display.print(line);
}
void updateDisplay() {
  if (!displayReady) {
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawLine("KHO LANH PID", 0, 1);
  drawLine("SP: " + formatFloat(setpointC) + " C", 12, 1);
  drawLine("DS: " + formatFloat(sensorData.coldTempC) + " C", 24, 1);
  drawLine("DHT: " + formatFloat(sensorData.ambientTempC) + " C", 36, 1);
  drawLine("PID: " + formatFloat(pidOutputPercent, 0) + " %", 48, 1);
  display.setCursor(88, 0);
  display.print(fanOn ? "ON" : "OFF");
  display.display();
}
void pushToBlynk() {
  if (!Blynk.connected()) {
    return;
  }
  Blynk.virtualWrite(VPIN_SETPOINT, setpointC);
  Blynk.virtualWrite(VPIN_COLD_TEMP, sensorData.dsValid ? sensorData.coldTempC : 0.0f);
  Blynk.virtualWrite(VPIN_AMBIENT_TEMP, sensorData.dhtValid ? sensorData.ambientTempC : 0.0f);
  Blynk.virtualWrite(VPIN_HUMIDITY, sensorData.dhtValid ? sensorData.humidity : 0.0f);
  Blynk.virtualWrite(VPIN_FAN_STATE, fanOn ? 1 : 0);
  Blynk.virtualWrite(VPIN_PID_OUTPUT, pidOutputPercent);
  String status = sensorData.dsValid ? "He thong OK | PID" : "Mat du lieu DS18B20";
  if (!sensorData.dhtValid) {
    status += " | DHT loi";
  }
  Blynk.virtualWrite(VPIN_STATUS, status);
}

// ---------------------------------------------------------------------------
// Sensor reading
// ---------------------------------------------------------------------------

void readSensors() {
  // DS18B20 — blocking conversion (~750 ms at 12-bit; call infrequently)
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

  // DHT11 — non-blocking read (library handles 1 s minimum interval internally)
  const float t = dht.readTemperature();
  const float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    sensorData.dhtValid      = false;
    sensorData.ambientTempC  = NAN;
    sensorData.humidity      = NAN;
  } else {
    sensorData.ambientTempC = t;
    sensorData.humidity     = h;
    sensorData.dhtValid     = true;
  }
}

// ---------------------------------------------------------------------------
// Button handling — debounce + auto-repeat
// ---------------------------------------------------------------------------

// Processes one button. Calls adjustSetpoint(delta) on initial press and then
// every BUTTON_REPEAT_MS while the button is held.
void processButton(ButtonState &btn, float delta) {
  const bool reading       = digitalRead(btn.pin);
  const unsigned long now  = millis();

  // Detect any level change and restart debounce timer
  if (reading != btn.lastReading) {
    btn.lastDebounceMs = now;
    btn.lastReading    = reading;
  }

  // Not yet stable — wait out the debounce window
  if (now - btn.lastDebounceMs < BUTTON_DEBOUNCE_MS) {
    return;
  }

  // Level is stable; check for state transition
  if (btn.stableLevel != reading) {
    btn.stableLevel = reading;
    if (reading == LOW) {
      // Initial press edge
      adjustSetpoint(delta);
      btn.lastRepeatMs = now;
    }
  } else if (reading == LOW && now - btn.lastRepeatMs >= BUTTON_REPEAT_MS) {
    // Held — fire auto-repeat
    adjustSetpoint(delta);
    btn.lastRepeatMs = now;
  }
}

// ---------------------------------------------------------------------------
// Blynk callbacks
// ---------------------------------------------------------------------------

// Called when Blynk reconnects — re-sync the setpoint slider to the local value
BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_SETPOINT);
}

// Remote setpoint update from Blynk dashboard or automation rule
BLYNK_WRITE(VPIN_SETPOINT) {
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
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("[INFO] Booting kho-lanh-esp32...");

  // Output pins — set to safe state before enabling drivers
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);
  writeRelayPin(false);  // compressor OFF by default

  // Input pins with internal pull-ups (buttons active-LOW)
  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayReady = true;
    display.clearDisplay();
    display.display();
    Serial.println("[INFO] SSD1306 OK");
  } else {
    Serial.println("[ERROR] SSD1306 init failed — check I2C wiring");
  }

  // DHT11
  dht.begin();

  // DS18B20 — scan bus and latch the first device address
  ds18b20.begin();
  const uint8_t dsCount = ds18b20.getDeviceCount();
  if (dsCount > 0) {
    hasProbeAddress = ds18b20.getAddress(coldProbeAddress, 0);
    ds18b20.setResolution(coldProbeAddress, 12);  // 0.0625 °C resolution
    Serial.printf("[INFO] DS18B20 found (%u device(s))\n", dsCount);
  } else {
    Serial.println("[WARN] No DS18B20 found on 1-Wire bus — check wiring and 4.7k pull-up");
  }

  // Restore setpoint from NVS; resets PID state
  loadSetpoint();
  resetPid();
  Serial.printf("[INFO] Setpoint loaded: %.1f C\n", setpointC);

  // --- Wi-Fi: manual connect with hard timeout ---
  // Blynk.begin() blocks forever on network failure; this approach lets the
  // WDT fire (and restart cleanly) if the AP is unreachable at boot.
  Serial.printf("[INFO] Connecting to WiFi: %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  {
    const unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - wifiStart > WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println("[ERROR] WiFi timeout — restarting");
        ESP.restart();  // clean restart; WDT would also catch this eventually
      }
      delay(500);
      Serial.print(".");
    }
  }
  Serial.printf("\n[INFO] WiFi OK  IP: %s\n", WiFi.localIP().toString().c_str());

  // --- Blynk: non-blocking config + connect (5 s timeout) ---
  Blynk.config(auth);
  if (!Blynk.connect(5000)) {
    Serial.println("[WARN] Blynk unreachable at boot — will retry in loop");
  }

  // --- Watchdog: enable AFTER network init so setup delay doesn't trip it ---
  esp_task_wdt_init(WDT_TIMEOUT_S, true);  // true = panic + reset on trigger
  esp_task_wdt_add(NULL);                  // subscribe the main Arduino task
  Serial.printf("[INFO] WDT armed (%lu s)\n", (unsigned long)WDT_TIMEOUT_S);

  Serial.println("[INFO] Setup complete — entering loop");
}

void loop() {
  esp_task_wdt_reset();  // feed watchdog — must be called within WDT_TIMEOUT_S

  Blynk.run();  // keep-alive and incoming write handler

  // Non-blocking Blynk reconnect (1 s attempt, does not stall the loop)
  if (!Blynk.connected()) {
    Blynk.connect(1000);
  }

  const unsigned long now = millis();

  // Sensor polling
  if (now - lastSensorReadMs >= SENSOR_READ_MS) {
    lastSensorReadMs = now;
    readSensors();
  }

  // Physical button handling
  processButton(btnUp,   +SETPOINT_STEP_C);
  processButton(btnDown, -SETPOINT_STEP_C);

  // Persist setpoint to NVS after a 3-second quiet period
  constexpr unsigned long SAVE_QUIET_MS = 3000UL;
  if (setpointDirty && (now - lastSetpointChangeMs >= SAVE_QUIET_MS)) {
    saveSetpoint();
    Serial.printf("[INFO] Setpoint saved: %.1f C\n", setpointC);
  }

  // PID computation + relay output
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
}  Blynk.virtualWrite(VPIN_STATUS, status);
}

// ---------------------------------------------------------------------------
// Sensor reading
// ---------------------------------------------------------------------------

void readSensors() {
  // DS18B20 — blocking conversion (~750 ms at 12-bit; call infrequently)
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

  // DHT11 — non-blocking read (library handles 1 s minimum interval internally)
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
// ---------------------------------------------------------------------------

// Processes one button. Calls adjustSetpoint(delta) on initial press and then
// every BUTTON_REPEAT_MS while the button is held down.
void processButton(ButtonState &btn, float delta) {
  const bool reading      = digitalRead(btn.pin);
  const unsigned long now = millis();

  // Any level change resets the debounce timer
  if (reading != btn.lastReading) {
    btn.lastDebounceMs = now;
    btn.lastReading    = reading;
  }

  // Wait until the signal has been stable for at least BUTTON_DEBOUNCE_MS
  if (now - btn.lastDebounceMs < BUTTON_DEBOUNCE_MS) {
    return;
  }

  // Stable level changed — process edge
  if (btn.stableLevel != reading) {
    btn.stableLevel = reading;
    if (reading == LOW) {
      // Initial press: fire immediately
      adjustSetpoint(delta);
      btn.lastRepeatMs = now;
    }
  } else if (reading == LOW && now - btn.lastRepeatMs >= BUTTON_REPEAT_MS) {
    // Button held: auto-repeat
    adjustSetpoint(delta);
    btn.lastRepeatMs = now;
  }
}

// ---------------------------------------------------------------------------
// Blynk callbacks
// ---------------------------------------------------------------------------

// Re-sync setpoint slider whenever Blynk reconnects
BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_SETPOINT);
}

// Remote setpoint write from dashboard slider or automation rule
BLYNK_WRITE(VPIN_SETPOINT) {
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
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("[INFO] Booting kho-lanh-esp32...");

  // Output pins — drive to safe state before enabling any load
  pinMode(PIN_RELAY,      OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);
  writeRelayPin(false);  // compressor OFF by default

  // Input pins — active-LOW buttons with internal pull-ups
  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayReady = true;
    display.clearDisplay();
    display.display();
    Serial.println("[INFO] SSD1306 OK");
  } else {
    Serial.println("[ERROR] SSD1306 init failed — check I2C address and wiring");
  }

  // DHT11
  dht.begin();

  // DS18B20 — scan 1-Wire bus and latch the first sensor address
  ds18b20.begin();
  const uint8_t dsCount = ds18b20.getDeviceCount();
  if (dsCount > 0) {
    hasProbeAddress = ds18b20.getAddress(coldProbeAddress, 0);
    ds18b20.setResolution(coldProbeAddress, 12);  // 12-bit = 0.0625 C resolution
    Serial.printf("[INFO] DS18B20 found (%u device(s) on bus)\n", dsCount);
  } else {
    Serial.println("[WARN] No DS18B20 on 1-Wire bus — check wiring and 4.7k pull-up");
  }

  // Restore setpoint from NVS; initialize PID state clean
  loadSetpoint();
  resetPid();
  Serial.printf("[INFO] Setpoint restored: %.1f C\n", setpointC);

  // Connect to Wi-Fi and Blynk (blocking until connected or timeout)
  Blynk.begin(auth, ssid, pass);

  Serial.println("[INFO] Setup complete — entering loop");
}

void loop() {
  Blynk.run();  // must be called every iteration for keep-alive and incoming writes

  const unsigned long now = millis();

  // Poll sensors every SENSOR_READ_MS
  if (now - lastSensorReadMs >= SENSOR_READ_MS) {
    lastSensorReadMs = now;
    readSensors();
  }

  // Process physical buttons (UP = +0.5 C, DOWN = -0.5 C)
  processButton(btnUp,   +SETPOINT_STEP_C);
  processButton(btnDown, -SETPOINT_STEP_C);

  // Auto-save setpoint to NVS after a 3-second quiet period
  constexpr unsigned long SAVE_QUIET_MS = 3000UL;
  if (setpointDirty && (now - lastSetpointChangeMs >= SAVE_QUIET_MS)) {
    saveSetpoint();
    Serial.printf("[INFO] Setpoint saved to NVS: %.1f C\n", setpointC);
  }

  // PID compute + relay time-proportioning
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
