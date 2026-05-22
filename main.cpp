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
