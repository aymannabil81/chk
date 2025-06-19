#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <HTTPClient.h>

// Function prototypes
void handleRoot();
void handleControl();
void handleHeatingTempUpdate();
void handleCoolingTempUpdate();
void handleTempDiffUpdate();
void handleAmmoniaUpdate();
void handleVentilationTempUpdate();
void handleVentilation2TempUpdate();
void handleHeatingTimerUpdate();
void handleCoolingTimerUpdate();
void handleVentilationTimerUpdate();
void handleVentilation2TimerUpdate();
void handleRelayAutoUpdate();
void handleCalibrateMQ135();
void connectToWiFi();
void checkWiFiConnection();
void readTemperature();
float readNTC();
void checkAmmoniaLevel();
float readMQ135();
float readAmmoniaLevel();
float getMQ135Resistance(int rawADC);
void calibrateMQ135();
void handleHeatingTimer();
void handleCoolingTimer();
void handleVentilationTimer();
void handleVentilation2Timer();
String getTime();
String getHeatingRemainingTime();
String getCoolingRemainingTime();
String getVentilationRemainingTime();
String getVentilation2RemainingTime();
String formatDuration(unsigned long seconds);

// NEW: Humidity control (Relay5)
void handleHumidityTimer();
void handleHumidityTimerUpdate();
String getHumidityRemainingTime();

// EEPROM settings
#define EEPROM_SIZE 256
#define HEATING_TARGET_TEMP_ADDR 0
#define COOLING_TARGET_TEMP_ADDR 4
#define TEMP_DIFF_ADDR 8
#define AMMONIA_THRESHOLD_ADDR 12
#define VENTILATION_TEMP_ADDR 16
#define CONTROL_MODE_HEATING_ADDR 20
#define CONTROL_MODE_COOLING_ADDR 24
#define CONTROL_MODE_VENTILATION_ADDR 28
#define HEATING_TIMER_ON_ADDR 32
#define HEATING_TIMER_OFF_ADDR 36
#define COOLING_TIMER_ON_ADDR 40
#define COOLING_TIMER_OFF_ADDR 44
#define VENTILATION_TIMER_ON_ADDR 48
#define VENTILATION_TIMER_OFF_ADDR 52
#define MQ135_RZERO_ADDR 56
#define MQ135_R0_ADDR 60
#define VENTILATION2_TEMP_ADDR 64
#define VENTILATION2_TIMER_ON_ADDR 68
#define VENTILATION2_TIMER_OFF_ADDR 72
#define CONTROL_MODE_VENTILATION2_ADDR 76

// NEW: EEPROM addresses for Humidity Timer/Control
#define HUMIDITY_CONTROL_MODE_ADDR 80
#define HUMIDITY_TARGET_ADDR 84
#define HUMIDITY_TIMER_ON_ADDR 88
#define HUMIDITY_TIMER_OFF_ADDR 92

// Control modes
enum ControlMode { CM_DISABLED, CM_MANUAL, CM_TEMPERATURE };
ControlMode currentHeatingMode = CM_DISABLED;
ControlMode currentCoolingMode = CM_DISABLED;
ControlMode currentVentilationMode = CM_DISABLED;
ControlMode currentVentilation2Mode = CM_DISABLED;
ControlMode currentHumidityMode = CM_DISABLED; // NEW

// Relay definitions
const int relayPins[] = {23, 22, 21, 19, 18, 17, 16, 27};
#define HEATING_RELAY 0
#define COOLING_RELAY 1
#define VENTILATION_RELAY 2
#define VENTILATION2_RELAY 3
#define HUMIDITY_RELAY 4 // Relay5 assigned for humidity
const int relayCount = 8;

// Sensor pins
#define DS18B20_PIN 4
#define DHT_PIN 5
#define NTC_PIN 34
#define MQ135_PIN 35

// Sensor constants
#define DHT_TYPE DHT22
const int R1 = 10000;
const float Vcc = 5.0;

//ddns update
#define DYNU_UPDATE_INTERVAL 300000 // 5 minutes
const char* dynu_url = "https://api.dynu.com/nic/update?hostname=aymanfarm.kozow.com&password=c11324d0b6c4e88df7489c02164d28eb";

// MQ135 calibration
#define ATMOSPHERIC_CO2 410
#define RL 10.0
#define RZERO_CALIBRATION_SAMPLE_TIMES 50
#define RZERO_CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_INTERVAL 50
#define READ_SAMPLE_TIMES 5
float MQ135_RZERO = 76.63;
float MQ135_R0 = 10.0;

// System variables
float ammoniaThreshold = 50.0;
float ammoniaLevel = 0.0;
bool ammoniaAlert = false;
float ventilationTempThreshold = 30.0;
float ventilation2TempThreshold = 32.0;
float heatingTargetTemp = 25.0;
float coolingTargetTemp = 22.0;
float tempDifference = 2.0;
float currentTemp = 0;
float humidity = 0;

// NEW: Humidity control variables
float humidityTarget = 60.0; // Default 60%
unsigned long humidityOnDuration = 1800;
unsigned long humidityOffDuration = 1800;
unsigned long humidityTimerStartTime = 0;
bool isHumidityOn = true; // Always start ON

// Timer variables
unsigned long lastDynuUpdate = 0;

unsigned long heatingOnDuration = 1800;
unsigned long heatingOffDuration = 1800;
unsigned long heatingTimerStartTime = 0;
bool isHeatingOn = true; // Always start ON
unsigned long coolingOnDuration = 1800;
unsigned long coolingOffDuration = 1800;
unsigned long coolingTimerStartTime = 0;
bool isCoolingOn = true; // Always start ON
unsigned long ventilationOnDuration = 1800;
unsigned long ventilationOffDuration = 1800;
unsigned long ventilationTimerStartTime = 0;
bool isVentilationOn = true; // Always start ON
unsigned long ventilation2OnDuration = 1800;
unsigned long ventilation2OffDuration = 1800;
unsigned long ventilation2TimerStartTime = 0;
bool isVentilation2On = true; // Always start ON

// WiFi networks
const int MAX_NETWORKS = 3;
struct WiFiNetwork {
  const char* ssid;
  const char* password;
};

WiFiNetwork networks[MAX_NETWORKS] = {
  {"NAVK", "NEwdays<127912>"},
  {"CHK", "Anbawanas127982645127912"}
};

// Relay control settings
bool relayAllowAuto[relayCount] = {1,1,1,1,1,1,1,1};

// Sensor objects
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);

// --- Always start ON when timer starts or on settings change ---
void resetHeatingTimer() {
  isHeatingOn = true;
  heatingTimerStartTime = millis();
  if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], LOW);
}
void resetCoolingTimer() {
  isCoolingOn = true;
  coolingTimerStartTime = millis();
  if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], LOW);
}
void resetVentilationTimer() {
  isVentilationOn = true;
  ventilationTimerStartTime = millis();
  if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], LOW);
}
void resetVentilation2Timer() {
  isVentilation2On = true;
  ventilation2TimerStartTime = millis();
  if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], LOW);
}
// NEW: reset humidity timer
void resetHumidityTimer() {
  isHumidityOn = true;
  humidityTimerStartTime = millis();
  if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
}

void setup() {
  Serial.begin(115200);

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Read saved values
  EEPROM.get(HEATING_TARGET_TEMP_ADDR, heatingTargetTemp);
  EEPROM.get(COOLING_TARGET_TEMP_ADDR, coolingTargetTemp);
  EEPROM.get(TEMP_DIFF_ADDR, tempDifference);
  EEPROM.get(AMMONIA_THRESHOLD_ADDR, ammoniaThreshold);
  EEPROM.get(VENTILATION_TEMP_ADDR, ventilationTempThreshold);
  EEPROM.get(MQ135_RZERO_ADDR, MQ135_RZERO);
  EEPROM.get(MQ135_R0_ADDR, MQ135_R0);
  EEPROM.get(VENTILATION2_TEMP_ADDR, ventilation2TempThreshold);

  // NEW: Humidity settings
  EEPROM.get(HUMIDITY_TARGET_ADDR, humidityTarget);
  EEPROM.get(HUMIDITY_TIMER_ON_ADDR, humidityOnDuration);
  EEPROM.get(HUMIDITY_TIMER_OFF_ADDR, humidityOffDuration);

  // Read control modes
  int heatingMode, coolingMode, ventilationMode, ventilation2Mode, humidityMode;
  EEPROM.get(CONTROL_MODE_HEATING_ADDR, heatingMode);
  EEPROM.get(CONTROL_MODE_COOLING_ADDR, coolingMode);
  EEPROM.get(CONTROL_MODE_VENTILATION_ADDR, ventilationMode);
  EEPROM.get(CONTROL_MODE_VENTILATION2_ADDR, ventilation2Mode);
  EEPROM.get(HUMIDITY_CONTROL_MODE_ADDR, humidityMode);

  currentHeatingMode = static_cast<ControlMode>(heatingMode);
  currentCoolingMode = static_cast<ControlMode>(coolingMode);
  currentVentilationMode = static_cast<ControlMode>(ventilationMode);
  currentVentilation2Mode = static_cast<ControlMode>(ventilation2Mode);
  currentHumidityMode = static_cast<ControlMode>(humidityMode);

  // Read timer durations
  EEPROM.get(HEATING_TIMER_ON_ADDR, heatingOnDuration);
  EEPROM.get(HEATING_TIMER_OFF_ADDR, heatingOffDuration);
  EEPROM.get(COOLING_TIMER_ON_ADDR, coolingOnDuration);
  EEPROM.get(COOLING_TIMER_OFF_ADDR, coolingOffDuration);
  EEPROM.get(VENTILATION_TIMER_ON_ADDR, ventilationOnDuration);
  EEPROM.get(VENTILATION_TIMER_OFF_ADDR, ventilationOffDuration);
  EEPROM.get(VENTILATION2_TIMER_ON_ADDR, ventilation2OnDuration);
  EEPROM.get(VENTILATION2_TIMER_OFF_ADDR, ventilation2OffDuration);

  // Set default values if invalid
  if (isnan(heatingTargetTemp)) heatingTargetTemp = 25.0;
  if (isnan(coolingTargetTemp)) coolingTargetTemp = 22.0;
  if (isnan(tempDifference)) tempDifference = 2.0;
  if (isnan(ammoniaThreshold)) ammoniaThreshold = 50.0;
  if (isnan(ventilationTempThreshold)) ventilationTempThreshold = 30.0;
  if (isnan(ventilation2TempThreshold)) ventilation2TempThreshold = 32.0;
  if (isnan(humidityTarget) || humidityTarget < 10.0 || humidityTarget > 99.0) humidityTarget = 60.0;
  if (isnan(heatingMode)) currentHeatingMode = CM_DISABLED;
  if (isnan(coolingMode)) currentCoolingMode = CM_DISABLED;
  if (isnan(ventilationMode)) currentVentilationMode = CM_DISABLED;
  if (isnan(ventilation2Mode)) currentVentilation2Mode = CM_DISABLED;
  if (isnan(humidityMode)) currentHumidityMode = CM_DISABLED;
  if (isnan(heatingOnDuration)) heatingOnDuration = 1800;
  if (isnan(heatingOffDuration)) heatingOffDuration = 1800;
  if (isnan(coolingOnDuration)) coolingOnDuration = 1800;
  if (isnan(coolingOffDuration)) coolingOffDuration = 1800;
  if (isnan(ventilationOnDuration)) ventilationOnDuration = 1800;
  if (isnan(ventilationOffDuration)) ventilationOffDuration = 1800;
  if (isnan(ventilation2OnDuration)) ventilation2OnDuration = 1800;
  if (isnan(ventilation2OffDuration)) ventilation2OffDuration = 1800;
  if (isnan(humidityOnDuration)) humidityOnDuration = 1800;
  if (isnan(humidityOffDuration)) humidityOffDuration = 1800;
  if (isnan(MQ135_RZERO)) {
    MQ135_RZERO = 76.63;
    calibrateMQ135();
  }
  if (isnan(MQ135_R0)) MQ135_R0 = 10.0;

  // Initialize relay pins
  for(int i=0; i<relayCount; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);
  }

  // Start sensors
  ds18b20.begin();
  dht.begin();

  // Connect to WiFi
  connectToWiFi();

  // Define server routes
  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/update_heating_temp", HTTP_POST, handleHeatingTempUpdate);
  server.on("/update_cooling_temp", HTTP_POST, handleCoolingTempUpdate);
  server.on("/update_temp_diff", HTTP_POST, handleTempDiffUpdate);
  server.on("/update_ammonia", HTTP_POST, handleAmmoniaUpdate);
  server.on("/update_ventilation_temp", HTTP_POST, handleVentilationTempUpdate);
  server.on("/update_ventilation2_temp", HTTP_POST, handleVentilation2TempUpdate);
  server.on("/update_heating_timer", HTTP_POST, handleHeatingTimerUpdate);
  server.on("/update_cooling_timer", HTTP_POST, handleCoolingTimerUpdate);
  server.on("/update_ventilation_timer", HTTP_POST, handleVentilationTimerUpdate);
  server.on("/update_ventilation2_timer", HTTP_POST, handleVentilation2TimerUpdate);
  server.on("/update_humidity_timer", HTTP_POST, handleHumidityTimerUpdate); // NEW
  server.on("/calibrate_mq135", handleCalibrateMQ135);
  server.on("/relay_auto", HTTP_POST, handleRelayAutoUpdate);

  server.begin();

  // Always start timers ON on power-up
  resetHeatingTimer();
  resetCoolingTimer();
  resetVentilationTimer();
  resetVentilation2Timer();
  resetHumidityTimer();
}

void calibrateMQ135() {
  Serial.println(F("جارٍ معايرة حساس MQ135..."));

  float rs = 0;
  for (int i = 0; i < RZERO_CALIBRATION_SAMPLE_TIMES; i++) {
    rs += getMQ135Resistance(analogRead(MQ135_PIN));
    delay(RZERO_CALIBRATION_SAMPLE_INTERVAL);
  }
  rs = rs / RZERO_CALIBRATION_SAMPLE_TIMES;

  MQ135_RZERO = rs * pow((ATMOSPHERIC_CO2/398.107), 1/-2.612);
  MQ135_R0 = rs / 9.83; // نسبة RS/R0 في الهواء النظيف

  EEPROM.put(MQ135_RZERO_ADDR, MQ135_RZERO);
  EEPROM.put(MQ135_R0_ADDR, MQ135_R0);
  EEPROM.commit();

  Serial.print(F("تمت المعايرة. RZero الجديدة: "));
  Serial.println(MQ135_RZERO);
  Serial.print(F("R0 الجديدة: "));
  Serial.println(MQ135_R0);
}

float getMQ135Resistance(int rawADC) {
  return ((1023./(float)rawADC) * 5. - 1.) * RL;
}

float readMQ135() {
  float rs = 0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += getMQ135Resistance(analogRead(MQ135_PIN));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs / READ_SAMPLE_TIMES;

  float ratio = rs / MQ135_RZERO;
  float ppm = 116.6020682 * pow(ratio, -2.769034857);
  return ppm;
}

float readAmmoniaLevel() {
  float rs = 0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += getMQ135Resistance(analogRead(MQ135_PIN));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs / READ_SAMPLE_TIMES;

  float ratio = rs / MQ135_R0;
  float ppm = 0.0;
  ppm = pow(10, (log10(ratio) - 0.8) / 0.2);
  return ppm;
}

void handleCalibrateMQ135() {
  calibrateMQ135();
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), F("تمت معايرة الحساس بنجاح"));
}

void connectToWiFi() {
  Serial.println(F("محاولة الاتصال بشبكة WiFi..."));

  for(int i = 0; i < MAX_NETWORKS; i++) {
    if(networks[i].ssid == NULL || strlen(networks[i].ssid) == 0) {
      continue;
    }

    Serial.print(F("جاري الاتصال بشبكة: "));
    Serial.println(networks[i].ssid);

    WiFi.begin(networks[i].ssid, networks[i].password);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
      delay(500);
      Serial.print(F("."));
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("\nتم الاتصال بنجاح"));
      Serial.print(F("الشبكة: "));
      Serial.println(networks[i].ssid);
      Serial.print(F("عنوان IP: "));
      Serial.println(WiFi.localIP());
      return;
    }

    Serial.println(F("\nفشل الاتصال بهذه الشبكة، جاري المحاولة على الشبكة التالية..."));
    WiFi.disconnect();
    delay(1000);
  }

  Serial.println(F("\nفشل الاتصال بأي شبكة WiFi!"));
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  if(millis() - lastCheck > 30000) {
    if(WiFi.status() != WL_CONNECTED) {
      Serial.println(F("فقدان الاتصال بشبكة WiFi، جاري إعادة الاتصال..."));
      connectToWiFi();
    }
    lastCheck = millis();
  }
}

void updateDynuDNS() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(dynu_url);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println("Dynu DNS Update Response: " + payload);
    } else {
      Serial.println("Dynu DNS Update Failed");
    }
    http.end();
  }
}

// ---------- TIMER LOGIC: Always start ON when mode/settings change ----------

void handleHeatingTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - heatingTimerStartTime) / 1000;
  if (currentHeatingMode == CM_DISABLED) {
    if (currentTemp <= (heatingTargetTemp - tempDifference/2)) {
      if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], LOW);
      isHeatingOn = true;
      heatingTimerStartTime = currentTime;
    } else if (currentTemp >= (heatingTargetTemp + tempDifference/2)) {
      if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], HIGH);
      isHeatingOn = false;
      heatingTimerStartTime = currentTime;
    }
  } else if (currentHeatingMode == CM_MANUAL) {
    if (isHeatingOn && elapsedTime >= heatingOnDuration) {
      if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], HIGH);
      isHeatingOn = false;
      heatingTimerStartTime = currentTime;
    } else if (!isHeatingOn && elapsedTime >= heatingOffDuration) {
      if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], LOW);
      isHeatingOn = true;
      heatingTimerStartTime = currentTime;
    }
  } else if (currentHeatingMode == CM_TEMPERATURE) {
    if (currentTemp <= (heatingTargetTemp - tempDifference/2)) {
      if (isHeatingOn && elapsedTime >= heatingOnDuration) {
        if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], HIGH);
        isHeatingOn = false;
        heatingTimerStartTime = currentTime;
      } else if (!isHeatingOn && elapsedTime >= heatingOffDuration) {
        if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], LOW);
        isHeatingOn = true;
        heatingTimerStartTime = currentTime;
      }
    } else {
      if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], HIGH);
      isHeatingOn = false;
      heatingTimerStartTime = currentTime;
    }
  }
}

void handleCoolingTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - coolingTimerStartTime) / 1000;
  if (currentCoolingMode == CM_DISABLED) {
    if (currentTemp >= (coolingTargetTemp + tempDifference/2)) {
      if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], LOW);
      isCoolingOn = true;
      coolingTimerStartTime = currentTime;
    } else if (currentTemp <= (coolingTargetTemp - tempDifference/2)) {
      if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], HIGH);
      isCoolingOn = false;
      coolingTimerStartTime = currentTime;
    }
  } else if (currentCoolingMode == CM_MANUAL) {
    if (isCoolingOn && elapsedTime >= coolingOnDuration) {
      if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], HIGH);
      isCoolingOn = false;
      coolingTimerStartTime = currentTime;
    } else if (!isCoolingOn && elapsedTime >= coolingOffDuration) {
      if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], LOW);
      isCoolingOn = true;
      coolingTimerStartTime = currentTime;
    }
  } else if (currentCoolingMode == CM_TEMPERATURE) {
    if (currentTemp >= (coolingTargetTemp + tempDifference/2)) {
      if (isCoolingOn && elapsedTime >= coolingOnDuration) {
        if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], HIGH);
        isCoolingOn = false;
        coolingTimerStartTime = currentTime;
      } else if (!isCoolingOn && elapsedTime >= coolingOffDuration) {
        if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], LOW);
        isCoolingOn = true;
        coolingTimerStartTime = currentTime;
      }
    } else {
      if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], HIGH);
      isCoolingOn = false;
      coolingTimerStartTime = currentTime;
    }
  }
}

void handleVentilationTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - ventilationTimerStartTime) / 1000;
  if (currentVentilationMode == CM_DISABLED) {
    if (currentTemp >= ventilationTempThreshold || ammoniaLevel > ammoniaThreshold) {
      if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], LOW);
      isVentilationOn = true;
      ventilationTimerStartTime = currentTime;
    } else {
      if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], HIGH);
      isVentilationOn = false;
      ventilationTimerStartTime = currentTime;
    }
  } else if (currentVentilationMode == CM_MANUAL) {
    if (isVentilationOn && elapsedTime >= ventilationOnDuration) {
      if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], HIGH);
      isVentilationOn = false;
      ventilationTimerStartTime = currentTime;
    } else if (!isVentilationOn && elapsedTime >= ventilationOffDuration) {
      if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], LOW);
      isVentilationOn = true;
      ventilationTimerStartTime = currentTime;
    }
  } else if (currentVentilationMode == CM_TEMPERATURE) {
    if (currentTemp >= ventilationTempThreshold || ammoniaLevel > ammoniaThreshold) {
      if (isVentilationOn && elapsedTime >= ventilationOnDuration) {
        if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], HIGH);
        isVentilationOn = false;
        ventilationTimerStartTime = currentTime;
      } else if (!isVentilationOn && elapsedTime >= ventilationOffDuration) {
        if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], LOW);
        isVentilationOn = true;
        ventilationTimerStartTime = currentTime;
      }
    } else {
      if (relayAllowAuto[VENTILATION_RELAY]) digitalWrite(relayPins[VENTILATION_RELAY], HIGH);
      isVentilationOn = false;
      ventilationTimerStartTime = currentTime;
    }
  }
}

void handleVentilation2Timer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - ventilation2TimerStartTime) / 1000;
  if (!relayAllowAuto[VENTILATION2_RELAY]) {
    digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
    isVentilation2On = false;
    return;
  }
  if (currentVentilation2Mode == CM_DISABLED) {
    if (currentTemp >= ventilation2TempThreshold || ammoniaLevel > ammoniaThreshold) {
      digitalWrite(relayPins[VENTILATION2_RELAY], LOW);
      isVentilation2On = true;
      ventilation2TimerStartTime = currentTime;
    } else {
      digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
      isVentilation2On = false;
      ventilation2TimerStartTime = currentTime;
    }
  } else if (currentVentilation2Mode == CM_MANUAL) {
    if (isVentilation2On && elapsedTime >= ventilation2OnDuration) {
      digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
      isVentilation2On = false;
      ventilation2TimerStartTime = currentTime;
    } else if (!isVentilation2On && elapsedTime >= ventilation2OffDuration) {
      digitalWrite(relayPins[VENTILATION2_RELAY], LOW);
      isVentilation2On = true;
      ventilation2TimerStartTime = currentTime;
    }
  } else if (currentVentilation2Mode == CM_TEMPERATURE) {
    if (currentTemp >= ventilation2TempThreshold || ammoniaLevel > ammoniaThreshold) {
      if (isVentilation2On && elapsedTime >= ventilation2OnDuration) {
        digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
        isVentilation2On = false;
        ventilation2TimerStartTime = currentTime;
      } else if (!isVentilation2On && elapsedTime >= ventilation2OffDuration) {
        digitalWrite(relayPins[VENTILATION2_RELAY], LOW);
        isVentilation2On = true;
        ventilation2TimerStartTime = currentTime;
      }
    } else {
      digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
      isVentilation2On = false;
      ventilation2TimerStartTime = currentTime;
    }
  }
}

// --------- NEW: Humidity Timer Logic (Relay5) -----------

void handleHumidityTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - humidityTimerStartTime) / 1000;
  if (!relayAllowAuto[HUMIDITY_RELAY]) {
    digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
    isHumidityOn = false;
    return;
  }
  if (currentHumidityMode == CM_DISABLED) {
    if (humidity <= (humidityTarget - 1.0)) {
      digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
      isHumidityOn = true;
      humidityTimerStartTime = currentTime;
    } else if (humidity >= (humidityTarget + 1.0)) {
      digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
      isHumidityOn = false;
      humidityTimerStartTime = currentTime;
    }
  } else if (currentHumidityMode == CM_MANUAL) {
    if (isHumidityOn && elapsedTime >= humidityOnDuration) {
      digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
      isHumidityOn = false;
      humidityTimerStartTime = currentTime;
    } else if (!isHumidityOn && elapsedTime >= humidityOffDuration) {
      digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
      isHumidityOn = true;
      humidityTimerStartTime = currentTime;
    }
  } else if (currentHumidityMode == CM_TEMPERATURE) {
    if (humidity <= (humidityTarget - 1.0)) {
      if (isHumidityOn && elapsedTime >= humidityOnDuration) {
        digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
        isHumidityOn = false;
        humidityTimerStartTime = currentTime;
      } else if (!isHumidityOn && elapsedTime >= humidityOffDuration) {
        digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
        isHumidityOn = true;
        humidityTimerStartTime = currentTime;
      }
    } else {
      digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
      isHumidityOn = false;
      humidityTimerStartTime = currentTime;
    }
  }
}

String getHumidityRemainingTime() {
  if (humidityTimerStartTime == 0) return F("لم يبدأ");
  unsigned long elapsed = (millis() - humidityTimerStartTime) / 1000;
  unsigned long totalCycle = isHumidityOn ? humidityOnDuration : humidityOffDuration;
  if (elapsed >= totalCycle) return F("جاهز للتبديل");
  unsigned long remaining = totalCycle - elapsed;
  unsigned long days = remaining / 86400;
  remaining = remaining % 86400;
  unsigned long hours = remaining / 3600;
  remaining = remaining % 3600;
  unsigned long minutes = remaining / 60;
  unsigned long seconds = remaining % 60;
  char buf[50];
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  }
  return String(buf) + (isHumidityOn ? F(" (تشغيل)") : F(" (إيقاف)"));
}

// --------- END Humidity Timer -----------

void handleRelayAutoUpdate() {
  for (int i = 0; i < relayCount; i++) {
    String allowName = "allow_auto_" + String(i);
    relayAllowAuto[i] = server.hasArg(allowName) ? 1 : 0;
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void loop() {
  server.handleClient();
  checkWiFiConnection();

  if (millis() - lastDynuUpdate > DYNU_UPDATE_INTERVAL) {
    updateDynuDNS();
    lastDynuUpdate = millis();
  }

  static unsigned long lastTempRead = 0;
  if(millis() - lastTempRead > 2000) {
    readTemperature();
    handleHeatingTimer();
    handleCoolingTimer();
    handleVentilationTimer();
    handleVentilation2Timer();
    handleHumidityTimer(); // NEW
    checkAmmoniaLevel();
    lastTempRead = millis();
  }
  delay(2);
}

void readTemperature() {
  float tempDHT = dht.readTemperature();
  humidity = dht.readHumidity();

  if(!isnan(tempDHT)) {
    currentTemp = tempDHT;
    return;
  }

  ds18b20.requestTemperatures();
  float tempDS18 = ds18b20.getTempCByIndex(0);
  if(!isnan(tempDS18)) {
    currentTemp = tempDS18;
    Serial.println(F("فشل قراءة DHT22! جاري استخدام DS18B20"));
    return;
  }

  float tempNTC = readNTC();
  currentTemp = tempNTC;
  Serial.println(F("فشل قراءة DHT22 و DS18B20! جاري استخدام NTC"));
}

float readNTC() {
  int raw = analogRead(NTC_PIN);
  float Vout = raw * (Vcc / 4095.0) * 0.5;
  float Rntc = (Vcc - Vout) * R1 / Vout;
  float tempK = 1 / (1/298.15 + log(Rntc/10000)/3950.0);
  return tempK - 273.15;
}

void checkAmmoniaLevel() {
  ammoniaLevel = readAmmoniaLevel();

  if (ammoniaLevel > ammoniaThreshold) {
    ammoniaAlert = true;
  } else {
    ammoniaAlert = false;
  }
}

String getTime() {
  unsigned long seconds = millis() / 1000;
  unsigned long days = seconds / 86400;
  seconds = seconds % 86400;
  unsigned long hours = seconds / 3600;
  seconds = seconds % 3600;
  unsigned long minutes = seconds / 60;
  unsigned long secondsRemaining = seconds % 60;

  char buf[50];
  snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, secondsRemaining);
  return String(buf) + F(" منذ التشغيل");
}

String getHeatingRemainingTime() {
  if (heatingTimerStartTime == 0) return F("لم يبدأ");

  unsigned long elapsed = (millis() - heatingTimerStartTime) / 1000;
  unsigned long totalCycle = isHeatingOn ? heatingOnDuration : heatingOffDuration;

  if (elapsed >= totalCycle) return F("جاهز للتبديل");

  unsigned long remaining = totalCycle - elapsed;
  unsigned long days = remaining / 86400;
  remaining = remaining % 86400;
  unsigned long hours = remaining / 3600;
  remaining = remaining % 3600;
  unsigned long minutes = remaining / 60;
  unsigned long seconds = remaining % 60;

  char buf[50];
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  }
  return String(buf) + (isHeatingOn ? F(" (تشغيل)") : F(" (إيقاف)"));
}

String getCoolingRemainingTime() {
  if (coolingTimerStartTime == 0) return F("لم يبدأ");

  unsigned long elapsed = (millis() - coolingTimerStartTime) / 1000;
  unsigned long totalCycle = isCoolingOn ? coolingOnDuration : coolingOffDuration;

  if (elapsed >= totalCycle) return F("جاهز للتبديل");

  unsigned long remaining = totalCycle - elapsed;
  unsigned long days = remaining / 86400;
  remaining = remaining % 86400;
  unsigned long hours = remaining / 3600;
  remaining = remaining % 3600;
  unsigned long minutes = remaining / 60;
  unsigned long seconds = remaining % 60;

  char buf[50];
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  }
  return String(buf) + (isCoolingOn ? F(" (تشغيل)") : F(" (إيقاف)"));
}

String getVentilationRemainingTime() {
  if (ventilationTimerStartTime == 0) return F("لم يبدأ");

  unsigned long elapsed = (millis() - ventilationTimerStartTime) / 1000;
  unsigned long totalCycle = isVentilationOn ? ventilationOnDuration : ventilationOffDuration;

  if (elapsed >= totalCycle) return F("جاهز للتبديل");

  unsigned long remaining = totalCycle - elapsed;
  unsigned long days = remaining / 86400;
  remaining = remaining % 86400;
  unsigned long hours = remaining / 3600;
  remaining = remaining % 3600;
  unsigned long minutes = remaining / 60;
  unsigned long seconds = remaining % 60;

  char buf[50];
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  }
  return String(buf) + (isVentilationOn ? F(" (تشغيل)") : F(" (إيقاف)"));
}

String getVentilation2RemainingTime() {
  if (ventilation2TimerStartTime == 0) return F("لم يبدأ");
  unsigned long elapsed = (millis() - ventilation2TimerStartTime) / 1000;
  unsigned long totalCycle = isVentilation2On ? ventilation2OnDuration : ventilation2OffDuration;
  if (elapsed >= totalCycle) return F("جاهز للتبديل");
  unsigned long remaining = totalCycle - elapsed;
  unsigned long days = remaining / 86400;
  remaining = remaining % 86400;
  unsigned long hours = remaining / 3600;
  remaining = remaining % 3600;
  unsigned long minutes = remaining / 60;
  unsigned long seconds = remaining % 60;
  char buf[50];
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  }
  return String(buf) + (isVentilation2On ? F(" (تشغيل)") : F(" (إيقاف)"));
}

String formatDuration(unsigned long seconds) {
  unsigned long days = seconds / 86400;
  seconds = seconds % 86400;
  unsigned long hours = seconds / 3600;
  seconds = seconds % 3600;
  unsigned long minutes = seconds / 60;
  unsigned long secs = seconds % 60;

  char buf[50];
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, secs);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, secs);
  }
  return String(buf);
}

// --- PAGE: handleRoot() ---
// (No change except add Humidity Timer card below)

void handleRoot() {
  float tempDS18 = ds18b20.getTempCByIndex(0);
  float tempNTC = readNTC();
  float mq135 = readMQ135();
  float humidityVal = dht.readHumidity();
  float dhtTemp = dht.readTemperature();

  // ... (the HTML code as above, with the following insert for Humidity Timer) ...

  // ADD THIS SECTION right after the cards for heating/cooling/ventilation/ventilation2:
  // ---- START HUMIDITY TIMER CARD HTML ----
  String html = FPSTR(R"=====(...)====="); // ... rest of existing HTML before the timer cards

  // (insert after the other timer cards)
  html += FPSTR(R"=====(
      <div class="card">
        <h2 class="card-title"><i>💧</i> التحكم في الرطوبة (Relay5)</h2>
        <form action="/update_humidity_timer" method="post">
          <div class="radio-group">
            <div class="radio-option">
              <input type="radio" id="humidity_mode_disabled" name="humidity_control_mode" value="0" )=====");
  html += (currentHumidityMode == CM_DISABLED) ? "checked" : "";
  html += FPSTR(R"=====(>
              <label for="humidity_mode_disabled">معطل (حسب الرطوبة فقط)</label>
            </div>
            <div class="radio-option">
              <input type="radio" id="humidity_mode_manual" name="humidity_control_mode" value="1" )=====");
  html += (currentHumidityMode == CM_MANUAL) ? "checked" : "";
  html += FPSTR(R"=====(>
              <label for="humidity_mode_manual">يدوي (بالتايمر فقط)</label>
            </div>
            <div class="radio-option">
              <input type="radio" id="humidity_mode_temp" name="humidity_control_mode" value="2" )=====");
  html += (currentHumidityMode == CM_TEMPERATURE) ? "checked" : "";
  html += FPSTR(R"=====(>
              <label for="humidity_mode_temp">تلقائي (حسب الرطوبة والتايمر)</label>
            </div>
          </div>
          <div class="form-group">
            <label for="humidity_target">القيمة المطلوبة للرطوبة (%):</label>
            <input type="number" id="humidity_target" name="humidity_target" value=")=====");
  html += String(humidityTarget, 1);
  html += FPSTR(R"=====(" step="0.1" min="10" max="99" required>
          </div>
          <h3 style="font-size: var(--text-lg); margin: 0.5rem 0">إعدادات التايمر</h3>
          <div class="form-group">
            <label>زمن التشغيل:</label>
            <div class="time-segment">
              <input type="number" id="humidity_on_days" name="humidity_on_days" value=")=====");
  html += String(humidityOnDuration / 86400);
  html += FPSTR(R"=====(" min="0" max="30" placeholder="أيام">
              <span>يوم</span>
              <input type="number" id="humidity_on_hours" name="humidity_on_hours" value=")=====");
  html += String((humidityOnDuration % 86400) / 3600);
  html += FPSTR(R"=====(" min="0" max="23" placeholder="ساعات">
              <span>:</span>
              <input type="number" id="humidity_on_minutes" name="humidity_on_minutes" value=")=====");
  html += String((humidityOnDuration % 3600) / 60);
  html += FPSTR(R"=====(" min="0" max="59" placeholder="دقائق">
              <span>:</span>
              <input type="number" id="humidity_on_seconds" name="humidity_on_seconds" value=")=====");
  html += String(humidityOnDuration % 60);
  html += FPSTR(R"=====(" min="0" max="59" placeholder="ثواني">
            </div>
          </div>
          <div class="form-group">
            <label>زمن الإيقاف:</label>
            <div class="time-segment">
              <input type="number" id="humidity_off_days" name="humidity_off_days" value=")=====");
  html += String(humidityOffDuration / 86400);
  html += FPSTR(R"=====(" min="0" max="30" placeholder="أيام">
              <span>يوم</span>
              <input type="number" id="humidity_off_hours" name="humidity_off_hours" value=")=====");
  html += String((humidityOffDuration % 86400) / 3600);
  html += FPSTR(R"=====(" min="0" max="23" placeholder="ساعات">
              <span>:</span>
              <input type="number" id="humidity_off_minutes" name="humidity_off_minutes" value=")=====");
  html += String((humidityOffDuration % 3600) / 60);
  html += FPSTR(R"=====(" min="0" max="59" placeholder="دقائق">
              <span>:</span>
              <input type="number" id="humidity_off_seconds" name="humidity_off_seconds" value=")=====");
  html += String(humidityOffDuration % 60);
  html += FPSTR(R"=====(" min="0" max="59" placeholder="ثواني">
            </div>
          </div>
          <button type="submit" class="btn btn-ventilation">حفظ إعدادات الرطوبة</button>
        </form>
        <div class="timer-status">
          <p><strong>الحالة الحالية:</strong> )=====");
  if (currentHumidityMode == CM_DISABLED) html += F("معطل");
  else if (currentHumidityMode == CM_MANUAL) html += F("يدوي");
  else html += F("تلقائي");
  html += FPSTR(R"=====(</p>
          <p><strong>حالة رطوبة Relay5:</strong> )=====");
  html += isHumidityOn ? F("قيد التشغيل") : F("متوقف");
  html += FPSTR(R"=====(</p>
          <p><strong>الوقت المتبقي:</strong> )=====");
  html += getHumidityRemainingTime();
  html += FPSTR(R"=====(</p>
          <p><strong>الرطوبة المطلوبة:</strong> )=====");
  html += String(humidityTarget, 1);
  html += FPSTR(R"=====( %</p>
          <p><strong>الرطوبة الحالية:</strong> )=====");
  html += String(humidity, 1);
  html += FPSTR(R"=====( %</p>
        </div>
      </div>
  )=====");

  // ... (rest of your HTML page code)

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}

// --- END PAGE changes ---

void handleControl() {
  if (server.hasArg("relay")) {
    int relayNum = server.arg("relay").toInt();
    if (relayNum >= 0 && relayNum < relayCount) {
      // Only allow manual toggle if auto control is disabled
      if (!relayAllowAuto[relayNum]) {
        digitalWrite(relayPins[relayNum], !digitalRead(relayPins[relayNum]));
      }
    }
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleHeatingTempUpdate() {
  if (server.hasArg("heating_target")) {
    heatingTargetTemp = server.arg("heating_target").toFloat();
    EEPROM.put(HEATING_TARGET_TEMP_ADDR, heatingTargetTemp);
    EEPROM.commit();
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleCoolingTempUpdate() {
  if (server.hasArg("cooling_target")) {
    coolingTargetTemp = server.arg("cooling_target").toFloat();
    EEPROM.put(COOLING_TARGET_TEMP_ADDR, coolingTargetTemp);
    EEPROM.commit();
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleTempDiffUpdate() {
  if (server.hasArg("difference")) {
    tempDifference = server.arg("difference").toFloat();
    EEPROM.put(TEMP_DIFF_ADDR, tempDifference);
    EEPROM.commit();
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleAmmoniaUpdate() {
  if (server.hasArg("threshold")) {
    ammoniaThreshold = server.arg("threshold").toFloat();
    EEPROM.put(AMMONIA_THRESHOLD_ADDR, ammoniaThreshold);
    EEPROM.commit();
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleVentilationTempUpdate() {
  if (server.hasArg("ventilation_temp")) {
    ventilationTempThreshold = server.arg("ventilation_temp").toFloat();
    EEPROM.put(VENTILATION_TEMP_ADDR, ventilationTempThreshold);
    EEPROM.commit();
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleVentilation2TempUpdate() {
  if (server.hasArg("ventilation2_temp")) {
    ventilation2TempThreshold = server.arg("ventilation2_temp").toFloat();
    EEPROM.put(VENTILATION2_TEMP_ADDR, ventilation2TempThreshold);
    EEPROM.commit();
  }
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleVentilation2TimerUpdate() {
  if (server.hasArg("ventilation2_control_mode")) {
    currentVentilation2Mode = static_cast<ControlMode>(server.arg("ventilation2_control_mode").toInt());
    EEPROM.put(CONTROL_MODE_VENTILATION2_ADDR, static_cast<int>(currentVentilation2Mode));
    isVentilation2On = false;
    ventilation2TimerStartTime = 0;
  }
  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;
  if (server.hasArg("ventilation2_on_days") && server.hasArg("ventilation2_on_hours") && server.hasArg("ventilation2_on_minutes") && server.hasArg("ventilation2_on_seconds")) {
    newOnDuration = server.arg("ventilation2_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("ventilation2_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("ventilation2_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("ventilation2_on_seconds").toInt();
    if (newOnDuration > 0) {
      ventilation2OnDuration = newOnDuration;
      EEPROM.put(VENTILATION2_TIMER_ON_ADDR, ventilation2OnDuration);
    }
  }
  if (server.hasArg("ventilation2_off_days") && server.hasArg("ventilation2_off_hours") && server.hasArg("ventilation2_off_minutes") && server.hasArg("ventilation2_off_seconds")) {
    newOffDuration = server.arg("ventilation2_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("ventilation2_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("ventilation2_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("ventilation2_off_seconds").toInt();
    if (newOffDuration > 0) {
      ventilation2OffDuration = newOffDuration;
      EEPROM.put(VENTILATION2_TIMER_OFF_ADDR, ventilation2OffDuration);
    }
  }
  EEPROM.commit();
  isVentilation2On = false;
  ventilation2TimerStartTime = 0;
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

// ------------- NEW: Humidity Timer Update Handler ------------

void handleHumidityTimerUpdate() {
  if (server.hasArg("humidity_control_mode")) {
    currentHumidityMode = static_cast<ControlMode>(server.arg("humidity_control_mode").toInt());
    EEPROM.put(HUMIDITY_CONTROL_MODE_ADDR, static_cast<int>(currentHumidityMode));
    isHumidityOn = false;
    humidityTimerStartTime = 0;
  }
  if (server.hasArg("humidity_target")) {
    humidityTarget = server.arg("humidity_target").toFloat();
    if (humidityTarget < 10.0) humidityTarget = 10.0;
    if (humidityTarget > 99.0) humidityTarget = 99.0;
    EEPROM.put(HUMIDITY_TARGET_ADDR, humidityTarget);
  }
  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;
  if (server.hasArg("humidity_on_days") && server.hasArg("humidity_on_hours") && server.hasArg("humidity_on_minutes") && server.hasArg("humidity_on_seconds")) {
    newOnDuration = server.arg("humidity_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("humidity_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("humidity_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("humidity_on_seconds").toInt();
    if (newOnDuration > 0) {
      humidityOnDuration = newOnDuration;
      EEPROM.put(HUMIDITY_TIMER_ON_ADDR, humidityOnDuration);
    }
  }
  if (server.hasArg("humidity_off_days") && server.hasArg("humidity_off_hours") && server.hasArg("humidity_off_minutes") && server.hasArg("humidity_off_seconds")) {
    newOffDuration = server.arg("humidity_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("humidity_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("humidity_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("humidity_off_seconds").toInt();
    if (newOffDuration > 0) {
      humidityOffDuration = newOffDuration;
      EEPROM.put(HUMIDITY_TIMER_OFF_ADDR, humidityOffDuration);
    }
  }
  EEPROM.commit();
  isHumidityOn = false;
  humidityTimerStartTime = 0;
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

// ----------- END Humidity Handler ---------------

void handleHeatingTimerUpdate() {
  if (server.hasArg("heating_control_mode")) {
    currentHeatingMode = static_cast<ControlMode>(server.arg("heating_control_mode").toInt());
    EEPROM.put(CONTROL_MODE_HEATING_ADDR, static_cast<int>(currentHeatingMode));
    isHeatingOn = false;
    heatingTimerStartTime = 0;
  }

  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;

  if (server.hasArg("heating_on_days") && server.hasArg("heating_on_hours") && server.hasArg("heating_on_minutes") && server.hasArg("heating_on_seconds")) {
    newOnDuration = server.arg("heating_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("heating_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("heating_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("heating_on_seconds").toInt();
    if (newOnDuration > 0) {
      heatingOnDuration = newOnDuration;
      EEPROM.put(HEATING_TIMER_ON_ADDR, heatingOnDuration);
    }
  }

  if (server.hasArg("heating_off_days") && server.hasArg("heating_off_hours") && server.hasArg("heating_off_minutes") && server.hasArg("heating_off_seconds")) {
    newOffDuration = server.arg("heating_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("heating_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("heating_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("heating_off_seconds").toInt();
    if (newOffDuration > 0) {
      heatingOffDuration = newOffDuration;
      EEPROM.put(HEATING_TIMER_OFF_ADDR, heatingOffDuration);
    }
  }

  EEPROM.commit();
  isHeatingOn = false;
  heatingTimerStartTime = 0;

  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleCoolingTimerUpdate() {
  if (server.hasArg("cooling_control_mode")) {
    currentCoolingMode = static_cast<ControlMode>(server.arg("cooling_control_mode").toInt());
    EEPROM.put(CONTROL_MODE_COOLING_ADDR, static_cast<int>(currentCoolingMode));
    isCoolingOn = false;
    coolingTimerStartTime = 0;
  }

  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;

  if (server.hasArg("cooling_on_days") && server.hasArg("cooling_on_hours") && server.hasArg("cooling_on_minutes") && server.hasArg("cooling_on_seconds")) {
    newOnDuration = server.arg("cooling_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("cooling_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("cooling_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("cooling_on_seconds").toInt();
    if (newOnDuration > 0) {
      coolingOnDuration = newOnDuration;
      EEPROM.put(COOLING_TIMER_ON_ADDR, coolingOnDuration);
    }
  }

  if (server.hasArg("cooling_off_days") && server.hasArg("cooling_off_hours") && server.hasArg("cooling_off_minutes") && server.hasArg("cooling_off_seconds")) {
    newOffDuration = server.arg("cooling_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("cooling_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("cooling_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("cooling_off_seconds").toInt();
    if (newOffDuration > 0) {
      coolingOffDuration = newOffDuration;
      EEPROM.put(COOLING_TIMER_OFF_ADDR, coolingOffDuration);
    }
  }

  EEPROM.commit();
  isCoolingOn = false;
  coolingTimerStartTime = 0;

  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}

void handleVentilationTimerUpdate() {
  if (server.hasArg("ventilation_control_mode")) {
    currentVentilationMode = static_cast<ControlMode>(server.arg("ventilation_control_mode").toInt());
    EEPROM.put(CONTROL_MODE_VENTILATION_ADDR, static_cast<int>(currentVentilationMode));
    isVentilationOn = false;
    ventilationTimerStartTime = 0;
  }

  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;

  if (server.hasArg("ventilation_on_days") && server.hasArg("ventilation_on_hours") && server.hasArg("ventilation_on_minutes") && server.hasArg("ventilation_on_seconds")) {
    newOnDuration = server.arg("ventilation_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("ventilation_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("ventilation_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("ventilation_on_seconds").toInt();
    if (newOnDuration > 0) {
      ventilationOnDuration = newOnDuration;
      EEPROM.put(VENTILATION_TIMER_ON_ADDR, ventilationOnDuration);
    }
  }

  if (server.hasArg("ventilation_off_days") && server.hasArg("ventilation_off_hours") && server.hasArg("ventilation_off_minutes") && server.hasArg("ventilation_off_seconds")) {
    newOffDuration = server.arg("ventilation_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("ventilation_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("ventilation_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("ventilation_off_seconds").toInt();
    if (newOffDuration > 0) {
      ventilationOffDuration = newOffDuration;
      EEPROM.put(VENTILATION_TIMER_OFF_ADDR, ventilationOffDuration);
    }
  }

  EEPROM.commit();
  isVentilationOn = false;
  ventilationTimerStartTime = 0;

  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), "");
}