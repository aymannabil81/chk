#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <LiquidCrystal_I2C.h>

// إعدادات LCD
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// تعريفات الريلاي
const int relayPins[] = { 23, 32, 33, 19, 18, 17, 16, 27 };
#define HEATING_RELAY 0
#define COOLING_RELAY 1
#define VENTILATION_RELAY 2
#define VENTILATION2_RELAY 3
#define HUMIDITY_RELAY 4
#define AIR_CIRCULATION_RELAY 5
#define LIGHTING_RELAY 6
const int relayCount = 8;

// تعريفات الحساسات
#define DS18B20_PIN 4
#define DHT_PIN 5
#define NTC_PIN 34
#define MQ135_PIN 35
#define DHT_TYPE DHT22
const int R1 = 10000;
const float Vcc = 5.0;

// تعريفات LittleFS
#define HEATING_TARGET_TEMP_FILE "/heating_target_temp.txt"
#define COOLING_TARGET_TEMP_FILE "/cooling_target_temp.txt"
#define TEMP_DIFF_FILE "/temp_diff.txt"
#define AMMONIA_THRESHOLD_FILE "/ammonia_threshold.txt"
#define VENTILATION_TEMP_FILE "/ventilation_temp.txt"
#define CONTROL_MODE_HEATING_FILE "/control_mode_heating.txt"
#define CONTROL_MODE_COOLING_FILE "/control_mode_cooling.txt"
#define CONTROL_MODE_VENTILATION_FILE "/control_mode_ventilation.txt"
#define HEATING_TIMER_ON_FILE "/heating_timer_on.txt"
#define HEATING_TIMER_OFF_FILE "/heating_timer_off.txt"
#define COOLING_TIMER_ON_FILE "/cooling_timer_on.txt"
#define COOLING_TIMER_OFF_FILE "/cooling_timer_off.txt"
#define VENTILATION_TIMER_ON_FILE "/ventilation_timer_on.txt"
#define VENTILATION_TIMER_OFF_FILE "/ventilation_timer_off.txt"
#define MQ135_RZERO_FILE "/mq135_rzero.txt"
#define MQ135_R0_FILE "/mq135_r0.txt"
#define VENTILATION2_TEMP_FILE "/ventilation2_temp.txt"
#define VENTILATION2_TIMER_ON_FILE "/ventilation2_timer_on.txt"
#define VENTILATION2_TIMER_OFF_FILE "/ventilation2_timer_off.txt"
#define CONTROL_MODE_VENTILATION2_FILE "/control_mode_ventilation2.txt"
#define HUMIDITY_THRESHOLD_FILE "/humidity_threshold.txt"
#define HUMIDITY_TIMER_ON_FILE "/humidity_timer_on.txt"
#define HUMIDITY_TIMER_OFF_FILE "/humidity_timer_off.txt"
#define CONTROL_MODE_HUMIDITY_FILE "/control_mode_humidity.txt"
#define VENTILATION_AMMONIA_ENABLED_FILE "/ventilation_ammonia_enabled.txt"
#define VENTILATION2_AMMONIA_ENABLED_FILE "/ventilation2_ammonia_enabled.txt"
#define AIR_CIRCULATION_TIMER_ON_FILE "/air_circulation_timer_on.txt"
#define AIR_CIRCULATION_TIMER_OFF_FILE "/air_circulation_timer_off.txt"
#define CONTROL_MODE_AIR_CIRCULATION_FILE "/control_mode_air_circulation.txt"
#define LIGHTING_TIMER_ON_FILE "/lighting_timer_on.txt"
#define LIGHTING_TIMER_OFF_FILE "/lighting_timer_off.txt"
#define CONTROL_MODE_LIGHTING_FILE "/control_mode_lighting.txt"

// تعريفات MQ135
#define ATMOSPHERIC_CO2 410
#define RL 10.0
#define RZERO_CALIBRATION_SAMPLE_TIMES 50
#define RZERO_CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_INTERVAL 50
#define READ_SAMPLE_TIMES 5

// تعريفات DDNS
#define DYNU_UPDATE_INTERVAL 300000
const char* dynu_url = "https://api.dynu.com/nic/update?hostname=aymanfarm.kozow.com&password=aX.:DQVgeVqv2A9";

const char* firmwareUrl = "http://example.com/firmware.bin";  // لينك السكيتش الجديد


// تعريفات WiFi
const int MAX_NETWORKS = 3;
struct WiFiNetwork {
  const char* ssid;
  const char* password;
};

WiFiNetwork networks[MAX_NETWORKS] = {
  { "NAVK", "NEwdays<127912>" },
  { "CHK", "Anbawanas127982645127912" }
};

IPAddress staticIP(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);
bool useStaticIP = false;

// متغيرات النظام
enum ControlMode {
  CM_DISABLED,
  CM_MANUAL,
  CM_TEMPERATURE,
  CM_HUMIDITY
};

ControlMode currentHeatingMode = CM_DISABLED;
ControlMode currentCoolingMode = CM_DISABLED;
ControlMode currentVentilationMode = CM_DISABLED;
ControlMode currentVentilation2Mode = CM_DISABLED;
ControlMode currentHumidityMode = CM_DISABLED;
ControlMode currentAirCirculationMode = CM_DISABLED;
ControlMode currentLightingMode = CM_DISABLED;

bool relayAllowAuto[relayCount] = { 1, 1, 1, 1, 1, 1, 1, 1 };

float ammoniaThreshold = 50.0;
float ammoniaLevel = 0.0;
bool ammoniaAlert = false;
bool ventilationAmmoniaEnabled = true;
bool ventilation2AmmoniaEnabled = true;
float ventilationTempThreshold = 30.0;
float ventilation2TempThreshold = 32.0;
float humidityThreshold = 70.0;
float heatingTargetTemp = 25.0;
float coolingTargetTemp = 22.0;
float tempDifference = 2.0;
float currentTemp = 0;
float humidity = 0;

// متغيرات التايمر
unsigned long lastDynuUpdate = 0;
unsigned long heatingOnDuration = 1800;
unsigned long heatingOffDuration = 1800;
unsigned long heatingTimerStartTime = 0;
bool isHeatingOn = true;
unsigned long coolingOnDuration = 1800;
unsigned long coolingOffDuration = 1800;
unsigned long coolingTimerStartTime = 0;
bool isCoolingOn = true;
unsigned long ventilationOnDuration = 1800;
unsigned long ventilationOffDuration = 1800;
unsigned long ventilationTimerStartTime = 0;
bool isVentilationOn = true;
unsigned long ventilation2OnDuration = 1800;
unsigned long ventilation2OffDuration = 1800;
unsigned long ventilation2TimerStartTime = 0;
bool isVentilation2On = true;
unsigned long humidityOnDuration = 1800;
unsigned long humidityOffDuration = 1800;
unsigned long humidityTimerStartTime = 0;
bool isHumidityOn = true;
unsigned long airCirculationOnDuration = 1800;
unsigned long airCirculationOffDuration = 1800;
unsigned long airCirculationTimerStartTime = 0;
bool isAirCirculationOn = true;
unsigned long lightingOnDuration = 1800;
unsigned long lightingOffDuration = 1800;
unsigned long lightingTimerStartTime = 0;
bool isLightingOn = true;

// كائنات الحساسات
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);

// متغيرات MQ135
float MQ135_RZERO = 76.63;
float MQ135_R0 = 10.0;

// ========== دوال إدارة EEPROM الآمنة ==========

bool writeFloatToFile(const char* filename, float value) {
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للكتابة: "));
    Serial.println(filename);
    return false;
  }
  file.print(value, 6);
  file.close();
  return true;
}

bool readFloatFromFile(const char* filename, float& value) {
  if (!LittleFS.exists(filename)) {
    return false;
  }

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للقراءة: "));
    Serial.println(filename);
    return false;
  }

  String content = file.readString();
  file.close();

  value = content.toFloat();
  return true;
}

bool writeUlongToFile(const char* filename, unsigned long value) {
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للكتابة: "));
    Serial.println(filename);
    return false;
  }
  file.print(value);
  file.close();
  return true;
}

bool readUlongFromFile(const char* filename, unsigned long& value) {
  if (!LittleFS.exists(filename)) {
    return false;
  }

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للقراءة: "));
    Serial.println(filename);
    return false;
  }

  String content = file.readString();
  file.close();

  value = content.toInt();
  return true;
}

bool writeIntToFile(const char* filename, int value) {
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للكتابة: "));
    Serial.println(filename);
    return false;
  }
  file.print(value);
  file.close();
  return true;
}

bool readIntFromFile(const char* filename, int& value) {
  if (!LittleFS.exists(filename)) {
    return false;
  }

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للقراءة: "));
    Serial.println(filename);
    return false;
  }

  String content = file.readString();
  file.close();

  value = content.toInt();
  return true;
}

bool writeBoolToFile(const char* filename, bool value) {
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للكتابة: "));
    Serial.println(filename);
    return false;
  }
  file.print(value ? "1" : "0");
  file.close();
  return true;
}

bool readBoolFromFile(const char* filename, bool& value) {
  if (!LittleFS.exists(filename)) {
    return false;
  }

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.print(F("خطأ في فتح الملف للقراءة: "));
    Serial.println(filename);
    return false;
  }

  String content = file.readString();
  file.close();

  value = (content == "1");
  return true;
}

void checkLittleFSHealth() {
  Serial.println(F("=== فحص نظام الملفات LittleFS ==="));

  Serial.print(F("المساحة الكلية: "));
  Serial.print(LittleFS.totalBytes());
  Serial.println(F(" بايت"));

  Serial.print(F("المساحة المستخدمة: "));
  Serial.print(LittleFS.usedBytes());
  Serial.println(F(" بايت"));

  Serial.println(F("=== قائمة الملفات ==="));
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print(file.name());
    Serial.print(" - ");
    Serial.print(file.size());
    Serial.println(" بايت");
    file = root.openNextFile();
  }
}

void loadSettingsFromLittleFS() {
  // تحميل درجات الحرارة والعتبات
  readFloatFromFile(HEATING_TARGET_TEMP_FILE, heatingTargetTemp);
  readFloatFromFile(COOLING_TARGET_TEMP_FILE, coolingTargetTemp);
  readFloatFromFile(TEMP_DIFF_FILE, tempDifference);
  readFloatFromFile(AMMONIA_THRESHOLD_FILE, ammoniaThreshold);
  readFloatFromFile(VENTILATION_TEMP_FILE, ventilationTempThreshold);
  readFloatFromFile(VENTILATION2_TEMP_FILE, ventilation2TempThreshold);
  readFloatFromFile(HUMIDITY_THRESHOLD_FILE, humidityThreshold);
  readFloatFromFile(MQ135_RZERO_FILE, MQ135_RZERO);
  readFloatFromFile(MQ135_R0_FILE, MQ135_R0);
  readBoolFromFile(VENTILATION_AMMONIA_ENABLED_FILE, ventilationAmmoniaEnabled);
  readBoolFromFile(VENTILATION2_AMMONIA_ENABLED_FILE, ventilation2AmmoniaEnabled);

  // تحميل أنماط التحكم
  int heatingMode, coolingMode, ventilationMode, ventilation2Mode, humidityMode, airCirculationMode, lightingMode;
  readIntFromFile(CONTROL_MODE_HEATING_FILE, heatingMode);
  readIntFromFile(CONTROL_MODE_COOLING_FILE, coolingMode);
  readIntFromFile(CONTROL_MODE_VENTILATION_FILE, ventilationMode);
  readIntFromFile(CONTROL_MODE_VENTILATION2_FILE, ventilation2Mode);
  readIntFromFile(CONTROL_MODE_HUMIDITY_FILE, humidityMode);
  readIntFromFile(CONTROL_MODE_AIR_CIRCULATION_FILE, airCirculationMode);
  readIntFromFile(CONTROL_MODE_LIGHTING_FILE, lightingMode);

  currentHeatingMode = static_cast<ControlMode>(heatingMode);
  currentCoolingMode = static_cast<ControlMode>(coolingMode);
  currentVentilationMode = static_cast<ControlMode>(ventilationMode);
  currentVentilation2Mode = static_cast<ControlMode>(ventilation2Mode);
  currentHumidityMode = static_cast<ControlMode>(humidityMode);
  currentAirCirculationMode = static_cast<ControlMode>(airCirculationMode);
  currentLightingMode = static_cast<ControlMode>(lightingMode);

  // تحميل إعدادات التايمر
  readUlongFromFile(HEATING_TIMER_ON_FILE, heatingOnDuration);
  readUlongFromFile(HEATING_TIMER_OFF_FILE, heatingOffDuration);
  readUlongFromFile(COOLING_TIMER_ON_FILE, coolingOnDuration);
  readUlongFromFile(COOLING_TIMER_OFF_FILE, coolingOffDuration);
  readUlongFromFile(VENTILATION_TIMER_ON_FILE, ventilationOnDuration);
  readUlongFromFile(VENTILATION_TIMER_OFF_FILE, ventilationOffDuration);
  readUlongFromFile(VENTILATION2_TIMER_ON_FILE, ventilation2OnDuration);
  readUlongFromFile(VENTILATION2_TIMER_OFF_FILE, ventilation2OffDuration);
  readUlongFromFile(HUMIDITY_TIMER_ON_FILE, humidityOnDuration);
  readUlongFromFile(HUMIDITY_TIMER_OFF_FILE, humidityOffDuration);
  readUlongFromFile(AIR_CIRCULATION_TIMER_ON_FILE, airCirculationOnDuration);
  readUlongFromFile(AIR_CIRCULATION_TIMER_OFF_FILE, airCirculationOffDuration);
  readUlongFromFile(LIGHTING_TIMER_ON_FILE, lightingOnDuration);
  readUlongFromFile(LIGHTING_TIMER_OFF_FILE, lightingOffDuration);

  // تعيين قيم افتراضية إذا كانت غير صالحة
  if (isnan(heatingTargetTemp)) heatingTargetTemp = 25.0;
  if (isnan(coolingTargetTemp)) coolingTargetTemp = 22.0;
  if (isnan(tempDifference)) tempDifference = 2.0;
  if (isnan(ammoniaThreshold)) ammoniaThreshold = 50.0;
  if (isnan(ventilationTempThreshold)) ventilationTempThreshold = 30.0;
  if (isnan(ventilation2TempThreshold)) ventilation2TempThreshold = 32.0;
  if (isnan(humidityThreshold)) humidityThreshold = 70.0;
  if (isnan(heatingMode)) currentHeatingMode = CM_DISABLED;
  if (isnan(coolingMode)) currentCoolingMode = CM_DISABLED;
  if (isnan(ventilationMode)) currentVentilationMode = CM_DISABLED;
  if (isnan(ventilation2Mode)) currentVentilation2Mode = CM_DISABLED;
  if (isnan(humidityMode)) currentHumidityMode = CM_DISABLED;
  if (isnan(airCirculationMode)) currentAirCirculationMode = CM_DISABLED;
  if (isnan(lightingMode)) currentLightingMode = CM_DISABLED;
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
  if (isnan(airCirculationOnDuration)) airCirculationOnDuration = 1800;
  if (isnan(airCirculationOffDuration)) airCirculationOffDuration = 1800;
  if (isnan(lightingOnDuration)) lightingOnDuration = 1800;
  if (isnan(lightingOffDuration)) lightingOffDuration = 1800;
  if (isnan(MQ135_RZERO)) {
    MQ135_RZERO = 76.63;
    calibrateMQ135();
  }
  if (isnan(MQ135_R0)) MQ135_R0 = 10.0;
}


void resetAllTimers() {
  resetHeatingTimer();
  resetCoolingTimer();
  resetVentilationTimer();
  resetVentilation2Timer();
  resetHumidityTimer();
  resetAirCirculationTimer();
  resetLightingTimer();
}

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

void resetHumidityTimer() {
  isHumidityOn = true;
  humidityTimerStartTime = millis();
  if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
}

void resetAirCirculationTimer() {
  isAirCirculationOn = true;
  airCirculationTimerStartTime = millis();
  if (relayAllowAuto[AIR_CIRCULATION_RELAY]) digitalWrite(relayPins[AIR_CIRCULATION_RELAY], LOW);
}

void resetLightingTimer() {
  isLightingOn = true;
  lightingTimerStartTime = millis();
  if (relayAllowAuto[LIGHTING_RELAY]) digitalWrite(relayPins[LIGHTING_RELAY], LOW);
}
// ========== دوال إدارة WiFi ==========

void connectToWiFi() {
  Serial.println(F("محاولة الاتصال بشبكة WiFi..."));

  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (networks[i].ssid == NULL || strlen(networks[i].ssid) == 0) {
      continue;
    }

    Serial.print(F("جاري الاتصال بشبكة: "));
    Serial.println(networks[i].ssid);

    if (useStaticIP) {
      byte gw0 = gateway[0], gw1 = gateway[1], gw2 = gateway[2];
      byte st0 = staticIP[0], st1 = staticIP[1], st2 = staticIP[2];

      if (gw0 == st0 && gw1 == st1 && gw2 == st2) {
        IPAddress fixedIP(gw0, gw1, gw2, 200);
        WiFi.config(fixedIP, gateway, subnet, dns);
      } else {
        Serial.println(F("تحذير: الـ gateway خارج نطاق الـ static IP! سيتم استخدام DHCP بدلًا من static IP."));
        WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
      }
    } else {
      WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    }

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

      IPAddress gatewayIP = WiFi.gatewayIP();
      IPAddress localIP = WiFi.localIP();
      if (gatewayIP[0] == localIP[0] && gatewayIP[1] == localIP[1] && gatewayIP[2] == localIP[2]) {
        IPAddress fixedIP(gatewayIP[0], gatewayIP[1], gatewayIP[2], 200);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.config(fixedIP, gatewayIP, subnet, IPAddress(8, 8, 8, 8), IPAddress(8, 8, 4, 4));
        Serial.print("Static IP set to: ");
        Serial.println(fixedIP);
        updateDynuDNS();
      }
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
  if (millis() - lastCheck > 30000) {
    if (WiFi.status() != WL_CONNECTED) {
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

// ========== دوال الحساسات ==========

void readTemperature() {
  float tempDHT = dht.readTemperature();
  humidity = dht.readHumidity();

  if (!isnan(tempDHT)) {
    currentTemp = tempDHT;
    return;
  }

  ds18b20.requestTemperatures();
  float tempDS18 = ds18b20.getTempCByIndex(0);
  if (!isnan(tempDS18)) {
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
  float tempK = 1 / (1 / 298.15 + log(Rntc / 10000) / 3950.0);
  return tempK - 273.15;
}

void checkAmmoniaLevel() {
  ammoniaLevel = readAmmoniaLevel();
  ammoniaAlert = (ammoniaLevel > ammoniaThreshold);
}

float getMQ135Resistance(int rawADC) {
  return ((1023. / (float)rawADC) * 5. - 1.) * RL;
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

void calibrateMQ135() {
  Serial.println(F("جارٍ معايرة حساس MQ135..."));

  float rs = 0;
  for (int i = 0; i < RZERO_CALIBRATION_SAMPLE_TIMES; i++) {
    rs += getMQ135Resistance(analogRead(MQ135_PIN));
    delay(RZERO_CALIBRATION_SAMPLE_INTERVAL);
  }
  rs = rs / RZERO_CALIBRATION_SAMPLE_TIMES;

  MQ135_RZERO = rs * pow((ATMOSPHERIC_CO2 / 398.107), 1 / -2.612);
  MQ135_R0 = rs / 9.83;

  writeFloatToFile(MQ135_RZERO_FILE, MQ135_RZERO);
  writeFloatToFile(MQ135_R0_FILE, MQ135_R0);

  Serial.print(F("تمت المعايرة. RZero الجديدة: "));
  Serial.println(MQ135_RZERO);
  Serial.print(F("R0 الجديدة: "));
  Serial.println(MQ135_R0);
}

// ========== دوال التحكم في الريلاي ==========

void handleHeatingTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - heatingTimerStartTime) / 1000;

  if (currentHeatingMode == CM_DISABLED) {
    if (currentTemp <= (heatingTargetTemp - tempDifference / 2)) {
      if (relayAllowAuto[HEATING_RELAY]) digitalWrite(relayPins[HEATING_RELAY], LOW);
      isHeatingOn = true;
      heatingTimerStartTime = currentTime;
    } else if (currentTemp >= (heatingTargetTemp + tempDifference / 2)) {
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
    if (currentTemp <= (heatingTargetTemp - tempDifference / 2)) {
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
    if (currentTemp >= (coolingTargetTemp + tempDifference / 2)) {
      if (relayAllowAuto[COOLING_RELAY]) digitalWrite(relayPins[COOLING_RELAY], LOW);
      isCoolingOn = true;
      coolingTimerStartTime = currentTime;
    } else if (currentTemp <= (coolingTargetTemp - tempDifference / 2)) {
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
    if (currentTemp >= (coolingTargetTemp + tempDifference / 2)) {
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
    if (currentTemp >= ventilationTempThreshold || (ventilationAmmoniaEnabled && ammoniaLevel > ammoniaThreshold)) {
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
    if (currentTemp >= ventilationTempThreshold || (ventilationAmmoniaEnabled && ammoniaLevel > ammoniaThreshold)) {
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

  if (currentVentilation2Mode == CM_DISABLED) {
    if (currentTemp >= ventilation2TempThreshold || (ventilation2AmmoniaEnabled && ammoniaLevel > ammoniaThreshold)) {
      if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], LOW);
      isVentilation2On = true;
      ventilation2TimerStartTime = currentTime;
    } else {
      if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
      isVentilation2On = false;
      ventilation2TimerStartTime = currentTime;
    }
  } else if (currentVentilation2Mode == CM_MANUAL) {
    if (isVentilation2On && elapsedTime >= ventilation2OnDuration) {
      if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
      isVentilation2On = false;
      ventilation2TimerStartTime = currentTime;
    } else if (!isVentilation2On && elapsedTime >= ventilation2OffDuration) {
      if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], LOW);
      isVentilation2On = true;
      ventilation2TimerStartTime = currentTime;
    }
  } else if (currentVentilation2Mode == CM_TEMPERATURE) {
    if (currentTemp >= ventilation2TempThreshold || (ventilation2AmmoniaEnabled && ammoniaLevel > ammoniaThreshold)) {
      if (isVentilation2On && elapsedTime >= ventilation2OnDuration) {
        if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
        isVentilation2On = false;
        ventilation2TimerStartTime = currentTime;
      } else if (!isVentilation2On && elapsedTime >= ventilation2OffDuration) {
        if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], LOW);
        isVentilation2On = true;
        ventilation2TimerStartTime = currentTime;
      }
    } else {
      if (relayAllowAuto[VENTILATION2_RELAY]) digitalWrite(relayPins[VENTILATION2_RELAY], HIGH);
      isVentilation2On = false;
      ventilation2TimerStartTime = currentTime;
    }
  }
}

void handleHumidityTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - humidityTimerStartTime) / 1000;

  if (currentHumidityMode == CM_DISABLED) {
    if (humidity <= humidityThreshold) {
      if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
      isHumidityOn = true;
      humidityTimerStartTime = currentTime;
    } else {
      if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
      isHumidityOn = false;
      humidityTimerStartTime = currentTime;
    }
  } else if (currentHumidityMode == CM_MANUAL) {
    if (isHumidityOn && elapsedTime >= humidityOnDuration) {
      if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
      isHumidityOn = false;
      humidityTimerStartTime = currentTime;
    } else if (!isHumidityOn && elapsedTime >= humidityOffDuration) {
      if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
      isHumidityOn = true;
      humidityTimerStartTime = currentTime;
    }
  } else if (currentHumidityMode == CM_HUMIDITY) {
    if (humidity <= humidityThreshold) {
      if (isHumidityOn && elapsedTime >= humidityOnDuration) {
        if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
        isHumidityOn = false;
        humidityTimerStartTime = currentTime;
      } else if (!isHumidityOn && elapsedTime >= humidityOffDuration) {
        if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], LOW);
        isHumidityOn = true;
        humidityTimerStartTime = currentTime;
      }
    } else {
      if (relayAllowAuto[HUMIDITY_RELAY]) digitalWrite(relayPins[HUMIDITY_RELAY], HIGH);
      isHumidityOn = false;
      humidityTimerStartTime = currentTime;
    }
  }
}

void handleAirCirculationTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - airCirculationTimerStartTime) / 1000;

  if (currentAirCirculationMode == CM_MANUAL) {
    if (isAirCirculationOn && elapsedTime >= airCirculationOnDuration) {
      if (relayAllowAuto[AIR_CIRCULATION_RELAY]) digitalWrite(relayPins[AIR_CIRCULATION_RELAY], HIGH);
      isAirCirculationOn = false;
      airCirculationTimerStartTime = currentTime;
    } else if (!isAirCirculationOn && elapsedTime >= airCirculationOffDuration) {
      if (relayAllowAuto[AIR_CIRCULATION_RELAY]) digitalWrite(relayPins[AIR_CIRCULATION_RELAY], LOW);
      isAirCirculationOn = true;
      airCirculationTimerStartTime = currentTime;
    }
  }
}

void handleLightingTimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - lightingTimerStartTime) / 1000;

  if (currentLightingMode == CM_MANUAL) {
    if (isLightingOn && elapsedTime >= lightingOnDuration) {
      if (relayAllowAuto[LIGHTING_RELAY]) digitalWrite(relayPins[LIGHTING_RELAY], HIGH);
      isLightingOn = false;
      lightingTimerStartTime = currentTime;
    } else if (!isLightingOn && elapsedTime >= lightingOffDuration) {
      if (relayAllowAuto[LIGHTING_RELAY]) digitalWrite(relayPins[LIGHTING_RELAY], LOW);
      isLightingOn = true;
      lightingTimerStartTime = currentTime;
    }
  }
}

// ========== دوال عرض المعلومات ==========

void showLCDMainInfo() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C H:");
  lcd.print(humidity, 0);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("NH3:");
  lcd.print((int)ammoniaLevel);
  lcd.print("ppm ");

  // عرض حالة الريلايات
  char states[8];
  for (int i = 0; i < 7; i++) {
    states[i] = (digitalRead(relayPins[i]) == LOW) ? '*' : '-';
  }
  states[7] = 0;
  lcd.print(states);
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
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lu يوم %02lu:%02lu:%02lu", days, hours, minutes, secondsRemaining);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, secondsRemaining);
  }
  return String(buf) + F(" منذ التشغيل");
}

void checkMemoryUsage() {
  Serial.print(F("الذاكرة الحرة: "));
  Serial.print(ESP.getFreeHeap());
  Serial.print(F(" / "));
  Serial.print(ESP.getHeapSize());
  Serial.println(F(" بايت"));

  Serial.print(F("مساحة الفلاش الحرة: "));
  Serial.print(ESP.getFreeSketchSpace());
  Serial.print(F(" / "));
  Serial.print(ESP.getFlashChipSize());
  Serial.println(F(" بايت"));
}



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

void handleCalibrateMQ135() {
  calibrateMQ135();
  server.sendHeader(F("Location"), F("/"));
  server.send(302, F("text/plain"), F("تمت معايرة الحساس بنجاح"));
}

void handleRelayAutoUpdate() {
  // التحقق من صلاحية المستخدم أولاً
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  // تحديث حالة الريلايات من بيانات النموذج
  for (int i = 0; i < relayCount; i++) {
    String allowName = "allow_auto_" + String(i);
    relayAllowAuto[i] = server.hasArg(allowName) ? 1 : 0;

    // حفظ كل إعداد في ملف منفصل في LittleFS
    String filename = "/relay_auto_" + String(i) + ".txt";
    writeBoolToFile(filename.c_str(), relayAllowAuto[i]);
  }

  // إعادة توجيه إلى صفحة التحكم مع رسالة نجاح
  server.sendHeader(F("Location"), F("/relay_control?success=1"));
  server.send(302, F("text/plain"), "تم تحديث الإعدادات بنجاح");
}

void handleRelayControlPage() {
  // التحقق من صلاحية المستخدم أولاً
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  // تحميل إعدادات التحكم الآلي من LittleFS
  for (int i = 0; i < relayCount; i++) {
    String filename = "/relay_auto_" + String(i) + ".txt";
    bool autoMode;
    if (readBoolFromFile(filename.c_str(), autoMode)) {
      relayAllowAuto[i] = autoMode;
    }
  }

  // إرسال صفحة التحكم
  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), getRelayControlPage());
}

void setupOTA() {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("ChickenFarmController");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
      LittleFS.end();
    }

    // Turn off all relays before OTA update
    for (int i = 0; i < relayCount; i++) {
      digitalWrite(relayPins[i], HIGH);
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OTA Update Start");
    lcd.setCursor(0, 1);
    lcd.print("Type: " + type);

    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OTA Update Done");
    lcd.setCursor(0, 1);
    lcd.print("Restarting...");

    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percentage = (progress / (total / 100));

    lcd.setCursor(0, 1);
    lcd.print("Progress: " + String(percentage) + "%");

    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OTA Error");
    lcd.setCursor(0, 1);

    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      lcd.print("Auth Failed");
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      lcd.print("Begin Failed");
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      lcd.print("Connect Failed");
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      lcd.print("Receive Failed");
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      lcd.print("End Failed");
      Serial.println("End Failed");
    }

    delay(3000);
    ESP.restart();
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void performOTA() {
  HTTPClient http;
  http.begin(firmwareUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    WiFiClient * stream = http.getStreamPtr();

    if (Update.begin(contentLength)) {
      size_t written = Update.writeStream(*stream);
      if (written == contentLength && Update.end() && Update.isFinished()) {
        Serial.println("Update complete. Rebooting...");
        ESP.restart();
      } else {
        Serial.println("Update failed.");
      }
    }
  } else {
    Serial.printf("HTTP GET failed, code: %d\n", httpCode);
  }

  http.end();
}
// ========== الإعدادات الرئيسية ==========

void setup() {
  Serial.begin(115200);

  // تهيئة LCD
  lcd.init();
  lcd.backlight();

  // تهيئة LittleFS
  if (!LittleFS.begin()) {
    Serial.println(F("خطأ في تهيئة نظام الملفات LittleFS"));
    Serial.println(F("جارٍ تهيئة نظام الملفات..."));
    LittleFS.format();
    if (!LittleFS.begin()) {
      Serial.println(F("فشل تهيئة نظام الملفات!"));
    } else {
      Serial.println(F("تم تهيئة نظام الملفات بنجاح"));
    }
  }

  checkLittleFSHealth();

  // تحميل الإعدادات من LittleFS
  loadSettingsFromLittleFS();

  // تهيئة أطراف الريلاي
  for (int i = 0; i < relayCount; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);
  }

  // بدء الحساسات
  ds18b20.begin();
  dht.begin();

  // الاتصال بشبكة WiFi
  connectToWiFi();

  // إعداد OTA
  setupOTA();

  // إعداد مسارات السيرفر
  setupServerRoutes();

  // بدء تشغيل التايمرات
  resetAllTimers();

  Serial.println(F("تم تهيئة النظام بنجاح"));
}

void loop() {
  static unsigned long lastLcdUpdate = 0;
  if (millis() - lastLcdUpdate > 2000) {
    showLCDMainInfo();
    lastLcdUpdate = millis();
  }

  ArduinoOTA.handle();  // Handle OTA updates
  server.handleClient();
  checkWiFiConnection();

  if (millis() - lastDynuUpdate > DYNU_UPDATE_INTERVAL) {
    updateDynuDNS();
    lastDynuUpdate = millis();
  }

  static unsigned long lastTempRead = 0;
  if (millis() - lastTempRead > 2000) {
    readTemperature();
    handleHeatingTimer();
    handleCoolingTimer();
    handleVentilationTimer();
    handleVentilation2Timer();
    handleHumidityTimer();
    handleAirCirculationTimer();
    handleLightingTimer();
    checkAmmoniaLevel();
    lastTempRead = millis();
  }

  static unsigned long lastMemoryCheck = 0;
  if (millis() - lastMemoryCheck > 60000) {
    checkMemoryUsage();
    lastMemoryCheck = millis();
  }

  delay(2);
}
void setupServerRoutes() {
  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/heating", handleHeatingPage);
  server.on("/cooling", handleCoolingPage);
  server.on("/ventilation", handleVentilationPage);
  server.on("/ventilation2", handleVentilation2Page);
  server.on("/humidity", handleHumidityPage);
  server.on("/air_circulation", handleAirCirculationPage);
  server.on("/lighting", handleLightingPage);
  server.on("/update_heating_temp", HTTP_POST, handleHeatingTempUpdate);
  server.on("/update_cooling_temp", HTTP_POST, handleCoolingTempUpdate);
  server.on("/update_temp_diff", HTTP_POST, handleTempDiffUpdate);
  server.on("/update_ammonia", HTTP_POST, handleAmmoniaUpdate);
  server.on("/update_ventilation_temp", HTTP_POST, handleVentilationTempUpdate);
  server.on("/update_ventilation2_temp", HTTP_POST, handleVentilation2TempUpdate);
  server.on("/update_humidity_threshold", HTTP_POST, handleHumidityThresholdUpdate);
  server.on("/update_heating_timer", HTTP_POST, handleHeatingTimerUpdate);
  server.on("/update_cooling_timer", HTTP_POST, handleCoolingTimerUpdate);
  server.on("/update_ventilation_timer", HTTP_POST, handleVentilationTimerUpdate);
  server.on("/update_ventilation2_timer", HTTP_POST, handleVentilation2TimerUpdate);
  server.on("/update_humidity_timer", HTTP_POST, handleHumidityTimerUpdate);
  server.on("/update_air_circulation_timer", HTTP_POST, handleAirCirculationTimerUpdate);
  server.on("/update_lighting_timer", HTTP_POST, handleLightingTimerUpdate);
  server.on("/calibrate_mq135", handleCalibrateMQ135);
  server.on("/relay_auto", HTTP_POST, handleRelayAutoUpdate);
  server.on("/relay_control", handleRelayControlPage);
  server.on("/restart", handleRestart);

  server.begin();
  Serial.println(F("تم بدء خادم الويب"));
}

// ========== دوال تحديث الإعدادات ==========

void handleHeatingTempUpdate() {
  if (server.hasArg("heating_target")) {
    heatingTargetTemp = server.arg("heating_target").toFloat();
    if (writeFloatToFile(HEATING_TARGET_TEMP_FILE, heatingTargetTemp)) {
      server.sendHeader(F("Location"), F("/heating?success=1"));
    } else {
      server.sendHeader(F("Location"), F("/heating?error=1"));
    }
  } else {
    server.sendHeader(F("Location"), F("/heating?error=2"));
  }
  server.send(302, F("text/plain"), "");
}

void handleCoolingTempUpdate() {
  if (server.hasArg("cooling_target")) {
    coolingTargetTemp = server.arg("cooling_target").toFloat();
    if (writeFloatToFile(COOLING_TARGET_TEMP_FILE, coolingTargetTemp)) {
      server.sendHeader(F("Location"), F("/cooling?success=1"));
    } else {
      server.sendHeader(F("Location"), F("/cooling?error=1"));
    }
  } else {
    server.sendHeader(F("Location"), F("/cooling?error=2"));
  }
  server.send(302, F("text/plain"), "");
}

void handleTempDiffUpdate() {
  if (server.hasArg("difference")) {
    tempDifference = server.arg("difference").toFloat();
    if (writeFloatToFile(TEMP_DIFF_FILE, tempDifference)) {
      server.sendHeader(F("Location"), F("/?success=1"));
    } else {
      server.sendHeader(F("Location"), F("/?error=1"));
    }
  } else {
    server.sendHeader(F("Location"), F("/?error=2"));
  }
  server.send(302, F("text/plain"), "");
}

void handleAmmoniaUpdate() {
  if (server.hasArg("threshold")) {
    ammoniaThreshold = server.arg("threshold").toFloat();
    if (writeFloatToFile(AMMONIA_THRESHOLD_FILE, ammoniaThreshold)) {
      ammoniaAlert = (ammoniaLevel > ammoniaThreshold);
      server.sendHeader(F("Location"), F("/#ammonia"));
    } else {
      server.sendHeader(F("Location"), F("/#error"));
    }
  } else {
    server.sendHeader(F("Location"), F("/#error"));
  }
  server.send(302, F("text/plain"), "");
}


void handleVentilationTempUpdate() {
  if (server.hasArg("ventilation_temp")) {
    ventilationTempThreshold = server.arg("ventilation_temp").toFloat();
    if (writeFloatToFile(VENTILATION_TEMP_FILE, ventilationTempThreshold)) {
      server.sendHeader(F("Location"), F("/ventilation?success=1"));
    } else {
      server.sendHeader(F("Location"), F("/ventilation?error=1"));
    }
  } else {
    server.sendHeader(F("Location"), F("/ventilation?error=2"));
  }
  server.send(302, F("text/plain"), "");
}

void handleVentilation2TempUpdate() {
  if (server.hasArg("ventilation2_temp")) {
    ventilation2TempThreshold = server.arg("ventilation2_temp").toFloat();
    if (writeFloatToFile(VENTILATION2_TEMP_FILE, ventilation2TempThreshold)) {
      server.sendHeader(F("Location"), F("/ventilation2?success=1"));
    } else {
      server.sendHeader(F("Location"), F("/ventilation2?error=1"));
    }
  } else {
    server.sendHeader(F("Location"), F("/ventilation2?error=2"));
  }
  server.send(302, F("text/plain"), "");
}

void handleHumidityThresholdUpdate() {
  if (server.hasArg("humidity_threshold")) {
    humidityThreshold = server.arg("humidity_threshold").toFloat();
    if (writeFloatToFile(HUMIDITY_THRESHOLD_FILE, humidityThreshold)) {
      server.sendHeader(F("Location"), F("/humidity?success=1"));
    } else {
      server.sendHeader(F("Location"), F("/humidity?error=1"));
    }
  } else {
    server.sendHeader(F("Location"), F("/humidity?error=2"));
  }
  server.send(302, F("text/plain"), "");
}

void handleRestart() {
  server.send(200, "text/plain", "جارٍ إعادة التشغيل...");
  delay(1000);
  ESP.restart();
}



String getRelayStatusTable() {
  String table = F("<div style='overflow-x:auto;'>");
  table += F("<table style='width:100%; border-collapse:collapse; font-size:14px;'>");
  table += F("<thead><tr>");
  table += F("<th style='padding:8px; text-align:right; background:#f1f5f9; border:1px solid #e2e8f0;'>المرحلة</th>");
  table += F("<th style='padding:8px; text-align:center; background:#f1f5f9; border:1px solid #e2e8f0; min-width:70px;'>الحالة</th>");
  table += F("<th style='padding:8px; text-align:center; background:#f1f5f9; border:1px solid #e2e8f0; min-width:80px;'>التحكم</th>");
  table += F("<th style='padding:8px; text-align:center; background:#f1f5f9; border:1px solid #e2e8f0; min-width:70px;'>الآلي</th>");
  table += F("</tr></thead>");
  table += F("<tbody>");

  const char* relayNames[relayCount] = { "التسخين", "التبريد", "التهوية", "التهوية2", "الرطوبة", "تقليب الهواء", "الإضاءة", "غير مستخدم" };

  for (int i = 0; i < relayCount; i++) {
    table += "<tr style='border-bottom:1px solid #e2e8f0;'>";

    // اسم المرحلة
    table += "<td style='padding:8px; text-align:right; border:1px solid #e2e8f0;'>" + String(relayNames[i]) + "</td>";

    // حالة المرحلة
    table += "<td style='padding:8px; text-align:center; border:1px solid #e2e8f0;'>";
    table += "<span style='display:inline-block; padding:4px 8px; border-radius:12px; font-size:12px; font-weight:600;";
    table += digitalRead(relayPins[i]) == LOW ? "background:#2ecc71; color:white;'>يعمل" : "background:#e74c3c; color:white;'>متوقف";
    table += "</span>";
    table += "</td>";

    // زر التحكم اليدوي
    table += "<td style='padding:8px; text-align:center; border:1px solid #e2e8f0;'>";
    table += "<button type='button' style='padding:6px 12px; border:none; border-radius:4px; font-weight:600; cursor:pointer;";
    table += digitalRead(relayPins[i]) == LOW ? "background:#e74c3c; color:white;" : "background:#2ecc71; color:white;";
    if (relayAllowAuto[i]) table += "background:#d1d5db; color:#888; cursor:not-allowed;";
    table += "' onclick=\"controlRelay(" + String(i) + ")\" ";
    if (relayAllowAuto[i]) table += "disabled";
    table += ">";
    table += digitalRead(relayPins[i]) == LOW ? "إيقاف" : "تشغيل";
    table += "</button>";
    table += "</td>";

    // خيار التحكم الآلي
    table += "<td style='padding:8px; text-align:center; border:1px solid #e2e8f0;'>";
    table += "<label style='display:inline-block; position:relative; width:40px; height:20px;'>";
    table += "<input type='checkbox' name='allow_auto_" + String(i) + "' ";
    if (relayAllowAuto[i]) table += "checked";
    table += " onchange='document.getElementById(\"relayForm\").submit();'";
    table += " style='opacity:0; width:0; height:0; position:absolute;'>";
    table += "<span style='position:absolute; cursor:pointer; top:0; left:0; right:0; bottom:0; background-color:#ccc; transition:.4s; border-radius:20px;'></span>";
    table += "<span style='position:absolute; content:\"\"; height:16px; width:16px; left:2px; bottom:2px; background-color:white; transition:.4s; border-radius:50%;";
    if (relayAllowAuto[i]) table += "transform:translateX(20px); background-color:#3498db;";
    table += "'></span>";
    table += "</label>";
    table += "</td>";

    table += "</tr>";
  }

  table += F("</tbody>");
  table += F("</table>");
  table += F("</div>");

  return table;
}

String getPerformanceCard() {
  String card = F("<div class='info-card'>");
  card += F("<h3><i>⚙️</i> أداء النظام</h3>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>وقت التشغيل:</span>");
  card += F("<span class='info-value'>");
  card += getTime();
  card += F("</span>");
  card += F("</div>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>ذاكرة RAM:</span>");
  card += F("<span class='info-value'>");
  card += String(ESP.getFreeHeap() / 1024.0, 1);
  card += F(" / ");
  card += String(ESP.getHeapSize() / 1024.0, 1);
  card += F(" KB</span>");
  card += F("</div>");

  card += F("<div class='progress-container'>");
  card += F("<div class='progress-bar' style='width:");
  int ramUsage = 100 - (ESP.getFreeHeap() * 100 / ESP.getHeapSize());
  card += String(ramUsage);
  card += F("%'></div>");
  card += F("</div>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>ذاكرة الفلاش:</span>");
  card += F("<span class='info-value'>");
  card += String(ESP.getFreeSketchSpace() / 1024.0, 1);
  card += F(" / ");
  card += String(ESP.getFlashChipSize() / 1024.0, 1);
  card += F(" KB</span>");
  card += F("</div>");

  card += F("<div class='progress-container'>");
  card += F("<div class='progress-bar' style='width:");
  int flashUsage = 100 - (ESP.getFreeSketchSpace() * 100 / ESP.getFlashChipSize());
  card += String(flashUsage);
  card += F("%'></div>");
  card += F("</div>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>معالج:</span>");
  card += F("<span class='info-value'>");
  card += String(ESP.getCpuFreqMHz());
  card += F(" MHz</span>");
  card += F("</div>");

  // إضافة درجة حرارة المعالج
  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>حرارة المعالج:</span>");
  card += F("<span class='info-value'>");
  card += String(temperatureRead());
  card += F(" °C</span>");
  card += F("</div>");

  // إضافة استخدام المعالج (تقديري)
  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>استخدام المعالج:</span>");
  card += F("<span class='info-value'>");
  card += String(100 - (ESP.getFreeHeap() * 100 / ESP.getHeapSize()));  // تقدير استخدام المعالج
  card += F(" %</span>");
  card += F("</div>");

  card += F("</div>");
  return card;
}

String getRelayControlPage() {
  String html = getHeader();
  html += F("<div class='header'>");
  html += F("<h1>التحكم في المراحل والأمونيا</h1>");
  html += F("<p>آخر تحديث: ");
  html += getTime();
  html += F("</p>");
  html += F("</div>");

  if (ammoniaAlert) {
    html += F("<div class='alert-banner'>");
    html += F("<div><i>⚠️</i>");
    html += F("<span>تحذير: مستوى الأمونيا مرتفع! ");
    html += String(ammoniaLevel, 1);
    html += F(" ppm (الحد: ");
    html += String(ammoniaThreshold, 1);
    html += F(" ppm)</span>");
    html += F("</div>");
    html += F("</div>");
  }

  html += F("<div class='card'>");
  html += F("<h2 class='card-title'><i>⚡</i> التحكم في المراحل</h2>");
  html += F("<form id='relayForm' action='/relay_auto' method='post' onsubmit='saveScrollPosition();'>");
  html += getRelayStatusTable();
  html += F("<div style='text-align:center; margin-top:15px;'>");
  html += F("<button class='btn' type='submit' style='padding:10px 20px; font-size:16px;'>حفظ الإعدادات</button>");
  html += F("</div>");
  html += F("</form>");
  html += F("</div>");

  html += getAmmoniaControlCard();
  html += getNavBar();
  html += getFooter();

  return html;
}

String getConnectionInfoCard() {
  String card = F("<div class='info-card'>");
  card += F("<h3><i>📶</i> بيانات الاتصال</h3>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>حالة الاتصال:</span>");
  card += F("<span class='info-value'>");
  card += (WiFi.status() == WL_CONNECTED) ? "متصل" : "غير متصل";
  card += F("</span>");
  card += F("</div>");

  if (WiFi.status() == WL_CONNECTED) {
    card += F("<div class='info-row'>");
    card += F("<span class='info-label'>الشبكة:</span>");
    card += F("<span class='info-value'>");
    card += WiFi.SSID();
    card += F("</span>");
    card += F("</div>");

    card += F("<div class='info-row'>");
    card += F("<span class='info-label'>عنوان IP:</span>");
    card += F("<span class='info-value'>");
    card += WiFi.localIP().toString();
    card += F("</span>");
    card += F("</div>");

    card += F("<div class='info-row'>");
    card += F("<span class='info-label'>قوة الإشارة:</span>");
    card += F("<span class='info-value'>");
    card += String(WiFi.RSSI());
    card += F(" dBm</span>");
    card += F("</div>");

    card += F("<div class='progress-container'>");
    card += F("<div class='progress-bar' style='width:");
    int quality = map(WiFi.RSSI(), -100, -50, 0, 100);
    if (quality > 100) quality = 100;
    if (quality < 0) quality = 0;
    card += String(quality);
    card += F("%'></div>");
    card += F("</div>");

    card += F("<div class='info-row'>");
    card += F("<span class='info-label'>آخر تحديث DNS:</span>");
    card += F("<span class='info-value'>");
    card += formatDuration((millis() - lastDynuUpdate) / 1000);
    card += F("</span>");
    card += F("</div>");
  }

  card += F("<button class='btn' style='margin-top:10px;' onclick='restartSystem()'>إعادة تشغيل النظام</button>");
  card += F("</div>");
  return card;
}

String getAmmoniaControlCard() {
  String card = F("<div class='info-card'>");
  card += F("<h3><i>⚠️</i> تحكم في الأمونيا</h3>");

  card += F("<div class='info-row' style='align-items: center;'>");
  card += F("<span class='info-label'>القراءة الحالية:</span>");
  card += F("<span class='info-value' style='font-size: 1.3rem; color:");
  card += (ammoniaLevel > ammoniaThreshold) ? "#e74c3c" : "#27ae60";
  card += F(";'>");
  card += String(ammoniaLevel, 1);
  card += F(" ppm</span>");
  card += F("</div>");

  card += F("<form action='/update_ammonia' method='post' style='margin-top: 15px;' onsubmit='saveScrollPosition()'>");
  card += F("<div class='info-row' style='align-items: center;'>");
  card += F("<span class='info-label'>الحد المسموح:</span>");
  card += F("<input type='number' name='threshold' value='");
  card += String(ammoniaThreshold, 1);
  card += F("' step='0.1' min='0' max='1000' style='width: 80px; padding: 8px; border: 1px solid #ddd; border-radius: 5px;'>");
  card += F(" ppm");
  card += F("</div>");

  card += F("<div style='display: flex; justify-content: space-between; margin-top: 15px;'>");
  card += F("<button type='submit' class='btn' style='background-color: #3498db;'>تحديث الحد</button>");
  card += F("<a href='/calibrate_mq135' class='btn' style='background-color: #f39c12;'>معايرة الحساس</a>");
  card += F("</div>");
  card += F("</form>");

  card += F("</div>");
  return card;
}

String getSensorReadingsCard() {
  String card = F("<div class='info-card'>");
  card += F("<h3><i>📊</i> قراءات الحساسات</h3>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>درجة الحرارة:</span>");
  card += F("<span class='info-value'>");
  card += String(currentTemp, 1);
  card += F(" °C</span>");
  card += F("</div>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>الرطوبة:</span>");
  card += F("<span class='info-value'>");
  card += String(humidity, 1);
  card += F(" %</span>");
  card += F("</div>");

  card += F("<div class='info-row'>");
  card += F("<span class='info-label'>مستوى الأمونيا:</span>");
  card += F("<span class='info-value'>");
  card += String(ammoniaLevel, 1);
  card += F(" ppm</span>");
  card += F("</div>");

  card += F("</div>");
  return card;
}

String getHeader() {
  String header = F("<!DOCTYPE html>");
  header += F("<html>");
  header += F("<head>");
  header += F("<meta charset='UTF-8'>");
  header += F("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  header += F("<title>مزرعه فراخ مومون</title>");
  header += F("<style>");
  header += F(":root {");
  header += F("--primary: #3498db;");
  header += F("--success: #2ecc71;");
  header += F("--danger: #e74c3c;");
  header += F("--warning: #f39c12;");
  header += F("--dark: #2c3e50;");
  header += F("--light: #ecf0f1;");
  header += F("--alert: #ff5252;");
  header += F("}");
  header += F("body { background-color: #f5f7fa; color: #333; line-height: 1.6; padding: 10px; font-size: 1rem; font-family: Arial, sans-serif; }");
  header += F(".container { max-width: 1200px; margin: 0 auto; padding: 5px; }");
  header += F(".card { background: white; border-radius: 10px; padding: 15px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); margin-bottom: 15px; }");
  header += F(".card-title { color: var(--dark); margin-bottom: 12px; font-size: 1.25rem; border-bottom: 2px solid var(--primary); padding-bottom: 8px; display: flex; align-items: center; gap: 8px; }");
  header += F(".control-panel { background: #f8f9fa; border-radius: 8px; padding: 12px; margin-bottom: 15px; display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; gap: 10px; }");
  header += F(".threshold-display { background: linear-gradient(135deg, #f5f7fa, #e4e7eb); border-radius: 8px; padding: 10px 15px; text-align: center; box-shadow: 0 2px 4px rgba(0,0,0,0.05); flex: 1; min-width: 120px; margin: 5px; }");
  header += F(".threshold-value { font-size: 1.2rem; font-weight: 700; }");
  header += F(".threshold-label { font-size: 0.9rem; color: #555; }");
  header += F(".status-badge { display: inline-block; padding: 6px 12px; border-radius: 20px; font-size: 0.875rem; font-weight: 600; }");
  header += F(".status-on { background-color: var(--success); color: white; }");
  header += F(".status-off { background-color: var(--danger); color: white; }");
  header += F(".alert-banner { background-color: var(--alert); color: white; padding: 10px 15px; border-radius: 8px; margin-bottom: 15px; display: flex; align-items: center; gap: 10px; }");
  header += F(".form-group { margin-bottom: 12px; }");
  header += F("label { display: block; margin-bottom: 5px; font-weight: 600; color: #444; }");
  header += F("input[type='number'] { width: 100%; padding: 8px; border: 1px solid #e2e8f0; border-radius: 6px; background: #f8fafc; }");
  header += F(".radio-group { margin: 10px 0; }");
  header += F(".radio-option { margin: 8px 0; display: flex; align-items: center; gap: 8px; }");
  header += F(".checkbox-option { margin: 8px 0; display: flex; align-items: center; gap: 8px; }");
  header += F(".btn { display: inline-block; background-color: var(--primary); color: white; border: none; padding: 10px 20px; border-radius: 6px; cursor: pointer; font-weight: 600; transition: all 0.2s; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }");
  header += F(".btn:hover { opacity: 0.9; transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.15); }");
  header += F(".time-inputs { display: flex; gap: 8px; margin: 10px 0; flex-wrap: wrap; }");
  header += F(".time-segment { display: flex; gap: 5px; align-items: center; background: #f8fafc; padding: 8px; border-radius: 6px; flex-grow: 1; }");
  header += F(".time-segment input { width: 60px; padding: 6px; border: 1px solid #e2e8f0; border-radius: 4px; text-align: center; }");
  header += F(".time-segment span { font-size: 0.875rem; color: #666; }");
  header += F(".timer-status { margin-top: 12px; padding: 10px; background-color: #f8fafc; border-radius: 8px; border-left: 4px solid var(--primary); }");
  header += F(".header { text-align: center; margin-bottom: 15px; padding: 15px; background: linear-gradient(135deg, #3b82f6, #1e40af); color: white; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }");
  header += F(".header h1 { margin-bottom: 8px; }");
  header += F(".nav { background: linear-gradient(135deg, #3b82f6, #1e40af); padding: 10px; border-radius: 10px; margin: 15px 0; }");
  header += F(".nav ul { list-style-type: none; display: flex; justify-content: center; margin: 0; padding: 0; flex-wrap: wrap; gap: 5px; }");
  header += F(".nav li { margin: 0; }");
  header += F(".nav a { color: white; text-decoration: none; font-weight: 600; padding: 10px 15px; border-radius: 5px; transition: all 0.2s; display: flex; align-items: center; gap: 5px; white-space: nowrap; }");
  header += F(".nav a:hover { background-color: rgba(255,255,255,0.2); transform: translateY(-2px); }");
  header += F(".info-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 15px; margin-bottom: 15px; }");
  header += F(".info-card { background: white; border-radius: 10px; padding: 15px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }");
  header += F(".info-card h3 { margin-top: 0; display: flex; align-items: center; gap: 8px; color: var(--dark); border-bottom: 1px solid #eee; padding-bottom: 8px; }");
  header += F(".info-row { display: flex; justify-content: space-between; margin-bottom: 8px; }");
  header += F(".info-label { font-weight: 600; color: #555; }");
  header += F(".info-value { font-weight: 500; }");
  header += F(".progress-container { height: 8px; background: #f1f5f9; border-radius: 4px; margin: 8px 0; overflow: hidden; }");
  header += F(".progress-bar { height: 100%; background: linear-gradient(90deg, #3b82f6, #1e40af); border-radius: 4px; }");

  // Media Queries for Mobile
  header += F("@media (max-width: 768px) {");
  header += F(".threshold-display { width: calc(100% - 20px); margin: 5px 0; }");
  header += F(".control-panel > div { flex-direction: column; }");
  header += F(".time-segment { width: 100%; justify-content: space-between; }");
  header += F(".time-segment input { width: 25%; }");
  header += F(".control-panel { flex-direction: column; align-items: stretch; }");
  header += F(".info-grid { grid-template-columns: 1fr; }");
  header += F(".info-card { padding: 12px; }");
  header += F(".nav ul { flex-direction: column; align-items: stretch; }");
  header += F(".nav a { justify-content: center; }");
  header += F("}");

  header += F("</style>");
  header += F("<script>");
  header += F("function saveScrollPosition() {");
  header += F("  localStorage.setItem('scrollPosition', window.scrollY);");
  header += F("}");
  header += F("window.onload = function() {");
  header += F("  const scrollPosition = localStorage.getItem('scrollPosition');");
  header += F("  if (scrollPosition) {");
  header += F("    window.scrollTo(0, parseInt(scrollPosition));");
  header += F("    localStorage.removeItem('scrollPosition');");
  header += F("  }");
  header += F("};");
  header += F("function controlRelay(relayNum) {");
  header += F("  fetch('/control?relay=' + relayNum, { method: 'POST' })");
  header += F("    .then(() => window.location.reload());");
  header += F("}");
  header += F("function restartSystem() {");
  header += F("  if(confirm('هل أنت متأكد من إعادة تشغيل النظام؟')) {");
  header += F("    fetch('/restart', { method: 'POST' });");
  header += F("  }");
  header += F("}");
  header += F("</script>");
  header += F("</head>");
  header += F("<body>");
  header += F("<div class='container'>");

  return header;
}

String getFooter() {
  String footer = F("<div style='text-align: center; margin-top: 20px; padding: 10px; color: #666; font-size: 0.9rem;'>");
  footer += F("<div>© 2025 نظام التحكم الآلي في مزارع الدواجن</div>");
  footer += F("<div>تصميم وتنفيذ <strong>Eng. Ayman N. Attia</strong></div>");
  footer += F("</div>");
  footer += F("</body>");
  footer += F("</html>");
  return footer;
}

String getNavBar() {
  String nav = F("<nav class='nav'>");
  nav += F("<ul>");
  nav += F("<li><a href='/'><i>🏠</i> الرئيسية</a></li>");
  nav += F("<li><a href='/heating'><i>🔥</i> التسخين</a></li>");
  nav += F("<li><a href='/cooling'><i>❄️</i> التبريد</a></li>");
  nav += F("<li><a href='/ventilation'><i>💨</i> التهوية</a></li>");
  nav += F("<li><a href='/ventilation2'><i>💨</i> التهوية2</a></li>");
  nav += F("<li><a href='/humidity'><i>💧</i> الرطوبة</a></li>");
  nav += F("<li><a href='/air_circulation'><i>🌀</i> تقليب الهواء</a></li>");
  nav += F("<li><a href='/lighting'><i>💡</i> الإضاءة</a></li>");
  nav += F("</ul>");
  nav += F("</nav>");
  return nav;
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

String getControlForm(String page, String title, String icon, String btnClass, ControlMode currentMode,
                      unsigned long onDuration, unsigned long offDuration,
                      bool isOn, bool showAmmoniaOption = false,
                      bool ammoniaEnabled = false, float threshold = 0.0,
                      float currentValue = 0.0, String valueUnit = "") {
  String form = "<div class='card'>";
  form += "<h2 class='card-title'><i>" + icon + "</i> " + title;
  form += "<span class='status-badge ";
  form += (isOn ? "status-on" : "status-off");
  form += "' style='margin-right:10px;'>";
  form += isOn ? "يعمل" : "متوقف";
  form += "</span></h2>";

  // Control Panel - Modified to show values in a row on mobile
  form += "<div class='control-panel' style='flex-direction: column; gap: 10px;'>";

  // Values row - will be horizontal on desktop and stacked on mobile
  form += "<div style='display: flex; flex-wrap: wrap; gap: 10px; justify-content: space-between;'>";

  if (page != "lighting" && page != "air_circulation") {
    form += "<div class='threshold-display' style='flex: 1; min-width: 120px;'>";
    form += "<div class='threshold-value'>" + String(threshold, 1) + " " + valueUnit + "</div>";
    form += "<div class='threshold-label'>المطلوبة</div>";
    form += "</div>";

    form += "<div class='threshold-display' style='flex: 1; min-width: 120px; background:#e3f2fd;'>";
    form += "<div class='threshold-value'>" + String(currentValue, 1) + " " + valueUnit + "</div>";
    form += "<div class='threshold-label'>الحالية</div>";
    form += "</div>";

    form += "<div class='threshold-display' style='flex: 1; min-width: 120px; background:#fff8e1;'>";
    form += "<div class='threshold-value'>" + String(fabs(currentValue - threshold), 1) + " " + valueUnit + "</div>";
    form += "<div class='threshold-label'>الفرق</div>";
    form += "</div>";
  }

  form += "</div>";  // End values row

  form += "</div>";  // End control panel

  // Rest of the form remains the same
  form += "<form action='/update_" + page + "_timer' method='post' onsubmit='saveScrollPosition()'>";

  form += F("<div class='radio-group'>");
  form += F("<div class='radio-option'>");
  form += "<input type='radio' id='" + page + "_mode_disabled' name='" + page + "_control_mode' value='0' ";
  form += (currentMode == CM_DISABLED) ? "checked" : "";
  form += ">";
  form += "<label for='" + page + "_mode_disabled'>معطل</label>";
  form += F("</div>");

  form += F("<div class='radio-option'>");
  form += "<input type='radio' id='" + page + "_mode_manual' name='" + page + "_control_mode' value='1' ";
  form += (currentMode == CM_MANUAL) ? "checked" : "";
  form += ">";
  form += "<label for='" + page + "_mode_manual'>يدوي (بالتايمر فقط)</label>";
  form += F("</div>");

  if (page == "heating" || page == "cooling" || page == "ventilation" || page == "ventilation2") {
    form += F("<div class='radio-option'>");
    form += "<input type='radio' id='" + page + "_mode_temp' name='" + page + "_control_mode' value='2' ";
    form += (currentMode == CM_TEMPERATURE) ? "checked" : "";
    form += ">";
    form += "<label for='" + page + "_mode_temp'>تلقائي (حسب الحرارة والتايمر)</label>";
    form += F("</div>");
  }

  if (page == "humidity") {
    form += F("<div class='radio-option'>");
    form += "<input type='radio' id='" + page + "_mode_humidity' name='" + page + "_control_mode' value='3' ";
    form += (currentMode == CM_HUMIDITY) ? "checked" : "";
    form += ">";
    form += "<label for='" + page + "_mode_humidity'>تلقائي (حسب الرطوبة والتايمر)</label>";
    form += F("</div>");
  }
  form += F("</div>");

  if (showAmmoniaOption) {
    form += F("<div class='checkbox-option'>");
    form += "<input type='checkbox' id='" + page + "_ammonia_enabled' name='" + page + "_ammonia_enabled' ";
    form += ammoniaEnabled ? "checked" : "";
    form += ">";
    form += "<label for='" + page + "_ammonia_enabled'>التفاعل مع ارتفاع الأمونيا</label>";
    form += F("</div>");
  }


  if (page != "lighting" && page != "air_circulation") {
    form += F("<div class='form-group'>");
    form += F("<label>درجة الحرارة المطلوبة:</label>");
    form += "<input type='number' id='" + page + "_target_temp' name='" + page + "_target_temp' value='" + String(threshold, 1) + "' step='0.1' min='-40' max='80' required>";
    form += F("</div>");
  }

  form += F("<h3 style='margin: 10px 0 5px 0'>إعدادات التايمر</h3>");

  // On duration
  form += F("<div class='form-group'>");
  form += F("<label>زمن التشغيل:</label>");
  form += F("<div class='time-inputs'>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_on_days' name='" + page + "_on_days' value='" + String(onDuration / 86400) + "' min='0' max='30' placeholder='أيام'>";
  form += F("<span>يوم</span>");
  form += F("</div>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_on_hours' name='" + page + "_on_hours' value='" + String((onDuration % 86400) / 3600) + "' min='0' max='23' placeholder='ساعات'>";
  form += F("<span>ساعة</span>");
  form += F("</div>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_on_minutes' name='" + page + "_on_minutes' value='" + String((onDuration % 3600) / 60) + "' min='0' max='59' placeholder='دقائق'>";
  form += F("<span>دقيقة</span>");
  form += F("</div>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_on_seconds' name='" + page + "_on_seconds' value='" + String(onDuration % 60) + "' min='0' max='59' placeholder='ثواني'>";
  form += F("<span>ثانية</span>");
  form += F("</div>");
  form += F("</div>");
  form += F("</div>");

  // Off duration
  form += F("<div class='form-group'>");
  form += F("<label>زمن الإيقاف:</label>");
  form += F("<div class='time-inputs'>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_off_days' name='" + page + "_off_days' value='" + String(offDuration / 86400) + "' min='0' max='30' placeholder='أيام'>";
  form += F("<span>يوم</span>");
  form += F("</div>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_off_hours' name='" + page + "_off_hours' value='" + String((offDuration % 86400) / 3600) + "' min='0' max='23' placeholder='ساعات'>";
  form += F("<span>ساعة</span>");
  form += F("</div>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_off_minutes' name='" + page + "_off_minutes' value='" + String((offDuration % 3600) / 60) + "' min='0' max='59' placeholder='دقائق'>";
  form += F("<span>دقيقة</span>");
  form += F("</div>");
  form += F("<div class='time-segment'>");
  form += "<input type='number' id='" + page + "_off_seconds' name='" + page + "_off_seconds' value='" + String(offDuration % 60) + "' min='0' max='59' placeholder='ثواني'>";
  form += F("<span>ثانية</span>");
  form += F("</div>");
  form += F("</div>");
  form += F("</div>");

  form += "<button type='submit' class='btn " + btnClass + "'>حفظ إعدادات " + title + "</button>";
  form += F("</form>");

  // Status section
  form += F("<div class='timer-status'>");
  form += F("<p><strong>وضع التشغيل الحالي:</strong> ");
  if (currentMode == CM_DISABLED) form += F("معطل");
  else if (currentMode == CM_MANUAL) form += F("يدوي (تائمر)");
  else if (currentMode == CM_TEMPERATURE) form += F("تلقائي (حسب الحرارة)");
  else if (currentMode == CM_HUMIDITY) form += F("تلقائي (حسب الرطوبة)");

  form += F("<br><strong>زمن التشغيل المحدد:</strong> ");
  form += formatDuration(onDuration);

  form += F("<br><strong>زمن الإيقاف المحدد:</strong> ");
  form += formatDuration(offDuration);
  form += F("</p>");
  form += F("</div>");
  form += F("</div>");

  return form;
}

void handleRoot() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header'>");
  html += F("<h1>مزرعه فراخ مومون</h1>");
  html += F("<p>آخر تحديث: ");
  html += getTime();
  html += F("</p>");
  html += F("</div>");

  if (ammoniaAlert) {
    html += F("<div class='alert-banner'>");
    html += F("<div><i>⚠️</i>");
    html += F("<span>تحذير: مستوى الأمونيا مرتفع! ");
    html += String(ammoniaLevel, 1);
    html += F(" ppm (الحد: ");
    html += String(ammoniaThreshold, 1);
    html += F(" ppm)</span>");
    html += F("</div>");
    html += F("</div>");
  }

  // إضافة بطاقات المعلومات الجديدة
  html += F("<div class='info-grid'>");
  html += getSensorReadingsCard();
  html += getConnectionInfoCard();
  html += getPerformanceCard();
  html += F("</div>");

  // إضافة جدول حالة الريلايات
  html += F("<div class='card'>");
  html += F("<h2 class='card-title'><i>⚡</i> حالة المراحل</h2>");
  html += F("<table>");
  html += F("<tr><th>اسم الريلاي</th><th>الحالة</th><th>الدرجة المطلوبة</th><th>الدرجة الحالية</th><th>الفرق</th><th>وضع التشغيل</th></tr>");

  const char* relayNames[relayCount] = { "التسخين", "التبريد", "التهوية", "التهوية2", "الرطوبة", "تقليب الهواء", "الإضاءة", "غير مستخدم" };
  float thresholds[relayCount] = { heatingTargetTemp, coolingTargetTemp, ventilationTempThreshold, ventilation2TempThreshold, humidityThreshold, 0, 0, 0 };
  float currentValues[relayCount] = { currentTemp, currentTemp, currentTemp, currentTemp, humidity, 0, 0, 0 };
  String units[relayCount] = { "°C", "°C", "°C", "°C", "%", "", "", "" };
  ControlMode modes[relayCount] = { currentHeatingMode, currentCoolingMode, currentVentilationMode, currentVentilation2Mode, currentHumidityMode, currentAirCirculationMode, currentLightingMode, CM_DISABLED };

  for (int i = 0; i < relayCount; i++) {
    if (i == 7) continue;  // تخطي الريلاي غير المستخدم

    html += "<tr>";
    html += "<td>" + String(relayNames[i]) + "</td>";

    // Status
    html += "<td>";
    html += "<span class='status-badge ";
    html += digitalRead(relayPins[i]) == LOW ? "status-on" : "status-off";
    html += "'>";
    html += digitalRead(relayPins[i]) == LOW ? "يعمل" : "متوقف";
    html += "</span>";
    html += "</td>";

    // Threshold
    html += "<td>";
    if (i != 5 && i != 6) {  // تخطي تقليب الهواء والإضاءة
      html += String(thresholds[i], 1) + " " + units[i];
    } else {
      html += "-";
    }
    html += "</td>";

    // Current value
    html += "<td>";
    if (i != 5 && i != 6) {
      html += String(currentValues[i], 1) + " " + units[i];
    } else {
      html += "-";
    }
    html += "</td>";

    // Difference
    html += "<td>";
    if (i != 5 && i != 6) {
      html += String(fabs(currentValues[i] - thresholds[i]), 1) + " " + units[i];
    } else {
      html += "-";
    }
    html += "</td>";

    // Mode
    html += "<td>";
    switch (modes[i]) {
      case CM_DISABLED: html += "معطل"; break;
      case CM_MANUAL: html += "يدوي (تائمر)"; break;
      case CM_TEMPERATURE: html += "تلقائي (حرارة)"; break;
      case CM_HUMIDITY: html += "تلقائي (رطوبة)"; break;
      default: html += "-"; break;
    }
    html += "</td>";
    html += "</tr>";
  }
  html += F("</table>");
  html += F("<div style='text-align: center; margin-top: 15px;'>");
  html += F("<a href='/relay_control' class='btn'>التحكم الكامل في المراحل</a>");
  html += F("</div>");
  html += F("</div>");

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}


void handleHeatingPage() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header' style='background: linear-gradient(135deg, #e74c3c, #c0392b);'>");
  html += F("<h1>إعدادات التسخين</h1>");
  html += F("</div>");

  html += getControlForm("heating", "التسخين", "🔥", "btn-heating", currentHeatingMode,
                         heatingOnDuration, heatingOffDuration, true, false, false,
                         heatingTargetTemp, currentTemp, "°C");

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}

void handleCoolingPage() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header' style='background: linear-gradient(135deg, #3498db, #2980b9);'>");
  html += F("<h1>إعدادات التبريد</h1>");
  html += F("</div>");

  html += getControlForm("cooling", "التبريد", "❄️", "btn-cooling", currentCoolingMode,
                         coolingOnDuration, coolingOffDuration, true, false, false,
                         coolingTargetTemp, currentTemp, "°C");

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}

void handleVentilationPage() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header' style='background: linear-gradient(135deg, #27ae60, #219653);'>");
  html += F("<h1>إعدادات التهوية</h1>");
  html += F("</div>");

  html += getControlForm("ventilation", "التهوية", "💨", "btn-ventilation", currentVentilationMode,
                         ventilationOnDuration, ventilationOffDuration, true, true,
                         ventilationAmmoniaEnabled, ventilationTempThreshold, currentTemp, "°C");

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}

void handleVentilation2Page() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header' style='background: linear-gradient(135deg, #2ecc71, #27ae60);'>");
  html += F("<h1>إعدادات التهوية2</h1>");
  html += F("</div>");

  html += getControlForm("ventilation2", "التهوية2", "💨", "btn-ventilation", currentVentilation2Mode,
                         ventilation2OnDuration, ventilation2OffDuration, true, true,
                         ventilation2AmmoniaEnabled, ventilation2TempThreshold, currentTemp, "°C");

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}

void handleHumidityPage() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header' style='background: linear-gradient(135deg, #9b59b6, #8e44ad);'>");
  html += F("<h1>إعدادات الرطوبة</h1>");
  html += F("</div>");

  html += getControlForm("humidity", "الرطوبة", "💧", "btn-humidity", currentHumidityMode,
                         humidityOnDuration, humidityOffDuration, true, false, false,
                         humidityThreshold, humidity, "%");

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}



















void handleAirCirculationPage() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header' style='background: linear-gradient(135deg, #16a085, #1abc9c);'>");
  html += F("<h1>إعدادات تقليب الهواء</h1>");
  html += F("</div>");

  html += getControlForm("air_circulation", "تقليب الهواء", "🌀", "btn-air-circulation", currentAirCirculationMode,
                         airCirculationOnDuration, airCirculationOffDuration, true);

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}

void handleLightingPage() {
  if (!server.authenticate("tchfshn", "godislove")) {
    return server.requestAuthentication();
  }

  String html = getHeader();
  html += F("<div class='header' style='background: linear-gradient(135deg, #f1c40f, #f39c12);'>");
  html += F("<h1>إعدادات الإضاءة</h1>");
  html += F("</div>");

  html += getControlForm("lighting", "الإضاءة", "💡", "btn-lighting", currentLightingMode,
                         lightingOnDuration, lightingOffDuration, true);

  html += getNavBar();
  html += getFooter();

  server.sendHeader(F("Content-Type"), F("text/html; charset=utf-8"));
  server.send(200, F("text/html"), html);
}

// ========== معالجات تحديث الإعدادات ==========

void handleHeatingTimerUpdate() {
  if (server.hasArg("heating_control_mode")) {
    currentHeatingMode = static_cast<ControlMode>(server.arg("heating_control_mode").toInt());
    writeIntToFile(CONTROL_MODE_HEATING_FILE, currentHeatingMode);
  }

  if (server.hasArg("heating_target_temp")) {
    heatingTargetTemp = server.arg("heating_target_temp").toFloat();
    writeFloatToFile(HEATING_TARGET_TEMP_FILE, heatingTargetTemp);
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
      writeUlongToFile(HEATING_TIMER_ON_FILE, heatingOnDuration);
    }
  }

  if (server.hasArg("heating_off_days") && server.hasArg("heating_off_hours") && server.hasArg("heating_off_minutes") && server.hasArg("heating_off_seconds")) {
    newOffDuration = server.arg("heating_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("heating_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("heating_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("heating_off_seconds").toInt();
    if (newOffDuration > 0) {
      heatingOffDuration = newOffDuration;
      writeUlongToFile(HEATING_TIMER_OFF_FILE, heatingOffDuration);
    }
  }

  server.sendHeader(F("Location"), F("/heating#settings"));
  server.send(302, F("text/plain"), "تم تحديث إعدادات التسخين بنجاح");
}

void handleCoolingTimerUpdate() {
  if (server.hasArg("cooling_control_mode")) {
    currentCoolingMode = static_cast<ControlMode>(server.arg("cooling_control_mode").toInt());
    writeIntToFile(CONTROL_MODE_COOLING_FILE, currentCoolingMode);
  }

  if (server.hasArg("cooling_target_temp")) {
    coolingTargetTemp = server.arg("cooling_target_temp").toFloat();
    writeFloatToFile(COOLING_TARGET_TEMP_FILE, coolingTargetTemp);
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
      writeUlongToFile(COOLING_TIMER_ON_FILE, coolingOnDuration);
    }
  }

  if (server.hasArg("cooling_off_days") && server.hasArg("cooling_off_hours") && server.hasArg("cooling_off_minutes") && server.hasArg("cooling_off_seconds")) {
    newOffDuration = server.arg("cooling_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("cooling_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("cooling_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("cooling_off_seconds").toInt();
    if (newOffDuration > 0) {
      coolingOffDuration = newOffDuration;
      writeUlongToFile(COOLING_TIMER_OFF_FILE, coolingOffDuration);
    }
  }

  server.sendHeader(F("Location"), F("/cooling#settings"));
  server.send(302, F("text/plain"), "تم تحديث إعدادات التبريد بنجاح");
}

void handleVentilationTimerUpdate() {
  if (server.hasArg("ventilation_control_mode")) {
    currentVentilationMode = static_cast<ControlMode>(server.arg("ventilation_control_mode").toInt());
    writeIntToFile(CONTROL_MODE_VENTILATION_FILE, currentVentilationMode);
  }

  if (server.hasArg("ventilation_target_temp")) {
    ventilationTempThreshold = server.arg("ventilation_target_temp").toFloat();
    writeFloatToFile(VENTILATION_TEMP_FILE, ventilationTempThreshold);
  }

  if (server.hasArg("ventilation_ammonia_enabled")) {
    ventilationAmmoniaEnabled = true;
  } else {
    ventilationAmmoniaEnabled = false;
  }
  writeBoolToFile(VENTILATION_AMMONIA_ENABLED_FILE, ventilationAmmoniaEnabled);

  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;

  if (server.hasArg("ventilation_on_days") && server.hasArg("ventilation_on_hours") && server.hasArg("ventilation_on_minutes") && server.hasArg("ventilation_on_seconds")) {
    newOnDuration = server.arg("ventilation_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("ventilation_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("ventilation_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("ventilation_on_seconds").toInt();
    if (newOnDuration > 0) {
      ventilationOnDuration = newOnDuration;
      writeUlongToFile(VENTILATION_TIMER_ON_FILE, ventilationOnDuration);
    }
  }

  if (server.hasArg("ventilation_off_days") && server.hasArg("ventilation_off_hours") && server.hasArg("ventilation_off_minutes") && server.hasArg("ventilation_off_seconds")) {
    newOffDuration = server.arg("ventilation_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("ventilation_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("ventilation_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("ventilation_off_seconds").toInt();
    if (newOffDuration > 0) {
      ventilationOffDuration = newOffDuration;
      writeUlongToFile(VENTILATION_TIMER_OFF_FILE, ventilationOffDuration);
    }
  }

  server.sendHeader(F("Location"), F("/ventilation#settings"));
  server.send(302, F("text/plain"), "تم تحديث إعدادات التهوية بنجاح");
}

void handleVentilation2TimerUpdate() {
  if (server.hasArg("ventilation2_control_mode")) {
    currentVentilation2Mode = static_cast<ControlMode>(server.arg("ventilation2_control_mode").toInt());
    writeIntToFile(CONTROL_MODE_VENTILATION2_FILE, currentVentilation2Mode);
  }

  if (server.hasArg("ventilation2_target_temp")) {
    ventilation2TempThreshold = server.arg("ventilation2_target_temp").toFloat();
    writeFloatToFile(VENTILATION2_TEMP_FILE, ventilation2TempThreshold);
  }

  if (server.hasArg("ventilation2_ammonia_enabled")) {
    ventilation2AmmoniaEnabled = true;
  } else {
    ventilation2AmmoniaEnabled = false;
  }
  writeBoolToFile(VENTILATION2_AMMONIA_ENABLED_FILE, ventilation2AmmoniaEnabled);

  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;

  if (server.hasArg("ventilation2_on_days") && server.hasArg("ventilation2_on_hours") && server.hasArg("ventilation2_on_minutes") && server.hasArg("ventilation2_on_seconds")) {
    newOnDuration = server.arg("ventilation2_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("ventilation2_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("ventilation2_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("ventilation2_on_seconds").toInt();
    if (newOnDuration > 0) {
      ventilation2OnDuration = newOnDuration;
      writeUlongToFile(VENTILATION2_TIMER_ON_FILE, ventilation2OnDuration);
    }
  }

  if (server.hasArg("ventilation2_off_days") && server.hasArg("ventilation2_off_hours") && server.hasArg("ventilation2_off_minutes") && server.hasArg("ventilation2_off_seconds")) {
    newOffDuration = server.arg("ventilation2_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("ventilation2_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("ventilation2_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("ventilation2_off_seconds").toInt();
    if (newOffDuration > 0) {
      ventilation2OffDuration = newOffDuration;
      writeUlongToFile(VENTILATION2_TIMER_OFF_FILE, ventilation2OffDuration);
    }
  }

  server.sendHeader(F("Location"), F("/ventilation2#settings"));
  server.send(302, F("text/plain"), "تم تحديث إعدادات التهوية2 بنجاح");
}

void handleHumidityTimerUpdate() {
  if (server.hasArg("humidity_control_mode")) {
    currentHumidityMode = static_cast<ControlMode>(server.arg("humidity_control_mode").toInt());
    writeIntToFile(CONTROL_MODE_HUMIDITY_FILE, currentHumidityMode);
  }

  if (server.hasArg("humidity_target_temp")) {
    humidityThreshold = server.arg("humidity_target_temp").toFloat();
    writeFloatToFile(HUMIDITY_THRESHOLD_FILE, humidityThreshold);
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
      writeUlongToFile(HUMIDITY_TIMER_ON_FILE, humidityOnDuration);
    }
  }

  if (server.hasArg("humidity_off_days") && server.hasArg("humidity_off_hours") && server.hasArg("humidity_off_minutes") && server.hasArg("humidity_off_seconds")) {
    newOffDuration = server.arg("humidity_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("humidity_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("humidity_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("humidity_off_seconds").toInt();
    if (newOffDuration > 0) {
      humidityOffDuration = newOffDuration;
      writeUlongToFile(HUMIDITY_TIMER_OFF_FILE, humidityOffDuration);
    }
  }

  server.sendHeader(F("Location"), F("/humidity#settings"));
  server.send(302, F("text/plain"), "تم تحديث إعدادات الرطوبة بنجاح");
}

void handleAirCirculationTimerUpdate() {
  if (server.hasArg("air_circulation_control_mode")) {
    currentAirCirculationMode = static_cast<ControlMode>(server.arg("air_circulation_control_mode").toInt());
    writeIntToFile(CONTROL_MODE_AIR_CIRCULATION_FILE, currentAirCirculationMode);
  }

  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;

  if (server.hasArg("air_circulation_on_days") && server.hasArg("air_circulation_on_hours") && server.hasArg("air_circulation_on_minutes") && server.hasArg("air_circulation_on_seconds")) {
    newOnDuration = server.arg("air_circulation_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("air_circulation_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("air_circulation_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("air_circulation_on_seconds").toInt();
    if (newOnDuration > 0) {
      airCirculationOnDuration = newOnDuration;
      writeUlongToFile(AIR_CIRCULATION_TIMER_ON_FILE, airCirculationOnDuration);
    }
  }

  if (server.hasArg("air_circulation_off_days") && server.hasArg("air_circulation_off_hours") && server.hasArg("air_circulation_off_minutes") && server.hasArg("air_circulation_off_seconds")) {
    newOffDuration = server.arg("air_circulation_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("air_circulation_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("air_circulation_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("air_circulation_off_seconds").toInt();
    if (newOffDuration > 0) {
      airCirculationOffDuration = newOffDuration;
      writeUlongToFile(AIR_CIRCULATION_TIMER_OFF_FILE, airCirculationOffDuration);
    }
  }

  server.sendHeader(F("Location"), F("/air_circulation#settings"));
  server.send(302, F("text/plain"), "تم تحديث إعدادات تقليب الهواء بنجاح");
}

void handleLightingTimerUpdate() {
  if (server.hasArg("lighting_control_mode")) {
    currentLightingMode = static_cast<ControlMode>(server.arg("lighting_control_mode").toInt());
    writeIntToFile(CONTROL_MODE_LIGHTING_FILE, currentLightingMode);
  }

  unsigned long newOnDuration = 0;
  unsigned long newOffDuration = 0;

  if (server.hasArg("lighting_on_days") && server.hasArg("lighting_on_hours") && server.hasArg("lighting_on_minutes") && server.hasArg("lighting_on_seconds")) {
    newOnDuration = server.arg("lighting_on_days").toInt() * 86400UL;
    newOnDuration += server.arg("lighting_on_hours").toInt() * 3600UL;
    newOnDuration += server.arg("lighting_on_minutes").toInt() * 60UL;
    newOnDuration += server.arg("lighting_on_seconds").toInt();
    if (newOnDuration > 0) {
      lightingOnDuration = newOnDuration;
      writeUlongToFile(LIGHTING_TIMER_ON_FILE, lightingOnDuration);
    }
  }

  if (server.hasArg("lighting_off_days") && server.hasArg("lighting_off_hours") && server.hasArg("lighting_off_minutes") && server.hasArg("lighting_off_seconds")) {
    newOffDuration = server.arg("lighting_off_days").toInt() * 86400UL;
    newOffDuration += server.arg("lighting_off_hours").toInt() * 3600UL;
    newOffDuration += server.arg("lighting_off_minutes").toInt() * 60UL;
    newOffDuration += server.arg("lighting_off_seconds").toInt();
    if (newOffDuration > 0) {
      lightingOffDuration = newOffDuration;
      writeUlongToFile(LIGHTING_TIMER_OFF_FILE, lightingOffDuration);
    }
  }

  server.sendHeader(F("Location"), F("/lighting#settings"));
  server.send(302, F("text/plain"), "تم تحديث إعدادات الإضاءة بنجاح");
}