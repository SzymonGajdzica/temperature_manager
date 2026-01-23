#include <Arduino.h>
#include <ESPDateTime.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Json.h>
#include <LinkedList.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Bounce2.h>
#include <SHT4x.h>

// 200L, 4KW od 50C do 70C w 70 minut
// 200L, 2KW od 50C do 70C w 140 minut
// 140L, 2KW od 10C do 70C w 5h

#define version 10

#define storageStartGuard 0xDEADBEEF
#define storageEndGuard 0xBEEFDEAD

#define ssid "Dom"
#define password "123456789a"
#define hostId1 100
#define hostId2 181

#define loopDelay 1
const int maxNumberOfRetries = 10;
const int httpReadDelayMillis = 30 * 1000;
const int wifiCheckDelay = 60 * 5;
const int wifiConnectTimeoutMillis = 30 * 1000;

#define nightHoursSize 12

#define humidityPinSDA 16
#define humidityPinSCL 17
#define temperaturePin 18
#define moveDetectorBottomPin 21
#define moveDetectorMiddlePin 22
#define moveDetectorUpPin 23
#define upButtonPin 4
#define heater200Pin1 25
#define heater200Pin2 26
#define heater140Pin 27
#define pumpPin 14
#define ventPin 33
#define ventGearPin 32

void print(String message) {
  Serial.print(message);
}

void println(String message) {
  Serial.println(message);
}

String formatSeconds(int seconds) {
    int h = seconds / 3600;
    int m = (seconds % 3600) / 60;
    int s = seconds % 60;

    String result = "PT";

    if (h > 0) result += String(h) + "H";
    if (m > 0) result += String(m) + "M";
    if (s > 0 || seconds == 0) result += String(s) + "S";

    return result;
}

struct WifiManager {
  private:
  time_t disconnectTime;
  time_t lastDisconnectTime;

  bool isWifiConnected() {
    return WiFi.status() == WL_CONNECTED;
  }

  void waitForWifiConnection(int timeoutMillis) {
    print("Connecting to WiFi");

    unsigned long start = millis();

    while (!isWifiConnected()) {
      if (millis() - start >= timeoutMillis) {
        println(" Failed (timeout)");
        return;
      }

      delay(100);
      print(".");
    }

    println(" Connected!");
    print("IP address: ");
    println(WiFi.localIP().toString());
  }

  void setupInternetConnection() {
    WiFi.disconnect(true, true);
    delay(300);

    WiFi.mode(WIFI_STA);
    delay(200);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    IPAddress localIP(192, 168, hostId1, hostId2);
    IPAddress gateway(192, 168, hostId1, 1); 
    IPAddress subnet(255, 255, 0, 0); 
    IPAddress primaryDNS(8, 8, 8, 8);
    IPAddress secondaryDNS(8, 8, 4, 4); 

    while (!WiFi.config(localIP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("STA Failed to configure");
        delay(10000);
    }

    WiFi.begin("Dom", "123456789a");
  }

  void setupLocalTime() {
    DateTime.setTimeZone("CET-1CEST,M3.5.0,M10.5.0/3");
    println("Fetching current time from server");
    while(!DateTime.isTimeValid()) {
      if(!DateTime.begin(20000)) {
        println("Failed to fetch current time from server");
        delay(httpReadDelayMillis);
      }
    }
    print("Fetched current time from server ");
    println(DateTime.toString());
  }

  public:
  WifiManager() {}
  ~WifiManager() {}

  void begin() {
    setupInternetConnection();
    waitForWifiConnection(wifiConnectTimeoutMillis);
    setupLocalTime();
  }

  time_t getLastDisconnectTime() {
    return lastDisconnectTime;
  }

  void invalidate() {
    bool isConnected = isWifiConnected();
    if(isConnected) {
      disconnectTime = 0;
      return;
    }
    if(disconnectTime == 0) {
      lastDisconnectTime = DateTime.getTime();
      disconnectTime = DateTime.getTime();
    } else if(DateTime.getTime() - disconnectTime > wifiCheckDelay) {
      setupInternetConnection();
      disconnectTime = 0;
    }
  }

  bool ensureConnection() {
    if(!isWifiConnected()) {
      setupInternetConnection();
      waitForWifiConnection(wifiConnectTimeoutMillis);
    }
    return isWifiConnected();
  }

};

WifiManager wifiManager = WifiManager();

struct Switch {
  private:
  int pin;
  bool enabled;
  String name;
  time_t lastEnableTime;
  time_t lastDisableTime;

  void setPinOutput(int pin, bool value) {
    digitalWrite(pin, value ? HIGH : LOW);
    delay(1000);
  }

  public:
  Switch(int pin, String name) : pin(pin), enabled(false), name(name) { }
  ~Switch() {}

  void begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  void setState(bool newState) {
    if (enabled != newState) {
      enabled = newState;
      if(enabled) {
        lastEnableTime = DateTime.getTime();
      } else {
        lastDisableTime = DateTime.getTime();
      }
      setPinOutput(pin, enabled);
    }
  }

  bool isOn() {
    return enabled;
  }

  String getName() {
    return name;
  }

  time_t getLastEnableTime() {
    return lastEnableTime;
  }

  time_t getLastDisableTime() {
    return lastDisableTime;
  }

  time_t getLastChangeTime() {
    return enabled ? lastEnableTime : lastDisableTime;
  }

  Json getJson() {
    Json json;
    json["name"] = name;
    json["enabled"] = enabled;
    json["lastEnableTime"] = DateTimeParts::from(lastEnableTime).toString();
    json["lastDisableTime"] = DateTimeParts::from(lastDisableTime).toString();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["name"] = "Nazwa przełącznika";
    json["enabled"] = "Stan przełącznika (true - włączony, false - wyłączony)";
    json["lastEnableTime"] = "Czas ostatniego włączenia przełącznika";
    json["lastDisableTime"] = "Czas ostatniego wyłączenia przełącznika";
    return json;
  }

};

Switch ventSwitch = Switch(ventPin, "Wentylacja środek");
Switch ventGearSwitch = Switch(ventGearPin, "Wentylacja środek bieg");
Switch pumpSwitch = Switch(pumpPin, "Pompa obiegowa");
Switch heater200Switch1 = Switch(heater200Pin1, "Grzałka pierwsza 200L");
Switch heater200Switch2 = Switch(heater200Pin2, "Grzałka druga 200L");
Switch heater140Switch = Switch(heater140Pin, "Grzałka 140L");

Bounce2::Button upButtonKitchen = Bounce2::Button();
AsyncWebServer server(80);

struct OtherParams {
  public:
  OtherParams() {}
  ~OtherParams() {}

  Json getJson() {
    Json json;
    json["time"] = DateTime.getParts().toString();
    json["bootTime"] = DateTimeParts::from(DateTime.getBootTime()).toString();
    json["version"] = String(version);
    json["lastWifiDisconnectTime"] = DateTimeParts::from(wifiManager.getLastDisconnectTime()).toString();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["time"] = "Aktualny czas systemowy";
    json["bootTime"] = "Czas uruchomienia urządzenia";
    json["version"] = "Wersja oprogramowania urządzenia";
    json["lastWifiDisconnectTime"] = "Czas ostatniego rozłączenia z siecią WiFi";
    return json;
  }

};

OtherParams otherParams = OtherParams();

struct PumpConfig {
  public:
  int nightStartHour;
  int nightEndHour;
  int workingTemperatureReadDelay;
  int idleTemperatureReadDelay;
  float pumpOffTemperature;
  int pumpOnTimeBottomBathroom;
  int pumpOnTimeMiddleBathroom;
  int pumpOnTimeUpKitchen;
  int pumpOnTimeUpBathroom;
  int pumpDelayTime;
  float pumpLowTemperatureGuard;
  int pumpOnTimeGuard;
  bool pumpAutoMode;
  bool pumpEnabled;

  PumpConfig() : nightStartHour(23), nightEndHour(6) ,workingTemperatureReadDelay(2), idleTemperatureReadDelay(120), pumpOffTemperature(40), pumpOnTimeBottomBathroom(0), pumpOnTimeMiddleBathroom(60), pumpOnTimeUpKitchen(0), pumpOnTimeUpBathroom(60), pumpDelayTime(60 * 30), pumpLowTemperatureGuard(5), pumpOnTimeGuard(240), pumpAutoMode(true), pumpEnabled(false) {}
  ~PumpConfig() {}

  int getPumpOnTime(int index) {
    switch (index) {
      case 0:
        return pumpOnTimeBottomBathroom;
      case 1:
        return pumpOnTimeMiddleBathroom;
      case 2:
        return pumpOnTimeUpBathroom;
      case 3:
        return pumpOnTimeUpKitchen;
      default:
        return 0;
    }
  }

  Json getJson() {
    Json json;
    json["nightStartHour"] = nightStartHour;
    json["nightEndHour"] = nightEndHour;
    json["workingTemperatureReadDelay"] = workingTemperatureReadDelay;
    json["idleTemperatureReadDelay"] = idleTemperatureReadDelay;
    json["pumpOffTemperature"] = pumpOffTemperature;
    json["pumpOnTimeBottomBathroom"] = pumpOnTimeBottomBathroom;
    json["pumpOnTimeMiddleBathroom"] = pumpOnTimeMiddleBathroom;
    json["pumpOnTimeUpKitchen"] = pumpOnTimeUpKitchen;
    json["pumpOnTimeUpBathroom"] = pumpOnTimeUpBathroom;
    json["pumpDelayTime"] = pumpDelayTime;
    json["pumpLowTemperatureGuard"] = pumpLowTemperatureGuard;
    json["pumpOnTimeGuard"] = pumpOnTimeGuard;
    json["pumpAutoMode"] = pumpAutoMode;
    json["pumpEnabled"] = pumpEnabled;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["nightStartHour"] = "Godzina rozpoczęcia trybu nocnego (w trybie nocnym pompa jest wyłączona niezależnie od innych waronkow)";
    json["nightEndHour"] = "Godzina zakończenia trybu nocnego (w trybie nocnym pompa jest wyłączona niezależnie od innych warunkow)";
    json["workingTemperatureReadDelay"] = "Opóźnienie odczytu temperatury gdy pompa pracuje w sekundach";
    json["idleTemperatureReadDelay"] = "Opóźnienie odczytu temperatury gdy pompa nie pracuje w sekundach";
    json["pumpOffTemperature"] = "Temperatura przy której pompa się wyłącza (optymalizacja pracy pompy, nie pompujemy niepotrzebnie jeśli woda jest już ciepła)";
    json["pumpOnTimeBottomBathroom"] = "Czas pracy pompy przy wykryciu ruchu na czujniku dolnym w sekundach (łazienka)";
    json["pumpOnTimeMiddleBathroom"] = "Czas pracy pompy przy wykryciu ruchu na czujniku środkowym w sekundach (łazienka)";
    json["pumpOnTimeUpKitchen"] = "Czas pracy pompy przy wykryciu ruchu na czujniku górnym w sekundach (kuchnia)";
    json["pumpOnTimeUpBathroom"] = "Czas pracy pompy przy wykryciu ruchu na czujniku górnym w sekundach (łazienka)";
    json["pumpDelayTime"] = "Minimalny czas pomiędzy kolejnymi uruchomieniami pompy w sekundach (optymalizacja pracy pompy, nie włączamy pompy zbyt często)";
    json["pumpLowTemperatureGuard"] = "Temperatura przy której pompa się włącza, zabezpieczenie przed zamarznięciem (jeśli temperatura spadnie poniżej tej wartości, pompa zostanie włączona na czas określony w pumpOnTimeGuard)";
    json["pumpOnTimeGuard"] = "Czas pracy pompy przy włączeniu przez zabezpieczenie przed zamarznięciem w sekundach (pumpLowTemperatureGuard)";
    json["pumpAutoMode"] = "Tryb automatyczny pompy (jeśli wyłączony, pompa może być sterowana ręcznie)";
    json["pumpEnabled"] = "Czy pompa jest włączona (jeśli pumpAutoMode jest wyłączony, można ręcznie włączyć lub wyłączyć pompę)";
    return json;
  }

  bool updateParams(AsyncWebServerRequest* request) {
    bool hasChanges = false;
    if (request->hasParam("nightStartHour")) {
      nightStartHour = request->getParam("nightStartHour")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("nightEndHour")) {
      nightEndHour = request->getParam("nightEndHour")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("workingTemperatureReadDelay")) {
      workingTemperatureReadDelay = request->getParam("workingTemperatureReadDelay")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("idleTemperatureReadDelay")) {
      idleTemperatureReadDelay = request->getParam("idleTemperatureReadDelay")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("pumpOffTemperature")) {
      pumpOffTemperature = request->getParam("pumpOffTemperature")->value().toFloat();
      hasChanges = true;
    }
    if (request->hasParam("pumpOnTimeBottomBathroom")) {
      pumpOnTimeBottomBathroom = request->getParam("pumpOnTimeBottomBathroom")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("pumpOnTimeMiddleBathroom")) {
      pumpOnTimeMiddleBathroom = request->getParam("pumpOnTimeMiddleBathroom")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("pumpOnTimeUpKitchen")) {
      pumpOnTimeUpKitchen = request->getParam("pumpOnTimeUpKitchen")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("pumpOnTimeUpBathroom")) {
      pumpOnTimeUpBathroom = request->getParam("pumpOnTimeUpBathroom")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("pumpDelayTime")) {
      pumpDelayTime = request->getParam("pumpDelayTime")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("pumpLowTemperatureGuard")) {
      pumpLowTemperatureGuard = request->getParam("pumpLowTemperatureGuard")->value().toFloat();
      hasChanges = true;
    }
    if (request->hasParam("pumpOnTimeGuard")) {
      pumpOnTimeGuard = request->getParam("pumpOnTimeGuard")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("pumpAutoMode")) {
      pumpAutoMode = request->getParam("pumpAutoMode")->value().compareTo("true") == 0;
      hasChanges = true;
    }
    if (request->hasParam("pumpEnabled")) {
      pumpEnabled = request->getParam("pumpEnabled")->value().compareTo("true") == 0;
      hasChanges = true;
    }
    return hasChanges;
  }

};

PumpConfig pumpConfig = PumpConfig();

struct VentConfig {
  public:
  int periodicVentilationDelay;
  int periodicVentilationTime;
  int humidityReadDelay;
  float highHumidityThreshold;
  float veryHighHumidityThreshold;
  float humidityHysteresis;
  int maxWorkTime;
  int minDelayTime;
  int nightStartHour;
  int nightEndHour;
  bool useHighGear;
  bool ventAutoMode;
  bool ventEnabled;

  VentConfig() : periodicVentilationDelay(3 * 60 * 60), periodicVentilationTime(0), humidityReadDelay(30), highHumidityThreshold(55), veryHighHumidityThreshold(65), humidityHysteresis(5), maxWorkTime(30 * 60), minDelayTime(60 * 60), nightStartHour(23), nightEndHour(6), useHighGear(false), ventAutoMode(true), ventEnabled(false) {}
  ~VentConfig() {}

  Json getJson() {
    Json json;
    json["periodicVentilationDelay"] = periodicVentilationDelay;
    json["periodicVentilationTime"] = periodicVentilationTime;
    json["humidityReadDelay"] = humidityReadDelay;
    json["highHumidityThreshold"] = highHumidityThreshold;
    json["veryHighHumidityThreshold"] = veryHighHumidityThreshold;
    json["humidityHysteresis"] = humidityHysteresis;
    json["maxWorkTime"] = maxWorkTime;
    json["minDelayTime"] = minDelayTime;
    json["nightStartHour"] = nightStartHour;
    json["nightEndHour"] = nightEndHour;
    json["useHighGear"] = useHighGear;
    json["ventAutoMode"] = ventAutoMode;
    json["ventEnabled"] = ventEnabled;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["periodicVentilationDelay"] = "Częstotliwość okresowej wentylacji w sekundach";
    json["periodicVentilationTime"] = "Czas trwania okresowej wentylacji w sekundach";
    json["humidityReadDelay"] = "Częstotliwość odczytu wilgotności w sekundach";
    json["highHumidityThreshold"] = "Próg wilgotności powyżej którego włączamy wentylacje";
    json["veryHighHumidityThreshold"] = "Próg wilgotności powyżej którego włączamy wentylacje bez wzgledu na ograniczenia czasowe (maxWorkTime i minDelayTime)";
    json["humidityHysteresis"] = "Histereza dla wilgotności (różnica pomiędzy progiem włączania i wyłączania wentylacji)";
    json["maxWorkTime"] = "Maksymalny czas pracy wentylacji w sekundach (jeśli wentylacja jest włączona, to po tym czasie zostanie wyłączona, jeśli jest ponizej highHumidityThreshold)";
    json["minDelayTime"] = "Minimalny czas przerwy pomiędzy kolejnymi uruchomieniami wentylacji w sekundach (jeśli wentylacja została wyłączona, to nie może być ponownie włączona przed upływem tego czasu, chyba że wilgotność przekroczy highHumidityThreshold)";
    json["nightStartHour"] = "Godzina rozpoczęcia trybu nocnego (w trybie nocnym wentylacja jest wyłączona niezależnie od wilgotności)";
    json["nightEndHour"] = "Godzina zakończenia trybu nocnego (w trybie nocnym wentylacja jest wyłączona niezależnie od wilgotności)";
    json["useHighGear"] = "Czy wentylacja ma pracować na wysokich obrotach";
    json["ventAutoMode"] = "Tryb automatyczny wentylacji (jeśli wyłączony, wentylacja może być sterowana ręcznie)";
    json["ventEnabled"] = "Czy wentylacja jest włączona (jeśli ventAutoMode jest wyłączony, można ręcznie włączyć lub wyłączyć wentylację)";
    return json;
  }

  bool updateParams(AsyncWebServerRequest* request) {
    bool hasChanges = false;
    if (request->hasParam("periodicVentilationDelay")) {
      periodicVentilationDelay = request->getParam("periodicVentilationDelay")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("periodicVentilationTime")) {
      periodicVentilationTime = request->getParam("periodicVentilationTime")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("humidityReadDelay")) {
      humidityReadDelay = request->getParam("humidityReadDelay")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("highHumidityThreshold")) {
      highHumidityThreshold = request->getParam("highHumidityThreshold")->value().toFloat();
      hasChanges = true;
    }
    if (request->hasParam("veryHighHumidityThreshold")) {
      veryHighHumidityThreshold = request->getParam("veryHighHumidityThreshold")->value().toFloat();
      hasChanges = true;
    }
    if (request->hasParam("humidityHysteresis")) {
      humidityHysteresis = request->getParam("humidityHysteresis")->value().toFloat();
      hasChanges = true;
    }
    if(request->hasParam("maxWorkTime")) {
      maxWorkTime = request->getParam("maxWorkTime")->value().toInt();
      hasChanges = true;
    }
    if(request->hasParam("minDelayTime")) {
      minDelayTime = request->getParam("minDelayTime")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("nightStartHour")) {
      nightStartHour = request->getParam("nightStartHour")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("nightEndHour")) {
      nightEndHour = request->getParam("nightEndHour")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("useHighGear")) {
      useHighGear = request->getParam("useHighGear")->value().compareTo("true") == 0;
      hasChanges = true;
    }
    if (request->hasParam("ventAutoMode")) {
      ventAutoMode = request->getParam("ventAutoMode")->value().compareTo("true") == 0;
      hasChanges = true;
    }
    if (request->hasParam("ventEnabled")) {
      ventEnabled = request->getParam("ventEnabled")->value().compareTo("true") == 0;
      hasChanges = true;
    }
    return hasChanges;
  }

};

VentConfig ventConfig = VentConfig();

struct MoveDetector {
  private:
  int pin;
  String name;
  bool currentState;
  time_t lastDetectionTime;

  bool isMoveDetected() {
    return digitalRead(pin) == HIGH;
  }

  public:
  MoveDetector(int pin, String name) : pin(pin), name(name), currentState(false), lastDetectionTime(0) {}
  ~MoveDetector() {}

  void begin() {
    pinMode(pin, INPUT);
  }

  void update() {
    bool oldState = currentState;
    currentState = isMoveDetected();
    if(oldState != currentState && currentState) {
      lastDetectionTime = DateTime.getTime();
    }
  }

  bool getCurrentState() {
    return currentState;
  }

  String getName() {
    return name;
  }

  time_t getLastDetectionTime() {
    return lastDetectionTime;
  }

  Json getJson() {
    Json json;
    json["name"] = name;
    json["currentState"] = currentState;
    json["lastDetectionTime"] = DateTimeParts::from(lastDetectionTime).toString();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["name"] = "Nazwa czujnika ruchu";
    json["currentState"] = "Aktualny stan czujnika ruchu (true - wykryto ruch, false - brak ruchu)";
    json["lastDetectionTime"] = "Czas ostatniego wykrycia ruchu";
    return json;
  }

};

#define numberOfMoveDetectors 3

MoveDetector bottomMoveDetector = MoveDetector(moveDetectorBottomPin, "Łazienka dół");
MoveDetector middleMoveDetector = MoveDetector(moveDetectorMiddlePin, "Łazienka środek");
MoveDetector upMoveDetector = MoveDetector(moveDetectorUpPin, "Łazienka góra");

MoveDetector* moveDetectors[numberOfMoveDetectors] = {
  &bottomMoveDetector,
  &middleMoveDetector,
  &upMoveDetector
};

struct HumiditySensor {
  private:
  uint8_t pinSDA;
  uint8_t pinSCL;
  SHT4x sht;
  float lastReadHumidity;
  float lastGoodReadHumidity;
  time_t lastReadHumidityTime;
  time_t lastGoodReadHumidityTime;
  time_t lastBadReadHumidityTime;
  bool serialInitialized = false;

  float readHumidity() {
    float humidity = -1.0f;
    delay(100);
    if(serialInitialized && sht.measure() == SHT4X_STATUS_OK && sht.RHcrcOK) {
      humidity = sht.RHtoPercent();
    }
    time_t mTime = DateTime.getTime();
    lastReadHumidityTime = mTime;
    lastReadHumidity = humidity;
    if (isHumidityValid(humidity)) {
      lastGoodReadHumidity = humidity;
      lastGoodReadHumidityTime = mTime;
    } else {
      lastBadReadHumidityTime = mTime;
    }
    return humidity;
  }

  public:
  HumiditySensor(uint8_t pinSDA, uint8_t pinSCL) : pinSDA(pinSDA), pinSCL(pinSCL), sht(), lastReadHumidity(-1.0f), lastGoodReadHumidity(-1.0f), lastReadHumidityTime(0), lastGoodReadHumidityTime(0), lastBadReadHumidityTime(0) {}
  ~HumiditySensor() {}

  void begin() {
    Wire.begin(pinSDA, pinSCL);
    sht.setChipType(SHT4X_CHIPTYPE_A);
    sht.setMode(SHT4X_CMD_MEAS_MED_PREC);
    serialInitialized = sht.checkSerial() == SHT4X_STATUS_OK;
  }

  float getLastHumidity() {
    return lastReadHumidity;
  }

  float getLastGoodHumidity() {
    return lastGoodReadHumidity;
  }

  time_t getLastGoodHumidityReadTime() {
    return lastGoodReadHumidityTime;
  }

  time_t getLastHumidityReadTime() {
    return lastReadHumidityTime;
  }

  float readHumidityIfNeeded() {
    if(DateTime.getTime() - lastReadHumidityTime < ventConfig.humidityReadDelay) {
      return lastReadHumidity;
    }
    return readHumidity();
  }

  bool isHumidityValid(float humidity) {
    return !isnan(humidity) && humidity >= 0.0 && humidity <= 100.0;
  }

    Json getJson() {
    Json json;
    json["lastHumidity"] = getLastHumidity();
    json["lastHumidityReadTime"] = DateTimeParts::from(lastReadHumidityTime).toString();
    json["lastGoodHumidity"] = getLastGoodHumidity();
    json["lastGoodHumidityReadTime"] = DateTimeParts::from(lastGoodReadHumidityTime).toString();
    json["lastBadHumidityReadTime"] = DateTimeParts::from(lastBadReadHumidityTime).toString();
    json["serialInitialized"] = serialInitialized;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["lastHumidity"] = "Ostatnia odczytana wilgotność powietrza w procentach";
    json["lastHumidityReadTime"] = "Czas ostatniego odczytu wilgotności powietrza";
    json["lastGoodHumidity"] = "Ostatnia poprawna odczytana wilgotność powietrza w procentach";
    json["lastGoodHumidityReadTime"] = "Czas ostatniego poprawnego odczytu wilgotności powietrza";
    json["lastBadHumidityReadTime"] = "Czas ostatniego niepoprawnego odczytu wilgotności powietrza";
    json["serialInitialized"] = "Czy komunikacja z czujnikiem wilgotności została poprawnie zainicjalizowana";
    return json;
  }

};

HumiditySensor humiditySensor = HumiditySensor(humidityPinSDA, humidityPinSCL);

struct TemperatureSensor {
  private:
  OneWire oneWire;
  DallasTemperature sensors;
  float lastReadTemperature;
  float lastGoodReadTemperature;
  time_t lastReadTemperatureTime;
  time_t lastGoodReadTemperatureTime;
  time_t lastBadReadTemperatureTime;

  float readTemperature() {
    sensors.requestTemperatures(); 
    delay(750);
    float mTemperature = sensors.getTempCByIndex(0);
    float temperature = DEVICE_DISCONNECTED_C;
    if(isTemperatureValid(mTemperature)) {
      temperature = mTemperature;
    }
    time_t mTime = DateTime.getTime();
    lastReadTemperatureTime = mTime;
    lastReadTemperature = temperature;
    if (isTemperatureValid(temperature)) {
      lastGoodReadTemperature = temperature;
      lastGoodReadTemperatureTime = mTime;
    } else {
      lastBadReadTemperatureTime = mTime;
    }
    return temperature;
  }

  public:
  TemperatureSensor(uint8_t pin) : oneWire(pin), sensors(&oneWire), lastReadTemperature(DEVICE_DISCONNECTED_C), lastGoodReadTemperature(DEVICE_DISCONNECTED_C), lastReadTemperatureTime(0), lastGoodReadTemperatureTime(0), lastBadReadTemperatureTime(0) {}
  ~TemperatureSensor() {}

  void begin() {
    sensors.begin();
  }

  float getLastTemperature() {
    return lastReadTemperature;
  }

  float getLastGoodTemperature() {
    return lastGoodReadTemperature;
  }

  time_t getLastGoodTemperatureReadTime() {
    return lastGoodReadTemperatureTime;
  }

  time_t getLastTemperatureReadTime() {
    return lastReadTemperatureTime;
  }

  float readWorkingTemperatureIfNeeded() {
    if(DateTime.getTime() - lastReadTemperatureTime < pumpConfig.workingTemperatureReadDelay) {
      return lastReadTemperature;
    }
    return readTemperature();
  }

  float readIdleTemperatureIfNeeded() {
    if(DateTime.getTime() - lastReadTemperatureTime < pumpConfig.idleTemperatureReadDelay) {
      return lastReadTemperature;
    }
    return readTemperature();
  }

  bool isTemperatureValid(float temperature) {
    return !isnan(temperature) && temperature > DEVICE_DISCONNECTED_C && temperature != 85.0f;
  }

  Json getJson() {
    Json json;
    json["lastTemperature"] = getLastTemperature();
    json["lastTemperatureReadTime"] = DateTimeParts::from(lastReadTemperatureTime).toString();
    json["lastGoodTemperature"] = getLastGoodTemperature();
    json["lastGoodTemperatureReadTime"] = DateTimeParts::from(lastGoodReadTemperatureTime).toString();
    json["lastBadTemperatureReadTime"] = DateTimeParts::from(lastBadReadTemperatureTime).toString();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["lastTemperature"] = "Ostatnia odczytana temperatura w stopniach Celsjusza";
    json["lastTemperatureReadTime"] = "Czas ostatniego odczytu temperatury";
    json["lastGoodTemperature"] = "Ostatnia poprawna odczytana temperatura w stopniach Celsjusza";
    json["lastGoodTemperatureReadTime"] = "Czas ostatniego poprawnego odczytu temperatury";
    json["lastBadTemperatureReadTime"] = "Czas ostatniego niepoprawnego odczytu temperatury";
    return json;
  }

};

TemperatureSensor temperatureSensor = TemperatureSensor(temperaturePin);

struct VentStatus {
  private:

  public:
  VentStatus() {}
  ~VentStatus() {}

  void setVentMode(bool ventEnabled) {
    ventSwitch.setState(ventEnabled);
  }

  void setGear(bool highGear) {
    ventGearSwitch.setState(highGear);
  }

  bool isEnabled() {
    return ventSwitch.isOn();
  }

  bool isHighGear() {
    return ventGearSwitch.isOn();
  }

  Json getJson() {
    Json json;
    json["ventEnabled"] = isEnabled();
    json["ventHighGear"] = isHighGear();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["enabled"] = "Czy wentylator jest włączony";
    json["highGear"] = "Czy wentylator pracuje na wysokich obrotach";
    return json;
  }

};

VentStatus ventStatus = VentStatus();

struct HeaterConfig {
  public:
  float minHourProduction200;
  int numberOfHours200;
  bool use2Heaters200;
  int numberOfHours140;
  float minHourProduction140;
  int nightHours[nightHoursSize];

  HeaterConfig() : minHourProduction200(2.8), numberOfHours200(3), use2Heaters200(false), numberOfHours140(5), minHourProduction140(2.5) {
    for (int i = 0; i < nightHoursSize; i++) {
      nightHours[i] = 0;
    }
    int defaults[] = {13, 22, 5, 14, 23, 4, 3, 2, 1};
    int defaultsCount = 9;
    for (int i = 0; i < defaultsCount; i++) {
      nightHours[i] = defaults[i];
    }
  }
  ~HeaterConfig() {}

  Json getJson() {
    Json json;
    json["minHourProduction200"] = minHourProduction200;
    json["numberOfHours200"] = numberOfHours200;
    json["use2Heaters200"] = use2Heaters200;
    json["numberOfHours140"] = numberOfHours140;
    json["minHourProduction140"] = minHourProduction140;
    String nightHoursString = "";
    for (int i = 0; i < nightHoursSize; i++) {
      int nightHour = nightHours[i];
      if(nightHour > 0) {
        nightHoursString += String(nightHours[i]);
        if (i < nightHoursSize - 1 && nightHours[i + 1] != -1) {
          nightHoursString += ",";
        }
      }
    }
    json["nightHours"] = nightHoursString;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["minHourProduction200"] = "Minimalna produkcja energii w kWh przy której włączamy grzanie zbiornika 200L (jeśli produkcja jest mniejsza, grzanie nie zostanie włączone)";
    json["numberOfHours200"] = "Liczba godzin w ciągu doby, przez które chcemy grzać zbiornik 200L (godziny z największą produkcją energii)";
    json["use2Heaters200"] = "Czy używamy dwóch grzałek do zbiornika 200L (jeśli tak, to obie grzałki będą włączane jednocześnie, jeśli nie, to grzałki będą włączane naprzemiennie w zależności od parzystości dnia w roku)";
    json["numberOfHours140"] = "Liczba godzin w ciągu doby, przez które chcemy grzać zbiornik 140L (godziny z największą produkcją energii)";
    json["minHourProduction140"] = "Minimalna produkcja energii w kWh przy której włączamy grzanie zbiornika 140L (jeśli produkcja jest mniejsza, grzanie nie zostanie włączone)";
    json["nightHours"] = "Godziny nocne, w których grzejemy zbiornik 200L w przypadku gdy produkcja energii jest niska (oddzielone przecinkami).";
    return json;
  }

  bool updateParams(AsyncWebServerRequest* request) {
    bool hasChanges = false;
    if(request->hasParam("nightHours")) {
      String nightHoursString = request->getParam("nightHours")->value();
      for (int i = 0; i < nightHoursSize; i++) {
        nightHours[i] = 0;
      }
      int index = 0;
      int start = 0;

      while (index < nightHoursSize) {
        int separatorIndex = nightHoursString.indexOf(',', start);

        if (separatorIndex == -1) {
          nightHours[index++] = nightHoursString.substring(start).toInt();
          break;
        }

        nightHours[index++] = nightHoursString.substring(start, separatorIndex).toInt();
        start = separatorIndex + 1;
      }
      hasChanges = true;
    }
    if (request->hasParam("minHourProduction200")) {
      minHourProduction200 = request->getParam("minHourProduction200")->value().toFloat();
      hasChanges = true;
    }
    if (request->hasParam("minHourProduction140")) {
      minHourProduction140 = request->getParam("minHourProduction140")->value().toFloat();
      hasChanges = true;
    }
    if (request->hasParam("numberOfHours200")) {
      numberOfHours200 = request->getParam("numberOfHours200")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("numberOfHours140")) {
      numberOfHours140 = request->getParam("numberOfHours140")->value().toInt();
      hasChanges = true;
    }
    if (request->hasParam("use2Heaters200")) {
      use2Heaters200 = request->getParam("use2Heaters200")->value().compareTo("true") == 0;
      hasChanges = true;
    }
    return hasChanges;
}

};

HeaterConfig heaterConfig = HeaterConfig();

struct StorageConfig {
  public:
  uint32_t mStorageStartGuard;
  HeaterConfig heaterConfig;
  PumpConfig pumpConfig;
  VentConfig ventConfig;
  uint32_t mStorageEndGuard;
  StorageConfig(HeaterConfig heaterConfig, PumpConfig pumpConfig, VentConfig ventConfig) : mStorageStartGuard(storageStartGuard), heaterConfig(heaterConfig), pumpConfig(pumpConfig), ventConfig(ventConfig), mStorageEndGuard(storageEndGuard) {}
  ~StorageConfig() {}

  bool isValid() {
    return mStorageStartGuard == storageStartGuard && mStorageEndGuard == storageEndGuard;
  }

};

struct HourProductionPlan {
  private:
  int hour;
  float production;
  bool heater200Enabled1;
  bool heater200Enabled2;
  bool heater140Enabled;

  public:
  HourProductionPlan() : hour(0), production(0), heater200Enabled1(false), heater200Enabled2(false), heater140Enabled(false) {}
  HourProductionPlan(int hour, float production) : hour(hour), production(production), heater200Enabled1(false), heater200Enabled2(false), heater140Enabled(false) {}
  ~HourProductionPlan() {}

  void setHeater200Enabled1(bool heater200Enabled1) {
      this->heater200Enabled1 = heater200Enabled1;
  }

  void setHeater200Enabled2(bool heater200Enabled2) {
    this->heater200Enabled2 = heater200Enabled2;
  }

  void setHeater140Enabled(bool heater140Enabled) {
    this->heater140Enabled = heater140Enabled;
  }

  void enableHeater140() {
    setHeater140Enabled(true);
  }

  void enableHeater200() {
    if (heaterConfig.use2Heaters200) {
      setHeater200Enabled1(true);
      setHeater200Enabled2(true);
    } else {
      if(DateTime.getParts().getYearDay() % 2 == 0) {
        setHeater200Enabled1(true);
      } else {
        setHeater200Enabled2(true);
      }
    }
  }

  bool getHeater200Enabled1() {
    return heater200Enabled1;
  }

  bool getHeater200Enabled2() {
    return heater200Enabled2;
  }

  bool getHeater200EnabledAny() {
    return heater200Enabled1 || heater200Enabled2;
  }

  bool getHeater140Enabled() {
    return heater140Enabled;
  }

  int getHour() {
    return hour;
  }

  float getProduction() {
    return production;
  }

};

int compareHourPredictionByHourAsc(HourProductionPlan &a, HourProductionPlan &b) {
  return a.getHour() - b.getHour();
}

int compareHourPredictionByValueDesc(HourProductionPlan &a, HourProductionPlan &b) {
  if (a.getProduction() > b.getProduction()) {
    return -1;
  } else if (a.getProduction() < b.getProduction()) {
    return 1;
  }
  return 0;
}

struct HeaterStatus {
  private:
  int mHeaterSetHour;

  public:
  HeaterStatus() : mHeaterSetHour(-1) {}
  ~HeaterStatus() {}

  bool shouldUpdateHeaters() {
    return mHeaterSetHour != DateTime.getParts().getHours();
  }

  void setHeaterMode(bool heater200Enabled1, bool heater200Enabled2, bool heater140Enabled) {
    heater200Switch1.setState(heater200Enabled1);
    heater200Switch2.setState(heater200Enabled2);
    heater140Switch.setState(heater140Enabled);
    mHeaterSetHour = DateTime.getParts().getHours();
  }

  void disableAllHeaters() {
    setHeaterMode(false, false, false);
    mHeaterSetHour = -1;
  }

  bool isHeater200Enabled1() {
    return heater200Switch1.isOn();
  }

  bool isHeater200Enabled2() {
    return heater200Switch2.isOn();
  }

  bool isHeater140Enabled() {
    return heater140Switch.isOn();
  }

  Json getJson() {
    Json json;
    json["heater200Enabled1"] = isHeater200Enabled1();
    json["heater200Enabled2"] = isHeater200Enabled2();
    json["heater140Enabled"] = isHeater140Enabled();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["heater200Enabled1"] = "Czy grzałka 1 do zbiornika 200L jest włączona";
    json["heater200Enabled2"] = "Czy grzałka 2 do zbiornika 200L jest włączona";
    json["heater140Enabled"] = "Czy grzałka do zbiornika 140L jest włączona";
    return json;
  }

};

HeaterStatus heaterStatus = HeaterStatus();

struct PumpStatus {
  private:

  public:
  PumpStatus() {}
  ~PumpStatus() {}

  void setPumpMode(bool pumpEnabled) {
    pumpSwitch.setState(pumpEnabled);
  }

  bool isPumpEnabled() {
    return pumpSwitch.isOn();
  }

  Json getJson() {
    Json json;
    json["pumpEnabled"] = isPumpEnabled();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["pumpEnabled"] = "Czy pompa jest włączona";
    return json;
  }

};

PumpStatus pumpStatus = PumpStatus();

struct ProductionPlansManager {
  private:
  String mProductionPlansCreateDate;
  int mProductionPlansDay;
  LinkedList<HourProductionPlan>* mHourProductionPlans;

  String getPanelSetupString() {
    Json result;
    JsonArray panels;
    Json panelsHome;
    Json panelsWarehouse;

    panelsHome.add("kwp", 0.4);
    panelsHome.add("count", 4);
    panelsHome.add("tilt", 70.0);
    panelsHome.add("azimuth", 180.0);

    panelsWarehouse.add("kwp", 0.25);
    panelsWarehouse.add("count", 15);
    panelsWarehouse.add("tilt", 10.0);
    panelsWarehouse.add("azimuth", 180.0);

    panels.push(panelsHome);
    panels.push(panelsWarehouse);

    result.add("latitude", 49.7423);
    result.add("longitude", 18.7370);
    result.add("days", 1);
    result.add("past_days", 0);
    result.add("timezone", "Europe/Warsaw");
    result.add("panels", panels);

    return result.toString();
  }

  String getPredictionsStringOrEmpty() {
    wifiManager.ensureConnection();
    println("Fetching prediction from server");
    String payload = getPanelSetupString();
    HTTPClient http;
    WiFiClient client;
    client.setTimeout(httpReadDelayMillis);
    http.begin(client, "http://solar.datatask.net/api/predict/");
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(payload);
    String result = http.getString();
    http.end();
    if(httpCode != 200 || result.length() == 0) {
      print("Failed to get prediction, error code: ");
      print(String(httpCode));
      print(", Result: ");
      println(result);
    }
    return result;
  }

  String getPredictionsString() {
    String result = "";
    int numberOfRetries = 0;
    while (result.length() == 0 && numberOfRetries <= maxNumberOfRetries) {
      result = getPredictionsStringOrEmpty();
      numberOfRetries++;
      if(result.length() == 0) {
        delay(10000);
      }
    }
    return result;
  }

  int getIndexOfHour(LinkedList<HourProductionPlan>* hourProductionPlans, int hour) {
    for (int i = 0; i < hourProductionPlans->size(); i++) {
      if (hourProductionPlans->get(i).getHour() == hour) {
        return i;
      }
    }
    return -1;
  }

  void fillProductionPlan(LinkedList<HourProductionPlan>* hourProductionPlans) {
    hourProductionPlans->sort(compareHourPredictionByValueDesc);
    int counter140 = 0;
    int counter200 = 0;
    bool heater1 = false;
    for (int i = 0; i < hourProductionPlans->size(); i++) {
      HourProductionPlan hourProductionPlan = hourProductionPlans->get(i);
      if (hourProductionPlan.getProduction() >= heaterConfig.minHourProduction140 && counter140 < heaterConfig.numberOfHours140) {
        hourProductionPlan.enableHeater140();
        counter140++;
      }
      if (hourProductionPlan.getProduction() >= heaterConfig.minHourProduction200 && counter200 < heaterConfig.numberOfHours200) {
        hourProductionPlan.enableHeater200();
        counter200++;
      }
      hourProductionPlans->set(i, hourProductionPlan);
    }
    int missingHours = heaterConfig.numberOfHours200 - counter200;
    int nightHourIndex = 0;
    while (missingHours > 0 && nightHourIndex < nightHoursSize) {
      int hour = heaterConfig.nightHours[nightHourIndex];
      if(hour > 0) {
        int index = getIndexOfHour(hourProductionPlans, hour);
        if(index >= 0) {
          HourProductionPlan hourProductionPlan1 = hourProductionPlans->get(index);
          if(!hourProductionPlan1.getHeater200EnabledAny()) {
            hourProductionPlan1.enableHeater200();
            hourProductionPlans->set(index, hourProductionPlan1);
            missingHours--;
          }
        }
      }
      nightHourIndex++;
    }

    hourProductionPlans->sort(compareHourPredictionByHourAsc);
  }

  LinkedList<HourProductionPlan>* createProductionPlans() {
    Json json = Json(getPredictionsString());
    JsonArray days = json["days"];
    Json day = days[0];
    JsonArray hourPredictions = day["hourly_predictions"];
    LinkedList<HourProductionPlan>* result = new LinkedList<HourProductionPlan>();
    for (int i = 0; i < 24; i++) {
      Json hourPrediction = hourPredictions[i];
      float kwh = hourPrediction["kwh"];
      result->add(HourProductionPlan(i, kwh));
    }
    fillProductionPlan(result);
    return result;
  }
  
  public:
  ProductionPlansManager() : mProductionPlansCreateDate(""), mProductionPlansDay(-1), mHourProductionPlans(nullptr) {}
  ~ProductionPlansManager() {
    if(mHourProductionPlans != nullptr) {
      delete(mHourProductionPlans);
    }
  }

  void clearProductionPlans() {
    if(mHourProductionPlans != nullptr) {
      delete(mHourProductionPlans);
      mHourProductionPlans = nullptr;
    }
    mProductionPlansDay = -1;
    mProductionPlansCreateDate = "";
  }

  bool shouldUpdatePlans() {
    return mProductionPlansDay != DateTime.getParts().getYearDay();
  }

  void updateProductionPlansIfNeeded() {
    if(!shouldUpdatePlans()) {
      return;
    }
    clearProductionPlans();
    mHourProductionPlans = createProductionPlans();
    mProductionPlansDay = DateTime.getParts().getYearDay();
    mProductionPlansCreateDate = DateTime.toISOString();
  }

  Json getJson() {
    Json json;
    if(mHourProductionPlans != nullptr) {
      JsonArray plansJson;
      for (int i = 0; i < mHourProductionPlans->size(); i++) {
        HourProductionPlan plan = mHourProductionPlans->get(i);
        Json planJson;
        planJson["hour"] = plan.getHour();
        planJson["production"] = plan.getProduction();
        planJson["heater200Enabled1"] = plan.getHeater200Enabled1();
        planJson["heater200Enabled2"] = plan.getHeater200Enabled2();
        planJson["heater140Enabled"] = plan.getHeater140Enabled();
        plansJson.push(planJson);
      }
      json["plans"] = plansJson;
      json["createDate"] = mProductionPlansCreateDate;
    }
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["plans"] = "Lista planów produkcji energii i ustawień grzałek dla poszczególnych godzin (jeśli brak planów, to pusta lista)";
    json["createDate"] = "Data i czas utworzenia aktualnych planów produkcji energii";
    Json details;
    details["hour"] = "Godzina (0-23)";
    details["production"] = "Przewidywana produkcja energii w kWh w danej godzinie";
    details["heater200Enabled1"] = "Czy grzałka 1 do zbiornika 200L ma być włączona w danej godzinie";
    details["heater200Enabled2"] = "Czy grzałka 2 do zbiornika 200L ma być włączona w danej godzinie";
    details["heater140Enabled"] = "Czy grzałka do zbiornika 140L ma być włączona w danej godzinie";
    json["details"] = details;
    return json;
  }

  void executeProductionPlan() {
    int hour = DateTime.getParts().getHours();
    int index = getIndexOfHour(mHourProductionPlans, hour);
    if(index >= 0) {
      HourProductionPlan plan = mHourProductionPlans->get(index);
      heaterStatus.setHeaterMode(plan.getHeater200Enabled1(), plan.getHeater200Enabled2(), plan.getHeater140Enabled());
    } else {
      heaterStatus.disableAllHeaters();
    }
  }

  LinkedList<HourProductionPlan>* getProductionPlans() {
    return mHourProductionPlans;
  }

  String getProductionPlansCreateDate() {
    return mProductionPlansCreateDate;
  }

  int getProductionPlansDay() {
    return mProductionPlansDay;
  }

};

ProductionPlansManager productionPlansManager = ProductionPlansManager();

struct PumpManager {
  private:
  int dayOfYear = -1;
  int triggerCounters[numberOfMoveDetectors + 1];
  int pumpOnTime = 0;
  time_t pumpEnableTime = 0;
  time_t pumpDisableTime = 0;
  time_t pumpUptime = 0;
  String statusReason = "unknown";

  void enablePump(int time, String reason) {
    if(pumpStatus.isPumpEnabled()) {
      return;
    }
    statusReason = reason;
    pumpEnableTime = DateTime.getTime();
    pumpOnTime = time;
    pumpStatus.setPumpMode(true);
  }

  void disablePump(String reason) {
    if(!pumpStatus.isPumpEnabled()) {
      return;
    }
    statusReason = statusReason + " | " + reason;
    pumpUptime += DateTime.getTime() - pumpEnableTime;
    pumpDisableTime = DateTime.getTime();
    pumpStatus.setPumpMode(false);
  }

  public:
  void resetStats() {
    disablePump("stats reset");
    dayOfYear = DateTime.getParts().getYearDay();
    for (int i = 0; i < numberOfMoveDetectors; i++) {
      triggerCounters[i] = 0;
    }
    pumpUptime = 0;
    pumpEnableTime = 0;
    pumpDisableTime = 0;
    pumpOnTime = 0;
  }

  void invalidate() {
    if(dayOfYear != DateTime.getParts().getYearDay()) {
      resetStats();
    }

    if(!pumpConfig.pumpAutoMode) {
      if(pumpConfig.pumpEnabled) {
        enablePump(0, "manual mode");
      } else {
        disablePump("manual mode");
      }
      return;
    }

    if(pumpStatus.isPumpEnabled()) {
      float temperature = temperatureSensor.readWorkingTemperatureIfNeeded();
      if(DateTime.getTime() > (pumpEnableTime + pumpOnTime)) {
        disablePump("finished pumping after pumpOnTime");
        return;
      }
      if(temperatureSensor.isTemperatureValid(temperature) && temperature > pumpConfig.pumpOffTemperature && DateTime.getTime() > (pumpEnableTime + 2)) {
        disablePump("temperature above pumpOffTemperature");
        return;
      }
      return;
    }

    if(pumpDisableTime + pumpConfig.pumpDelayTime > DateTime.getTime()) {
      return;
    }

    float temperature = temperatureSensor.readIdleTemperatureIfNeeded();
    if(temperatureSensor.isTemperatureValid(temperature) && temperature < pumpConfig.pumpLowTemperatureGuard) {
      enablePump(pumpConfig.pumpOnTimeGuard, "temperature below pumpLowTemperatureGuard");
      return;
    }

    int currentHour = DateTime.getParts().getHours();
    if(currentHour >= pumpConfig.nightStartHour || currentHour < pumpConfig.nightEndHour) {
      return;
    }

    if(temperatureSensor.isTemperatureValid(temperature) && temperature > pumpConfig.pumpOffTemperature) {
      temperature = temperatureSensor.readWorkingTemperatureIfNeeded();
    }
    if(temperatureSensor.isTemperatureValid(temperature) && temperature > pumpConfig.pumpOffTemperature) {
      pumpDisableTime = DateTime.getTime();
      statusReason = "temperature still above pumpOffTemperature";
      return;
    }

    if(upButtonKitchen.pressed()) {
      int time = pumpConfig.getPumpOnTime(3);
      if(time > 0) {
        triggerCounters[3]++;
        enablePump(time, "Up kitchen button pressed");
        return;
      }
    }
    for (int i = 0; i < numberOfMoveDetectors; i++) {
      MoveDetector* moveDetector = moveDetectors[i];
      if(moveDetector->getCurrentState()) {
        int time = pumpConfig.getPumpOnTime(i);
        if(time > 0) {
          triggerCounters[i]++;
          enablePump(time, "motion detected at " + moveDetector->getName());
          return;
        }
      }
    }
  }

  Json getJson() {
    Json json;
    json["pumpUptime"] = formatSeconds(pumpUptime);
    json["lastPumpEnableTime"] = DateTimeParts::from(pumpEnableTime).toString();
    json["bottomBathroomTriggers"] = triggerCounters[0];
    json["middleBathroomTriggers"] = triggerCounters[1];
    json["upBathroomTriggers"] = triggerCounters[2];
    json["upKitchenTriggers"] = triggerCounters[3];
    json["statusReason"] = statusReason;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["pumpUptime"] = "Czas pracy pompy w sekundach aktualnego dnia";
    json["lastPumpEnableTime"] = "Czas ostatniego włączenia pompy";
    json["bottomBathroomTriggers"] = "Liczba uruchomień pompy z czujnika dolnego (łazienka) aktualnego dnia";
    json["middleBathroomTriggers"] = "Liczba uruchomień pompy z czujnika środkowego (łazienka) aktualnego dnia";
    json["upKitchenTriggers"] = "Liczba uruchomień pompy z przycisku górnego (kuchnia) aktualnego dnia";
    json["upBathroomTriggers"] = "Liczba uruchomień pompy z czujnika górnego (łazienka) aktualnego dnia";
    json["statusReason"] = "Powód aktualnego stanu pompy (np. wykryto ruch, niska temperatura, itp.)";
    return json;
  }

};

PumpManager pumpManager = PumpManager();

struct VentManager {
  private:
  int dayOfYear = -1;
  int ventOnTime = 0;
  time_t ventEnableTime = 0;
  time_t ventDisableTime = 0;
  bool periodicVentilationActive = false;
  bool veryHighHumidityActive = false;
  String statusReason = "unknown";

  void enableVent(String reason) {
    if(ventStatus.isEnabled()) {
      return;
    }
    statusReason = reason;
    ventEnableTime = DateTime.getTime();
    ventStatus.setVentMode(true);
  }

  void disableVent(String reason) {
    if(!ventStatus.isEnabled()) {
      return;
    }
    statusReason = statusReason + " | " + reason;
    periodicVentilationActive = false;
    veryHighHumidityActive = false;
    ventDisableTime = DateTime.getTime();
    ventOnTime += DateTime.getTime() - ventEnableTime;
    ventStatus.setVentMode(false);
    ventStatus.setGear(false);
  }

  public:
  void resetStats() {
    disableVent("stats reset");
    dayOfYear = DateTime.getParts().getYearDay();
    ventOnTime = 0;
    ventEnableTime = 0;
  }

  void invalidate() {
    if(dayOfYear != DateTime.getParts().getYearDay()) {
      resetStats();
    }
    if(ventStatus.isEnabled()) {
      ventStatus.setGear(ventConfig.useHighGear);
    }
    if(!ventConfig.ventAutoMode) {
      if(ventConfig.ventEnabled) {
        enableVent("manual mode");
      } else {
        disableVent("manual mode");
      }
      return;
    }
    int currentHour = DateTime.getParts().getHours();
    if(currentHour >= ventConfig.nightStartHour || currentHour < ventConfig.nightEndHour) {
      disableVent("night mode");
      return;
    }
    if(periodicVentilationActive) {
      if(ventEnableTime + ventConfig.periodicVentilationTime <= DateTime.getTime()) {
        disableVent("periodic ventilation time exceeded");
      } else {
        enableVent("periodic ventilation");
      }
      return;
    }
    if(!ventStatus.isEnabled() && ventDisableTime + ventConfig.periodicVentilationDelay < DateTime.getTime() && ventConfig.periodicVentilationTime > 0) {
      int mHours = ventConfig.periodicVentilationDelay / 3600;
      if(currentHour + mHours < ventConfig.nightStartHour && currentHour - mHours >= ventConfig.nightEndHour) {
        periodicVentilationActive = true;
        enableVent("periodic ventilation");
        return;
      }
    }

    float humidity = humiditySensor.readHumidityIfNeeded();
    if(!humiditySensor.isHumidityValid(humidity)) {
      disableVent("invalid humidity");
      return;
    }

    if(veryHighHumidityActive) {
      if(humidity < ventConfig.veryHighHumidityThreshold - ventConfig.humidityHysteresis) {
        disableVent("humidity below veryHighHumidityThreshold - humidityHysteresis");
      }
      return;
    }

    if(humidity > ventConfig.veryHighHumidityThreshold + ventConfig.humidityHysteresis) {
      veryHighHumidityActive = true;
      if(ventStatus.isEnabled()) {
        statusReason = "humidity over veryHighHumidityThreshold";
      } else {
        enableVent("humidity over veryHighHumidityThreshold + humidityHysteresis");
      }
      return;
    }

    if(humidity < ventConfig.highHumidityThreshold - ventConfig.humidityHysteresis) {
      disableVent("humidity below highHumidityThreshold - humidityHysteresis");
      return;
    }

    if(ventEnableTime != 0 && ventEnableTime + ventConfig.maxWorkTime < DateTime.getTime()) {
      disableVent("maxWorkTime exceeded");
      return;
    }

    if(humidity > ventConfig.highHumidityThreshold + ventConfig.humidityHysteresis && ventDisableTime + ventConfig.minDelayTime < DateTime.getTime()) {
      enableVent("humidity above highHumidityThreshold + humidityHysteresis and within time limits");
      return;
    }

  }

  Json getJson() {
    Json json;
    json["ventOnTime"] = formatSeconds(ventOnTime);
    json["lastVentEnableTime"] = DateTimeParts::from(ventEnableTime).toString();
    json["lastVentDisableTime"] = DateTimeParts::from(ventDisableTime).toString();
    json["statusReason"] = statusReason;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["ventOnTime"] = "Czas pracy wentylacji w sekundach aktualnego dnia";
    json["lastVentEnableTime"] = "Czas ostatniego włączenia wentylacji";
    json["lastVentDisableTime"] = "Czas ostatniego wyłączenia wentylacji";
    json["statusReason"] = "Powód aktualnego stanu wentylacji (np. wysoka wilgotność, tryb nocny, itp.)";
    return json;
  }

};

VentManager ventManager = VentManager();

void updateLocalStorage() {
  EEPROM.put(0, StorageConfig(heaterConfig, pumpConfig, ventConfig));
  EEPROM.commit();
}

Json getDocResponse(String path, String description) {
  Json statusJson;
  statusJson["path"] = "192.168.100.181/" + path;
  statusJson["method"] = "GET";
  statusJson["description"] = description;
  return statusJson;
}

void setupServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    JsonArray edpointsJson;
    edpointsJson.push(getDocResponse("status", "Returns current status, config and plans"));
    edpointsJson.push(getDocResponse("docs", "Description of all status fields and config parameters"));
    edpointsJson.push(getDocResponse("heater_config", "Allow update of heater config parameters by using query params. For example /heater_config?minHourProduction200=1.43"));
    edpointsJson.push(getDocResponse("pump_config", "Allow update of pump config parameters by using query params. For example /pump_config?pumpOffTemperature=1.43"));
    edpointsJson.push(getDocResponse("vent_config", "Allow update of vent config parameters by using query params. For example /vent_config?bottomHumidityThreshold=55.5"));
    edpointsJson.push(getDocResponse("force_update_plans", "Forces update of production plans"));
    edpointsJson.push(getDocResponse("reset_config", "Reset all config to default values"));
    edpointsJson.push(getDocResponse("reset_stats", "Reset all statistics"));

    Json resultJson;
    resultJson["edpoints"] = edpointsJson;

    request->send(200, "application/json", resultJson.toString());
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    Json resultJson;
    resultJson["heaterConfig"] = heaterConfig.getJson();
    resultJson["heaterStatus"] = heaterStatus.getJson();
    resultJson["pumpConfig"] = pumpConfig.getJson();
    resultJson["pumpStatus"] = pumpStatus.getJson();
    resultJson["pumpStatistics"] = pumpManager.getJson();
    resultJson["ventConfig"] = ventConfig.getJson();
    resultJson["ventStatus"] = ventStatus.getJson();
    resultJson["ventStatistics"] = ventManager.getJson();
    resultJson["other"] = otherParams.getJson();
    resultJson["humiditySensor"] = humiditySensor.getJson();
    resultJson["temperatureSensor"] = temperatureSensor.getJson();
    resultJson["upMoveDetector"] = upMoveDetector.getJson();
    resultJson["middleMoveDetector"] = middleMoveDetector.getJson();
    resultJson["bottomMoveDetector"] = bottomMoveDetector.getJson();
    resultJson["productionPlans"] = productionPlansManager.getJson();

    request->send(200, "application/json", resultJson.toString());
  });

  server.on("/docs", HTTP_GET, [](AsyncWebServerRequest *request) {
    Json resultJson;
    resultJson["heaterConfig"] = heaterConfig.getDocumentationJson();
    resultJson["heaterStatus"] = heaterStatus.getDocumentationJson();
    resultJson["pumpConfig"] = pumpConfig.getDocumentationJson();
    resultJson["pumpStatus"] = pumpStatus.getDocumentationJson();
    resultJson["pumpStatistics"] = pumpManager.getDocumentationJson();
    resultJson["ventConfig"] = ventConfig.getDocumentationJson();
    resultJson["ventStatus"] = ventStatus.getDocumentationJson();
    resultJson["ventStatistics"] = ventManager.getDocumentationJson();
    resultJson["other"] = otherParams.getDocumentationJson();
    resultJson["humiditySensor"] = humiditySensor.getDocumentationJson();
    resultJson["temperatureSensor"] = temperatureSensor.getDocumentationJson();
    resultJson["upMoveDetector"] = upMoveDetector.getDocumentationJson();
    resultJson["middleMoveDetector"] = middleMoveDetector.getDocumentationJson();
    resultJson["bottomMoveDetector"] = bottomMoveDetector.getDocumentationJson();
    resultJson["productionPlans"] = productionPlansManager.getDocumentationJson();

    request->send(200, "application/json", resultJson.toString());
  });

  server.on("/pump_config", HTTP_GET, [](AsyncWebServerRequest *request){
    if (pumpConfig.updateParams(request)) {
      updateLocalStorage();
    }
    request->send(200, "application/json", pumpConfig.getJson().toString());
  });

  server.on("/heater_config", HTTP_GET, [](AsyncWebServerRequest *request){
    if (heaterConfig.updateParams(request)) {
      updateLocalStorage();
    }
    request->send(200, "application/json", heaterConfig.getJson().toString());
  });

  server.on("/vent_config", HTTP_GET, [](AsyncWebServerRequest *request){
    if (ventConfig.updateParams(request)) {
      updateLocalStorage();
    }
    request->send(200, "application/json", ventConfig.getJson().toString());
  });

  server.on("/force_update_plans", HTTP_GET, [](AsyncWebServerRequest *request){
    heaterStatus.disableAllHeaters();
    productionPlansManager.clearProductionPlans();
    Json resultJson;
    resultJson["message"] = "Production plans will update soon";
    request->send(200, "application/json", resultJson.toString());
  });

  server.on("/reset_config", HTTP_GET, [](AsyncWebServerRequest *request){
    ventConfig = VentConfig();
    pumpConfig = PumpConfig();
    heaterConfig = HeaterConfig();
    updateLocalStorage();
    Json resultJson;
    resultJson["message"] = "Config reseted to default values";
    request->send(200, "application/json", resultJson.toString());
  });

  server.on("/reset_stats", HTTP_GET, [](AsyncWebServerRequest *request){
    ventManager.resetStats();
    pumpManager.resetStats();
    Json resultJson;
    resultJson["message"] = "Stats reseted";
    request->send(200, "application/json", resultJson.toString());
  });

  server.onNotFound([](AsyncWebServerRequest *request){
    Json resultJson;
    resultJson["message"] = "Path not found, check 192.168.100.181/ for availible paths";
    request->send(404, "application/json", resultJson.toString());
  });

  server.begin();
}

void initializeLocalStorage() {
  StorageConfig storageConfig = StorageConfig(heaterConfig, pumpConfig, ventConfig);
  EEPROM.begin(sizeof(StorageConfig));
  EEPROM.get(0, storageConfig);
  if(storageConfig.isValid()) {
    pumpConfig = storageConfig.pumpConfig;
    heaterConfig = storageConfig.heaterConfig;
    ventConfig = storageConfig.ventConfig;
  }
}

void setup() {
  heater140Switch.begin();
  heater200Switch1.begin();
  heater200Switch2.begin();
  pumpSwitch.begin();
  ventSwitch.begin();
  ventGearSwitch.begin();
  bottomMoveDetector.begin();
  middleMoveDetector.begin();
  upMoveDetector.begin();
  upButtonKitchen.attach(moveDetectorUpPin, INPUT);
  upButtonKitchen.interval(5); 
  upButtonKitchen.setPressedState(LOW); 
  Serial.begin(9600);
  while (!Serial)
    delay(100);
  delay(1000);
  temperatureSensor.begin();
  humiditySensor.begin();
  initializeLocalStorage();
  wifiManager.begin();
  setupServer();
  println("Finished setup waiting 120 secs to start loop");
  delay(120000);
}

void loop() {
  wifiManager.invalidate();
  upButtonKitchen.update();
  bottomMoveDetector.update();
  middleMoveDetector.update();
  upMoveDetector.update();
  pumpManager.invalidate();
  ventManager.invalidate();
  if(productionPlansManager.shouldUpdatePlans()) {
    heaterStatus.disableAllHeaters();
    productionPlansManager.updateProductionPlansIfNeeded();
  }
  if(heaterStatus.shouldUpdateHeaters()) {
    productionPlansManager.executeProductionPlan();
  }
  delay(loopDelay);
}