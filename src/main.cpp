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

#define version 5

const int maxNumberOfRetries = 10;
const int httpReadDelay = 30000;
time_t lastInternetInterruption = 0;

#define humidityPinSDA 17
#define humidityPinSCL 19
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

void setPinOutput(int pin, bool value) {
  if (value) {
    digitalWrite(pin, HIGH);
  } else {
    digitalWrite(pin, LOW);
  }
  delay(1000);
}

void print(String message) {
  Serial.print(message);
}

void println(String message) {
  Serial.println(message);
}

Bounce2::Button upButtonKitchen = Bounce2::Button();
AsyncWebServer server(80);

bool isWifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

void setupInternetConnectionIfNeeded() {
  if(isWifiConnected()) {
    return;
  }
  IPAddress localIP(192, 168, 100, 181);
  IPAddress gateway(192,168,100,1); 
  IPAddress subnet(255,255,0,0); 
  IPAddress primaryDNS(8, 8, 8, 8);
  IPAddress secondaryDNS(8, 8, 4, 4); 
  if (!WiFi.config(localIP, gateway, subnet, primaryDNS, secondaryDNS)) {  
    while (1) {
        Serial.println("STA Failed to configure");
        delay(10000);
    }
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin("Dom", "123456789a");
  print("Connecting to WiFi");
  // Wait for connection
  while (!isWifiConnected()) {
    delay(100);
    print(".");
  }
  lastInternetInterruption = DateTime.getTime();
  println(" Connected!");

  print("IP address: ");
  println(WiFi.localIP().toString());
}

void setupLocalTimefNeeded() {
  if(DateTime.isTimeValid()) {
    return;
  }
  setupInternetConnectionIfNeeded();
  DateTime.setTimeZone("CET-1CEST,M3.5.0,M10.5.0/3");
  println("Fetching current time from server");
  while(!DateTime.isTimeValid()) {
    if(!DateTime.begin(20000)) {
      println("Failed to fetch current time from server");
      delay(httpReadDelay);
    }
  }
  print("Fetched current time from server ");
  println(DateTime.toString());
}

struct OtherParams {
  public:
  OtherParams() {}
  ~OtherParams() {}

  Json getJson() {
    Json json;
    json["time"] = DateTime.getParts().toString();
    json["bootTime"] = DateTimeParts::from(DateTime.getBootTime()).toString();
    json["lastInternetInterruption"] = DateTimeParts::from(lastInternetInterruption).toString();
    json["version"] = version;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["time"] = "Aktualny czas systemowy";
    json["bootTime"] = "Czas ostatniego uruchomienia urządzenia";
    json["lastInternetInterruption"] = "Czas ostatniej utraty łączności z internetem";
    json["version"] = "Wersja oprogramowania urządzenia";
    return json;
  }

};

OtherParams otherParams = OtherParams();

struct PumpConfig {
  public:
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

  PumpConfig() : workingTemperatureReadDelay(2), idleTemperatureReadDelay(120), pumpOffTemperature(30), pumpOnTimeBottomBathroom(0), pumpOnTimeMiddleBathroom(120), pumpOnTimeUpKitchen(0), pumpOnTimeUpBathroom(0), pumpDelayTime(60 * 30), pumpLowTemperatureGuard(5), pumpOnTimeGuard(120), pumpAutoMode(true), pumpEnabled(false) {}
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
  float bottomHumidityThreshold;
  float highHumidityThreshold;
  int maxWorkTime;
  int minDelayTime;
  int nightStartHour;
  int nightEndHour;
  bool useHighGear;
  bool ventAutoMode;
  bool ventEnabled;

  VentConfig() : periodicVentilationDelay(3 * 60 * 60), periodicVentilationTime(5 * 60), humidityReadDelay(30), bottomHumidityThreshold(55), highHumidityThreshold(70), maxWorkTime(30 * 60), minDelayTime(60 * 60), nightStartHour(23), nightEndHour(6), useHighGear(false), ventAutoMode(true), ventEnabled(false) {}
  ~VentConfig() {}

  Json getJson() {
    Json json;
    json["periodicVentilationDelay"] = periodicVentilationDelay;
    json["periodicVentilationTime"] = periodicVentilationTime;
    json["humidityReadDelay"] = humidityReadDelay;
    json["bottomHumidityThreshold"] = bottomHumidityThreshold;
    json["highHumidityThreshold"] = highHumidityThreshold;
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
    json["bottomHumidityThreshold"] = "Próg wilgotności poniżej którego wyłączamy wentylację";
    json["highHumidityThreshold"] = "Próg wilgotności powyżej którego włączamy wentylacje bez wzgledu na ograniczenia czasowe (maxWorkTime i minDelayTime)";
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
    if (request->hasParam("bottomHumidityThreshold")) {
      bottomHumidityThreshold = request->getParam("bottomHumidityThreshold")->value().toFloat();
      hasChanges = true;
    }
    if (request->hasParam("highHumidityThreshold")) {
      highHumidityThreshold = request->getParam("highHumidityThreshold")->value().toFloat();
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
  time_t lastChangeTime;

  bool isMoveDetected() {
    return digitalRead(pin) == HIGH;
  }

  public:
  MoveDetector(int pin, String name) : pin(pin), name(name), currentState(false), lastChangeTime(0) {}
  ~MoveDetector() {}

  void begin() {
    pinMode(pin, INPUT);
  }

  void update() {
    bool oldState = currentState;
    currentState = isMoveDetected();
    if(oldState != currentState) {
      lastChangeTime = DateTime.getTime();
    }
  }

  bool getCurrentState() {
    return currentState;
  }

  String getName() {
    return name;
  }

  time_t getLastChangeTime() {
    return lastChangeTime;
  }

  Json getJson() {
    Json json;
    json["name"] = name;
    json["currentState"] = currentState;
    json["lastChangeTime"] = DateTimeParts::from(lastChangeTime).toString();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["name"] = "Nazwa czujnika ruchu";
    json["currentState"] = "Aktualny stan czujnika ruchu (true - wykryto ruch, false - brak ruchu)";
    json["lastChangeTime"] = "Czas ostatniej zmiany stanu czujnika ruchu";
    return json;
  }

};

#define numberOfMoveDetectors 3

MoveDetector bottomMoveDetector = MoveDetector(moveDetectorBottomPin, "Łazienka dół");
MoveDetector middleMoveDetector = MoveDetector(moveDetectorMiddlePin, "Łazienka środek");
MoveDetector upMoveDetector = MoveDetector(moveDetectorUpPin, "Łazienka góra");

MoveDetector moveDetectors[] = {
  bottomMoveDetector,
  middleMoveDetector,
  upMoveDetector
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
    }
    return humidity;
  }

  public:
  HumiditySensor(uint8_t pinSDA, uint8_t pinSCL) : pinSDA(pinSDA), pinSCL(pinSCL), sht(), lastReadHumidity(-1.0f), lastGoodReadHumidity(-1.0f), lastReadHumidityTime(0), lastGoodReadHumidityTime(0) {}
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
    json["lastHumidityReadTime"] = DateTimeParts::from(getLastHumidityReadTime()).toString();
    json["lastGoodHumidity"] = getLastGoodHumidity();
    json["lastGoodHumidityReadTime"] = DateTimeParts::from(getLastGoodHumidityReadTime()).toString();
    json["serialInitialized"] = serialInitialized;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["lastHumidity"] = "Ostatnia odczytana wilgotność powietrza w procentach";
    json["lastHumidityReadTime"] = "Czas ostatniego odczytu wilgotności powietrza";
    json["lastGoodHumidity"] = "Ostatnia poprawna odczytana wilgotność powietrza w procentach";
    json["lastGoodHumidityReadTime"] = "Czas ostatniego poprawnego odczytu wilgotności powietrza";
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
    }
    return temperature;
  }

  public:
  TemperatureSensor(uint8_t pin) : oneWire(pin), sensors(&oneWire), lastReadTemperature(DEVICE_DISCONNECTED_C), lastGoodReadTemperature(DEVICE_DISCONNECTED_C), lastReadTemperatureTime(0), lastGoodReadTemperatureTime(0) {}
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
    json["lastTemperatureReadTime"] = DateTimeParts::from(getLastTemperatureReadTime()).toString();
    json["lastGoodTemperature"] = getLastGoodTemperature();
    json["lastGoodTemperatureReadTime"] = DateTimeParts::from(getLastGoodTemperatureReadTime()).toString();
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["lastTemperature"] = "Ostatnia odczytana temperatura w stopniach Celsjusza";
    json["lastTemperatureReadTime"] = "Czas ostatniego odczytu temperatury";
    json["lastGoodTemperature"] = "Ostatnia poprawna odczytana temperatura w stopniach Celsjusza";
    json["lastGoodTemperatureReadTime"] = "Czas ostatniego poprawnego odczytu temperatury";
    return json;
  }

};

TemperatureSensor temperatureSensor = TemperatureSensor(temperaturePin);

struct VentStatus {
  private:
  bool mEnabled;
  bool mHighGear;

  public:
  VentStatus() : mEnabled(false), mHighGear(false) {}
  ~VentStatus() {}

  void setVentMode(bool ventEnabled, bool highGear) {
    if(mEnabled != ventEnabled) {
      mEnabled = ventEnabled;
      setPinOutput(ventPin, ventEnabled);
    }
    if(mHighGear != highGear) {
      mHighGear = highGear;
      setPinOutput(ventGearPin, highGear);
    }
  }

  bool isEnabled() {
    return mEnabled;
  }

  bool isHighGear() {
    return mHighGear;
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

  HeaterConfig() : minHourProduction200(2.8), numberOfHours200(3), use2Heaters200(false), numberOfHours140(5), minHourProduction140(2.5) {}
  ~HeaterConfig() {}

  Json getJson() {
    Json json;
    json["minHourProduction200"] = minHourProduction200;
    json["numberOfHours200"] = numberOfHours200;
    json["use2Heaters200"] = use2Heaters200;
    json["numberOfHours140"] = numberOfHours140;
    json["minHourProduction140"] = minHourProduction140;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["minHourProduction200"] = "Minimalna produkcja energii w kWh przy której włączamy grzanie zbiornika 200L (jeśli produkcja jest mniejsza, grzanie nie zostanie włączone)";
    json["numberOfHours200"] = "Liczba godzin w ciągu doby, przez które chcemy grzać zbiornik 200L (godziny z największą produkcją energii)";
    json["use2Heaters200"] = "Czy używamy dwóch grzałek do zbiornika 200L (jeśli tak, to obie grzałki będą włączane jednocześnie, jeśli nie, to grzałki będą włączane naprzemiennie w zależności od parzystości dnia w roku)";
    json["numberOfHours140"] = "Liczba godzin w ciągu doby, przez które chcemy grzać zbiornik 140L (godziny z największą produkcją energii)";
    json["minHourProduction140"] = "Minimalna produkcja energii w kWh przy której włączamy grzanie zbiornika 140L (jeśli produkcja jest mniejsza, grzanie nie zostanie włączone)";
    return json;
  }

  bool updateParams(AsyncWebServerRequest* request) {
    bool hasChanges = false;
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
  HeaterConfig heaterConfig;
  PumpConfig pumpConfig;
  VentConfig ventConfig;
  StorageConfig(HeaterConfig heaterConfig, PumpConfig pumpConfig, VentConfig ventConfig) : heaterConfig(heaterConfig), pumpConfig(pumpConfig), ventConfig(ventConfig) {}
  ~StorageConfig() {}
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
  bool mHeater200Enabled1;
  bool mHeater200Enabled2;
  bool mHeater140Enabled;

  public:
  HeaterStatus() : mHeaterSetHour(-1), mHeater200Enabled1(false), mHeater200Enabled2(false), mHeater140Enabled(false) {}
  ~HeaterStatus() {}

  bool shouldUpdateHeaters() {
    return mHeaterSetHour != DateTime.getParts().getHours();
  }

  void setHeaterMode(bool heater200Enabled1, bool heater200Enabled2, bool heater140Enabled) {
    if(mHeater200Enabled1 != heater200Enabled1) {
      mHeater200Enabled1 = heater200Enabled1;
      setPinOutput(heater200Pin1, heater200Enabled1);
    }
    if(mHeater200Enabled2 != heater200Enabled2) {
      mHeater200Enabled2 = heater200Enabled2;
      setPinOutput(heater200Pin2, heater200Enabled2);
    }
    if(mHeater140Enabled != heater140Enabled) {
      mHeater140Enabled = heater140Enabled;
      setPinOutput(heater140Pin, heater140Enabled);
    }
    mHeaterSetHour = DateTime.getParts().getHours();
  }

  void disableAllHeaters() {
    setHeaterMode(false, false, false);
    mHeaterSetHour = -1;
  }

  bool isHeater200Enabled1() {
    return mHeater200Enabled1;
  }

  bool isHeater200Enabled2() {
    return mHeater200Enabled2;
  }

  bool isHeater140Enabled() {
    return mHeater140Enabled;
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
  bool mPumpEnabled;

  public:
  PumpStatus() : mPumpEnabled(false) {}
  ~PumpStatus() {}

  void setPumpMode(bool pumpEnabled) {
    if(mPumpEnabled != pumpEnabled) {
      mPumpEnabled = pumpEnabled;
      setPinOutput(pumpPin, pumpEnabled);
    }
  }

  bool isPumpEnabled() {
    return mPumpEnabled;
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
    setupInternetConnectionIfNeeded();
    println("Fetching prediction from server");
    String payload = getPanelSetupString();
    HTTPClient http;
    WiFiClient client;
    client.setTimeout(httpReadDelay);
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
    int nightHour1 = 14;
    int nightHour2 = 5;
    while (missingHours > 0) {
      println("Filling missing heater200 hour, missing: " + String(missingHours));
      int index1 = getIndexOfHour(hourProductionPlans, nightHour1);
      if(missingHours > 0 && nightHour1 >= 13 && index1 >= 0) {
        HourProductionPlan hourProductionPlan1 = hourProductionPlans->get(index1);
        if(!hourProductionPlan1.getHeater200EnabledAny()) {
          hourProductionPlan1.enableHeater200();
          hourProductionPlans->set(index1, hourProductionPlan1);
          nightHour1--;
          missingHours--;
        }
      }
      int index2 = getIndexOfHour(hourProductionPlans, nightHour2);
      if(missingHours > 0 && nightHour2 >= 0 && index2 >= 0) {
        HourProductionPlan hourProductionPlan2 = hourProductionPlans->get(index2);
        if(!hourProductionPlan2.getHeater200EnabledAny()) {
          hourProductionPlan2.enableHeater200();
          hourProductionPlans->set(index2, hourProductionPlan2);
          nightHour2--;
          missingHours--;
        }
      }
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
      MoveDetector moveDetector = moveDetectors[i];
      if(moveDetector.getCurrentState()) {
        int time = pumpConfig.getPumpOnTime(i);
        if(time > 0) {
          triggerCounters[i]++;
          enablePump(time, "motion detected at " + moveDetector.getName());
          return;
        }
      }
    }
    if(temperatureSensor.isTemperatureValid(temperature) && temperature < pumpConfig.pumpLowTemperatureGuard) {
      enablePump(pumpConfig.pumpOnTimeGuard, "temperature below pumpLowTemperatureGuard");
      return;
    }
  }

  Json getJson() {
    Json json;
    json["pumpUptime"] = int(pumpUptime);
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
  time_t lastVentWorkTime = 0;
  String statusReason = "unknown";

  void enableVent(String reason, bool highGear) {
    if(ventStatus.isEnabled() && ventStatus.isHighGear() == highGear) {
      return;
    }
    if(!ventStatus.isEnabled()) {
      statusReason = reason;
      ventEnableTime = DateTime.getTime();
      ventStatus.setVentMode(true, highGear);
    } else {
      statusReason = reason;
      ventStatus.setVentMode(true, highGear);
    }
  }

  void disableVent(String reason) {
    if(!ventStatus.isEnabled()) {
      return;
    }
    statusReason = statusReason + " | " + reason;
    lastVentWorkTime = DateTime.getTime();
    if(ventEnableTime != 0) {
      ventOnTime += DateTime.getTime() - ventEnableTime;
    }
    ventEnableTime = 0;
    ventStatus.setVentMode(false, false);
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
    bool highGear = ventConfig.useHighGear;
    if(!ventConfig.ventAutoMode) {
      if(ventConfig.ventEnabled) {
        enableVent("manual mode", highGear);
      } else {
        disableVent("manual mode");
      }
      return;
    }
    int currentHour = DateTime.getParts().getHours();
    if(currentHour >= ventConfig.nightStartHour || currentHour <= ventConfig.nightEndHour) {
      disableVent("night mode");
      return;
    }
    if(lastVentWorkTime == 0 || lastVentWorkTime + ventConfig.periodicVentilationDelay > DateTime.getTime()) {
      if(!ventStatus.isEnabled()) {
        enableVent("periodic ventilation", highGear);
      } else if(ventEnableTime + ventConfig.periodicVentilationTime <= DateTime.getTime()) {
        disableVent("periodic ventilation time exceeded");
      }
      return;
    }

    float humidity = humiditySensor.readHumidityIfNeeded();
    if(!humiditySensor.isHumidityValid(humidity)) {
      disableVent("invalid humidity");
      return;
    }

    if(humidity <= ventConfig.bottomHumidityThreshold) {
      disableVent("humidity below bottomHumidityThreshold");
      return;
    }

    if(humidity >= ventConfig.highHumidityThreshold) {
      enableVent("humidity over highHumidityThreshold", highGear);
      return;
    }

    if(ventEnableTime != 0 && ventEnableTime + ventConfig.maxWorkTime <= DateTime.getTime()) {
      disableVent("maxWorkTime exceeded");
      return;
    }

    if(humidity > ventConfig.bottomHumidityThreshold && lastVentWorkTime + ventConfig.minDelayTime > DateTime.getTime()) {
      enableVent("humidity above bottomHumidityThreshold and within time limits", highGear);
      return;
    }

  }

  Json getJson() {
    Json json;
    json["ventOnTime"] = int(ventOnTime);
    json["lastVentEnableTime"] = DateTimeParts::from(ventEnableTime).toString();
    json["lastVentWorkTime"] = DateTimeParts::from(lastVentWorkTime).toString();
    json["statusReason"] = statusReason;
    return json;
  }

  Json getDocumentationJson() {
    Json json;
    json["ventOnTime"] = "Czas pracy wentylacji w sekundach aktualnego dnia";
    json["lastVentEnableTime"] = "Czas ostatniego włączenia wentylacji";
    json["lastVentWorkTime"] = "Czas ostatniej pracy wentylacji (zarejestrowana w momencie wyłączenia)";
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
  setupInternetConnectionIfNeeded();

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
    resultJson["productionPlans"] = productionPlansManager.getJson();
    resultJson["humiditySensor"] = humiditySensor.getJson();
    resultJson["temperatureSensor"] = temperatureSensor.getJson();
    resultJson["upMoveDetector"] = upMoveDetector.getJson();
    resultJson["middleMoveDetector"] = middleMoveDetector.getJson();
    resultJson["bottomMoveDetector"] = bottomMoveDetector.getJson();

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
    resultJson["productionPlans"] = productionPlansManager.getDocumentationJson();
    resultJson["humiditySensor"] = humiditySensor.getDocumentationJson();
    resultJson["temperatureSensor"] = temperatureSensor.getDocumentationJson();
    resultJson["upMoveDetector"] = upMoveDetector.getDocumentationJson();
    resultJson["middleMoveDetector"] = middleMoveDetector.getDocumentationJson();
    resultJson["bottomMoveDetector"] = bottomMoveDetector.getDocumentationJson();

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

void setup() {
  pinMode(heater200Pin1, OUTPUT);
  pinMode(heater200Pin2, OUTPUT);
  pinMode(heater140Pin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(ventPin, OUTPUT);
  pinMode(ventGearPin, OUTPUT);
  bottomMoveDetector.begin();
  middleMoveDetector.begin();
  upMoveDetector.begin();
  digitalWrite(heater200Pin1, LOW);
  digitalWrite(heater200Pin2, LOW);
  digitalWrite(heater140Pin, LOW);
  digitalWrite(pumpPin, LOW);
  digitalWrite(ventPin, LOW);
  digitalWrite(ventGearPin, LOW);
  upButtonKitchen.attach(moveDetectorUpPin, INPUT);
  upButtonKitchen.interval(5); 
  upButtonKitchen.setPressedState(LOW); 
  Serial.begin(9600);
  while (!Serial)
    delay(100);
  temperatureSensor.begin();
  humiditySensor.begin();
  StorageConfig storageConfig = StorageConfig(heaterConfig, pumpConfig, ventConfig);
  EEPROM.begin(sizeof(StorageConfig));
  if(EEPROM.read(0) == 0xFF) {
    println("EEPROM is empty, writing default config");
    EEPROM.put(0, storageConfig);
    EEPROM.commit();
  }
  EEPROM.get(0, storageConfig);
  pumpConfig = storageConfig.pumpConfig;
  heaterConfig = storageConfig.heaterConfig;
  ventConfig = storageConfig.ventConfig;
  
  setupLocalTimefNeeded();
  setupServer();
  println("Finished setup");
}

void loop() {
  upButtonKitchen.update();
  pumpManager.invalidate();
  ventManager.invalidate();
  setupInternetConnectionIfNeeded();
  if(productionPlansManager.shouldUpdatePlans()) {
    heaterStatus.disableAllHeaters();
    productionPlansManager.updateProductionPlansIfNeeded();
  }
  if(heaterStatus.shouldUpdateHeaters()) {
    productionPlansManager.executeProductionPlan();
  }
  delay(1);
}