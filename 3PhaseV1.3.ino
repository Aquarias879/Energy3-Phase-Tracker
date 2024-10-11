#include <WiFiManage.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

// Define pins
#define ERRLED 14
#define NORLED 12

// Define variables
const int serverPort = 8069;
const char* addr = "ammeter.esg4u.com.tw";
const unsigned long measurementInterval = 15 * 60 * 1000; // 15 minutes
const unsigned long btInterval = 2 * 60 * 1000;           // 2 minutes
unsigned long lastMeasurementTime = 0;
const char* host = "esp32";
bool checked = false;

// Define libraries
WiFiConfig config(host);
WiFiClient wifiClient;
SoftwareSerial P3P50(16, 17);
ModbusMaster node;

void setup() {
  Serial.begin(9600);
  P3P50.begin(9600);
  pinMode(ERRLED, OUTPUT);
  pinMode(NORLED, OUTPUT);
  config.autoconnect();
  config.ConnectMode();
}

void connect_check() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(NORLED, HIGH);
  } else {
    digitalWrite(NORLED, LOW);
  }
}

void loop() {
  connect_check();
  unsigned long elapsedTime = millis() - lastMeasurementTime;
  if (elapsedTime >= measurementInterval) {
    phaseA("A");
    delay(3000); // Delay between phases

    phaseB("B");
    delay(3000); // Delay between phases

    phaseC("C");
    delay(3000); // Delay before looping again

    // Update the last measurement time
    lastMeasurementTime = millis();
  }

  if (checked && elapsedTime >= btInterval) {
    checked = false;
  }
  delay(1);
}

void sendPhaseData(const char* phaseName, double Vol, double Curr, double Freq,int AP, double PF) {
  String deviceID = generateDeviceID();
  float hr = (measurementInterval / 60000.0) / 60.0;
  float kw = (Vol * Curr * PF) / 1000.0; // Voltage (V) x Current (I) x Power Factor (PF) 
  float kWh = kw * hr;

  double apparentPower = Vol * Curr;
  //double pf =  AP / apparentPower;

  char kwString[10];
  char kWhString[10];
  dtostrf(kw, 5, 4, kwString); // dtostrf(value, width, precision, output)
  dtostrf(kWh, 5, 4, kWhString); // dtostrf(value, width, precision, output)

  DynamicJsonDocument json(200);

  // Constructing the JSON object
  json["current"] = Curr;
  json["kw"] = kwString;
  json["voltage"] = Vol;
  json["pf"] = PF;
  json["freq"] = Freq;
  json["kwh"] = kWhString;
  json["machineid"] = String(deviceID.substring(0, 5)) + "_" + phaseName;

  // Serializing JSON to a string
  String jsonString;
  serializeJson(json, jsonString);
  String jsonPayload = "[" + jsonString + "]";
  Serial.println(jsonPayload);

  // Sending JSON data to server
  HttpClient http(wifiClient, addr, serverPort);

  http.beginRequest();
  http.post("/electriciot/create_data");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", jsonPayload.length());
  http.beginBody();
  http.print(jsonPayload);
  http.endRequest();

  // Read the response from the server
  int httpResponseCode = http.responseStatusCode();
  String response = http.responseBody();
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  Serial.print("Response payload: ");
  Serial.println(response);

  // Clear the JSON string and end the HTTP request
  jsonString.clear();
  jsonPayload.clear();
  http.stop();
}

void phaseA(const char* phase) {
  double Vol = Read_RTU(1, 3000) / 10.0;
  double Curr = Read_RTU(1, 3001) / 10.0;
  double Freq = Read_RTU(1, 3008) / 10.0;
  int AP = Read_RTU(1,3002);
  double PF = Read_RTU(1,3005)/1000.0;
  sendPhaseData(phase, Vol, Curr, Freq, AP, PF);
}

void phaseB(const char* phase) {
  double Vol = Read_RTU(1, 3010) / 10.0;
  double Curr = Read_RTU(1, 3011) / 10.0;
  double Freq = Read_RTU(1, 3018) / 10.0;
  int AP = Read_RTU(1,3012);
  double PF = Read_RTU(1,3015)/1000.0;
  sendPhaseData(phase, Vol, Curr, Freq, AP, PF);
}

void phaseC(const char* phase) {
  double Vol = Read_RTU(1, 3020) / 10.0;
  double Curr = Read_RTU(1, 3021) / 10.0;
  double Freq = Read_RTU(1, 3028) / 10.0;
  int AP = Read_RTU(1,3022);
  double PF = Read_RTU(1,3025)/1000.0;
  sendPhaseData(phase, Vol, Curr, Freq, AP, PF);
}

uint16_t Read_RTU(char addr, uint16_t REG) {
  uint8_t result;
  uint16_t data[2];

  node.begin(addr, P3P50);
  result = node.readHoldingRegisters(REG, 1);
  delay(500);
  if (result == node.ku8MBSuccess) {
    data[0] = node.getResponseBuffer(0);
    return data[0];
  } else {
    Serial.print("Modbus fail. REG:");
    delay(20);
    for (int i = 0; i < 10; i++) {
      digitalWrite(ERRLED, HIGH);
      delay(100);
      digitalWrite(ERRLED, LOW);
      delay(100);
    }
    return 0;
  }
}

String generateDeviceID() {
  uint64_t chipId = ESP.getEfuseMac();
  char deviceID[17];
  sprintf(deviceID, "%04X%08X", (uint16_t)(chipId >> 32), (uint32_t)chipId);

  return String(deviceID);
}
