#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_AHT10.h>         // Library for AHT10 temperature/humidity sensor
#include "DFRobot_MICS.h"          // Library for MiCS 4514 gas sensor
#include <SensirionI2CSdp.h>

// WiFi credentials
const char* ssid = "YashRedmi";
const char* password = "yashyash";

// Server details
const char* serverUrl = "http://192.168.177.127:5001/api/sensor-data"; 
const char* apiKey = "YOUR_API_KEY"; // Must match ESP32_API_KEY in your .env file

// Use a fixed device ID - no registration needed
const char* deviceId = "esp32_001";

// [Keep all sensor pin/config definitions from your original code]
#define MICS_SDA 21
#define MICS_SCL 22
#define ENV_SDA 17
#define ENV_SCL 18
#define BUZZER_PIN 25
#define MICS_ADDRESS 0x75
#define AHT_ADDRESS 0x38
#define SDP_ADDRESS 0x25
#define CALIBRATION_TIME 3
#define WARMUP_TIMEOUT 300
#define SDP_READ_DELAY 100

// Sensor objects
TwoWire I2C_MICS = TwoWire(0);
TwoWire I2C_ENV = TwoWire(1);
DFRobot_MICS_I2C mics(&I2C_MICS, MICS_ADDRESS);
Adafruit_AHT10 aht;
SensirionI2CSdp sdp;

void initializeSDP() {
  Serial.println("Initializing SDP sensor...");
  
  for(int attempt = 1; attempt <= 3; attempt++) {
    sdp.begin(I2C_ENV, SDP_ADDRESS);
    sdp.stopContinuousMeasurement();
    delay(50);

    uint32_t productNumber;
    uint8_t serialNumber[8];
    char errorMessage[256];
    uint16_t error = sdp.readProductIdentifier(productNumber, serialNumber, 8);

    if (!error) {
      Serial.print("SDP initialized. Product: 0x");
      Serial.println(productNumber, HEX);
      return;
    }
    
    Serial.print("Attempt ");
    Serial.print(attempt);
    Serial.print(" failed: ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
    delay(200);
  }

  Serial.println("SDP initialization failed!");
  while(1);
}

void i2cScan(TwoWire &wire, const char* busName) {
  Serial.print("Scanning ");
  Serial.print(busName);
  Serial.println(" I2C bus...");
  
  byte count = 0;
  for(byte addr = 1; addr < 127; addr++) {
    wire.beginTransmission(addr);
    byte error = wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  
  if (count == 0) Serial.println("No devices found");
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for serial monitor
  Serial.println("\n=== Sensor System Initialization ===");

  // Initialize MICS sensor
  I2C_MICS.begin(MICS_SDA, MICS_SCL);
  Serial.println("Initializing MICS4514...");
  if (!mics.begin()) {
    Serial.println("MICS4514 not found!");
    i2cScan(I2C_MICS, "MICS Bus");
    while(1);
  }
  Serial.println("MICS4514 connected");

  // Initialize ENV bus sensors
  I2C_ENV.begin(ENV_SDA, ENV_SCL);
  
  // Initialize AHT10
  Serial.println("Initializing AHT10...");
  if (!aht.begin(&I2C_ENV)) {
    Serial.println("AHT10 not found!");
    i2cScan(I2C_ENV, "ENV Bus");
    while(1);
  }
  Serial.println("AHT10 connected");

  // Initialize SDP
  initializeSDP();
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  // Start SDP measurements
  uint16_t error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
  if (error) {
    char errorMessage[256];
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.print("SDP start error: ");
    Serial.println(errorMessage);
  }
  delay(500);  // Critical stabilization delay

  // MICS warm-up sequence
  Serial.println("\nStarting MICS warm-up...");
  if (mics.getPowerState() == SLEEP_MODE) mics.wakeUpMode();
  
  unsigned long warmupStart = millis();
  bool warmupComplete = false;
  
  while (!warmupComplete) {
    if (mics.warmUpTime(CALIBRATION_TIME)) {
      warmupComplete = true;
      break;
    }
    
    if (millis() - warmupStart > (WARMUP_TIMEOUT * 1000)) {
      Serial.println("\nWarm-up timeout! Check MICS sensor");
      break;
    }
    
    Serial.print(".");
    delay(1000);
  }
  
  if (warmupComplete) {
    Serial.println("\nMICS ready!");
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());

  // Create a device entry directly in the server
  createDeviceIfNeeded();
}

void createDeviceIfNeeded() {
  // This function could make an API call to create a device
  // But since we're simplifying, we'll just log that we're using a predefined device
  Serial.println("Using device ID: " + String(deviceId));
  Serial.println("No registration needed - server will handle data directly");
}

void loop() {
  // Only proceed if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.begin(ssid, password);
    delay(5000); // Wait 5 seconds before retrying
    return;
  }
  
  // Read all sensor data
  Serial.println("\n--- Reading Sensor Data ---");
  
 // Read gas sensors
  float co = mics.getGasData(CO);
  float no2 = mics.getGasData(NO2);
  float ethanol = mics.getGasData(C2H5OH);
  
  // Read environmental sensors
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  // Read SDP sensor
  float pressure, sdpTemp;
  char errorMessage[256];
  uint16_t error = sdp.readMeasurement(pressure, sdpTemp);

  // Print sensor data
  Serial.println("\n=== Sensor Readings ===");
  Serial.print("CO: "); Serial.print(co); Serial.println(" ppm");
  Serial.print("NO2: "); Serial.print(no2); Serial.println(" ppm");
  Serial.print("Ethanol: "); Serial.print(ethanol); Serial.println(" ppm");
  
  Serial.print("Temperature: "); 
  Serial.print(temp.temperature); 
  Serial.println(" °C");
  
  Serial.print("Humidity: "); 
  Serial.print(humidity.relative_humidity); 
  Serial.println(" %");

  if (error) {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.print("SDP Error: ");
    Serial.println(errorMessage);
    digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer off on error
  } else {
    Serial.print("Pressure: "); 
    Serial.print(pressure); 
    Serial.print(" Pa | SDP Temp: ");
    Serial.print(sdpTemp); 
    Serial.println(" °C");

    // Buzzer control logic
    if (pressure < -0.1) {
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("ALERT: Low pressure detected!");
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
  // Prepare JSON data to send to server
  DynamicJsonDocument jsonDoc(1024);
  jsonDoc["device_id"] = deviceId;
  
  // Gas sensor data
  JsonObject gas = jsonDoc.createNestedObject("gas");
  gas["co"] = co;
  gas["no2"] = no2;
  gas["c2h5oh"] = ethanol;
  // gas["nh4"] = mics.getNH3PPM();  // Note: NH3 is ammmonia, NH4 is ammonium - using NH3 from library
  // gas["ch4"] = mics.getCH4PPM();
  // gas["co2"] = mics.getCO2PPM();
  
  // Temperature and humidity
  JsonObject env = jsonDoc.createNestedObject("environmental");
  env["temperature"] = temp.temperature;
  env["humidity"] = humidity.relative_humidity;
  
  // Pressure data
  jsonDoc["differential_pressure"] = pressure;
  
  // Battery level (simulated - implement actual monitoring if available)
  jsonDoc["battery_level"] = 85;
  
  // Serialize JSON to string
  String jsonPayload;
  serializeJson(jsonDoc, jsonPayload);
  
  // Send data to server
  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", apiKey);
  
  int httpResponseCode = http.POST(jsonPayload);
  Serial.println("Sending POST request to: " + String(serverUrl));
  Serial.println("Payload: " + jsonPayload);
  
  if (httpResponseCode > 0) {
    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    Serial.println("Response: " + http.getString());
  } else {
    Serial.printf("HTTP Error code: %d\n", httpResponseCode);
    Serial.println("Error: " + http.errorToString(httpResponseCode));
  }
  
  http.end();
  
  // Wait 30 seconds before next reading
  Serial.println("Waiting 3 seconds until next reading...");
  delay(3000);
}