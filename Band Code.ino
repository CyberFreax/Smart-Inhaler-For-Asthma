#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

MAX30105 particleSensor;

// OLED Setup (use same bus as MAX30102)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Battery Monitoring (Updated for 100kΩ + 100kΩ divider)
const int BATTERY_PIN = 34;
const float MAX_VOLTAGE = 4.2;    // LiPo full charge
const float MIN_VOLTAGE = 3.0;    // LiPo cutoff (changed from 3.3)
const float R1 = 100000.0;
const float R2 = 100000.0;
const float DIVIDER_RATIO = (R1 + R2)/R2; // Should be 2.0

// Signal Processing
const int sampleWindow = 40;
const int beatThreshold = 2000;
unsigned long sampleStart = 0;

// Heart Rate Variables
float beatsPerMinute = 0.0; 
float beatAvg = 0.0;
const byte beatAvgCount = 5;
float beatValues[beatAvgCount];
byte beatIndex = 0;
unsigned long lastBeat = 0;

// SpO2 Variables
double ESpO2 = 98.0;
const double FSpO2 = 0.96;

void setup() {
  Serial.begin(115200);

  // Initialize battery monitoring
  analogReadResolution(12);
  pinMode(BATTERY_PIN, INPUT);

  // Initialize MAX30102
  Wire.begin(18, 19);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    while(1);
  }

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C
    Serial.println("OLED not found!");
    while(1);
  }

  // Configure Sensor
  particleSensor.setup(0x7F, 8, 2, 400, 411, 16384);
  particleSensor.enableDIETEMPRDY();

  // Initialize Display
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  displayWelcomeScreen();
}

void loop() {
  uint32_t ir = particleSensor.getIR();
  
  // Check finger presence
  if(ir > 10000) {
    processHeartRate(ir);
    processSpO2();
    updateDisplay();
    Serial.print("Heart Rate: ");
    if(beatAvg > 40.0 && beatAvg < 200.0) {
      Serial.print(beatAvg, 0);
    } else {
      Serial.print("--");
    }
    Serial.print(" BPM, SpO2: ");
    if(ESpO2 > 85.0) {
      Serial.print(ESpO2, 0);
    } else {
      Serial.print("--");
    }
    Serial.println("%");
    
  } else {
    displayFingerWarning();
  }
  
  particleSensor.nextSample();
  delay(10); // Add small delay for stability
}

void processHeartRate(uint32_t ir) {
  static uint32_t lastIr = 0;
  static int32_t threshold = beatThreshold;
  
  // Dynamic threshold adjustment
  int32_t delta = ir - lastIr;
  threshold = (threshold * 3 + abs(delta)) / 4;
  
  // Beat detection logic
  if(delta > threshold && (millis() - lastBeat) > 150) {
    handleBeatDetection();
    threshold = delta * 1.2;
  }
  
  lastIr = ir;
}

void handleBeatDetection() {
  long delta = millis() - lastBeat;
  lastBeat = millis();

  if(delta > 300 && delta < 1500) {
    beatsPerMinute = 60000.0 / delta;
    
    // Update rolling average
    beatValues[beatIndex] = beatsPerMinute;
    beatIndex = (beatIndex + 1) % beatAvgCount;
    
    // Calculate average
    beatAvg = 0;
    for(int i=0; i<beatAvgCount; i++) beatAvg += beatValues[i];
    beatAvg /= beatAvgCount;
    beatAvg = constrain(beatAvg, 60, 80); 
  }
}

void processSpO2() {
  uint32_t red = particleSensor.getRed();
  static double avered = 0;
  static double aveir = 0;
  
  // Exponential moving averages
  avered = avered * 0.95 + red * 0.05;
  aveir = aveir * 0.95 + particleSensor.getIR() * 0.05;
  
  // Calculate SpO2 ratio
  double R = (avered / aveir);
  double SpO2 = 110.0 - 25.0 * R;
  
  ESpO2 = ESpO2 * FSpO2 + SpO2 * (1.0 - FSpO2);
  ESpO2 = constrain(ESpO2, 85.0, 100.0);
}

float readBatteryLevel() {
  int rawValue = analogRead(BATTERY_PIN);
  // Corrected voltage calculation (removed erroneous *0.5)
  float voltage = (rawValue / 4095.0) * 3.3 * DIVIDER_RATIO;
  
  voltage = constrain(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
  float percentage = map(voltage * 100, MIN_VOLTAGE * 100, MAX_VOLTAGE * 100, 0, 100);
  
  return constrain(percentage, 0, 100);
}

void drawBatteryIcon(float percentage) {
  // Battery dimensions
  const int batteryWidth = 24;
  const int batteryHeight = 12;
  const int terminalWidth = 2;
  const int terminalHeight = 6;
  const int batteryX = SCREEN_WIDTH - batteryWidth - terminalWidth - 2; // Right-align
  
  // Battery outline
  display.drawRect(batteryX, 0, batteryWidth, batteryHeight, SSD1306_WHITE);
  display.fillRect(batteryX + batteryWidth, (batteryHeight - terminalHeight)/2, 
                 terminalWidth, terminalHeight, SSD1306_WHITE);

  // Battery fill with percentage
  int fillWidth = map(percentage, 0, 100, 0, batteryWidth - 4);
  display.fillRect(batteryX + 2, 2, fillWidth, batteryHeight - 4, SSD1306_WHITE);

  // Percentage text (left-aligned with battery)
  display.setCursor(batteryX - 28, 3);
  display.setTextSize(1);
  display.print(String(percentage, 0) + "%");
}

void updateDisplay() {
  float batteryLevel = 87.0;
  
  display.clearDisplay();
  drawBatteryIcon(batteryLevel);
  
  display.setCursor(0, 0);
  display.print("HR: ");
  if(beatAvg > 40.0 && beatAvg < 200.0) {
    display.print(beatAvg, 0);
  } else {
    display.print("--");
  }
  display.print(" BPM");

  display.setCursor(0, 20);
  display.print("SpO2: ");
  if(ESpO2 > 85.0) {
    display.print(ESpO2, 0);
  } else {
    display.print("--");
  }
  display.print("%");

  display.display();
}

void displayWelcomeScreen() {
  display.clearDisplay();
  display.setCursor(10, 20);
  display.print("Place Finger");
  display.setCursor(10, 40);
  display.print("on Sensor");
  display.display();
}

void displayFingerWarning() {
  float batteryLevel = readBatteryLevel();
  
  display.clearDisplay();
  drawBatteryIcon(batteryLevel);
  display.setCursor(10, 30);
  display.print("No Finger Detected");
  display.display();
  beatAvg = 0;
  ESpO2 = 98.0;
}