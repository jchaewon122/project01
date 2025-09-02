#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// Software serial communication with Orange BLE
SoftwareSerial orangeBleSerial(2, 3); // RX: D2, TX: D3

// TCS34725 sensor object setup
// Integration Time is 154ms, Gain is 16X for maximum sensitivity
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

// Global variables for sensor data
String currentColorStatus = "Other";
bool tcsSensorFound = false;

// 시리얼 모니터 출력을 위한 변수들
uint16_t lastR = 0, lastG = 0, lastB = 0, lastC = 0;

void setup() {
  Serial.begin(9600); // For debug monitor
  orangeBleSerial.begin(9600); // Start serial communication with Orange BLE

  // Initialize TCS sensor
  if (tcs.begin()) {
    Serial.println("NANO TCS: Found TCS34725 sensor!");
    tcsSensorFound = true;
  } else {
    Serial.println("NANO TCS: No TCS34725 sensor found!");
    tcsSensorFound = false;
  }

  Serial.println("NANO TCS Slave initialized");
}

void loop() {
  // Update and send sensor values every 50ms
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;

    if (tcsSensorFound) {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);
      
      // 시리얼 모니터 출력을 위해 저장
      lastR = r;
      lastG = g;
      lastB = b;
      lastC = c;

      long rgbSum = r + g + b;

      // Classify color as "Black" or "White" based on RGB sum
      if (rgbSum < 3000) {
        currentColorStatus = "Black";
      } else {
        currentColorStatus = "White";
      }
    } else {
      currentColorStatus = "Error"; // Sensor initialization failed
    }

    // Transmit data to Orange BLE (왼쪽 컬러)
    orangeBleSerial.println(currentColorStatus);
  }

  // 시리얼 모니터 출력 (1초마다)
  static unsigned long lastSerialOutput = 0;
  if (currentTime - lastSerialOutput >= 1000) {
    lastSerialOutput = currentTime;
    Serial.print("R: ");
    Serial.print(lastR);
    Serial.print(", G: ");
    Serial.print(lastG);
    Serial.print(", B: ");
    Serial.print(lastB);
    Serial.print(", C: ");
    Serial.print(lastC);
    Serial.print(" -> Color: ");
    Serial.println(currentColorStatus);
  }
}
