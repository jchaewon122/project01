#include <Adafruit_TCS34725.h>

// TCS34725 sensor configuration
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// Global variables for sensor data
String currentColorStatus = "Other";
bool tcsSensorFound = false;

void setup() {
  Serial.begin(9600); // Start serial communication with Orange BLE
  
  // Initialize TCS sensor
  if (tcs.begin()) {
    Serial.println("NANO1: Found TCS34725 sensor!");
    tcsSensorFound = true;
  } else {
    Serial.println("NANO1: No TCS34725 sensor found!");
    tcsSensorFound = false;
  }
  
  Serial.println("NANO1 Slave initialized (Serial Mode)");
}

void loop() {
  // Update and send sensor values every 50ms
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    
    if (tcsSensorFound) {
      uint16_t r, g, b, c;
      if (tcs.getRawData(&r, &g, &b, &c)) {
        if (c < 200) {
          currentColorStatus = "Black";
        } else if (c > 800) {
          currentColorStatus = "White";
        } else {
          currentColorStatus = "Other";
        }
      } else {
        currentColorStatus = "Error";
      }
    } else {
      currentColorStatus = "Error";
    }
    
    // Transmit data
    Serial.println(currentColorStatus);
  }
}
