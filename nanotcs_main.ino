#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

SoftwareSerial orangeBleSerial(2, 3);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

String currentColorStatus = "Other";
bool tcsSensorFound = false;
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 50;

void setup() {
  Serial.begin(9600);
  orangeBleSerial.begin(9600);

  if (tcs.begin()) {
    tcsSensorFound = true;
    Serial.println("NANO_TCS: Found TCS34725 sensor!");
  } else {
    tcsSensorFound = false;
    Serial.println("NANO_TCS: No TCS34725 sensor found!");
  }

  Serial.println("NANO_TCS Slave initialized");
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    if (tcsSensorFound) {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);
      long rgbSum = r + g + b;

      if (rgbSum < 1600) {
        currentColorStatus = "Black";
      } else {
        currentColorStatus = "White";
      }

      Serial.print("R: "); Serial.print(r);
      Serial.print(", G: "); Serial.print(g);
      Serial.print(", B: "); Serial.print(b);
      Serial.print(", C: "); Serial.print(c);
      Serial.print(" -> Color: "); Serial.println(currentColorStatus);
    } else {
      currentColorStatus = "Error";
      Serial.println("R: Error, G: Error, B: Error, C: Error -> Color: Error");
    }
  }

  if (orangeBleSerial.available()) {
    String command = orangeBleSerial.readStringUntil('\n');
    command.trim();

    if (command == "GET_TCS") {
      orangeBleSerial.print("NANO_TCS:");
      orangeBleSerial.println(currentColorStatus);
    }
  }
}
