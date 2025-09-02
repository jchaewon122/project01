#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// Software serial communication with Orange BLE
// Connect TX pin of Nano TCS (D2) to RX pin of Orange BLE (D6)
// Connect RX pin of Nano TCS (D3) to TX pin of Orange BLE (D7)
SoftwareSerial orangeBleSerial(2, 3);

// TCS34725 sensor object setup
// Integration Time is 154ms, Gain is 16X for maximum sensitivity
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

// Global variables for sensor data
String currentColorStatus = "Other";
bool tcsSensorFound = false;

void setup() {
  Serial.begin(9600); // For PC debug monitor
  orangeBleSerial.begin(9600);

  // Initialize TCS sensor
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
  // Listen for command from master (Orange Board)
  if (orangeBleSerial.available()) {
    String command = orangeBleSerial.readStringUntil('\n');
    command.trim();

    // If the command is "GET_TCS", update and send data
    if (command == "GET_TCS") {
      if (tcsSensorFound) {
        uint16_t r, g, b, c;
        tcs.getRawData(&r, &g, &b, &c);

        long rgbSum = r + g + b;

        // Classify color as "Black" or "White" based on RGB sum
        if (rgbSum < 3000) {
          currentColorStatus = "Black";
        } else {
          currentColorStatus = "White";
        }

        // Print to Serial Monitor for debug
        Serial.print("R: "); Serial.print(r);
        Serial.print(", G: "); Serial.print(g);
        Serial.print(", B: "); Serial.print(b);
        Serial.print(", C: "); Serial.print(c);
        Serial.print(" -> Color: "); Serial.println(currentColorStatus);

      } else {
        currentColorStatus = "Error"; // Sensor initialization failed
        Serial.println("R: Error, G: Error, B: Error, C: Error -> Color: Error");
      }

      // Transmit data to Orange BLE with a unique ID
      orangeBleSerial.print("NANO_TCS:");
      orangeBleSerial.println(currentColorStatus);
    }
  }
}
