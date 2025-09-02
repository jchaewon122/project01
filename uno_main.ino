#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <LiquidCrystal_I2C.h>

// Software serial communication with Orange BLE
// Connect TX pin of Uno (D2) to RX pin of Orange BLE (D6)
// Connect RX pin of Uno (D3) to TX pin of Orange BLE (D7)
SoftwareSerial orangeBleSerial(2, 3);

// LCD I2C address (default is 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Ultrasonic sensor pins
#define TRIG_PIN 9
#define ECHO_PIN 10

// TCS34725 sensor object setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

// Global variables for sensor data
long currentDistance = -1;
String currentColorStatus = "Other";
bool tcsSensorFound = false;

// New variable for continuous reading
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 50; // 50ms interval for continuous reading

void setup() {
  Serial.begin(9600); // For PC debug monitor
  orangeBleSerial.begin(9600); // Start serial communication with Orange BLE

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize TCS sensor
  if (tcs.begin()) {
    tcsSensorFound = true;
    Serial.println("UNO: Found TCS34725 sensor!");
  } else {
    tcsSensorFound = false;
    Serial.println("UNO: No TCS34725 sensor found!");
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("UNO Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Check for incoming command from Orange BLE
  if (orangeBleSerial.available()) {
    String command = orangeBleSerial.readStringUntil('\n');
    command.trim();

    // If the command is "GET_UNO", update and send data
    if (command == "GET_UNO") {
      updateSensorData();

      // Transmit data to Orange BLE with a unique ID
      orangeBleSerial.print("UNO:");
      orangeBleSerial.print(currentDistance);
      orangeBleSerial.print(",");
      orangeBleSerial.println(currentColorStatus);
    }
    
    // Check for incoming score from Orange BLE
    if (command.startsWith("SCORE:")) {
      String scoreString = command.substring(6);
      int receivedScore = scoreString.toInt();

      // Display the received score on the LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Score:");
      lcd.print(receivedScore);
    }
  }

  // --- New Logic for continuous monitoring ---
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    updateSensorData();
  }
}

// Function to update sensor data
void updateSensorData() {
  // Read data from ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration > 0) {
    long newDistance = (duration / 2) / 29.1;
    currentDistance = newDistance;
  } else {
    currentDistance = -1;
  }

  // Read data from TCS color sensor
  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    long rgbSum = r + g + b;

    // Use a robust threshold for color detection
    if (rgbSum < 1600) { 
      currentColorStatus = "Black";
    } else {
      currentColorStatus = "White";
    }

    // Print to Serial Monitor for debug
    Serial.print("Distance: "); Serial.print(currentDistance); Serial.print(" cm, ");
    Serial.print("R: "); Serial.print(r); Serial.print(", G: "); Serial.print(g); Serial.print(", B: "); Serial.print(b); Serial.print(", C: "); Serial.print(c);
    Serial.print(" -> Color: "); Serial.println(currentColorStatus);

  } else {
    currentColorStatus = "Error"; // Sensor initialization failed
    Serial.println("Distance: Error, Color: Error");
  }
}
