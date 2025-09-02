#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <LiquidCrystal_I2C.h>

SoftwareSerial orangeBleSerial(2, 3);
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define TRIG_PIN 9
#define ECHO_PIN 10

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

long currentDistance = -1;
String currentColorStatus = "Other";
bool tcsSensorFound = false;
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 50;

void setup() {
  Serial.begin(9600);
  orangeBleSerial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (tcs.begin()) {
    tcsSensorFound = true;
    Serial.println("UNO: Found TCS34725 sensor!");
  } else {
    tcsSensorFound = false;
    Serial.println("UNO: No TCS34725 sensor found!");
  }

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("UNO Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  if (orangeBleSerial.available()) {
    String command = orangeBleSerial.readStringUntil('\n');
    command.trim();

    if (command == "GET_UNO") {
      updateSensorData();
      orangeBleSerial.print("UNO:");
      orangeBleSerial.print(currentDistance);
      orangeBleSerial.print(",");
      orangeBleSerial.println(currentColorStatus);
    }
    
    if (command.startsWith("SCORE:")) {
      String scoreString = command.substring(6);
      int receivedScore = scoreString.toInt();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Score:");
      lcd.print(receivedScore);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    updateSensorData();
  }
}

void updateSensorData() {
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

  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    long rgbSum = r + g + b;

    if (rgbSum < 1600) {
      currentColorStatus = "Black";
    } else {
      currentColorStatus = "White";
    }

    Serial.print("Distance: "); Serial.print(currentDistance); Serial.print(" cm, ");
    Serial.print("R: "); Serial.print(r); Serial.print(", G: "); Serial.print(g); Serial.print(", B: "); Serial.print(b); Serial.print(", C: "); Serial.print(c);
    Serial.print(" -> Color: "); Serial.println(currentColorStatus);
  } else {
    currentColorStatus = "Error";
    Serial.println("Distance: Error, Color: Error");
  }
}
