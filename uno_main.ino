#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <LiquidCrystal_I2C.h>

// Software serial communication with Orange BLE
SoftwareSerial orangeBleSerial(2, 3); // RX: D2, TX: D3

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
int receivedScore = 0;

// 시리얼 모니터 출력을 위한 변수들
uint16_t lastR = 0, lastG = 0, lastB = 0, lastC = 0;

void setup() {
  Serial.begin(9600); // For debug monitor
  orangeBleSerial.begin(9600); // Start serial communication with Orange BLE

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize TCS sensor
  if (tcs.begin()) {
    tcsSensorFound = true;
    Serial.println("TCS34725 sensor found!");
  } else {
    tcsSensorFound = false;
    Serial.println("TCS34725 sensor not found!");
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
  // Update sensor values every 50ms
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    updateSensorData();

    // Transmit data to Orange BLE (거리, 오른쪽 컬러)
    orangeBleSerial.print(currentDistance);
    orangeBleSerial.print(",");
    orangeBleSerial.println(currentColorStatus);
  }

  // Check for incoming data (score) from Orange BLE
  if (orangeBleSerial.available()) {
    String incomingData = orangeBleSerial.readStringUntil('\n');
    incomingData.trim();
    
    // SCORE: 형태로 온다고 가정
    if (incomingData.startsWith("SCORE:")) {
      String scoreString = incomingData.substring(6);
      receivedScore = scoreString.toInt();

      // Display the received score on the LCD
      updateLCD();
    }
  }

  // 시리얼 모니터 출력 (1초마다)
  static unsigned long lastSerialOutput = 0;
  if (currentTime - lastSerialOutput >= 1000) {
    lastSerialOutput = currentTime;
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.print(" cm, R: ");
    Serial.print(lastR);
    Serial.print(", G: ");
    Serial.print(lastG);
    Serial.print(", B: ");
    Serial.print(lastB);
    Serial.print(", C: ");
    Serial.println(lastC);
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
    // 거리가 너무 크면 제한
    if (newDistance > 400) {
      currentDistance = 400;
    } else {
      currentDistance = newDistance;
    }
  } else {
    currentDistance = -1;
  }

  // Read data from TCS color sensor (오른쪽 컬러)
  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    
    // 시리얼 모니터 출력을 위해 저장
    lastR = r;
    lastG = g;
    lastB = b;
    lastC = c;

    long rgbSum = r + g + b;

    // Use a robust threshold for color detection
    if (rgbSum < 3000) {
      currentColorStatus = "Black";
    } else {
      currentColorStatus = "White";
    }
  } else {
    currentColorStatus = "Error"; // Sensor initialization failed
  }
}

// LCD 업데이트 함수
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Score: ");
  lcd.print(receivedScore);
  
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(currentDistance);
  lcd.print("cm");
}
