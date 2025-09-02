#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <LiquidCrystal_I2C.h>

SoftwareSerial orangeSerial(2, 3);
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define SLAVE_ID 1
#define TRIG_PIN 9
#define ECHO_PIN 10

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

long currentDistance = -1;
String currentColorStatus = "Other";
bool tcsSensorFound = false;
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 100; // 센서 읽기 간격을 늘림

void updateSensorData();
void sendDataToMaster();
void displayScore(int score);

void setup() {
  Serial.begin(9600);
  orangeSerial.begin(9600);

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
  lcd.print("UNO Ready (ID:1)");
  delay(1000);
  lcd.clear();
  
  Serial.print("UNO Slave initialized with ID: ");
  Serial.println(SLAVE_ID);
}

void loop() {
  // Check for commands from master
  if (orangeSerial.available()) {
    String command = orangeSerial.readStringUntil('\n');
    command.trim();

    // Check for data request
    if (command == "REQ_DATA") {
      sendDataToMaster();
    }
    
    // Handle score updates
    if (command.startsWith("SCORE:")) {
      String scoreString = command.substring(6);
      int receivedScore = scoreString.toInt();
      displayScore(receivedScore);
    }
  }

  // Periodic sensor reading for local processing
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    updateSensorData();
  }
}

void sendDataToMaster() {
  orangeSerial.print("UNO:");
  orangeSerial.print(currentDistance);
  orangeSerial.print(",");
  orangeSerial.println(currentColorStatus);
  
  // 전송 완료 대기
  orangeSerial.flush();
  
  Serial.print("Sent to master - Distance: ");
  Serial.print(currentDistance);
  Serial.print(", Color: ");
  Serial.println(currentColorStatus);
}

void updateSensorData() {
  // Read ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  if (duration > 0) {
    currentDistance = (duration * 0.034) / 2; // 더 정확한 계산식
    // 범위 제한 (2cm ~ 400cm)
    if (currentDistance < 2) currentDistance = 2;
    if (currentDistance > 400) currentDistance = 400;
  } else {
    currentDistance = -1; // 측정 실패
  }

  // Read color sensor
  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    
    // 센서 에러 체크
    if (c == 0) {
      currentColorStatus = "Error";
      return;
    }
    
    long rgbSum = r + g + b;

    // 임계값 조정 및 추가 검증
    if (rgbSum < 1600) {
      currentColorStatus = "Black";
    } else if (rgbSum > 1600) {
      currentColorStatus = "White";
    } else {
      currentColorStatus = "Other";
    }
  } else {
    currentColorStatus = "Error";
  }
}

void displayScore(int score) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Score: ");
  lcd.print(score);
  
  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(currentDistance);
  lcd.print(" C:");
  lcd.print(currentColorStatus.substring(0, 5)); // 처음 5글자만
}
