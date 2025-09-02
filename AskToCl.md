# 임베디드 소프트웨어 대회



목적: 초보 운전자를 위한 운전 습관 분석 임베디드 시스템 개발



## 각각의 보드 별 설명



### 오렌지 BLE 보드



#### 기능



- 우노로부터 거리와 오른쪽 컬러 값, 나노(TCS)로부터 왼쪽 컬러 값, 나노(MPU)로부터 브레이크 상태를 받음

- 다른 보드로부터 받은 정보를 분석하여, 점수를 차감하는 등의 일을 수행.

- 계산한 점수를 우노로 보냄

- 블루투스로 휴대폰으로 데이터를 보냄 (미구현)



#### 코드



#include <SoftwareSerial.h>

#include <LiquidCrystal_I2C.h>



// LCD I2C address (default is 0x27)

LiquidCrystal_I2C lcd(0x27, 16, 2);



// Software serial communication with slave boards

// 오렌지보드 RX(데이터 받는 핀) -> 슬레이브보드 TX

// 오렌지보드 TX(데이터 보내는 핀) -> 슬레이브보드 RX



// 우노 보드 연결: 오렌지 보드 6번 핀(RX)이 우노 보드의 D1(TX)에, 오렌지 보드 7번 핀(TX)이 우노 보드의 D0(RX)에 연결

SoftwareSerial unoSerial(6, 7);



// 나노 TCS 보드 연결: 오렌지 보드 2번 핀(RX)이 나노 TCS 보드의 D1(TX)에, 오렌지 보드 3번 핀(TX)이 나노 TCS 보드의 D0(RX)에 연결

SoftwareSerial nanoTcsSerial(2, 3);



// 나노 MPU 보드 연결: 오렌지 보드 4번 핀(RX)이 나노 MPU 보드의 D1(TX)에, 오렌지 보드 5번 핀(TX)이 나노 MPU 보드의 D0(RX)에 연결

SoftwareSerial nanoMpuSerial(4, 5);



// Driving score and penalty factors

int drivingScore = 100;

int laneChangeCount = 0;

int hardBrakeCount = 0;

int obstacleWarningCount = 0;

int solidLinePenaltyCount = 0;



// State machine for lane detection

enum LaneState {

  ON_ROAD,

  ON_LINE

};

LaneState currentLaneState = ON_ROAD;

unsigned long lineDetectionStartTime = 0;



// Criteria for solid/dotted lines

const unsigned long SOLID_LINE_MIN_DURATION = 800; // Solid line if on for > 0.8s



// Debouncing for hard brake and obstacle warnings

unsigned long lastHardBrakeTime = 0;

const unsigned long HARD_BRAKE_COOLDOWN = 2000;

unsigned long lastObstacleWarningTime = 0;

const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000;



// Sensor error flags

bool nanoTcsError = false;

bool nanoMpuError = false;

bool unoError = false;



void setup() {

  Serial.begin(9600); // For PC debug monitor

  unoSerial.begin(9600);

  nanoTcsSerial.begin(9600);

  nanoMpuSerial.begin(9600);



  // Initialize LCD

  lcd.init();

  lcd.backlight();

  lcd.setCursor(0, 0);

  lcd.print("Driving Start!");

  delay(2000);

  lcd.clear();



  Serial.println("Master initialized (Serial Mode)");

}



void loop() {

  // Read data from slaves

  String unoData = readSlaveData(unoSerial);

  String nanoTcsData = readSlaveData(nanoTcsSerial);

  String nanoMpuData = readSlaveData(nanoMpuSerial);



  // Parse and validate data

  int distance = -1;

  String leftColor = "Other";

  String rightColor = "Other";

  String brakeStatus = "Normal";



  // Parse UNO data

  if (unoData != "") {

    int firstComma = unoData.indexOf(',');

    if (firstComma != -1) {

      distance = unoData.substring(0, firstComma).toInt();

      leftColor = unoData.substring(firstComma + 1);

    }

  }



  // Parse Nano TCS data

  if (nanoTcsData != "") {

    rightColor = nanoTcsData;

  }



  // Parse Nano MPU data

  if (nanoMpuData != "") {

    brakeStatus = nanoMpuData;

  }



  // Update error flags

  if (leftColor == "Error" || distance == -1) unoError = true; else unoError = false;

  if (rightColor == "Error") nanoTcsError = true; else nanoTcsError = false;

  if (brakeStatus == "Error") nanoMpuError = true; else nanoMpuError = false;



  // Calculate score based on sensor data

  calculateScore(distance, leftColor, rightColor, brakeStatus);



  // Update LCD display and send score to Uno every 500ms

  static unsigned long lastDisplayTime = 0;

  unsigned long currentTime = millis();

  if (currentTime - lastDisplayTime >= 500) {

    lastDisplayTime = currentTime;

    displayOnLcd();

    unoSerial.println(drivingScore); // Uno 보드로 점수 전송

  }



  // Debug output

  static unsigned long lastDebugTime = 0;

  if (currentTime - lastDebugTime >= 1000) {

    lastDebugTime = currentTime;

    Serial.print("Uno: "); Serial.print(unoData);

    Serial.print(" | Nano TCS: "); Serial.print(nanoTcsData);

    Serial.print(" | Nano MPU: "); Serial.println(nanoMpuData);

    Serial.print("Parsed - Dist: "); Serial.print(distance);

    Serial.print(" | Left: "); Serial.print(leftColor);

    Serial.print(" | Right: "); Serial.print(rightColor);

    Serial.print(" | Brake: "); Serial.println(brakeStatus);

    Serial.print("State: "); Serial.print(currentLaneState == ON_ROAD ? "ON_ROAD" : "ON_LINE");

    Serial.print(" | Score: "); Serial.println(drivingScore);

    Serial.println("---");

  }

}



// Function to read data from a serial stream

String readSlaveData(Stream &stream) {

  if (stream.available()) {

    String data = stream.readStringUntil('\n');

    data.trim();

    return data;

  }

  return "";

}



// Improved logic for lane change and line type detection

void detectLaneChangeAndType(String leftColor, String rightColor) {

  unsigned long currentTime = millis();

  bool isOnWhiteLine = (leftColor == "White" || rightColor == "White");



  switch (currentLaneState) {

    case ON_ROAD:

      if (isOnWhiteLine) {

        currentLaneState = ON_LINE;

        lineDetectionStartTime = currentTime;

      }

      break;



    case ON_LINE:

      if (!isOnWhiteLine) {

        unsigned long lineDuration = currentTime - lineDetectionStartTime;

        if (lineDuration >= SOLID_LINE_MIN_DURATION) {

            solidLinePenaltyCount++;

            drivingScore -= 10;

        }

        laneChangeCount++;

        currentLaneState = ON_ROAD; // Return to ON_ROAD after passing the line

      }

      break;

  }

}



// Function to calculate driving score

void calculateScore(int distance, String leftColor, String rightColor, String brakeStatus) {

  unsigned long currentTime = millis();



  detectLaneChangeAndType(leftColor, rightColor);



  if (brakeStatus == "HardBrake" && currentTime - lastHardBrakeTime > HARD_BRAKE_COOLDOWN) {

    hardBrakeCount++;

    drivingScore -= 10;

    lastHardBrakeTime = currentTime;

  }



  if (distance > 0 && distance < 30 && currentTime - lastObstacleWarningTime > OBSTACLE_WARNING_COOLDOWN) {

    obstacleWarningCount++;

    drivingScore -= 3;

    lastObstacleWarningTime = currentTime;

  }



  if (drivingScore < 0) {

    drivingScore = 0;

  }

}



// Function to display score on LCD

void displayOnLcd() {

  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("Score: ");

  lcd.print(drivingScore);



  lcd.setCursor(11, 0);

  if (unoError || nanoTcsError || nanoMpuError) {

    lcd.print("ERROR");

  } else {

    if (currentLaneState == ON_ROAD) {

      lcd.print("ROAD");

    } else {

      lcd.print("LINE");

    }

  }



  lcd.setCursor(0, 1);

  lcd.print("HB:");

  lcd.print(hardBrakeCount);

  lcd.print(" LC:");

  lcd.print(laneChangeCount);

  lcd.print(" SL:");

  lcd.print(solidLinePenaltyCount);

}



### 아두이노 우노 보드



#### 기능



- 초음파 센서를 통해서 앞 차와의 거리를 파악하여 오렌지 보드로 보냄

- 컬러 센서를 통하여 감지한 색(BLACk or WHITE)을 오렌지 보드로 보냄

- 오렌지 보드로부터 받은 점수를 LCD로 출력



#### 코드



#include <SoftwareSerial.h>

#include <Wire.h>

#include <Adafruit_TCS34725.h>

#include <LiquidCrystal_I2C.h> // LCD 라이브러리 추가



// Software serial communication with Orange BLE

// Connect TX pin of Uno (D1) to RX pin of Orange BLE (D6)

// Connect RX pin of Uno (D0) to TX pin of Orange BLE (D7)

SoftwareSerial orangeBleSerial(0, 1); // RX: D0, TX: D1



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



void setup() {

  orangeBleSerial.begin(9600); // Start serial communication with Orange BLE



  pinMode(TRIG_PIN, OUTPUT);

  pinMode(ECHO_PIN, INPUT);



  // Initialize TCS sensor

  if (tcs.begin()) {

    tcsSensorFound = true;

  } else {

    tcsSensorFound = false;

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



    // Transmit data to Orange BLE

    orangeBleSerial.print(currentDistance);

    orangeBleSerial.print(",");

    orangeBleSerial.println(currentColorStatus);

  }



  // Check for incoming data (score) from Orange BLE

  if (orangeBleSerial.available()) {

    String scoreString = orangeBleSerial.readStringUntil('\n');

    scoreString.trim();

    int receivedScore = scoreString.toInt();



    // Display the received score on the LCD

    lcd.clear();

    lcd.setCursor(0, 0);

    lcd.print("Score:");

    lcd.print(receivedScore);

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

    if (rgbSum < 3000) {

      currentColorStatus = "Black";

    } else {

      currentColorStatus = "White";

    }

  } else {

    currentColorStatus = "Error"; // Sensor initialization failed

  }

}



### 아두이노 나노 보드 (TCS)



#### 기능



- 컬러 센서를 통하여 감지한 색(BLACk or WHITE)을 오렌지 보드로 보냄



#### 코드



#include <SoftwareSerial.h>

#include <Wire.h>

#include <Adafruit_TCS34725.h>



// Software serial communication with Orange BLE

// Connect TX pin of Nano TCS (D1) to RX pin of Orange BLE (D2)

// Connect RX pin of Nano TCS (D0) to TX pin of Orange BLE (D3)

SoftwareSerial orangeBleSerial(0, 1); // RX: D0, TX: D1



// TCS34725 sensor object setup

// Integration Time is 154ms, Gain is 16X for maximum sensitivity

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);



// Global variables for sensor data

String currentColorStatus = "Other";

bool tcsSensorFound = false;



void setup() {

  // Start serial communication with Orange BLE

  orangeBleSerial.begin(9600);



  // Initialize TCS sensor

  if (tcs.begin()) {

    orangeBleSerial.println("NANO1: Found TCS34725 sensor!");

    tcsSensorFound = true;

  } else {

    orangeBleSerial.println("NANO1: No TCS34725 sensor found!");

    tcsSensorFound = false;

  }



  orangeBleSerial.println("NANO1 Slave initialized");

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



    // Transmit data to Orange BLE

    orangeBleSerial.println(currentColorStatus);

  }

}



### 아두이노 나노 보드 MPU)



#### 기능



- 가속도 센서를 통하여 현재 브레이크 여부를 오렌지 보드로 보냄



#### 코드



#include <SoftwareSerial.h>

#include <Wire.h>

#include <math.h>



// Software serial communication with Orange BLE

// Connect TX pin of Nano MPU (D1) to RX pin of Orange BLE (D4)

// Connect RX pin of Nano MPU (D0) to TX pin of Orange BLE (D5)

SoftwareSerial orangeBleSerial(0, 1); // RX: D0, TX: D1



// MPU6500 I2C address and registers

#define MPU6500_ADDRESS 0x68

#define WHO_AM_I        0x75

#define PWR_MGMT_1      0x6B

#define PWR_MGMT_2      0x6C

#define ACCEL_XOUT_H    0x3B



// Variables for hard brake detection

float accelThreshold = 12.0; // Hard brake threshold (m/s²)

float prevAccelMagnitude = 0.0;

unsigned long lastHardBrakeTime = 0;

const unsigned long BRAKE_DETECTION_COOLDOWN = 1000; // 1s cooldown

bool mpuSensorFound = false;



// Acceleration scale factor (±8g range)

const float ACCEL_SCALE = 8.0 / 32768.0 * 9.81; // m/s² conversion



void setup() {

  orangeBleSerial.begin(9600); // Start serial communication with Orange BLE



  // I2C initialization

  Wire.begin();

  Wire.setClock(100000);

  delay(100);



  // Initialize MPU6500 sensor

  if (initializeMPU6500()) {

    orangeBleSerial.println("NANO2: Found MPU6500 sensor!");

    mpuSensorFound = true;



    // MPU6500 configuration

    configureMPU6500();



    delay(100);



    // Test read

    float testX, testY, testZ;

    if (readAcceleration(testX, testY, testZ)) {

      orangeBleSerial.println("NANO2: Sensor test read successful!");

    }

  } else {

    orangeBleSerial.println("NANO2: Failed to find MPU6500 chip");

    mpuSensorFound = false;

  }



  orangeBleSerial.println("NANO2 Slave initialized");

}



void loop() {

  // Update sensor values every 20ms

  static unsigned long lastSensorReadTime = 0;

  unsigned long currentTime = millis();



  if (currentTime - lastSensorReadTime >= 20) {

    lastSensorReadTime = currentTime;



    String currentBrakeStatus = "Normal";

    if (mpuSensorFound) {

      float accelX, accelY, accelZ;



      if (readAcceleration(accelX, accelY, accelZ)) {

        float accelMagnitude = sqrt(

          accelX * accelX +

          accelY * accelY +

          accelZ * accelZ

        );



        float accelChange = abs(accelMagnitude - prevAccelMagnitude);



        if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {

          if (accelZ > accelThreshold || accelChange > 8.0) {

            currentBrakeStatus = "HardBrake";

            lastHardBrakeTime = currentTime;

          }

        }

        prevAccelMagnitude = accelMagnitude;

      } else {

        currentBrakeStatus = "Error";

      }

    } else {

      currentBrakeStatus = "Error";

    }



    // Transmit data

    orangeBleSerial.println(currentBrakeStatus);

  }

}



// MPU6500 initialization function

bool initializeMPU6500() {

  uint8_t who_am_i = readRegister(WHO_AM_I);

  if (who_am_i != 0x70) { // MPU6500

    return false;

  }



  writeRegister(PWR_MGMT_1, 0x80);

  delay(100);



  writeRegister(PWR_MGMT_1, 0x00);

  delay(50);



  writeRegister(PWR_MGMT_2, 0x00);

  delay(10);



  return true;

}



// MPU6500 configuration function (±8g, 500°/s equivalent)

void configureMPU6500() {

  writeRegister(0x19, 0x07); // SMPLRT_DIV

  writeRegister(0x1A, 0x04); // CONFIG

  writeRegister(0x1B, 0x08); // GYRO_CONFIG

  writeRegister(0x1C, 0x10); // ACCEL_CONFIG

}



// Read acceleration data (in m/s²)

bool readAcceleration(float &x, float &y, float &z) {

  Wire.beginTransmission(MPU6500_ADDRESS);

  Wire.write(ACCEL_XOUT_H);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6500_ADDRESS, 6, true);



  if (Wire.available() >= 6) {

    int16_t rawX = (Wire.read() << 8) | Wire.read();

    int16_t rawY = (Wire.read() << 8) | Wire.read();

    int16_t rawZ = (Wire.read() << 8) | Wire.read();



    x = rawX * ACCEL_SCALE;

    y = rawY * ACCEL_SCALE;

    z = rawZ * ACCEL_SCALE;



    return true;

  }



  return false;

}



// Read register

uint8_t readRegister(uint8_t reg) {

  Wire.beginTransmission(MPU6500_ADDRESS);

  Wire.write(reg);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6500_ADDRESS, 1, true);



  if (Wire.available()) {

    return Wire.read();

  }

  return 0;

}



// Write register

void writeRegister(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(MPU6500_ADDRESS);

  Wire.write(reg);

  Wire.write(value);

  Wire.endTransmission();

  delay(1);

}



## 설명 및 부탁



### 설명



#### 핀 연결



- 오렌지 보드의 0, 1핀은 비어둔 상태.

- 오렌지 보드의 2, 3핀은 각각 나노 TCS의 1(TX), 0(RX)

- 오렌지 보드의 4, 5핀은 각각 나노 MPU의 1(TX), 0(RX)

- 오렌지 보드의 6, 7핀은 각각 우노의 1(TX), 0(RX)



### 부탁



- 현재 코드와 기능을 바탕으로, 시리얼 통신에 오류가 있다면 이를 수정해줘.

- 현재 우노 보드가 LCD로 점수를 출력하는 코드가 존재하는지 분석하고, 없다면 이를 추가해줘.

- 오렌지 보드가 다른 보드로부터 어떤 값을 받고 이를 어떻게 처리하는지 알려줘.
