#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>
#include <math.h>
#include <string.h> // strcmp 함수 사용을 위해 추가

// 5V 수동 부저 핀 설정. tone() 함수를 사용하므로 PWM 핀이 아니어도 됩니다.
#define BUZZER_PIN 8

// LCD I2C 설정
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 초음파 센서 핀 설정
#define TRIG_PIN 9
#define ECHO_PIN 10

// TCS34725 색상 센서 설정
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

// MPU6500 레지스터 주소
#define MPU6500_ADDRESS 0x68
#define WHO_AM_I         0x75
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define ACCEL_XOUT_H     0x3B

// 점수 및 카운터 변수
int drivingScore = 100;
int hardBrakeCount = 0;
int obstacleWarningCount = 0;
int laneChangeCount = 0;

// 초음파 센서 관련 변수
long currentDistance = -1;
unsigned long lastObstacleWarningTime = 0;
const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000;
const int OBSTACLE_BUZZER_THRESHOLD = 10; // 소리가 나기 시작할 거리 (cm)

// TCS 센서 관련 변수 (String -> const char* 로 변경)
const char* currentColorStatus = "Other";
const char* previousColorStatus = "Other";
bool tcsSensorFound = false;
unsigned long lastLaneChangeTime = 0;
const unsigned long LANE_CHANGE_COOLDOWN = 200; // 차선 변경 감지 쿨다운 시간

// 수동 부저 관련 변수
bool buzzerOn = false;
unsigned long lastBuzzTime = 0;
const int BUZZER_FREQUENCY = 1000; // 부저 음높이 (Hz), 수동 부저만 가능
const int MIN_PULSE_DELAY = 50; // 최대 소리 속도 (가장 짧은 간격)
const int MAX_PULSE_DELAY = 500; // 최소 소리 속도 (가장 긴 간격)

// MPU6500 관련 변수
float hardBrakeThreshold = -3.0;
unsigned long lastHardBrakeTime = 0;
const unsigned long BRAKE_DETECTION_COOLDOWN = 1000;
bool mpuSensorFound = false;
const float ACCEL_SCALE = 8.0 / 32768.0 * 9.81;

// 가속도 필터링을 위한 변수
const int FILTER_SIZE = 3;
float accelBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// 캘리브레이션 오프셋
float accelXOffset = 0.0;
float accelYOffset = 0.0;
float accelZOffset = 0.0;
bool calibrated = false;

// 타이밍 변수
unsigned long lastSensorReadTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastDebugTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 100;
const unsigned long DISPLAY_INTERVAL = 500;
const unsigned long DEBUG_INTERVAL = 2000;

// LCD 디스플레이 모드
int displayMode = 0; // 0: 메인화면, 1: 센서상태, 2: 상세정보
const int MAX_DISPLAY_MODE = 2;
unsigned long displayModeChangeTime = 0;
const unsigned long DISPLAY_MODE_DURATION = 3000; // 각 화면을 3초씩 보여줌

// 함수 선언
void readUltrasonicSensor();
void readColorSensor();
void detectLaneChange();
void buzzIfObstacleDetected();
bool initializeMPU6500();
void configureMPU6500();
void calibrateSensor();
bool readAcceleration(float &x, float &y, float &z);
float getMovingAverage();
void processMPUData();
void calculateScore();
void displayOnLcd();
void printDebugInfo();
uint8_t readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t value);

void setup() {
  Serial.begin(9600);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("System Starting..")); // F() 매크로 사용
  delay(1000);
  
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  
  if (tcs.begin()) {
    tcsSensorFound = true;
    Serial.println(F("Found TCS34725 color sensor!")); // F() 매크로 사용
    lcd.setCursor(0, 1);
    lcd.print(F("TCS: OK")); // F() 매크로 사용
    delay(500);
  } else {
    tcsSensorFound = false;
    Serial.println(F("No TCS34725 color sensor found!")); // F() 매크로 사용
    lcd.setCursor(0, 1);
    lcd.print(F("TCS: ERROR")); // F() 매크로 사용
    delay(500);
  }
  
  for (int i = 0; i < FILTER_SIZE; i++) {
    accelBuffer[i] = 0.0;
  }
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Init MPU6500...")); // F() 매크로 사용
  
  if (initializeMPU6500()) {
    Serial.println(F("Found MPU6500 sensor!")); // F() 매크로 사용
    mpuSensorFound = true;
    configureMPU6500();
    delay(100);
    calibrateSensor();
    
    float testX, testY, testZ;
    if (readAcceleration(testX, testY, testZ)) {
      Serial.println(F("Sensor test read successful!")); // F() 매크로 사용
    }
  } else {
    Serial.println(F("Failed to find MPU6500 chip")); // F() 매크로 사용
    mpuSensorFound = false;
    lcd.setCursor(0, 1);
    lcd.print(F("MPU: ERROR")); // F() 매크로 사용
    delay(1000);
  }
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("All Systems")); // F() 매크로 사용
  lcd.setCursor(0, 1);
  lcd.print(F("Ready!")); // F() 매크로 사용
  delay(1500);
  lcd.clear();
  
  Serial.println(F("=== Integrated Arduino System ===")); // F() 매크로 사용
  Serial.println(F("Sensors: Ultrasonic + MPU6500 + TCS34725 + LCD + Buzzer")); // F() 매크로 사용
  Serial.print(F("TCS Status: ")); // F() 매크로 사용
  Serial.println(tcsSensorFound ? F("ACTIVE") : F("ERROR")); // F() 매크로 사용
  Serial.print(F("MPU Status: ")); // F() 매크로 사용
  Serial.println(mpuSensorFound ? F("ACTIVE") : F("ERROR")); // F() 매크로 사용
  Serial.println(F("===============================")); // F() 매크로 사용
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    readUltrasonicSensor();
    readColorSensor();
    detectLaneChange();
    processMPUData();
  }
  
  buzzIfObstacleDetected();
  calculateScore();
  
  if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = currentTime;
    
    if (currentTime - displayModeChangeTime >= DISPLAY_MODE_DURATION) {
      displayMode = (displayMode + 1) % (MAX_DISPLAY_MODE + 1);
      displayModeChangeTime = currentTime;
    }
    
    displayOnLcd();
  }
  
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
    lastDebugTime = currentTime;
    printDebugInfo();
  }
}

void readUltrasonicSensor() {
  unsigned long currentTime = millis();
  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration > 0) {
    currentDistance = (duration * 0.034) / 2;
    
    if (currentDistance < 2) currentDistance = 2;
    if (currentDistance > 400) currentDistance = 400;
    
    if (currentDistance > 0 && currentDistance < 30 && 
        currentTime - lastObstacleWarningTime > OBSTACLE_WARNING_COOLDOWN) {
      obstacleWarningCount++;
      lastObstacleWarningTime = currentTime;
      drivingScore -= 3;
      if (drivingScore < 0) drivingScore = 0;
      
      Serial.print(F("Obstacle warning! Distance: "));
      Serial.print(currentDistance);
      Serial.print(F("cm, Count: "));
      Serial.println(obstacleWarningCount);
    }
  } else {
    currentDistance = -1;
  }
}

void readColorSensor() {
  if (!tcsSensorFound) {
    currentColorStatus = "Error"; // String 객체 사용
    return;
  }
  
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  uint32_t rgbSum = (uint32_t)r + g + b;
  const uint32_t COLOR_THRESHOLD_RGB = 7000;

  // **(변경)** currentColorStatus를 String 대신 const char*로 변경
  if (rgbSum < COLOR_THRESHOLD_RGB) {
    currentColorStatus = "Black";
  } else {
    currentColorStatus = "White";
  }
}

void detectLaneChange() {
  unsigned long currentTime = millis();
  
  // **(변경)** String 비교 대신 strcmp() 사용
  if (currentTime - lastLaneChangeTime > LANE_CHANGE_COOLDOWN) {
    if ((strcmp(previousColorStatus, "White") == 0 && strcmp(currentColorStatus, "Black") == 0) ||
        (strcmp(previousColorStatus, "Black") == 0 && strcmp(currentColorStatus, "White") == 0)) {
      
      laneChangeCount++;
      lastLaneChangeTime = currentTime;
      drivingScore -= 2;
      if (drivingScore < 0) drivingScore = 0;
      
      Serial.print(F("Lane change detected! "));
      Serial.print(previousColorStatus);
      Serial.print(F(" -> "));
      Serial.print(currentColorStatus);
      Serial.print(F(", Count: "));
      Serial.println(laneChangeCount);
    }
  }
  
  // **(수정)** 현재 색상이 이전 색상과 다를 때만 업데이트
  if (strcmp(previousColorStatus, currentColorStatus) != 0) {
      previousColorStatus = currentColorStatus;
  }
}

void buzzIfObstacleDetected() {
  if (currentDistance > 0 && currentDistance <= OBSTACLE_BUZZER_THRESHOLD) {
    long pulseDelay = map(currentDistance, 2, OBSTACLE_BUZZER_THRESHOLD, MIN_PULSE_DELAY, MAX_PULSE_DELAY);
    
    if (millis() - lastBuzzTime >= pulseDelay) {
      lastBuzzTime = millis();
      
      if (buzzerOn) {
        noTone(BUZZER_PIN);
        buzzerOn = false;
      } else {
        tone(BUZZER_PIN, BUZZER_FREQUENCY);
        buzzerOn = true;
      }
    }
  } else {
    if (buzzerOn) {
      noTone(BUZZER_PIN);
      buzzerOn = false;
    }
  }
}

void processMPUData() {
  if (mpuSensorFound) {
    unsigned long currentTime = millis();
    float accelX, accelY, accelZ;
    
    if (readAcceleration(accelX, accelY, accelZ)) {
      accelX -= accelXOffset;
      accelY -= accelYOffset;
      accelZ -= accelZOffset;
      
      float forwardAccel = accelY;
      
      accelBuffer[bufferIndex] = forwardAccel;
      bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
      if (bufferIndex == 0) bufferFilled = true;
      
      float smoothedAccel = getMovingAverage();
      
      if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {
        if (smoothedAccel < hardBrakeThreshold && bufferFilled) {
          lastHardBrakeTime = currentTime;
          hardBrakeCount++;
          drivingScore -= 5;
          if (drivingScore < 0) drivingScore = 0;
          
          Serial.print(F("Hard brake detected! Accel: "));
          Serial.print(smoothedAccel, 3);
          Serial.print(F(" m/s², Count: "));
          Serial.println(hardBrakeCount);
        }
      }
    } else {
      Serial.println(F("Failed to read MPU acceleration data"));
    }
  }
}

void calculateScore() {
  // 점수는 실시간으로 각 이벤트에서 차감됨
}

void displayOnLcd() {
  lcd.clear();
  
  switch(displayMode) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print(F("Score:"));
      if (drivingScore >= 100) {
        lcd.print(drivingScore);
      } else if (drivingScore >= 10) {
        lcd.print(F(" "));
        lcd.print(drivingScore);
      } else {
        lcd.print(F("  "));
        lcd.print(drivingScore);
      }
      
      lcd.setCursor(11, 0);
      if (currentDistance >= 0) {
        if (currentDistance >= 100) {
          lcd.print(currentDistance);
        } else if (currentDistance >= 10) {
          lcd.print(F(" "));
          lcd.print(currentDistance);
        } else {
          lcd.print(F("  "));
          lcd.print(currentDistance);
        }
        lcd.print(F("cm"));
      } else {
        lcd.print(F(" --cm"));
      }
      
      lcd.setCursor(0, 1);
      lcd.print(F("Color:"));
      // **(변경)** String 비교 대신 strcmp() 사용
      if (strcmp(currentColorStatus, "White") == 0) {
        lcd.print(F("WHT"));
      } else if (strcmp(currentColorStatus, "Black") == 0) {
        lcd.print(F("BLK"));
      } else if (strcmp(currentColorStatus, "Error") == 0) {
        lcd.print(F("ERR"));
      } else {
        lcd.print(F("OTH"));
      }
      
      lcd.setCursor(11, 1);
      lcd.print(F("HB:"));
      if (hardBrakeCount >= 10) {
        lcd.print(hardBrakeCount);
      } else {
        lcd.print(F(" "));
        lcd.print(hardBrakeCount);
      }
      break;
      
    case 1:
      lcd.setCursor(0, 0);
      lcd.print(F("Sensor Status"));
      
      lcd.setCursor(0, 1);
      lcd.print(F("TCS:"));
      lcd.print(tcsSensorFound ? F("OK") : F("ER"));
      
      lcd.setCursor(7, 1);
      lcd.print(F("MPU:"));
      lcd.print(mpuSensorFound ? F("OK") : F("ER"));
      
      lcd.setCursor(14, 1);
      lcd.print(millis() / 1000 % 100);
      break;
      
    case 2:
      lcd.setCursor(0, 0);
      lcd.print(F("OB:"));
      if (obstacleWarningCount >= 10) {
        lcd.print(obstacleWarningCount);
      } else {
        lcd.print(F(" "));
        lcd.print(obstacleWarningCount);
      }
      
      lcd.setCursor(6, 0);
      lcd.print(F("LC:"));
      if (laneChangeCount >= 10) {
        lcd.print(laneChangeCount);
      } else {
        lcd.print(F(" "));
        lcd.print(laneChangeCount);
      }
      
      lcd.setCursor(12, 0);
      lcd.print(F("UP:"));
      lcd.print(millis() / 60000);
      
      lcd.setCursor(0, 1);
      lcd.print(F("Driving Analysis"));
      break;
  }
}

void printDebugInfo() {
  Serial.println(F("=== INTEGRATED SYSTEM DEBUG ==="));
  Serial.print(F("Distance: "));
  Serial.print(currentDistance);
  Serial.println(F("cm"));
  Serial.print(F("Color Status: "));
  Serial.println(currentColorStatus);
  Serial.print(F("Hard Brake Count: "));
  Serial.println(hardBrakeCount);
  Serial.print(F("Obstacle Warning Count: "));
  Serial.println(obstacleWarningCount);
  Serial.print(F("Lane Change Count: "));
  Serial.println(laneChangeCount);
  Serial.print(F("Current Score: "));
  Serial.println(drivingScore);
  Serial.print(F("TCS34725 Status: "));
  Serial.println(tcsSensorFound ? F("ACTIVE") : F("ERROR"));
  Serial.print(F("MPU6500 Status: "));
  Serial.println(mpuSensorFound ? F("ACTIVE") : F("ERROR"));
  Serial.print(F("System Uptime: "));
  Serial.print(millis() / 1000);
  Serial.println(F(" seconds"));
  Serial.print(F("Display Mode: "));
  Serial.println(displayMode);
  Serial.println(F("==============================="));
}

float getMovingAverage() {
  if (!bufferFilled && bufferIndex < 2) {
    return accelBuffer[0];
  }
  
  float sum = 0;
  int count = bufferFilled ? FILTER_SIZE : bufferIndex;
  for (int i = 0; i < count; i++) {
    sum += accelBuffer[i];
  }
  return sum / count;
}

bool initializeMPU6500() {
  uint8_t who_am_i = readRegister(WHO_AM_I);
  if (who_am_i != 0x70) {
    Serial.print(F("MPU6500 WHO_AM_I check failed. Got: 0x"));
    Serial.println(who_am_i, HEX);
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

void configureMPU6500() {
  writeRegister(0x19, 0x07);
  writeRegister(0x1A, 0x04);
  writeRegister(0x1B, 0x08);
  writeRegister(0x1C, 0x10);
}

void calibrateSensor() {
  Serial.println(F("Calibrating sensor... Keep device still for 3 seconds"));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Calibrating..."));
  lcd.setCursor(0, 1);
  lcd.print(F("Keep still 3sec"));
  
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 0;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 3000) {
    float x, y, z;
    if (readAcceleration(x, y, z)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      samples++;
    }
    delay(20);
  }
  
  if (samples > 0) {
    accelXOffset = sumX / samples;
    accelYOffset = sumY / samples;
    accelZOffset = (sumZ / samples) - 9.81;
    calibrated = true;
    
    Serial.print(F("Calibration complete. Offsets: X="));
    Serial.print(accelXOffset, 3);
    Serial.print(F(", Y="));
    Serial.print(accelYOffset, 3);
    Serial.print(F(", Z="));
    Serial.println(accelZOffset, 3);
  }
  
  lcd.clear();
}

bool readAcceleration(float &x, float &y, float &z) {
  Wire.beginTransmission(MPU6500_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  
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

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6500_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  delay(1);
}
