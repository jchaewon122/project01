#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// 5V 수동 부저 핀 설정. tone() 함수를 사용하므로 PWM 핀이 아니어도 됩니다.
#define BUZZER_PIN 8

// LCD I2C 설정
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 초음파 센서 핀 설정
#define TRIG_PIN 9
#define ECHO_PIN 10

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

// 초음파 센서 관련 변수
long currentDistance = -1;
unsigned long lastObstacleWarningTime = 0;
const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000;
const int OBSTACLE_BUZZER_THRESHOLD = 10; // 소리가 나기 시작할 거리 (cm)

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

// 함수 선언
void readUltrasonicSensor();
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
  
  // 부저 핀 설정 (수동 부저는 핀모드만 설정)
  pinMode(BUZZER_PIN, OUTPUT);
  
  // 초음파 센서 핀 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Starting..");
  delay(1000);
  
  // I2C 초기화
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  
  // 가속도 버퍼 초기화
  for (int i = 0; i < FILTER_SIZE; i++) {
    accelBuffer[i] = 0.0;
  }
  
  // MPU6500 초기화
  if (initializeMPU6500()) {
    Serial.println("Found MPU6500 sensor!");
    mpuSensorFound = true;
    configureMPU6500();
    delay(100);
    calibrateSensor();
    
    // 테스트 읽기
    float testX, testY, testZ;
    if (readAcceleration(testX, testY, testZ)) {
      Serial.println("Sensor test read successful!");
    }
  } else {
    Serial.println("Failed to find MPU6500 chip");
    mpuSensorFound = false;
  }
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  delay(1000);
  lcd.clear();
  
  Serial.println("Integrated Arduino system initialized");
  Serial.println("Sensors: Ultrasonic + MPU6500 + LCD + Buzzer");
}

void loop() {
  unsigned long currentTime = millis();
  
  // 센서 데이터 읽기 (100ms마다)
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    readUltrasonicSensor();
    processMPUData();
  }
  
  // 부저 울리기
  buzzIfObstacleDetected();
  
  // 점수 계산
  calculateScore();
  
  // LCD 디스플레이 업데이트 (500ms마다)
  if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = currentTime;
    displayOnLcd();
  }
  
  // 디버그 정보 출력 (2초마다)
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
    lastDebugTime = currentTime;
    printDebugInfo();
  }
}

void readUltrasonicSensor() {
  unsigned long currentTime = millis();
  
  // 초음파 센서 트리거
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 에코 시간 측정
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration > 0) {
    currentDistance = (duration * 0.034) / 2;
    
    // 거리 범위 제한
    if (currentDistance < 2) currentDistance = 2;
    if (currentDistance > 400) currentDistance = 400;
    
    // 장애물 경고 로직 (30cm 이내)
    if (currentDistance > 0 && currentDistance < 30 && 
        currentTime - lastObstacleWarningTime > OBSTACLE_WARNING_COOLDOWN) {
      obstacleWarningCount++;
      lastObstacleWarningTime = currentTime;
      drivingScore -= 3;  // 장애물 접근 패널티
      if (drivingScore < 0) drivingScore = 0;
      
      Serial.print("Obstacle warning! Distance: ");
      Serial.print(currentDistance);
      Serial.print("cm, Count: ");
      Serial.println(obstacleWarningCount);
    }
  } else {
    currentDistance = -1;  // 측정 실패
  }
}

void buzzIfObstacleDetected() {
  // 장애물 감지 범위 내에 있으면
  if (currentDistance > 0 && currentDistance <= OBSTACLE_BUZZER_THRESHOLD) {
    // 거리에 따라 펄스 속도 조절
    // 거리가 30cm -> 500ms 간격, 2cm -> 50ms 간격으로 소리
    long pulseDelay = map(currentDistance, 2, OBSTACLE_BUZZER_THRESHOLD, MIN_PULSE_DELAY, MAX_PULSE_DELAY);
    
    // 마지막 소리 발생 시간으로부터 충분한 시간이 지났다면
    if (millis() - lastBuzzTime >= pulseDelay) {
      lastBuzzTime = millis();
      
      if (buzzerOn) {
        noTone(BUZZER_PIN); // 소리 끄기
        buzzerOn = false;
      } else {
        tone(BUZZER_PIN, BUZZER_FREQUENCY); // 소리 켜기 (특정 주파수)
        buzzerOn = true;
      }
    }
  } else {
    // 장애물이 감지되지 않으면 부저 끄기
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
      // 캘리브레이션 오프셋 적용
      accelX -= accelXOffset;
      accelY -= accelYOffset;
      accelZ -= accelZOffset;
      
      // 전진/후진 가속도 (Y축)
      float forwardAccel = accelY;
      
      // 이동평균 필터 적용
      accelBuffer[bufferIndex] = forwardAccel;
      bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
      if (bufferIndex == 0) bufferFilled = true;
      
      float smoothedAccel = getMovingAverage();
      
      // 급브레이크 감지
      if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {
        if (smoothedAccel < hardBrakeThreshold && bufferFilled) {
          lastHardBrakeTime = currentTime;
          hardBrakeCount++;
          drivingScore -= 5;  // 급브레이크 패널티
          if (drivingScore < 0) drivingScore = 0;
          
          Serial.print("Hard brake detected! Accel: ");
          Serial.print(smoothedAccel, 3);
          Serial.print(" m/s², Count: ");
          Serial.println(hardBrakeCount);
        }
      }
    } else {
      Serial.println("Failed to read MPU acceleration data");
    }
  }
}

void calculateScore() {
  // 점수는 실시간으로 각 이벤트에서 차감됨
}

void displayOnLcd() {
  lcd.clear();
  
  // 첫 번째 줄: 점수와 거리
  lcd.setCursor(0, 0);
  lcd.print("Score: ");
  lcd.print(drivingScore);
  
  lcd.setCursor(11, 0);
  if (currentDistance >= 0) {
    lcd.print(currentDistance);
    lcd.print("cm");
  } else {
    lcd.print("--cm");
  }
  
  // 두 번째 줄: 급브레이크와 장애물 카운트
  lcd.setCursor(0, 1);
  lcd.print("HB:");
  lcd.print(hardBrakeCount);
  
  lcd.setCursor(8, 1);
  lcd.print("OB:");
  lcd.print(obstacleWarningCount);
  
  // MPU 상태 표시
  lcd.setCursor(14, 1);
  if (mpuSensorFound) {
    lcd.print("OK");
  } else {
    lcd.print("ER");
  }
}

void printDebugInfo() {
  Serial.println("=== CONTEST DEBUG INFO ===");
  Serial.print("Distance: ");
  Serial.print(currentDistance);
  Serial.println("cm");
  Serial.print("Hard Brake Count: ");
  Serial.println(hardBrakeCount);
  Serial.print("Obstacle Warning Count: ");
  Serial.println(obstacleWarningCount);
  Serial.print("Current Score: ");
  Serial.println(drivingScore);
  Serial.print("MPU6500 Status: ");
  Serial.println(mpuSensorFound ? "ACTIVE" : "ERROR");
  Serial.print("System Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.println("========================");
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
    Serial.print("MPU6500 WHO_AM_I check failed. Got: 0x");
    Serial.println(who_am_i, HEX);
    return false;
  }
  
  // 리셋
  writeRegister(PWR_MGMT_1, 0x80);
  delay(100);
  
  // 파워 온
  writeRegister(PWR_MGMT_1, 0x00);
  delay(50);
  writeRegister(PWR_MGMT_2, 0x00);
  delay(10);
  
  return true;
}

void configureMPU6500() {
  // 샘플 레이트 설정 (125Hz)
  writeRegister(0x19, 0x07);
  
  // 디지털 로우패스 필터 설정
  writeRegister(0x1A, 0x04);
  
  // 자이로스코프 설정 (±500dps)
  writeRegister(0x1B, 0x08);
  
  // 가속도계 설정 (±8g)
  writeRegister(0x1C, 0x10);
}

void calibrateSensor() {
  Serial.println("Calibrating sensor... Keep device still for 3 seconds");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Keep still 3sec");
  
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
    accelZOffset = (sumZ / samples) - 9.81;  // 중력 보정
    calibrated = true;
    
    Serial.print("Calibration complete. Offsets: X=");
    Serial.print(accelXOffset, 3);
    Serial.print(", Y=");
    Serial.print(accelYOffset, 3);
    Serial.print(", Z=");
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
