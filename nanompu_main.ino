#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

SoftwareSerial orangeSerial(2, 3);

#define SLAVE_ID 3
#define MPU6500_ADDRESS 0x68
#define WHO_AM_I         0x75
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define ACCEL_XOUT_H     0x3B

float hardBrakeThreshold = -3.0;
unsigned long lastHardBrakeTime = 0;
const unsigned long BRAKE_DETECTION_COOLDOWN = 1000;
bool mpuSensorFound = false;
int hardBrakeCount = 0;
const float ACCEL_SCALE = 8.0 / 32768.0 * 9.81;
const int FILTER_SIZE = 3;
float accelBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;
float accelXOffset = 0.0;
float accelYOffset = 0.0;
float accelZOffset = 0.0;
bool calibrated = false;

// 함수 선언
void sendDataToMaster();
void processMPUData(unsigned long currentTime);
void calibrateSensor();
float getMovingAverage();
bool initializeMPU6500();
void configureMPU6500();
bool readAcceleration(float &x, float &y, float &z);
uint8_t readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t value);

void setup() {
  Serial.begin(9600);
  orangeSerial.begin(9600);

  for (int i = 0; i < FILTER_SIZE; i++) {
    accelBuffer[i] = 0.0;
  }

  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  if (initializeMPU6500()) {
    Serial.println("NANO_MPU: Found MPU6500 sensor!");
    mpuSensorFound = true;
    configureMPU6500();
    delay(100);
    calibrateSensor();
    float testX, testY, testZ;
    if (readAcceleration(testX, testY, testZ)) {
      Serial.println("NANO_MPU: Sensor test read successful!");
    }
  } else {
    Serial.println("NANO_MPU: Failed to find MPU6500 chip");
    mpuSensorFound = false;
  }

  Serial.print("NANO_MPU Slave initialized with ID: ");
  Serial.println(SLAVE_ID);
}

void loop() {
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();

  // Check for commands from master
  if (orangeSerial.available()) {
    String command = orangeSerial.readStringUntil('\n');
    command.trim();

    // Check for data request
    if (command == "REQ_DATA") {
      sendDataToMaster();
    }
  }

  // Sensor reading and processing
  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    processMPUData(currentTime);
  }
}

void sendDataToMaster() {
  orangeSerial.print("MPU:HardBrakeCount:");
  orangeSerial.println(hardBrakeCount);
  
  // 전송 완료 대기
  orangeSerial.flush();
  
  Serial.print("Sent to master - HardBrake Count: ");
  Serial.println(hardBrakeCount);
}

void processMPUData(unsigned long currentTime) {
  if (mpuSensorFound) {
    float accelX, accelY, accelZ;
    if (readAcceleration(accelX, accelY, accelZ)) {
      // 캘리브레이션 오프셋 적용
      accelX -= accelXOffset;
      accelY -= accelYOffset;
      accelZ -= accelZOffset;
      
      // Y축을 전진 방향 가속도로 사용 (차량이 앞으로 갈 때 +, 뒤로 갈 때 -)
      float forwardAccel = accelY;
      
      // 이동 평균 필터 적용
      accelBuffer[bufferIndex] = forwardAccel;
      bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
      if (bufferIndex == 0) bufferFilled = true;
      
      float smoothedAccel = getMovingAverage();

      // 급제동 감지: 음수이고 절댓값이 기준값보다 클 때
      if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {
        if (smoothedAccel < hardBrakeThreshold && bufferFilled) {
          lastHardBrakeTime = currentTime;
          hardBrakeCount++;
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

void calibrateSensor() {
  Serial.println("Calibrating sensor... Keep device still for 3 seconds");
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
    Serial.print("Calibration complete. Offsets: X=");
    Serial.print(accelXOffset, 3); Serial.print(", Y=");
    Serial.print(accelYOffset, 3); Serial.print(", Z=");
    Serial.println(accelZOffset, 3);
  }
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

bool readAcceleration(float &x, float &y, float &z) {
  Wire.beginTransmission(MPU6500_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false; // I2C 통신 에러
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
