#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

// Software serial communication with Orange BLE
SoftwareSerial orangeBleSerial(2, 3); // RX: D2, TX: D3

// MPU6500 I2C address and registers
#define MPU6500_ADDRESS 0x68
#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C
#define ACCEL_XOUT_H    0x3B

// Variables for hard brake detection
float accelThreshold = 12.0; // Hard brake threshold (m/s²)
float prevAccelMagnitude = 0.0;
unsigned long lastHardBrakeTime = 0;
const unsigned long BRAKE_DETECTION_COOLDOWN = 1000; // 1s cooldown
bool mpuSensorFound = false;

// Acceleration scale factor (±8g range)
const float ACCEL_SCALE = 8.0 / 32768.0 * 9.81; // m/s² conversion

// 시리얼 모니터 출력을 위한 변수들
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;

void setup() {
  Serial.begin(9600); // For debug monitor
  orangeBleSerial.begin(9600); // Start serial communication with Orange BLE

  // I2C initialization
  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  // Initialize MPU6500 sensor
  if (initializeMPU6500()) {
    Serial.println("NANO MPU: Found MPU6500 sensor!");
    mpuSensorFound = true;

    // MPU6500 configuration
    configureMPU6500();
    delay(100);

    // Test read
    float testX, testY, testZ;
    if (readAcceleration(testX, testY, testZ)) {
      Serial.println("NANO MPU: Sensor test read successful!");
    }
  } else {
    Serial.println("NANO MPU: Failed to find MPU6500 chip");
    mpuSensorFound = false;
  }

  Serial.println("NANO MPU Slave initialized");
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
        // 시리얼 모니터 출력을 위해 저장
        lastAccelX = accelX;
        lastAccelY = accelY;
        lastAccelZ = accelZ;
        
        float accelMagnitude = sqrt(
          accelX * accelX +
          accelY * accelY +
          accelZ * accelZ
        );

        float accelChange = abs(accelMagnitude - prevAccelMagnitude);

        if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {
          // 급제동 감지 로직 개선
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

    // Transmit data to Orange BLE
    orangeBleSerial.println(currentBrakeStatus);
  }

  // 시리얼 모니터 출력 (1초마다)
  static unsigned long lastSerialOutput = 0;
  if (currentTime - lastSerialOutput >= 1000) {
    lastSerialOutput = currentTime;
    Serial.print("AccelX: ");
    Serial.print(lastAccelX, 2);
    Serial.print(" m/s², AccelY: ");
    Serial.print(lastAccelY, 2);
    Serial.print(" m/s², AccelZ: ");
    Serial.print(lastAccelZ, 2);
    Serial.print(" m/s² -> Brake Status: ");
    
    String currentStatus = "Normal";
    if (mpuSensorFound) {
      float accelX, accelY, accelZ;
      if (readAcceleration(accelX, accelY, accelZ)) {
        float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        float accelChange = abs(accelMagnitude - prevAccelMagnitude);
        if (accelZ > accelThreshold || accelChange > 8.0) {
          currentStatus = "HardBrake";
        }
      } else {
        currentStatus = "Error";
      }
    } else {
      currentStatus = "Error";
    }
    
    Serial.println(currentStatus);
  }
}

// MPU6500 initialization function
bool initializeMPU6500() {
  uint8_t who_am_i = readRegister(WHO_AM_I);
  if (who_am_i != 0x70) { // MPU6500
    Serial.print("WHO_AM_I: 0x");
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
