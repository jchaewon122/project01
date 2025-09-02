#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

// Software serial communication with Orange BLE
// Connect TX pin of Nano MPU (D2) to RX pin of Orange BLE (D6)
// Connect RX pin of Nano MPU (D3) to TX pin of Orange BLE (D7)
SoftwareSerial orangeBleSerial(2, 3);

// MPU6500 I2C address and registers
#define MPU6500_ADDRESS 0x68
#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C
#define ACCEL_XOUT_H    0x3B

// Variables for hard brake detection
// Changed the threshold to be less aggressive, based on previous test data.
float hardBrakeThreshold = -3.0; // Negative acceleration threshold for hard brake (m/s²)
float prevVelocity = 0.0;
float velocity = 0.0;
unsigned long lastHardBrakeTime = 0;
const unsigned long BRAKE_DETECTION_COOLDOWN = 1000; // 1s cooldown
bool mpuSensorFound = false;

// Acceleration scale factor (±8g range)
const float ACCEL_SCALE = 8.0 / 32768.0 * 9.81; // m/s² conversion

// Moving average filter for smoothing
// Reduced filter size for faster reaction to sudden changes.
const int FILTER_SIZE = 3;
float accelBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// Calibration values
float accelXOffset = 0.0;
float accelYOffset = 0.0;
float accelZOffset = 0.0;
bool calibrated = false;

void setup() {
  Serial.begin(9600); // For PC debug monitor
  orangeBleSerial.begin(9600); // For Orange BLE communication

  // Initialize buffer
  for (int i = 0; i < FILTER_SIZE; i++) {
    accelBuffer[i] = 0.0;
  }

  // I2C initialization
  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  // Initialize MPU6500 sensor
  if (initializeMPU6500()) {
    Serial.println("NANO_MPU: Found MPU6500 sensor!");
    mpuSensorFound = true;

    // MPU6500 configuration
    configureMPU6500();
    delay(100);

    // Sensor calibration (must be done while stationary)
    calibrateSensor();

    // Test read
    float testX, testY, testZ;
    if (readAcceleration(testX, testY, testZ)) {
      Serial.println("NANO_MPU: Sensor test read successful!");
    }
  } else {
    Serial.println("NANO_MPU: Failed to find MPU6500 chip");
    mpuSensorFound = false;
  }
}

void loop() {
  // Update sensor values every 50ms (20Hz)
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    float deltaTime = 0.05; // 50ms = 0.05 seconds

    String currentBrakeStatus = "Normal";
    if (mpuSensorFound) {
      float accelX, accelY, accelZ;

      if (readAcceleration(accelX, accelY, accelZ)) {
        // Apply calibration
        accelX -= accelXOffset;
        accelY -= accelYOffset;
        accelZ -= accelZOffset;

        // Forward acceleration (Y-axis is usually the forward direction)
        float forwardAccel = accelY;

        // Apply moving average filter
        accelBuffer[bufferIndex] = forwardAccel;
        bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
        if (bufferIndex == 0) bufferFilled = true;

        float smoothedAccel = getMovingAverage();

        // Hard brake detection: when negative acceleration is greater than the threshold
        if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {
          if (smoothedAccel < hardBrakeThreshold && bufferFilled) {
            currentBrakeStatus = "HardBrake";
            lastHardBrakeTime = currentTime;
          }
        }

        // Print to Serial Monitor for debug
        Serial.print("X: "); Serial.print(accelX, 2);
        Serial.print(", Y: "); Serial.print(accelY, 2);
        Serial.print(", Z: "); Serial.print(accelZ, 2);
        Serial.print(", Smoothed: "); Serial.print(smoothedAccel, 2);
        Serial.print(", Status: "); Serial.println(currentBrakeStatus);

      } else {
        currentBrakeStatus = "Error";
        Serial.println("Failed to read acceleration data");
      }
    } else {
      currentBrakeStatus = "Error";
      Serial.println("MPU sensor not found");
    }

    // Transmit data to Orange BLE with a unique ID
    orangeBleSerial.print("NANO_MPU:");
    orangeBleSerial.println(currentBrakeStatus);
  }
}

// Sensor calibration (calculate offsets while stationary)
void calibrateSensor() {
  Serial.println("Calibrating sensor... Keep device still for 3 seconds");
  
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 0;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 3000) { // Measure for 3 seconds
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
    accelZOffset = (sumZ / samples) - 9.81; // Remove gravity from Z-axis
    calibrated = true;
    
    Serial.print("Calibration complete. Offsets: X=");
    Serial.print(accelXOffset, 3); Serial.print(", Y=");
    Serial.print(accelYOffset, 3); Serial.print(", Z=");
    Serial.println(accelZOffset, 3);
  }
}

// Calculate moving average
float getMovingAverage() {
  if (!bufferFilled && bufferIndex < 2) {
    return accelBuffer[0]; // Return current value if not enough data
  }
  
  float sum = 0;
  int count = bufferFilled ? FILTER_SIZE : bufferIndex;
  
  for (int i = 0; i < count; i++) {
    sum += accelBuffer[i];
  }
  
  return sum / count;
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
  writeRegister(0x1A, 0x04); // CONFIG - 20Hz low-pass filter
  writeRegister(0x1B, 0x08); // GYRO_CONFIG
  writeRegister(0x1C, 0x10); // ACCEL_CONFIG - ±8g range
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
