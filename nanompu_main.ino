#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU6050 sensor object
Adafruit_MPU6050 mpu;

// Variables for hard brake detection
float accelThreshold = 12.0; // Hard brake threshold (m/s²)
float prevAccelMagnitude = 0.0;
unsigned long lastHardBrakeTime = 0;
const unsigned long BRAKE_DETECTION_COOLDOWN = 1000; // 1s cooldown
bool mpuSensorFound = false;

void setup() {
  Serial.begin(9600); // Start serial communication with Orange BLE
  
  // Initialize MPU6050 sensor
  if (mpu.begin()) {
    Serial.println("NANO2: Found MPU6050 sensor!");
    mpuSensorFound = true;
    
    // MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_GYRO_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    delay(100);
  } else {
    Serial.println("NANO2: Failed to find MPU6050 chip");
    mpuSensorFound = false;
  }
  
  Serial.println("NANO2 Slave initialized (Serial Mode)");
}

void loop() {
  // Update sensor values every 20ms
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 20) {
    lastSensorReadTime = currentTime;
    
    String currentBrakeStatus = "Normal";
    if (mpuSensorFound) {
      sensors_event_t accel, gyro, temp;
      
      if (mpu.getEvent(&accel, &gyro, &temp)) {
        float accelMagnitude = sqrt(
          accel.acceleration.x * accel.acceleration.x +
          accel.acceleration.y * accel.acceleration.y +
          accel.acceleration.z * accel.acceleration.z
        );
        
        float accelChange = abs(accelMagnitude - prevAccelMagnitude);
        
        if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {
          if (accel.acceleration.z > accelThreshold || accelChange > 8.0) {
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
    Serial.println(currentBrakeStatus);
  }
}
