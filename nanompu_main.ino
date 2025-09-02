#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU6050 센서 객체
Adafruit_MPU6050 mpu;

// 급제동 감지를 위한 변수들
float accelThreshold = 12.0; // 급제동 임계값 (m/s²)
float prevAccelMagnitude = 0.0;
unsigned long lastHardBrakeTime = 0;
const unsigned long BRAKE_DETECTION_COOLDOWN = 1000; // 1초 쿨다운
bool mpuSensorFound = false;

void setup() {
  Serial.begin(9600); // 오렌지 BLE와의 시리얼 통신 시작
  
  // MPU6050 센서 초기화
  if (mpu.begin()) {
    Serial.println("NANO2: Found MPU6050 sensor!");
    mpuSensorFound = true;
    
    // MPU6050 설정
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
  // 20ms 주기로 센서 값을 업데이트
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
    
    // 데이터 전송
    Serial.println(currentBrakeStatus);
  }
}
