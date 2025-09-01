#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const int slaveAddress = 0x0A;

void setup() {
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent);
  
  Serial.begin(9600);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // MPU6500 설정
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_GYRO_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // 메인 루프에서는 특별히 할 일이 없습니다.
}

void requestEvent() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Z축 가속도 값을 사용해 급제동 여부 판단
  String brakeStatus = "Normal";
  // 급제동 임계값 (수치는 실제 환경에서 조정 필요)
  if (a.acceleration.z > 2.0) { 
    brakeStatus = "HardBrake";
  }
  
  Wire.write(brakeStatus.c_str());
}
