#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// I2C 주소는 UNO와 달라야 함
#define SLAVE_ADDRESS 9

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  
  // 시리얼 모니터 디버깅 용
  Serial.begin(9600);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void loop() {
  // 별도의 작업 없음
}

void requestEvent() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // 가속도 센서 값 (X, Y, Z)을 문자열로 조합
  String data = "A_" + String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z);
  
  Wire.write(data.c_str());
}
