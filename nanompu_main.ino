#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU6050 센서 객체
Adafruit_MPU6050 mpu;

const int slaveAddress = 0x0A;

// 센서 데이터 저장용 변수
String currentBrakeStatus = "Normal";
bool mpuSensorFound = false;
bool dataReady = false;

// 급제동 감지를 위한 변수들
float accelThreshold = 12.0; // 급제동 임계값 (m/s²)
float prevAccelMagnitude = 0.0;
unsigned long lastHardBrakeTime = 0;
const unsigned long BRAKE_DETECTION_COOLDOWN = 1000; // 1초 쿨다운

void setup() {
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent);
  
  Serial.begin(9600);
  
  // MPU6050 센서 초기화
  if (mpu.begin()) {
    Serial.println("NANO2: Found MPU6050 sensor!");
    mpuSensorFound = true;
    
    // MPU6050 설정
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_GYRO_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // 센서 안정화를 위한 지연
    delay(100);
  } else {
    Serial.println("NANO2: Failed to find MPU6050 chip");
    mpuSensorFound = false;
  }
  
  // 초기 센서 읽기
  updateSensorData();
  
  Serial.println("NANO2 Slave initialized at address 0x0A");
}

void loop() {
  // 20ms 주기로 센서 값을 업데이트 (가속도는 더 빠른 샘플링 필요)
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 20) {
    lastSensorReadTime = currentTime;
    updateSensorData();
    
    // 디버그 출력 (1초에 한 번)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 1000) {
      lastDebugTime = currentTime;
      Serial.print("NANO2 Brake Status: ");
      Serial.print(currentBrakeStatus);
      Serial.print(", Sensor Found: ");
      Serial.println(mpuSensorFound ? "Yes" : "No");
    }
  }
}

void updateSensorData() {
  if (mpuSensorFound) {
    sensors_event_t accel, gyro, temp;
    
    // 센서 데이터 읽기
    if (mpu.getEvent(&accel, &gyro, &temp)) {
      // 가속도 벡터의 크기 계산
      float accelMagnitude = sqrt(
        accel.acceleration.x * accel.acceleration.x +
        accel.acceleration.y * accel.acceleration.y +
        accel.acceleration.z * accel.acceleration.z
      );
      
      // 가속도 변화량 계산 (급격한 변화 감지)
      float accelChange = abs(accelMagnitude - prevAccelMagnitude);
      
      unsigned long currentTime = millis();
      
      // 급제동 감지 로직 개선
      // 1. Z축 가속도가 급격히 증가 (앞으로 기울어짐)
      // 2. 전체 가속도 크기의 급격한 변화
      bool hardBrakeDetected = false;
      
      if (currentTime - lastHardBrakeTime > BRAKE_DETECTION_COOLDOWN) {
        // Z축 기준 급제동 감지 (차량이 앞으로 기울어짐)
        if (accel.acceleration.z > accelThreshold) {
          hardBrakeDetected = true;
        }
        
        // 가속도 변화량 기준 급제동 감지
        if (accelChange > 8.0) { // 급격한 가속도 변화
          hardBrakeDetected = true;
        }
      }
      
      if (hardBrakeDetected) {
        currentBrakeStatus = "HardBrake";
        lastHardBrakeTime = currentTime;
        Serial.print("NANO2: Hard brake detected! Z-accel: ");
        Serial.print(accel.acceleration.z);
        Serial.print(" m/s², Change: ");
        Serial.print(accelChange);
        Serial.println(" m/s²");
      } else {
        currentBrakeStatus = "Normal";
      }
      
      // 이전 가속도 값 업데이트
      prevAccelMagnitude = accelMagnitude;
      dataReady = true;
      
      // 추가 디버그 정보 (2초마다)
      if (millis() % 2000 < 20) {
        Serial.print("NANO2 Accel - X:");
        Serial.print(accel.acceleration.x, 2);
        Serial.print(" Y:");
        Serial.print(accel.acceleration.y, 2);
        Serial.print(" Z:");
        Serial.print(accel.acceleration.z, 2);
        Serial.print(" Mag:");
        Serial.print(accelMagnitude, 2);
        Serial.print(" -> ");
        Serial.println(currentBrakeStatus);
      }
    } else {
      currentBrakeStatus = "Error";
      dataReady = false;
    }
  } else {
    currentBrakeStatus = "Error";
    dataReady = false;
  }
}

void requestEvent() {
  char brakeToSend[16];
  
  // 데이터가 준비되었는지 확인
  if (dataReady) {
    // 안전한 문자열 복사
    strncpy(brakeToSend, currentBrakeStatus.c_str(), sizeof(brakeToSend) - 1);
    brakeToSend[sizeof(brakeToSend) - 1] = '\0'; // null terminator 보장
  } else {
    strcpy(brakeToSend, "Error");
  }
  
  // 데이터 전송
  int dataLength = strlen(brakeToSend);
  if (dataLength > 0) {
    Wire.write(brakeToSend, dataLength);
  } else {
    Wire.write("Error", 5);
  }
  
  // 디버그 출력
  Serial.print("NANO2 Sent: ");
  Serial.println(brakeToSend);
}
