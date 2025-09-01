#include <Wire.h>
#include <Adafruit_TCS34725.h>

// 초음파 센서 핀 설정
#define TRIG_PIN 9
#define ECHO_PIN 10

// TCS34725 센서 설정
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// I2C 슬레이브 주소 설정
const int slaveAddress = 0x08;

// 센서 데이터를 저장할 전역 변수
volatile long currentDistance = -1;
volatile String currentColorStatus = "Error";
bool tcsSensorFound = false;

void setup() {
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  
  // TCS 센서 초기화
  if (tcs.begin()) {
    Serial.println("Found TCS34725 sensor!");
    tcsSensorFound = true;
  } else {
    Serial.println("No TCS34725 sensor found ... check your wiring!");
    tcsSensorFound = false;
  }
}

void loop() {
  // 100ms 주기로 센서 값을 업데이트
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSensorReadTime >= 100) {
    lastSensorReadTime = currentTime;
    
    // 초음파 센서 데이터 읽기
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // 타임아웃 추가
    long duration = pulseIn(ECHO_PIN, HIGH, 100000); 
    
    if (duration > 0) {
      currentDistance = (duration / 2) / 29.1;
    } else {
      currentDistance = -1; // 타임아웃 발생
    }

    // TCS 컬러 센서 데이터 읽기
    if (tcsSensorFound) {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);
      if (c < 300) {
        currentColorStatus = "Black";
      } else if (c > 1000) {
        currentColorStatus = "White";
      } else {
        currentColorStatus = "Other";
      }
    } else {
      currentColorStatus = "Error";
    }

    // 시리얼 모니터로 값 출력 (디버깅용)
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.print(" cm, Color: ");
    Serial.println(currentColorStatus);
  }
}

void requestEvent() {
  // 미리 읽어둔 센서 데이터를 마스터에 전송
  char dataToSend[32]; // 전송할 데이터를 위한 char 배열
  
  // 데이터를 char 배열로 포맷
  sprintf(dataToSend, "%ld,%s", currentDistance, currentColorStatus.c_str());
  
  // char 배열을 전송
  Wire.write(dataToSend);
}
