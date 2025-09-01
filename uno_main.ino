#include <Wire.h>
#include <NewPing.h> // 초음파 센서 라이브러리
#include <Adafruit_TCS34725.h> // 컬러 센서 라이브러리 (TCS34725 기준)

#define SONAR_PIN_TRIG 9
#define SONAR_PIN_ECHO 10
#define MAX_DISTANCE 200

NewPing sonar(SONAR_PIN_TRIG, SONAR_PIN_ECHO, MAX_DISTANCE);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

// I2C 주소는 1부터 127 사이의 고유한 값
#define SLAVE_ADDRESS 8

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  
  // 시리얼 모니터 디버깅 용
  Serial.begin(9600);
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found");
  }
}

void loop() {
  // 별도의 작업 없음. 마스터의 요청을 기다림
}

void requestEvent() {
  // 초음파 센서 값 측정
  unsigned int uS = sonar.ping_median();
  float distanceCm = uS / US_ROUNDTRIP_CM;
  
  // 컬러 센서 값 측정 (예시로 R, G, B 값 중 하나만 사용)
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  // 검은색(0)과 흰색(1) 상태를 감지하는 로직
  String colorState = "0"; // 기본값은 0(검은색)
  if (c > 500) { // 임계값은 환경에 따라 조절
      colorState = "1"; // 1(흰색)
  }
  
  // 조도 센서 값 측정 (예시로 포토레지스터 사용)
  int lightValue = analogRead(A0);
  
  // 모든 센서 데이터를 문자열로 조합
  String data = "U_" + String(distanceCm) + ",C1_" + colorState + ",L_" + String(lightValue);
  
  Wire.write(data.c_str());
}
