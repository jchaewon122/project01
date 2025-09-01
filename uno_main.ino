#include <Wire.h>
#include <Adafruit_TCS34725.h>

// 초음파 센서 핀 설정
#define TRIG_PIN 9
#define ECHO_PIN 10

// TCS34725 센서 설정
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// I2C 슬레이브 주소 설정
const int slaveAddress = 0x08;

void setup() {
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  
  // TCS 센서 초기화
  if (tcs.begin()) {
    Serial.println("Found TCS34725 sensor!");
  } else {
    Serial.println("No TCS34725 sensor found ... check your wiring!");
    while (1);
  }
}

void loop() {
  // 메인 루프에서는 특별히 할 일이 없습니다.
  // 데이터를 요청받았을 때만 requestEvent 함수가 호출됩니다.
}

// 마스터로부터 데이터 요청이 들어왔을 때 실행되는 함수
void requestEvent() {
  // 1. 초음파 센서 데이터 읽기
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1;

  // 2. TCS 컬러 센서 데이터 읽기
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  // 검은색과 흰색 구별 (임계값은 환경에 따라 조절 필요)
  String colorStatus;
  if (c < 300) { // 어두울 경우 (검은색)
    colorStatus = "Black";
  } else if (c > 1000) { // 밝을 경우 (흰색)
    colorStatus = "White";
  } else {
    colorStatus = "Other";
  }

  // 3. 초음파 센서 데이터와 컬러 센서 데이터를 결합하여 전송
  String dataToSend = String(distance) + "," + colorStatus;
  Wire.write(dataToSend.c_str());
}
