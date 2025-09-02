#include <Adafruit_TCS34725.h>

// TCS34725 센서 설정
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// 센서 데이터 저장용 변수
String currentColorStatus = "Other";
bool tcsSensorFound = false;

void setup() {
  Serial.begin(9600); // 오렌지 BLE와의 시리얼 통신 시작
  
  // TCS 센서 초기화
  if (tcs.begin()) {
    Serial.println("NANO1: Found TCS34725 sensor!");
    tcsSensorFound = true;
  } else {
    Serial.println("NANO1: No TCS34725 sensor found!");
    tcsSensorFound = false;
  }
  
  Serial.println("NANO1 Slave initialized (Serial Mode)");
}

void loop() {
  // 50ms 주기로 센서 값을 업데이트하고 전송
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    
    if (tcsSensorFound) {
      uint16_t r, g, b, c;
      if (tcs.getRawData(&r, &g, &b, &c)) {
        if (c < 200) {
          currentColorStatus = "Black";
        } else if (c > 800) {
          currentColorStatus = "White";
        } else {
          currentColorStatus = "Other";
        }
      } else {
        currentColorStatus = "Error";
      }
    } else {
      currentColorStatus = "Error";
    }
    
    // 데이터 전송
    Serial.println(currentColorStatus);
  }
}
