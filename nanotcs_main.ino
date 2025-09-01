#include <Wire.h>
#include <Adafruit_TCS34725.h>

// I2C 주소는 다른 보드들과 달라야 함
#define SLAVE_ADDRESS 10

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  
  Serial.begin(9600);
  
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found");
  }
}

void loop() {
  // 별도의 작업 없음
}

void requestEvent() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  
  // 검은색(0)과 흰색(1) 상태를 감지하는 로직
  String colorState = "0";
  if (c > 500) { // 임계값은 환경에 따라 조절
      colorState = "1";
  }
  
  String data = "C2_" + colorState;
  
  Wire.write(data.c_str());
}
