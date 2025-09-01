#include <Wire.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

const int slaveAddress = 0x09;

void setup() {
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent);
  
  Serial.begin(9600);
  
  if (tcs.begin()) {
    Serial.println("Found TCS34725 sensor!");
  } else {
    Serial.println("No TCS34725 sensor found ... check your wiring!");
    while (1);
  }
}

void loop() {
  // 메인 루프에서는 특별히 할 일이 없습니다.
}

void requestEvent() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  char colorStatus[10];
  if (c < 300) {  
    strcpy(colorStatus, "Black");
  } else if (c > 1000) {
    strcpy(colorStatus, "White");
  } else {
    strcpy(colorStatus, "Other");
  }
  
  Wire.write(colorStatus);
}
