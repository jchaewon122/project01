#include <Wire.h>
#include <LiquidCrystal_I2C.h> // I2C LCD 라이브러리

// LCD 주소와 크기
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 주소는 다를 수 있으니 확인 필요

// 슬레이브 주소
#define UNO_ADDRESS 8
#define NANO1_ADDRESS 9
#define NANO2_ADDRESS 10

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  delay(2000);
  lcd.clear();
}

void loop() {
  // 각 슬레이브에서 데이터 읽어오기
  String unoData = readI2CData(UNO_ADDRESS);
  String nano1Data = readI2CData(NANO1_ADDRESS);
  String nano2Data = readI2CData(NANO2_ADDRESS);

  // 데이터 가공 및 출력
  processAndDisplay(unoData, nano1Data, nano2Data);
  
  delay(1000); // 1초마다 업데이트
}

// I2C 데이터 읽기 함수
String readI2CData(int address) {
  String receivedData = "";
  Wire.requestFrom(address, 64); // 슬레이브에 64바이트 데이터 요청
  
  while (Wire.available()) {
    char c = Wire.read();
    receivedData += c;
  }
  return receivedData;
}

// 데이터 가공 및 LCD 출력 함수
void processAndDisplay(String uno, String nano1, String nano2) {
  // unoData에서 각 센서 값 파싱
  float distance = uno.substring(uno.indexOf("U_") + 2, uno.indexOf(",")).toFloat();
  String color1 = uno.substring(uno.indexOf("C1_") + 3, uno.indexOf(",", uno.indexOf("C1_"))).c_str();
  float light = uno.substring(uno.indexOf("L_") + 2).toFloat();

  // nano1Data에서 가속도 값 파싱
  // (가속도 값은 복잡하므로 X축만 예시로)
  float accelX = nano1.substring(nano1.indexOf("A_") + 2, nano1.indexOf(",")).toFloat();
  
  // nano2Data에서 컬러 값 파싱
  String color2 = nano2.substring(nano2.indexOf("C2_") + 3).c_str();

  // LCD에 출력
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("D:");
  lcd.print(distance);
  lcd.print(" C1:");
  lcd.print(color1);
  lcd.print(" C2:");
  lcd.print(color2);

  lcd.setCursor(0, 1);
  lcd.print("L:");
  lcd.print(light);
  lcd.print(" A_X:");
  lcd.print(accelX);
}
