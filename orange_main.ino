#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C 주소 설정 (기본값은 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// 슬레이브 주소
const int unoSlave = 0x08;
const int nano1Slave = 0x09;
const int nano2Slave = 0x0A;

// 운전 점수 및 감점 요인
int drivingScore = 100;
int laneChangeCount = 0;
int hardBrakeCount = 0;
int obstacleWarningCount = 0;

// 차선 변경 감지용 상태 변수
String lastLeftColor = "";
String lastRightColor = "";

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Driving Start!");
  delay(2000);
  lcd.clear();
}

void loop() {
  // 1초마다 데이터 읽기
  delay(1000);

  // 슬레이브로부터 데이터 읽기
  String unoData = readSlaveData(unoSlave);
  String nano1Data = readSlaveData(nano1Slave);
  String nano2Data = readSlaveData(nano2Slave);

  // 데이터 파싱
  String distanceStr = unoData.substring(0, unoData.indexOf(','));
  String leftColor = unoData.substring(unoData.indexOf(',') + 1);
  String rightColor = nano1Data;
  String brakeStatus = nano2Data;

  // 점수 계산 및 LCD에 표시
  calculateScore(distanceStr.toInt(), leftColor, rightColor, brakeStatus);
  displayOnLcd();
}

// 슬레이브로부터 데이터 읽기 함수
String readSlaveData(int address) {
  Wire.requestFrom(address, 32); // 32바이트 요청
  String receivedData = "";
  while (Wire.available()) {
    char c = Wire.read();
    receivedData += c;
  }
  return receivedData;
}

// 운전 점수 계산 함수
void calculateScore(int distance, String leftColor, String rightColor, String brakeStatus) {
  // 1. 차선 변경 감지
  if (isLaneChange(leftColor, rightColor)) {
    laneChangeCount++;
    drivingScore -= 5;
    Serial.println("Lane Change Detected!");
  }

  // 2. 급브레이크 감지
  if (brakeStatus == "HardBrake") {
    hardBrakeCount++;
    drivingScore -= 10;
    Serial.println("Hard Brake Detected!");
  }

  // 3. 앞 차 간격 감지
  if (distance < 30 && distance > 0) { // 30cm 이내 접근 시 감점
    obstacleWarningCount++;
    drivingScore -= 3;
    Serial.println("Obstacle Warning! (Distance: " + String(distance) + "cm)");
  }
  
  // 점수 하한선
  if (drivingScore < 0) {
    drivingScore = 0;
  }
}

// 차선 변경 여부 판단
bool isLaneChange(String leftColor, String rightColor) {
  if (lastLeftColor == "Black" && leftColor == "White" && rightColor == "Black") {
    lastLeftColor = leftColor;
    lastRightColor = rightColor;
    return true;
  } else if (lastRightColor == "Black" && rightColor == "White" && leftColor == "Black") {
    lastLeftColor = leftColor;
    lastRightColor = rightColor;
    return true;
  }
  lastLeftColor = leftColor;
  lastRightColor = rightColor;
  return false;
}

// LCD에 점수 표시
void displayOnLcd() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Score: ");
  lcd.print(drivingScore);
  lcd.setCursor(0, 1);
  lcd.print("HB:");
  lcd.print(hardBrakeCount);
  lcd.print(" LC:");
  lcd.print(laneChangeCount);
  lcd.print(" OW:");
  lcd.print(obstacleWarningCount);
}
