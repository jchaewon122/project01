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

// 차선 변경 감지용 상태 변수 (이전 상태 저장)
String prevLeftColor = "Other";
String prevRightColor = "Other";
bool isChangingLane = false;

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
  // 1초 주기로 데이터를 읽도록 타이머 사용
  static unsigned long lastDataReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastDataReadTime >= 1000) {
    lastDataReadTime = currentTime;

    // 슬레이브로부터 데이터 읽기
    String unoData = readSlaveData(unoSlave);
    String nano1Data = readSlaveData(nano1Slave);
    String nano2Data = readSlaveData(nano2Slave);

    // 데이터 파싱
    String distanceStr = "-1";
    String leftColor = "Error";
    String rightColor = nano1Data;
    String brakeStatus = nano2Data;

    // unoData가 유효한지 확인하고 파싱
    int commaIndex = unoData.indexOf(',');
    if (unoData != "Error" && commaIndex != -1) {
      distanceStr = unoData.substring(0, commaIndex);
      leftColor = unoData.substring(commaIndex + 1);
    }

    // 점수 계산 및 LCD에 표시
    calculateScore(distanceStr.toInt(), leftColor, rightColor, brakeStatus);
    displayOnLcd();

    // 시리얼 모니터로 값 출력 (디버깅용)
    Serial.print("Uno: "); Serial.print(unoData);
    Serial.print(" | Nano1: "); Serial.print(nano1Data);
    Serial.print(" | Nano2: "); Serial.println(nano2Data);
    Serial.print("Parsed: Dist="); Serial.print(distanceStr);
    Serial.print(" Left="); Serial.print(leftColor);
    Serial.print(" Right="); Serial.print(rightColor);
    Serial.print(" Brake="); Serial.println(brakeStatus);
  }
}

// 슬레이브로부터 데이터 읽기 함수 (통신 오류 처리 추가)
String readSlaveData(int address) {
  Wire.requestFrom(address, 32);
  char receivedChars[32];
  int index = 0;
  unsigned long timeout = millis();
  
  while (Wire.available() && index < 31 && millis() - timeout < 500) {
    receivedChars[index++] = Wire.read();
  }
  receivedChars[index] = '\0';
  
  if (index == 0) {
    return "Error"; 
  }
  
  return String(receivedChars);
}

// 운전 점수 계산 함수
void calculateScore(int distance, String leftColor, String rightColor, String brakeStatus) {
  if (isLaneChange(leftColor, rightColor)) {
    laneChangeCount++;
    drivingScore -= 5;
    Serial.println("Lane Change Detected!");
  }

  if (brakeStatus == "HardBrake") {
    hardBrakeCount++;
    drivingScore -= 10;
    Serial.println("Hard Brake Detected!");
  }

  if (distance > 0 && distance < 30) {
    obstacleWarningCount++;
    drivingScore -= 3;
    Serial.println("Obstacle Warning! (Distance: " + String(distance) + "cm)");
  }
  
  if (drivingScore < 0) {
    drivingScore = 0;
  }
}

// 차선 변경 여부 판단 (플래그 사용으로 연속 감지 방지)
bool isLaneChange(String leftColor, String rightColor) {
  if (prevLeftColor == "Black" && leftColor == "White" && !isChangingLane) {
    isChangingLane = true;
  }
  
  if (isChangingLane && leftColor == "Black") {
    isChangingLane = false;
    prevLeftColor = leftColor;
    prevRightColor = rightColor;
    return true;
  }

  if (prevRightColor == "Black" && rightColor == "White" && !isChangingLane) {
    isChangingLane = true;
  }
  
  if (isChangingLane && rightColor == "Black") {
    isChangingLane = false;
    prevLeftColor = leftColor;
    prevRightColor = rightColor;
    return true;
  }

  prevLeftColor = leftColor;
  prevRightColor = rightColor;
  
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
