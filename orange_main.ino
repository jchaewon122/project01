#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C 주소 설정 (기본값은 0x27)
// LCD는 이 코드에서 직접 제어합니다.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 우노와의 시리얼 통신 (하드웨어 시리얼)
// 오렌지 BLE의 TX(1), RX(0) 핀을 사용합니다.
// 별도의 SoftwareSerial 객체 없이 Serial로 바로 사용합니다.

// 나노 TCS와의 시리얼 통신 (소프트웨어 시리얼)
SoftwareSerial nanoTcsSerial(2, 3); // RX, TX
// 나노 MPU와의 시리얼 통신 (소프트웨어 시리얼)
SoftwareSerial nanoMpuSerial(4, 5); // RX, TX

// 운전 점수 및 감점 요인
int drivingScore = 100;
int laneChangeCount = 0;
int hardBrakeCount = 0;
int obstacleWarningCount = 0;
int solidLinePenaltyCount = 0;

// 차선 변경 감지용 상태 변수
enum LaneState {
  ON_ROAD,
  ON_LINE,
  CHANGING_LANE
};

LaneState currentLaneState = ON_ROAD;
unsigned long lineDetectionStartTime = 0;
unsigned long lineDetectionEndTime = 0;

// 디바운싱을 위한 변수들
String prevLeftColor = "Other";
String prevRightColor = "Other";
int colorStabilityCount = 0;
const int STABILITY_THRESHOLD = 3; // 3회 연속 같은 색상이어야 인정

// 실선/점선 판정 기준
const unsigned long SOLID_LINE_MIN_DURATION = 800; // 0.8초 이상이면 실선
const unsigned long DOTTED_LINE_MAX_DURATION = 400; // 0.4초 이하면 점선

// 급제동 디바운싱
unsigned long lastHardBrakeTime = 0;
const unsigned long HARD_BRAKE_COOLDOWN = 2000; // 2초 쿨다운

// 장애물 경고 디바운싱
unsigned long lastObstacleWarningTime = 0;
const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000; // 1초 쿨다운

void setup() {
  Serial.begin(9600);
  nanoTcsSerial.begin(9600);
  nanoMpuSerial.begin(9600);

  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Driving Start!");
  delay(2000);
  lcd.clear();
  
  Serial.println("Master initialized (Serial Mode)");
}

void loop() {
  // 슬레이브로부터 데이터 읽기
  String unoData = readSlaveData(Serial);
  String nanoTcsData = readSlaveData(nanoTcsSerial);
  String nanoMpuData = readSlaveData(nanoMpuSerial);
  
  // 데이터 파싱 및 검증
  String distanceStr = "-1";
  String leftColor = "Other";
  String rightColor = "Other";
  String brakeStatus = "Normal";

  // UNO 데이터 파싱 (거리, 왼쪽 색상)
  if (unoData != "") {
    int commaIndex = unoData.indexOf(',');
    if (commaIndex != -1) {
      distanceStr = unoData.substring(0, commaIndex);
      leftColor = unoData.substring(commaIndex + 1);
    }
  }
  
  // 나노 TCS 데이터 (오른쪽 색상)
  if (nanoTcsData != "") {
    rightColor = nanoTcsData;
  }
  
  // 나노 MPU 데이터 (브레이크 상태)
  if (nanoMpuData != "") {
    brakeStatus = nanoMpuData;
  }
  
  // 색상 안정화 (노이즈 제거)
  stabilizeColorDetection(leftColor, rightColor);
  
  // 점수 계산
  calculateScore(distanceStr.toInt(), leftColor, rightColor, brakeStatus);
  
  // LCD 표시 (500ms 주기)
  static unsigned long lastDisplayTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDisplayTime >= 500) {
      lastDisplayTime = currentTime;
      displayOnLcd();
  }

  // 시리얼 출력 (디버깅용)
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 1000) {
      lastDebugTime = currentTime;
      Serial.print("Raw - Uno: "); Serial.print(unoData);
      Serial.print(" | Nano TCS: "); Serial.print(nanoTcsData);
      Serial.print(" | Nano MPU: "); Serial.println(nanoMpuData);
      Serial.print("Parsed - Dist: "); Serial.print(distanceStr);
      Serial.print(" | Left: "); Serial.print(leftColor);
      Serial.print(" | Right: "); Serial.print(rightColor);
      Serial.print(" | Brake: "); Serial.println(brakeStatus);
      Serial.print("State: "); Serial.print(currentLaneState);
      Serial.print(" | Score: "); Serial.println(drivingScore);
      Serial.println("---");
  }
}

// 각 시리얼 포트에서 데이터 읽기
String readSlaveData(Stream &stream) {
    if (stream.available()) {
        String data = stream.readStringUntil('\n');
        data.trim();
        return data;
    }
    return "";
}

// 색상 감지 안정화 (노이즈 제거)
void stabilizeColorDetection(String& leftColor, String& rightColor) {
  if (leftColor == prevLeftColor && rightColor == prevRightColor) {
    colorStabilityCount++;
  } else {
    colorStabilityCount = 0;
    prevLeftColor = leftColor;
    prevRightColor = rightColor;
  }
  
  if (colorStabilityCount < STABILITY_THRESHOLD) {
    leftColor = prevLeftColor;
    rightColor = prevRightColor;
  }
}

// 개선된 차선 변경 및 실선/점선 감지
void detectLaneChangeAndType(String leftColor, String rightColor) {
  unsigned long currentTime = millis();
  bool isOnWhiteLine = (leftColor == "White" || rightColor == "White");
  
  switch (currentLaneState) {
    case ON_ROAD:
      if (isOnWhiteLine) {
        currentLaneState = ON_LINE;
        lineDetectionStartTime = currentTime;
      }
      break;
      
    case ON_LINE:
      if (!isOnWhiteLine) {
        currentLaneState = CHANGING_LANE;
        lineDetectionEndTime = currentTime;
      }
      break;
      
    case CHANGING_LANE:
      if (isOnWhiteLine) {
        currentLaneState = ON_LINE;
        lineDetectionStartTime = currentTime;
      } else {
        if (currentTime - lineDetectionEndTime > 500) { // 0.5초 후 완료
          currentLaneState = ON_ROAD;
          unsigned long lineDuration = lineDetectionEndTime - lineDetectionStartTime;
          
          if (lineDuration >= SOLID_LINE_MIN_DURATION) {
            solidLinePenaltyCount++;
            drivingScore -= 10;
          }
          laneChangeCount++;
        }
      }
      break;
  }
}

// 운전 점수 계산 (개선된 버전)
void calculateScore(int distance, String leftColor, String rightColor, String brakeStatus) {
  unsigned long currentTime = millis();
  
  detectLaneChangeAndType(leftColor, rightColor);
  
  if (brakeStatus == "HardBrake" && currentTime - lastHardBrakeTime > HARD_BRAKE_COOLDOWN) {
    hardBrakeCount++;
    drivingScore -= 10;
    lastHardBrakeTime = currentTime;
  }

  if (distance > 0 && distance < 30 && currentTime - lastObstacleWarningTime > OBSTACLE_WARNING_COOLDOWN) {
    obstacleWarningCount++;
    drivingScore -= 3;
    lastObstacleWarningTime = currentTime;
  }
  
  if (drivingScore < 0) {
    drivingScore = 0;
  }
}

// LCD에 점수 표시
void displayOnLcd() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Score: ");
  lcd.print(drivingScore);
  
  lcd.setCursor(11, 0);
  switch (currentLaneState) {
    case ON_ROAD: lcd.print("ROAD"); break;
    case ON_LINE: lcd.print("LINE"); break;
    case CHANGING_LANE: lcd.print("CHNG"); break;
  }
  
  lcd.setCursor(0, 1);
  lcd.print("HB:");
  lcd.print(hardBrakeCount);
  lcd.print(" LC:");
  lcd.print(laneChangeCount);
  lcd.print(" SL:");
  lcd.print(solidLinePenaltyCount);
}
