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
int solidLinePenaltyCount = 0;

// 차선 변경 감지용 상태 변수 (개선된 버전)
enum LaneState {
  ON_ROAD,           // 도로 위 (검은색)
  ON_LINE,           // 차선 위 (흰색)
  CHANGING_LANE      // 차선 변경 중
};

LaneState currentLaneState = ON_ROAD;
unsigned long lineDetectionStartTime = 0;
unsigned long lineDetectionEndTime = 0;
bool lineChangeCompleted = false;

// 디바운싱을 위한 변수들
String prevLeftColor = "Other";
String prevRightColor = "Other";
int colorStabilityCount = 0;
const int STABILITY_THRESHOLD = 3; // 3회 연속 같은 색상이어야 인정

// 실선/점선 판정 기준 (조정된 값)
const unsigned long SOLID_LINE_MIN_DURATION = 800;  // 0.8초 이상이면 실선
const unsigned long DOTTED_LINE_MAX_DURATION = 400; // 0.4초 이하면 점선

// 급제동 디바운싱
unsigned long lastHardBrakeTime = 0;
const unsigned long HARD_BRAKE_COOLDOWN = 2000; // 2초 쿨다운

// 장애물 경고 디바운싱
unsigned long lastObstacleWarningTime = 0;
const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000; // 1초 쿨다운

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
  static unsigned long lastDataReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastDataReadTime >= 50) { // 더 빠른 샘플링 (20Hz)
    lastDataReadTime = currentTime;

    // 슬레이브 데이터 읽기
    String unoData = readSlaveData(unoSlave);
    String nano1Data = readSlaveData(nano1Slave);
    String nano2Data = readSlaveData(nano2Slave);

    // 데이터 파싱
    String distanceStr = "-1";
    String leftColor = "Other";
    String rightColor = "Other";
    String brakeStatus = "Other";

    int commaIndex = unoData.indexOf(',');
    if (unoData != "Error" && commaIndex != -1) {
      distanceStr = unoData.substring(0, commaIndex);
      leftColor = unoData.substring(commaIndex + 1);
    }
    
    if (nano1Data != "Error") {
      rightColor = nano1Data;
    }
    
    if (nano2Data != "Error") {
      brakeStatus = nano2Data;
    }
    
    // 색상 안정화 (노이즈 제거)
    stabilizeColorDetection(leftColor, rightColor);
    
    // 점수 계산
    calculateScore(distanceStr.toInt(), leftColor, rightColor, brakeStatus);
    
    // LCD 표시
    static unsigned long lastDisplayTime = 0;
    if (currentTime - lastDisplayTime >= 500) {
        lastDisplayTime = currentTime;
        displayOnLcd();
    }

    // 시리얼 출력 (디버깅용)
    Serial.print("State: "); Serial.print(currentLaneState);
    Serial.print(" | Left: "); Serial.print(leftColor);
    Serial.print(" | Right: "); Serial.print(rightColor);
    Serial.print(" | Score: "); Serial.println(drivingScore);
  }
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
  
  // 안정화되지 않은 색상은 이전 값 유지
  if (colorStabilityCount < STABILITY_THRESHOLD) {
    leftColor = prevLeftColor;
    rightColor = prevRightColor;
  }
}

// 슬레이브 데이터 읽기 (타임아웃 개선)
String readSlaveData(int address) {
  Wire.requestFrom(address, 32);
  char receivedChars[32];
  int index = 0;
  unsigned long timeout = millis();
  
  while (Wire.available() && index < 31 && millis() - timeout < 100) { // 타임아웃 단축
    receivedChars[index++] = Wire.read();
  }
  receivedChars[index] = '\0';
  
  if (index == 0) {
    return "Error"; 
  }
  
  return String(receivedChars);
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
        Serial.println("Line detection started");
      }
      break;
      
    case ON_LINE:
      if (!isOnWhiteLine) {
        // 흰색 라인에서 벗어남 - 차선 변경 시작
        currentLaneState = CHANGING_LANE;
        lineDetectionEndTime = currentTime;
        Serial.println("Lane change started");
      }
      break;
      
    case CHANGING_LANE:
      if (isOnWhiteLine) {
        // 다시 흰색 라인 감지 - 복잡한 차선 변경이거나 잘못된 감지
        currentLaneState = ON_LINE;
        lineDetectionStartTime = currentTime;
        Serial.println("Re-entered line during change");
      } else {
        // 일정 시간 후 차선 변경 완료로 간주
        if (currentTime - lineDetectionEndTime > 500) { // 0.5초 후 완료
          currentLaneState = ON_ROAD;
          lineChangeCompleted = true;
          
          // 실선/점선 판정
          unsigned long lineDuration = lineDetectionEndTime - lineDetectionStartTime;
          
          if (lineDuration >= SOLID_LINE_MIN_DURATION) {
            // 실선 위반
            solidLinePenaltyCount++;
            drivingScore -= 10;
            Serial.print("SOLID LINE VIOLATION! Duration: ");
            Serial.print(lineDuration);
            Serial.println("ms (-10 points)");
          } else if (lineDuration <= DOTTED_LINE_MAX_DURATION) {
            // 점선 변경 (정상)
            Serial.print("Dotted line change. Duration: ");
            Serial.print(lineDuration);
            Serial.println("ms (No penalty)");
          } else {
            // 애매한 경우 - 경고만
            Serial.print("Ambiguous line type. Duration: ");
            Serial.print(lineDuration);
            Serial.println("ms (No penalty)");
          }
          
          laneChangeCount++;
          Serial.println("Lane change completed");
        }
      }
      break;
  }
}

// 운전 점수 계산 (개선된 버전)
void calculateScore(int distance, String leftColor, String rightColor, String brakeStatus) {
  unsigned long currentTime = millis();
  
  // 차선 변경 및 종류 감지
  detectLaneChangeAndType(leftColor, rightColor);
  
  // 급제동 감지 (디바운싱 적용)
  if (brakeStatus == "HardBrake" && currentTime - lastHardBrakeTime > HARD_BRAKE_COOLDOWN) {
    hardBrakeCount++;
    drivingScore -= 10;
    lastHardBrakeTime = currentTime;
    Serial.println("Hard Brake Detected! (-10)");
  }

  // 장애물 경고 (디바운싱 적용)
  if (distance > 0 && distance < 30 && currentTime - lastObstacleWarningTime > OBSTACLE_WARNING_COOLDOWN) {
    obstacleWarningCount++;
    drivingScore -= 3;
    lastObstacleWarningTime = currentTime;
    Serial.print("Obstacle Warning! Distance: ");
    Serial.print(distance);
    Serial.println("cm (-3)");
  }
  
  // 점수 하한선
  if (drivingScore < 0) {
    drivingScore = 0;
  }
}

// LCD에 점수 표시 (개선된 레이아웃)
void displayOnLcd() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Score: ");
  lcd.print(drivingScore);
  
  // 상태 표시 추가
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
