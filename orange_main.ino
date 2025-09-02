#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C address (default is 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Separate SoftwareSerial for each slave
SoftwareSerial unoSerial(2, 3);      // UNO slave
SoftwareSerial tcsSerial(4, 5);      // NANO TCS slave
SoftwareSerial mpuSerial(6, 7);      // NANO MPU slave

// Driving score and penalty factors
int drivingScore = 100;
int laneChangeCount = 0;
int hardBrakeCount = 0;
int obstacleWarningCount = 0;
int solidLinePenaltyCount = 0;

// State machine for lane detection
enum LaneState {
  ON_ROAD,
  ON_LINE
};
LaneState currentLaneState = ON_ROAD;
unsigned long lineDetectionStartTime = 0;

// Criteria for solid/dotted lines
const unsigned long SOLID_LINE_MIN_DURATION = 800; // Solid line if on for > 0.8s

// Debouncing for hard brake and obstacle warnings
unsigned long lastObstacleWarningTime = 0;
unsigned long lastHardBrakeTime = 0;
const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000;
const unsigned long HARD_BRAKE_COOLDOWN = 1000;

// Sensor error flags
bool nanoTcsError = false;
bool nanoMpuError = false;
bool unoError = false;

// Global variables for received data
int distance = -1;
String unoRightColor = "Other";
String nanoLeftColor = "Other";

// Timing variables for Master-Slave requests
unsigned long lastUnoRequestTime = 0;
unsigned long lastTcsRequestTime = 0;
unsigned long lastMpuRequestTime = 0;
const unsigned long UNO_REQUEST_INTERVAL = 300;     // 300ms
const unsigned long TCS_REQUEST_INTERVAL = 300;     // 300ms
const unsigned long MPU_REQUEST_INTERVAL = 2000;    // 2초
const unsigned long RESPONSE_TIMEOUT = 200;         // 응답 대기 시간

// Communication timeout tracking
unsigned long lastUnoResponse = 0;
unsigned long lastTcsResponse = 0;
unsigned long lastMpuResponse = 0;
const unsigned long COMMUNICATION_TIMEOUT = 1500; // 1.5초

// 함수 선언
void requestAndReceiveData(SoftwareSerial &serialPort, const char* slaveName);
void parseUnoData(String data);
void parseTcsData(String data);
void parseMpuData(String data);
void checkCommunicationTimeouts(unsigned long currentTime);
void detectLaneChangeAndType(String leftColor, String rightColor);
void calculateScore();
void displayOnLcd();
void printDebugInfo();
void sendScoreToUno(int score);

void setup() {
  Serial.begin(9600); // For PC debug monitor
  
  // Initialize all serial communications
  unoSerial.begin(9600);
  tcsSerial.begin(9600);
  mpuSerial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Driving Start!");
  delay(2000);
  lcd.clear();

  Serial.println("Master initialized with separate serial communication");
  
  // Initialize response timestamps
  lastUnoResponse = millis();
  lastTcsResponse = millis();
  lastMpuResponse = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // 각 슬레이브에 순차적으로 요청하여 데이터 수신
  static byte currentSlave = 0;
  
  switch (currentSlave) {
    case 0: // UNO에 요청
      if (currentTime - lastUnoRequestTime >= UNO_REQUEST_INTERVAL) {
        lastUnoRequestTime = currentTime;
        requestAndReceiveData(unoSerial, "UNO");
        currentSlave = 1; // 다음 슬레이브로 전환
      }
      break;
    case 1: // TCS에 요청
      if (currentTime - lastTcsRequestTime >= TCS_REQUEST_INTERVAL) {
        lastTcsRequestTime = currentTime;
        requestAndReceiveData(tcsSerial, "TCS");
        currentSlave = 2; // 다음 슬레이브로 전환
      }
      break;
    case 2: // MPU에 요청
      if (currentTime - lastMpuRequestTime >= MPU_REQUEST_INTERVAL) {
        lastMpuRequestTime = currentTime;
        requestAndReceiveData(mpuSerial, "MPU");
        currentSlave = 0; // 다시 처음으로 돌아감
      }
      break;
  }
  
  // Check communication timeouts
  checkCommunicationTimeouts(currentTime);

  // Calculate score based on sensor data
  calculateScore();

  // Update LCD display and send score to Uno every 500ms
  static unsigned long lastDisplayTime = 0;
  if (currentTime - lastDisplayTime >= 500) {
    lastDisplayTime = currentTime;
    displayOnLcd();
    // Send score to UNO
    sendScoreToUno(drivingScore);
  }

  // Debug output
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 2000) {
    lastDebugTime = currentTime;
    printDebugInfo();
  }
}

// 각 슬레이브에 요청을 보내고 응답을 처리하는 통합 함수
void requestAndReceiveData(SoftwareSerial &serialPort, const char* slaveName) {
  // 요청 보낼 포트만 활성화
  serialPort.listen();
  
  // 요청 보내기
  serialPort.println("REQ_DATA");
  Serial.print("Requesting data from ");
  Serial.println(slaveName);

  // 응답 기다리기 (타임아웃 포함)
  unsigned long startTime = millis();
  String data = "";
  while (millis() - startTime < RESPONSE_TIMEOUT) {
    if (serialPort.available()) {
      data = serialPort.readStringUntil('\n');
      data.trim();
      break;
    }
  }

  // 데이터 파싱 및 에러 처리
  if (data.length() > 0) {
    if (strcmp(slaveName, "UNO") == 0) parseUnoData(data);
    else if (strcmp(slaveName, "TCS") == 0) parseTcsData(data);
    else if (strcmp(slaveName, "MPU") == 0) parseMpuData(data);
  } else {
    Serial.print("Timeout or no data from ");
    Serial.println(slaveName);
  }
  
  // SoftwareSerial 포트 비활성화
  serialPort.end();
}

// Send score to UNO
void sendScoreToUno(int score) {
  // UNO 시리얼 포트만 활성화하여 전송
  unoSerial.listen();
  unoSerial.print("SCORE:");
  unoSerial.println(score);
  unoSerial.flush();
  unoSerial.end();
}

// Check communication timeouts
void checkCommunicationTimeouts(unsigned long currentTime) {
  // 각 슬레이브의 응답 시간을 개별적으로 체크
  if (currentTime - lastUnoResponse > COMMUNICATION_TIMEOUT) {
    unoError = true;
  } else {
    unoError = false;
  }
  if (currentTime - lastTcsResponse > COMMUNICATION_TIMEOUT) {
    nanoTcsError = true;
  } else {
    nanoTcsError = false;
  }
  if (currentTime - lastMpuResponse > COMMUNICATION_TIMEOUT) {
    nanoMpuError = true;
  } else {
    nanoMpuError = false;
  }
}

// Parse data from each slave
void parseUnoData(String data) {
  lastUnoResponse = millis();
  unoError = false; // 응답받았으므로 에러 해제
  
  if (data.startsWith("UNO:")) {
    data = data.substring(4); // Remove "UNO:"
    int firstComma = data.indexOf(',');
    if (firstComma != -1) {
      String distanceStr = data.substring(0, firstComma);
      distance = distanceStr.toInt();
      unoRightColor = data.substring(firstComma + 1);
      
      // 데이터 유효성 검사
      if (distance < 0 || distance > 400) {
        distance = -1;
      }
      if (unoRightColor != "White" && unoRightColor != "Black" && unoRightColor != "Error") {
        unoRightColor = "Other";
      }
    } else {
      distance = -1;
      unoRightColor = "Error";
    }
    
    Serial.print("UNO Data - Distance: "); Serial.print(distance);
    Serial.print(", Color: "); Serial.println(unoRightColor);
  } else {
    Serial.println("Invalid UNO data format: " + data);
  }
}

void parseTcsData(String data) {
  lastTcsResponse = millis();
  nanoTcsError = false; // 응답받았으므로 에러 해제
  
  if (data.startsWith("TCS:")) {
    data = data.substring(4); // Remove "TCS:"
    nanoLeftColor = data;
    
    // 데이터 유효성 검사
    if (nanoLeftColor != "White" && nanoLeftColor != "Black" && nanoLeftColor != "Error") {
      nanoLeftColor = "Other";
    }
    
    Serial.print("TCS Data - Color: "); Serial.println(nanoLeftColor);
  } else {
    Serial.println("Invalid TCS data format: " + data);
    nanoLeftColor = "Error";
  }
}

void parseMpuData(String data) {
  lastMpuResponse = millis();
  nanoMpuError = false; // 응답받았으므로 에러 해제
  
  if (data.startsWith("MPU:")) {
    data = data.substring(4); // Remove "MPU:"
    int colonIndex = data.indexOf(':');
    if (colonIndex != -1) {
      String status = data.substring(0, colonIndex);
      if (status == "HardBrakeCount") {
        String countStr = data.substring(colonIndex + 1);
        int newCount = countStr.toInt();
        
        // 급제동 횟수가 증가했을 때만 점수 감점
        if (newCount > hardBrakeCount) {
          drivingScore -= 5; // 급제동 1회당 5점 감점
          if (drivingScore < 0) drivingScore = 0;
          Serial.print("Penalty! Hard Brake Detected. New Score: ");
          Serial.println(drivingScore);
        }
        hardBrakeCount = newCount;
      }
    } else {
      Serial.println("Invalid MPU data format: " + data);
    }
  } else {
    Serial.println("Invalid MPU data format: " + data);
  }
}

// Improved logic for lane change detection
void detectLaneChangeAndType(String leftColor, String rightColor) {
  unsigned long currentTime = millis();
  bool isOnWhiteLine = (leftColor == "White" || rightColor == "White");

  switch (currentLaneState) {
    case ON_ROAD:
      if (isOnWhiteLine) {
        currentLaneState = ON_LINE;
        lineDetectionStartTime = currentTime;
        Serial.println("Lane detection: Entered white line");
      }
      break;

    case ON_LINE:
      if (!isOnWhiteLine) {
        unsigned long lineDuration = currentTime - lineDetectionStartTime;
        
        if (lineDuration >= SOLID_LINE_MIN_DURATION) {
          solidLinePenaltyCount++;
          drivingScore -= 10;
          if (drivingScore < 0) drivingScore = 0;
          Serial.print("Solid line violation detected! Duration: ");
          Serial.print(lineDuration);
          Serial.println("ms");
        }
        
        laneChangeCount++;
        currentLaneState = ON_ROAD;
        
        Serial.print("Lane change completed! Count: ");
        Serial.print(laneChangeCount);
        Serial.print(", Line duration: ");
        Serial.print(lineDuration);
        Serial.println("ms");
      }
      break;
  }
}

// Function to calculate driving score
void calculateScore() {
  unsigned long currentTime = millis();

  // 차선 변경 감지 (좌측 또는 우측 센서에서 흰색 감지)
  detectLaneChangeAndType(nanoLeftColor, unoRightColor);

  // 앞 차와의 거리 체크 (30cm 미만일 때 경고)
  if (distance > 0 && distance < 30 && currentTime - lastObstacleWarningTime > OBSTACLE_WARNING_COOLDOWN) {
    obstacleWarningCount++;
    drivingScore -= 3;
    if (drivingScore < 0) drivingScore = 0;
    lastObstacleWarningTime = currentTime;
    Serial.print("Close obstacle detected! Distance: ");
    Serial.print(distance);
    Serial.println("cm");
  }

  // 점수가 음수가 되지 않도록 제한
  if (drivingScore < 0) {
    drivingScore = 0;
  }
  
  // 최대 점수 제한 (선택사항)
  if (drivingScore > 100) {
    drivingScore = 100;
  }
}

// Function to display score on LCD
void displayOnLcd() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Score: ");
  lcd.print(drivingScore);

  lcd.setCursor(11, 0);
  if (unoError || nanoTcsError || nanoMpuError) {
    lcd.print("ERROR");
  } else {
    if (currentLaneState == ON_ROAD) {
      lcd.print("ROAD");
    } else {
      lcd.print("LINE");
    }
  }

  lcd.setCursor(0, 1);
  lcd.print("HB:");
  lcd.print(hardBrakeCount);
  lcd.print(" LC:");
  lcd.print(laneChangeCount);
  lcd.print(" SL:");
  lcd.print(solidLinePenaltyCount);
  
  // LCD 버퍼 플러시
  delay(10);
}

// Debug information print
void printDebugInfo() {
  Serial.println("=== DEBUG INFO ===");
  Serial.print("Distance: "); Serial.print(distance); Serial.print("cm");
  Serial.print(" | UNO Right Color: "); Serial.print(unoRightColor);
  Serial.print(" | TCS Left Color: "); Serial.println(nanoLeftColor);
  Serial.print("HardBrake Count: "); Serial.print(hardBrakeCount);
  Serial.print(" | Lane Changes: "); Serial.print(laneChangeCount);
  Serial.print(" | Solid Line Violations: "); Serial.print(solidLinePenaltyCount);
  Serial.print(" | Obstacle Warnings: "); Serial.println(obstacleWarningCount);
  Serial.print("Current State: ");
  Serial.print(currentLaneState == ON_ROAD ? "ON_ROAD" : "ON_LINE");
  Serial.print(" | Score: "); Serial.println(drivingScore);
  Serial.print("Errors - UNO: "); Serial.print(unoError ? "YES" : "NO");
  Serial.print(" | TCS: "); Serial.print(nanoTcsError ? "YES" : "NO");
  Serial.print(" | MPU: "); Serial.println(nanoMpuError ? "YES" : "NO");
  Serial.println("==================");
}
