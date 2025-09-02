#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C address (default is 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Software serial communication with slave boards
SoftwareSerial unoSerial(6, 7);      // 우노와 통신
SoftwareSerial nanoTcsSerial(2, 3);  // 나노 TCS와 통신
SoftwareSerial nanoMpuSerial(4, 5);  // 나노 MPU와 통신

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
unsigned long lastHardBrakeTime = 0;
const unsigned long HARD_BRAKE_COOLDOWN = 2000;
unsigned long lastObstacleWarningTime = 0;
const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000;

// Sensor error flags
bool nanoTcsError = false;
bool nanoMpuError = false;
bool unoError = false;

// 시리얼 통신 최적화를 위한 변수들
String lastUnoData = "";
String lastNanoTcsData = "";
String lastNanoMpuData = "";
unsigned long lastUnoReceived = 0;
unsigned long lastTcsReceived = 0;
unsigned long lastMpuReceived = 0;
const unsigned long DATA_TIMEOUT = 500; // 500ms timeout

void setup() {
  Serial.begin(9600); // For PC debug monitor
  unoSerial.begin(9600);
  nanoTcsSerial.begin(9600);
  nanoMpuSerial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Driving Start!");
  delay(2000);
  lcd.clear();

  Serial.println("Master initialized (Serial Mode)");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read data from slaves with timeout checking
  readSlaveDataWithTimeout();

  // Parse and validate data
  int distance = -1;
  String leftColor = "Other";
  String rightColor = "Other";
  String brakeStatus = "Normal";

  // Parse UNO data (거리와 오른쪽 컬러)
  if (lastUnoData != "" && (currentTime - lastUnoReceived < DATA_TIMEOUT)) {
    int firstComma = lastUnoData.indexOf(',');
    if (firstComma != -1) {
      distance = lastUnoData.substring(0, firstComma).toInt();
      rightColor = lastUnoData.substring(firstComma + 1);
      rightColor.trim();
    }
    unoError = false;
  } else {
    unoError = true;
  }

  // Parse Nano TCS data (왼쪽 컬러)
  if (lastNanoTcsData != "" && (currentTime - lastTcsReceived < DATA_TIMEOUT)) {
    leftColor = lastNanoTcsData;
    leftColor.trim();
    nanoTcsError = (leftColor == "Error");
  } else {
    nanoTcsError = true;
  }

  // Parse Nano MPU data (브레이크 상태)
  if (lastNanoMpuData != "" && (currentTime - lastMpuReceived < DATA_TIMEOUT)) {
    brakeStatus = lastNanoMpuData;
    brakeStatus.trim();
    nanoMpuError = (brakeStatus == "Error");
  } else {
    nanoMpuError = true;
  }

  // Calculate score based on sensor data
  if (!unoError || !nanoTcsError || !nanoMpuError) {
    calculateScore(distance, leftColor, rightColor, brakeStatus);
  }

  // Update LCD display and send score to Uno every 500ms
  static unsigned long lastDisplayTime = 0;
  if (currentTime - lastDisplayTime >= 500) {
    lastDisplayTime = currentTime;
    displayOnLcd();
    
    // Uno 보드로 점수 전송
    unoSerial.print("SCORE:");
    unoSerial.println(drivingScore);
  }

  // Debug output
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 1000) {
    lastDebugTime = currentTime;
    Serial.print("Uno: "); Serial.print(lastUnoData);
    Serial.print(" | Nano TCS: "); Serial.print(lastNanoTcsData);
    Serial.print(" | Nano MPU: "); Serial.println(lastNanoMpuData);
    Serial.print("Parsed - Dist: "); Serial.print(distance);
    Serial.print(" | Left: "); Serial.print(leftColor);
    Serial.print(" | Right: "); Serial.print(rightColor);
    Serial.print(" | Brake: "); Serial.println(brakeStatus);
    Serial.print("State: "); Serial.print(currentLaneState == ON_ROAD ? "ON_ROAD" : "ON_LINE");
    Serial.print(" | Score: "); Serial.println(drivingScore);
    Serial.print("Errors - Uno:"); Serial.print(unoError);
    Serial.print(" TCS:"); Serial.print(nanoTcsError);
    Serial.print(" MPU:"); Serial.println(nanoMpuError);
    Serial.println("---");
  }
}

// 시리얼 통신 최적화된 데이터 읽기 함수
void readSlaveDataWithTimeout() {
  unsigned long currentTime = millis();
  
  // UNO 데이터 읽기
  if (unoSerial.available()) {
    String newData = unoSerial.readStringUntil('\n');
    newData.trim();
    if (newData.length() > 0) {
      lastUnoData = newData;
      lastUnoReceived = currentTime;
    }
  }
  
  // Nano TCS 데이터 읽기
  if (nanoTcsSerial.available()) {
    String newData = nanoTcsSerial.readStringUntil('\n');
    newData.trim();
    if (newData.length() > 0) {
      lastNanoTcsData = newData;
      lastTcsReceived = currentTime;
    }
  }
  
  // Nano MPU 데이터 읽기
  if (nanoMpuSerial.available()) {
    String newData = nanoMpuSerial.readStringUntil('\n');
    newData.trim();
    if (newData.length() > 0) {
      lastNanoMpuData = newData;
      lastMpuReceived = currentTime;
    }
  }
}

// Improved logic for lane change and line type detection
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
        unsigned long lineDuration = currentTime - lineDetectionStartTime;
        if (lineDuration >= SOLID_LINE_MIN_DURATION) {
          solidLinePenaltyCount++;
          drivingScore -= 10;
        }
        laneChangeCount++;
        currentLaneState = ON_ROAD;
      }
      break;
  }
}

// Function to calculate driving score
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
}
