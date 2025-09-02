#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C address (default is 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Single Software serial communication for all slave boards
// 오렌지보드 RX(데이터 받는 핀) -> 슬레이브보드 TX
// 오렌지보드 TX(데이터 보내는 핀) -> 슬레이브보드 RX
SoftwareSerial slaveSerial(6, 7);

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

// Global variables for received data
int distance = -1;
String unoRightColor = "Other";
String nanoLeftColor = "Other";
String brakeStatus = "Normal";

void setup() {
  Serial.begin(9600); // For PC debug monitor
  slaveSerial.begin(9600);

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
  // Read data from a single serial stream and parse by ID
  String slaveData = readSlaveData(slaveSerial);
  if (slaveData != "") {
    parseAndProcessData(slaveData);
  }

  // Calculate score based on sensor data
  calculateScore(distance, unoRightColor, nanoLeftColor, brakeStatus);

  // Update LCD display and send score to Uno every 500ms
  static unsigned long lastDisplayTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDisplayTime >= 500) {
    lastDisplayTime = currentTime;
    displayOnLcd();
    slaveSerial.print("SCORE:");
    slaveSerial.println(drivingScore); // Uno 보드로 점수 전송
  }

  // Debug output
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 1000) {
    lastDebugTime = currentTime;
    Serial.print("Parsed - Dist: "); Serial.print(distance);
    Serial.print(" | Left: "); Serial.print(unoRightColor);
    Serial.print(" | Right: "); Serial.print(nanoLeftColor);
    Serial.print(" | Brake: "); Serial.println(brakeStatus);
    Serial.print("State: "); Serial.print(currentLaneState == ON_ROAD ? "ON_ROAD" : "ON_LINE");
    Serial.print(" | Score: "); Serial.println(drivingScore);
    Serial.println("---");
  }
}

// Function to read data from a single serial stream
String readSlaveData(Stream &stream) {
  if (stream.available()) {
    String data = stream.readStringUntil('\n');
    data.trim();
    return data;
  }
  return "";
}

// Parse and process data based on the slave ID
void parseAndProcessData(String data) {
  // Check for the slave ID
  if (data.startsWith("UNO:")) {
    data = data.substring(4); // Remove "UNO:"
    int firstComma = data.indexOf(',');
    if (firstComma != -1) {
      distance = data.substring(0, firstComma).toInt();
      unoRightColor = data.substring(firstComma + 1);
    }
    if (unoRightColor == "Error" || distance == -1) unoError = true; else unoError = false;
  } else if (data.startsWith("NANO_TCS:")) {
    data = data.substring(9); // Remove "NANO_TCS:"
    nanoLeftColor = data;
    if (nanoLeftColor == "Error") nanoTcsError = true; else nanoTcsError = false;
  } else if (data.startsWith("NANO_MPU:")) {
    data = data.substring(9); // Remove "NANO_MPU:"
    brakeStatus = data;
    if (brakeStatus == "Error") nanoMpuError = true; else nanoMpuError = false;
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
        currentLaneState = ON_ROAD; // Return to ON_ROAD after passing the line
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
