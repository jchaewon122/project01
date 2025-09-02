#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C address (default is 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Software serial communication with Nano boards
SoftwareSerial nanoTcsSerial(2, 3); // RX, TX
SoftwareSerial nanoMpuSerial(4, 5); // RX, TX

// Driving score and penalty factors
int drivingScore = 100;
int laneChangeCount = 0;
int hardBrakeCount = 0;
int obstacleWarningCount = 0;
int solidLinePenaltyCount = 0;

// State machine for lane detection
enum LaneState {
  ON_ROAD,
  ON_LINE,
  CHANGING_LANE
};
LaneState currentLaneState = ON_ROAD;
unsigned long lineDetectionStartTime = 0;

// Debouncing for color detection
String prevLeftColor = "Other";
String prevRightColor = "Other";
int colorStabilityCount = 0;
const int STABILITY_THRESHOLD = 3;

// Criteria for solid/dotted lines
const unsigned long SOLID_LINE_MIN_DURATION = 800; // Solid line if on for > 0.8s

// Debouncing for hard brake and obstacle warnings
unsigned long lastHardBrakeTime = 0;
const unsigned long HARD_BRAKE_COOLDOWN = 2000;
unsigned long lastObstacleWarningTime = 0;
const unsigned long OBSTACLE_WARNING_COOLDOWN = 1000;

void setup() {
  Serial.begin(9600);
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
  // Read data from slaves
  String unoData = readSlaveData(Serial);
  String nanoTcsData = readSlaveData(nanoTcsSerial);
  String nanoMpuData = readSlaveData(nanoMpuSerial);
  
  // Parse and validate data
  String distanceStr = "-1";
  String leftColor = "Other";
  String rightColor = "Other";
  String brakeStatus = "Normal";

  if (unoData != "") {
    int commaIndex = unoData.indexOf(',');
    if (commaIndex != -1) {
      distanceStr = unoData.substring(0, commaIndex);
      leftColor = unoData.substring(commaIndex + 1);
    }
  }
  
  if (nanoTcsData != "") {
    rightColor = nanoTcsData;
  }
  
  if (nanoMpuData != "") {
    brakeStatus = nanoMpuData;
  }
  
  // Stabilize color detection to remove noise
  stabilizeColorDetection(leftColor, rightColor);
  
  // Calculate score based on sensor data
  calculateScore(distanceStr.toInt(), leftColor, rightColor, brakeStatus);
  
  // Update LCD display every 500ms
  static unsigned long lastDisplayTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDisplayTime >= 500) {
      lastDisplayTime = currentTime;
      displayOnLcd();
  }

  // Debug output
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 1000) {
      lastDebugTime = currentTime;
      Serial.print("Uno: "); Serial.print(unoData);
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

// Function to read data from a serial stream
String readSlaveData(Stream &stream) {
    if (stream.available()) {
        String data = stream.readStringUntil('\n');
        data.trim();
        return data;
    }
    return "";
}

// Improved logic for color debouncing
void stabilizeColorDetection(String& leftColor, String& rightColor) {
  static String stableLeftColor = "Other";
  static String stableRightColor = "Other";
  
  if (leftColor == prevLeftColor && rightColor == prevRightColor) {
    colorStabilityCount++;
  } else {
    colorStabilityCount = 0;
    prevLeftColor = leftColor;
    prevRightColor = rightColor;
  }
  
  if (colorStabilityCount >= STABILITY_THRESHOLD) {
    stableLeftColor = leftColor;
    stableRightColor = rightColor;
  }
  
  leftColor = stableLeftColor;
  rightColor = stableRightColor;
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
        currentLaneState = CHANGING_LANE;
        unsigned long lineDuration = currentTime - lineDetectionStartTime;
        if (lineDuration >= SOLID_LINE_MIN_DURATION) {
            solidLinePenaltyCount++;
            drivingScore -= 10;
        }
        laneChangeCount++;
        currentLaneState = ON_ROAD; // Return to ON_ROAD after passing the line
      }
      break;
      
    case CHANGING_LANE:
      // This state is now handled instantly upon leaving the line
      // and immediately returns to ON_ROAD. No complex timing needed.
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
