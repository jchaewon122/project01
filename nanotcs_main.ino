#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

SoftwareSerial orangeSerial(2, 3);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

#define SLAVE_ID 2

String currentColorStatus = "Other";
bool tcsSensorFound = false;
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 100;

void updateSensorData();
void sendDataToMaster();

void setup() {
  Serial.begin(9600);
  orangeSerial.begin(9600);

  if (tcs.begin()) {
    tcsSensorFound = true;
    Serial.println("NANO_TCS: Found TCS34725 sensor!");
  } else {
    tcsSensorFound = false;
    Serial.println("NANO_TCS: No TCS34725 sensor found!");
  }

  Serial.print("NANO_TCS Slave initialized with ID: ");
  Serial.println(SLAVE_ID);
}

void loop() {
  // Check for commands from master
  if (orangeSerial.available()) {
    String command = orangeSerial.readStringUntil('\n');
    command.trim();

    // Check for data request
    if (command == "REQ_DATA") {
      sendDataToMaster();
    }
  }

  // Periodic sensor reading
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    updateSensorData();
  }
}

void sendDataToMaster() {
  orangeSerial.print("TCS:");
  orangeSerial.println(currentColorStatus);
  
  // 전송 완료 대기
  orangeSerial.flush();
  
  Serial.print("Sent to master - Color: ");
  Serial.println(currentColorStatus);
}

void updateSensorData() {
  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    
    // 센서 에러 체크
    if (c == 0) {
      currentColorStatus = "Error";
      return;
    }
    
    long rgbSum = r + g + b;

    // 임계값 조정 및 추가 검증
    if (rgbSum < 1600) {
      currentColorStatus = "Black";
    } else if (rgbSum > 1600) {
      currentColorStatus = "White";
    } else {
      currentColorStatus = "Other";
    }
  } else {
    currentColorStatus = "Error";
  }
}
