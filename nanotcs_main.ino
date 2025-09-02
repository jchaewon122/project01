#include <Wire.h>
#include <Adafruit_TCS34725.h>

// TCS34725 센서 설정
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

const int slaveAddress = 0x09;

// 센서 데이터 저장용 변수
String currentColorStatus = "Other";
bool tcsSensorFound = false;
bool dataReady = false;

void setup() {
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent);
  
  Serial.begin(9600);
  
  // TCS 센서 초기화
  if (tcs.begin()) {
    Serial.println("NANO1: Found TCS34725 sensor!");
    tcsSensorFound = true;
  } else {
    Serial.println("NANO1: No TCS34725 sensor found ... check your wiring!");
    tcsSensorFound = false;
  }
  
  // 초기 센서 읽기
  updateSensorData();
  
  Serial.println("NANO1 Slave initialized at address 0x09");
}

void loop() {
  // 50ms 주기로 센서 값을 업데이트
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    updateSensorData();
    
    // 디버그 출력 (1초에 한 번)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 1000) {
      lastDebugTime = currentTime;
      Serial.print("NANO1 Color: ");
      Serial.print(currentColorStatus);
      Serial.print(", Sensor Found: ");
      Serial.println(tcsSensorFound ? "Yes" : "No");
    }
  }
}

void updateSensorData() {
  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    
    // 센서 데이터 읽기 시도
    if (tcs.getRawData(&r, &g, &b, &c)) {
      // 더 정확한 임계값 설정 (UNO와 동일)
      if (c < 200) {
        currentColorStatus = "Black";
      } else if (c > 800) {
        currentColorStatus = "White";
      } else {
        currentColorStatus = "Other";
      }
      dataReady = true;
      
      // 추가 디버그 정보
      if (millis() % 2000 < 50) { // 2초마다 한 번씩 상세 정보 출력
        Serial.print("NANO1 Raw values - R:");
        Serial.print(r);
        Serial.print(" G:");
        Serial.print(g);
        Serial.print(" B:");
        Serial.print(b);
        Serial.print(" C:");
        Serial.print(c);
        Serial.print(" -> ");
        Serial.println(currentColorStatus);
      }
    } else {
      currentColorStatus = "Error";
      dataReady = false;
    }
  } else {
    currentColorStatus = "Error";
    dataReady = false;
  }
}

void requestEvent() {
  char colorToSend[16];
  
  // 데이터가 준비되었는지 확인
  if (dataReady) {
    // 안전한 문자열 복사
    strncpy(colorToSend, currentColorStatus.c_str(), sizeof(colorToSend) - 1);
    colorToSend[sizeof(colorToSend) - 1] = '\0'; // null terminator 보장
  } else {
    strcpy(colorToSend, "Error");
  }
  
  // 데이터 전송
  int dataLength = strlen(colorToSend);
  if (dataLength > 0) {
    Wire.write(colorToSend, dataLength);
  } else {
    Wire.write("Error", 5);
  }
  
  // 디버그 출력
  Serial.print("NANO1 Sent: ");
  Serial.println(colorToSend);
}
