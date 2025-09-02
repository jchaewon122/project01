#include <Adafruit_TCS34725.h>
#include <LiquidCrystal_I2C.h>

// 초음파 센서 핀 설정
#define TRIG_PIN 9
#define ECHO_PIN 10

// TCS34725 센서 설정
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// LCD 주소 설정
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 센서 데이터를 저장할 전역 변수
long currentDistance = -1;
String currentColorStatus = "Other";
bool tcsSensorFound = false;

void setup() {
  Serial.begin(9600); // 오렌지 BLE와의 시리얼 통신 시작
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // TCS 센서 초기화
  if (tcs.begin()) {
    Serial.println("UNO: TCS34725 sensor found!");
    tcsSensorFound = true;
  } else {
    Serial.println("UNO: No TCS34725 sensor found!");
    tcsSensorFound = false;
  }
  
  Serial.println("UNO Slave initialized (Serial Mode)");
}

void loop() {
  // 50ms 주기로 센서 값을 업데이트
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    updateSensorData();
    
    // 오렌지 BLE로 데이터 전송
    Serial.print(currentDistance);
    Serial.print(",");
    Serial.println(currentColorStatus);
  }
}

// 센서 데이터 업데이트 함수
void updateSensorData() {
  // 초음파 센서 데이터 읽기
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration > 0) {
    long newDistance = (duration / 2) / 29.1;
    if (newDistance >= 2 && newDistance <= 400) {
      currentDistance = newDistance;
    } else {
      currentDistance = -1;
    }
  } else {
    currentDistance = -1;
  }
  
  // TCS 컬러 센서 데이터 읽기
  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    if (tcs.getRawData(&r, &g, &b, &c)) {
      if (c < 200) {
        currentColorStatus = "Black";
      } else if (c > 800) {
        currentColorStatus = "White";
      } else {
        currentColorStatus = "Other";
      }
    } else {
      currentColorStatus = "Error";
    }
  } else {
    currentColorStatus = "Error";
  }
}
