#include <Wire.h>
#include <Adafruit_TCS34725.h>

// 초음파 센서 핀 설정
#define TRIG_PIN 9
#define ECHO_PIN 10

// TCS34725 센서 설정
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// I2C 슬레이브 주소 설정
const int slaveAddress = 0x08;

// 센서 데이터를 저장할 전역 변수 (volatile로 인터럽트 안전성 확보)
volatile long currentDistance = -1;
volatile bool dataReady = false;
String currentColorStatus = "Other";
bool tcsSensorFound = false;

// 데이터 준비 상태 플래그
bool sensorDataValid = false;

void setup() {
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.begin(9600);
  
  // TCS 센서 초기화
  if (tcs.begin()) {
    Serial.println("Found TCS34725 sensor!");
    tcsSensorFound = true;
  } else {
    Serial.println("No TCS34725 sensor found ... check your wiring!");
    tcsSensorFound = false;
  }
  
  // 초기 센서 읽기
  updateSensorData();
  
  Serial.println("UNO Slave initialized at address 0x08");
}

void loop() {
  // 50ms 주기로 센서 값을 업데이트 (더 빠른 응답)
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    updateSensorData();
    
    // 디버그 출력 (1초에 한 번)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 1000) {
      lastDebugTime = currentTime;
      Serial.print("Distance: ");
      Serial.print(currentDistance);
      Serial.print(" cm, Color: ");
      Serial.print(currentColorStatus);
      Serial.print(", Valid: ");
      Serial.println(sensorDataValid ? "Yes" : "No");
    }
  }
}

void updateSensorData() {
  // 초음파 센서 데이터 읽기 (개선된 버전)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 타임아웃을 더 짧게 설정 (30ms)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration > 0) {
    long newDistance = (duration / 2) / 29.1;
    // 유효 범위 체크 (2cm ~ 400cm)
    if (newDistance >= 2 && newDistance <= 400) {
      currentDistance = newDistance;
    } else {
      currentDistance = -1; // 유효하지 않은 값
    }
  } else {
    currentDistance = -1; // 타임아웃 발생
  }
  
  // TCS 컬러 센서 데이터 읽기 (개선된 버전)
  if (tcsSensorFound) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    
    // 더 정확한 임계값 설정
    if (c < 200) {
      currentColorStatus = "Black";
    } else if (c > 800) {
      currentColorStatus = "White";
    } else {
      currentColorStatus = "Other";
    }
    sensorDataValid = true;
  } else {
    currentColorStatus = "Error";
    sensorDataValid = false;
  }
  
  // 데이터 준비 완료 플래그 설정
  dataReady = true;
}

void requestEvent() {
  char dataToSend[32];
  
  // 데이터가 준비되었는지 확인
  if (dataReady && sensorDataValid) {
    // 안전한 문자열 생성 (sprintf 사용)
    snprintf(dataToSend, sizeof(dataToSend), "%ld,%s", 
             currentDistance, currentColorStatus.c_str());
  } else {
    // 에러 상태 전송
    snprintf(dataToSend, sizeof(dataToSend), "-1,Error");
  }
  
  // 문자열 길이 확인 후 전송
  int dataLength = strlen(dataToSend);
  if (dataLength > 0 && dataLength < 32) {
    Wire.write(dataToSend, dataLength);
  } else {
    // 안전한 기본값 전송
    Wire.write("-1,Error", 8);
  }
  
  // 디버그 출력
  Serial.print("Sent: ");
  Serial.println(dataToSend);
}
