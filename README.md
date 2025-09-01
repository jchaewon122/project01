# project01

## 필요한 라이브러리

- Adafruit MPU6050
- Adafruit TCS34725
- NewPing
- LiquidCrystal I2C (by Marco Schwartz)

## Note by GPT

1. 4.7kΩ 풀업 저항(갈색 - 검정색 - 주황색 - 금색/은색) 2개를 SDA/SCL에 각각 연결한 뒤 초음파를 제외한 센서가 제대로 작동하는 지 확인할 것.
2. 조도 센서에 연결된 저항이 82kΩ로 높은 편이라 10-20kΩ(갈색 - 검정색 - 주황색 - 금색/은색)으로 낮은 저항으로 바꿀 것.

### 1.풀업 저항

- 오렌지 보드의 A4 - LCD, nano 1, mpu, nano 2, tcs 2, uno 1, tcs 1의 SCL
- 오렌지 보드의 A5 - LCD, nano 1, mpu, nano 2, tcs 2, uno 1, tcs 1의 SDA

>브레드 보드를 통해 저렇게 연결해두었고 모두 전원과 접지를 공유하는 상태에서 

- 오렌지 보드의 A4 - LCD, nano 1, mpu, nano 2, tcs 2, uno 1, tcs 1의 SCL - 풀업 저항 - 5V
- 오렌지 보드의 A5 - LCD, nano 1, mpu, nano 2, tcs 2, uno 1, tcs 1의 SDA - 풀업 저항 - 5V

>이렇게 풀업 저항의 한쪽 다리는 +쪽에 꼽고 나머지를  SDA/SCL에 꽂으라 gemini가 지시.

### 2.조도 값 확인하는 코드

```c++
int lightPin = A2;  // 조도 센서 연결 아날로그 핀
int lightValue = 0;

void setup() {
  Serial.begin(9600);  // 시리얼 모니터로 값 확인
}

void loop() {
  lightValue = analogRead(lightPin);  // 0~1023 범위 값 읽기
  Serial.print("조도 값: ");
  Serial.println(lightValue);
  delay(500);  // 0.5초 간격
}
```

### 3. 컬러 센서 확인하는 코드

```c++
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// TCS34725 센서 객체 생성
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(9600);
  
  // TCS 센서 초기화 시도
  if (tcs.begin()) {
    Serial.println("Found TCS34725 sensor!");
    Serial.println("------------------------------------");
    Serial.println("  R   |   G   |   B   |   C  ");
    Serial.println("------------------------------------");
  } else {
    // 센서 연결 실패 메시지 출력 후 무한 루프
    Serial.println("No TCS34725 sensor found ... check your wiring!");
    while (1);
  }
}

void loop() {
  uint16_t r, g, b, c;
  
  // 센서에서 원시 RGBC 데이터 읽기
  tcs.getRawData(&r, &g, &b, &c);

  // 시리얼 모니터로 값 출력
  Serial.print(r, DEC);
  Serial.print(" | ");
  Serial.print(g, DEC);
  Serial.print(" | ");
  Serial.print(b, DEC);
  Serial.print(" | ");
  Serial.println(c, DEC);
  
  delay(500); // 0.5초 대기
}
```

### 가속도 센서 확인하는 코드

```c++
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(9600);
  
  // MPU 센서 초기화 시도
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050/6500 chip");
    Serial.println("Check your wiring and I2C address.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU Sensor Found!");
  Serial.println("------------------------------------");

  // 센서 범위 설정
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_GYRO_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  /* 센서 값 읽기 */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* 가속도 값(m/s^2) 시리얼 모니터로 출력 */
  Serial.print("AccelX: ");
  Serial.print(a.acceleration.x, 2);
  Serial.print(" | AccelY: ");
  Serial.print(a.acceleration.y, 2);
  Serial.print(" | AccelZ: ");
  Serial.println(a.acceleration.z, 2);

  /* 자이로스코프 값(rad/s) 시리얼 모니터로 출력 */
  Serial.print("GyroX: ");
  Serial.print(g.gyro.x, 2);
  Serial.print(" | GyroY: ");
  Serial.print(g.gyro.y, 2);
  Serial.print(" | GyroZ: ");
  Serial.println(g.gyro.z, 2);

  Serial.println("------------------------------------");
  delay(500);
}
```

### LCD 확인하는 코드

```c++
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C 주소를 설정합니다.
// 대부분의 LCD 모듈은 0x27 또는 0x3F 주소를 사용합니다.
// 자신의 모듈에 맞는 주소를 사용하세요.
LiquidCrystal_I2C lcd(0x27, 16, 2);  // 0x27 주소, 16x2 문자열 LCD

void setup() {
  Serial.begin(9600);

  // LCD 초기화
  lcd.init();
  
  // 백라이트 켜기
  lcd.backlight();
  Serial.println("LCD Initialized and Backlight ON.");

  // 첫 번째 줄에 메시지 출력
  lcd.setCursor(0, 0); // (0, 0) 위치로 커서 이동
  lcd.print("Hello, World!");

  // 두 번째 줄에 메시지 출력
  lcd.setCursor(0, 1); // (0, 1) 위치로 커서 이동
  lcd.print("Test OK!");

  Serial.println("Text printed on LCD.");
}

void loop() {
  // LCD 테스트 코드는 loop 함수에서 추가적인 동작이 필요 없습니다.
  // setup() 함수에서 한 번만 실행하면 됩니다.
}
```
