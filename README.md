# project01

## 필요한 라이브러리

- Adafruit MPU6050
- Adafruit TCS34725
- NewPing
- LiquidCrystal I2C (by Marco Schwartz)

## Note by GPT

1. 풀업 저항을 연결한 뒤 초음파를 제외한 센서가 제대로 작동하는 지 확인할 것.
2. 조도 센서에 연결된 저항이 82로 높은 편이라 10-20으로 낮은 저항으로 바꿀 것.

### 조도 값 확인하는 코드

```
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

