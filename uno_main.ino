#include <Adafruit_TCS34725.h>

// Ultrasonic sensor pins
#define TRIG_PIN 9
#define ECHO_PIN 10

// TCS34725 sensor configuration
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// Global variables for sensor data
long currentDistance = -1;
String currentColorStatus = "Other";
bool tcsSensorFound = false;

void setup() {
  Serial.begin(9600); // Start serial communication with Orange BLE
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize TCS sensor
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
  // Update sensor values every 50ms
  static unsigned long lastSensorReadTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorReadTime >= 50) {
    lastSensorReadTime = currentTime;
    updateSensorData();
    
    // Transmit data to Orange BLE
    Serial.print(currentDistance);
    Serial.print(",");
    Serial.println(currentColorStatus);
  }
}

// Function to update sensor data
void updateSensorData() {
  // Read data from ultrasonic sensor
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
  
  // Read data from TCS color sensor
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
