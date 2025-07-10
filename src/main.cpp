#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RobotIMU.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RobotIMU.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>

// Helper to convert yaw degrees to cardinal direction
const char* toCardinal(float deg) {
  // //print degree to serial
  // Serial.print("Yaw in degrees: ");
  // Serial.println(deg);
  const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  return dirs[(int)((((int)deg % 360 + 360) % 360 + 22.5) / 45.0) % 8];
}

ESP32Encoder encoder;

// Set up the GPS object
TinyGPSPlus gps;

// Define which serial port we’re using
#define GPS_RX 16  // Connect to GT-U7 TXD
#define GPS_TX 17  // Connect to GT-U7 RXD

HardwareSerial gpsSerial(1);  // Use UART1

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C  // Most common I2C address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RobotIMU imu(18, 19);  // IMU I2C pins
unsigned long lastUpdate = 0;

const int graphWidth = 128;
float speedHistory[graphWidth] = {0};
int graphIndex = 0;

void setup() {
  Serial.begin(115200);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // GPS default baud is 9600
  Serial.println("Starting GPS test...");

  // Initialize IMU
  imu.begin();
  imu.beginMag();  // Optional: for yaw
  imu.setAlpha(0.2);
  imu.setMadgwickSampleRate(250.0f);

  lastUpdate = millis();

  // Initialize OLED Display
  Wire.begin(21, 22);  // SDA, SCL for display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("IMU Starting...");
  display.display();
  delay(1000);

  // Enable motor driver
  pinMode(32, OUTPUT); // R_EN
  pinMode(33, OUTPUT); // L_EN
  digitalWrite(32, HIGH);
  digitalWrite(33, HIGH);

  // Set up PWM on RPWM and LPWM
  ledcSetup(0, 1000, 8);  // channel 0, 1kHz, 8-bit
  ledcAttachPin(25, 0);   // RPWM

  ledcSetup(1, 1000, 8);  // channel 1
  ledcAttachPin(26, 1);   // LPWM

  // Initialize encoder on GPIO 34 (A) and 35 (B)
  // ESP32Encoder::useInternalWeakPullResistors = Pullup::up;
  encoder.attachFullQuad(34, 35); // A = GPIO 34, B = GPIO 35
  encoder.setCount(0);
}

void loop() {



  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // GPS display block removed. See below for OLED output.






  static long lastPos = 0;
  long pos = encoder.getCount();
  long deltaPos = pos - lastPos;
  lastPos = pos;

  // Approximate encoder speed as counts per loop iteration (scaled)
  float encoderSpeed = abs(deltaPos) * 20.0; // scale factor for visibility

 
  // Check for real-time alpha tuning input
  if (Serial.available()) {
    float newAlpha = Serial.parseFloat();
    if (newAlpha >= 0.0 && newAlpha <= 1.0) {
      imu.setAlpha(newAlpha);
      Serial.print("Updated alpha to: ");
      Serial.println(newAlpha);
    } else {
      Serial.println("Alpha must be between 0.0 and 1.0");
    }
  }

  unsigned long now = millis();
  float dt = min((float)(now - lastUpdate) / 1000.0f, 0.1f);
  lastUpdate = now;

  imu.update(dt);

  float pitch = imu.getPitch();
  float roll = imu.getRoll();  // Flip roll for correct orientation
  float yaw = imu.getMadgwickYaw();

  // Serial.print("Pitch: "); Serial.print(pitch);
  // Serial.print(" Roll: "); Serial.print(roll);
  // Serial.print(" Yaw: "); Serial.println(yaw);1

  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Pitch: %.2f\n", pitch);
  display.printf("Roll:  %.2f\n", roll);
  display.printf("Yaw:   %.2f\n", yaw);
  display.printf("Lat: %.5f", gps.location.lat());
  display.printf("\nLng: %.5f", gps.location.lng());
  display.printf("\nSat: %d", gps.satellites.value());
  display.printf("\nAlt: %.2fm", gps.altitude.meters());
  display.printf("\nDir: %s\n", toCardinal(yaw));
  // Draw balance circle
  const int centerX = 100;
  const int centerY = 32;
  const int radius = 20;

  display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);

  // Map pitch and roll to screen displacement (scaled)
  int xOffset = constrain((int)(roll), -radius, radius);
  int yOffset = constrain((int)(pitch), -radius, radius);

  int dotX = centerX + xOffset;
  int dotY = centerY + yOffset;

  display.fillCircle(dotX, dotY, 3, SSD1306_WHITE);

  // Update speed history buffer
  speedHistory[graphIndex] = encoderSpeed;
  graphIndex = (graphIndex + 1) % graphWidth;

  // Draw speed graph
  const int graphBaseY = SCREEN_HEIGHT - 1;
  const int graphMaxHeight = 64;

  // Find the current peak value in the history
  float peakSpeed = 1.0;  // prevent divide-by-zero
  for (int i = 0; i < graphWidth; i++) {
    if (speedHistory[i] > peakSpeed) {
      peakSpeed = speedHistory[i];
    }
  }

  for (int i = 0; i < graphWidth; i++) {
    int index = (graphIndex + i) % graphWidth;
    int height = map(speedHistory[index], 0, peakSpeed, 0, graphMaxHeight);
    height = constrain(height, 0, graphMaxHeight);
    display.drawLine(i, graphBaseY, i, graphBaseY - height, SSD1306_WHITE);
  }

  display.display();

  int motorSpeed = constrain(abs((int)pitch * 5), 0, 255);
  // Determine motor direction based on pitch angle
  if (pitch > 2) {  // Tilted forward
    ledcWrite(0, motorSpeed); // Move forward
    ledcWrite(1, 0);
  } else if (pitch < -2) {  // Tilted backward
    ledcWrite(0, 0);
    ledcWrite(1, motorSpeed); // Move backward
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0); // Stop
  }

  delay(50);
}