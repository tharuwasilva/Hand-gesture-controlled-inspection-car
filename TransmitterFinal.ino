#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// =================================================
// TRANSMITTER - MPU6050 + ESP-NOW + SAFETY BUTTON
// =================================================

// RECEIVER MAC ADDRESS
uint8_t receiverMAC[] = {
  0x6C, 0xC8, 0x40, 0x8C, 0x58, 0xB0
};

// =================================================
// SETTINGS
// =================================================

#define MPU_ADDR      0x68
#define SDA_PIN       21
#define SCL_PIN       22

// PUSH BUTTON
#define BUTTON_PIN    23

#define DEADZONE      10
#define SEND_INTERVAL 50

#define INVERT_FORWARD_BACK false
#define INVERT_LEFT_RIGHT   false

#define SMOOTHING 0.3f

// =================================================
// COMMAND DATA
// Must match receiver exactly
// =================================================

struct CommandData {
  char command;
  uint8_t tilt;
};

CommandData data;

float pitch = 0;
float roll  = 0;

// =================================================
// MPU INITIALIZATION
// =================================================

void mpuInit() {

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

// =================================================
// READ MPU ACCELEROMETER
// =================================================

bool readAccel(float &ax, float &ay, float &az) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6)
    return false;

  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();

  ax = rx / 16384.0f;
  ay = ry / 16384.0f;
  az = rz / 16384.0f;

  return true;
}

// =================================================
// READ TILT AND CALCULATE COMMAND
// =================================================

void readCommand() {

  float ax, ay, az;

  if (!readAccel(ax, ay, az)) {
    data.command = 'S';
    data.tilt = 0;
    return;
  }

  float angX = atan2(ax, sqrt(ay * ay + az * az))
               * 180.0f / PI;

  float angY = atan2(ay, sqrt(ax * ax + az * az))
               * 180.0f / PI;

  // FORWARD / BACKWARD
  float p = -angY;

  // LEFT / RIGHT
  float r = -angX;

  if (INVERT_FORWARD_BACK)
    p = -p;

  if (INVERT_LEFT_RIGHT)
    r = -r;

  // Smoothing
  pitch += SMOOTHING * (p - pitch);
  roll  += SMOOTHING * (r - roll);

  float absPitch = fabs(pitch);
  float absRoll  = fabs(roll);

  if (absPitch < DEADZONE && absRoll < DEADZONE) {

    data.command = 'S';
    data.tilt = 0;

  }
  else if (absPitch >= absRoll) {

    data.command = (pitch > 0) ? 'F' : 'B';

    data.tilt = (uint8_t)constrain(
      (int)absPitch, 0, 90
    );

  }
  else {

    data.command = (roll > 0) ? 'R' : 'L';

    data.tilt = (uint8_t)constrain(
      (int)absRoll, 0, 90
    );
  }
}

// =================================================
// SETUP
// =================================================

void setup() {

  Serial.begin(115200);

  // MPU6050
  mpuInit();

  // PUSH BUTTON
  // INPUT_PULLUP means:
  // Pressed  = LOW
  // Released = HIGH

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW initialization FAILED!");
    return;
  }

  // Add receiver
  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, receiverMAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {

    Serial.println("Failed to add receiver!");
    return;
  }

  Serial.println("==============================");
  Serial.println("TRANSMITTER READY");
  Serial.println("THUMB BUTTON SAFETY ENABLED");
  Serial.println("==============================");
}

// =================================================
// MAIN LOOP
// =================================================

void loop() {

  static unsigned long lastSend = 0;

  if (millis() - lastSend >= SEND_INTERVAL) {

    lastSend = millis();

    // Read MPU6050
    readCommand();

    // =============================================
    // THUMB BUTTON SAFETY CONTROL
    // =============================================

    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

    if (!buttonPressed) {

      // Button released: FORCE STOP
      data.command = 'S';
      data.tilt = 0;
    }

    // Send ESP-NOW data
    esp_now_send(
      receiverMAC,
      (uint8_t *)&data,
      sizeof(data)
    );

    // Debug information
    Serial.print("Button: ");

    Serial.print(buttonPressed ? "PRESSED" : "RELEASED");

    Serial.print(" | Command: ");
    Serial.print(data.command);

    Serial.print(" | Tilt: ");
    Serial.println(data.tilt);
  }
}