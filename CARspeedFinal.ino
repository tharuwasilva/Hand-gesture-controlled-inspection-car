
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

// =================================================
// TRANSMITTER MAC ADDRESS
// =================================================

uint8_t TRANSMITTER_MAC[] = {
  0x58, 0x2A, 0xBD, 0x78, 0xA8, 0x40
};

// Receiver MAC: 6C:C8:40:8C:58:B0

// =================================================
// L298N MOTOR PINS
// =================================================

// RIGHT MOTOR
#define ENA 27
#define IN1 26
#define IN2 25

// LEFT MOTOR
#define IN3 33
#define IN4 32
#define ENB 14

// =================================================
// IR SENSOR PINS
// =================================================

#define FRONT_IR_PIN 18
#define REAR_IR_PIN 19

// LOW = obstacle detected
#define IR_ACTIVE_LOW true

// =================================================
// MOTOR SETTINGS
// =================================================

#define MIN_SPEED 90
#define MAX_SPEED 255

#define TILT_MIN 10
#define TILT_MAX 30

#define RAMP_STEP 6
#define DECEL_STEP 20

#define TIMEOUT 500

// Obstacle braking settings
#define BRAKE_PULSE_MS 200
#define BRAKE_HOLD_MS 500

// =================================================
// ESP-NOW DATA
// =================================================

struct CommandData {
  char command;
  uint8_t tilt;
};

volatile char currentCommand = 'S';
volatile uint8_t lastTilt = 0;
volatile unsigned long lastReceiveTime = 0;

int targetSpeed = 0;
int motorSpeed = 0;
int turnSpeed = 0;

char activeCmd = 'S';

// Obstacle braking state
bool brakeHoldActive = false;
bool blockedForward = false;
unsigned long brakeStartedAt = 0;

// =================================================
// IR SENSOR DETECTION
// =================================================

bool obstacleDetected(int pin) {

  int state = digitalRead(pin);

  if (IR_ACTIVE_LOW) {
    return state == LOW;
  }

  return state == HIGH;
}

// =================================================
// SPEED FROM TILT
// =================================================

void updateSpeedFromTilt(int tilt) {

  tilt = constrain(tilt, TILT_MIN, TILT_MAX);

  targetSpeed = map(
    tilt,
    TILT_MIN,
    TILT_MAX,
    MIN_SPEED,
    MAX_SPEED
  );
}

// =================================================
// SPEED RAMP
// =================================================

void rampSpeed() {

  int current = motorSpeed;

  if (current < targetSpeed) {
    current = min(current + RAMP_STEP, targetSpeed);
  }
  else if (current > targetSpeed) {
    current = max(current - DECEL_STEP, targetSpeed);
  }

  motorSpeed = current;

  turnSpeed = current * 35 / 100;
}

// =================================================
// ESP-NOW RECEIVE
// =================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {

  // Verify transmitter MAC
  if (memcmp(info->src_addr, TRANSMITTER_MAC, 6) != 0) {
    return;
  }

  if (len != sizeof(CommandData)) {
    return;
  }

  CommandData data;
  memcpy(&data, incomingData, sizeof(data));

  // Validate command
  if (
    data.command != 'F' &&
    data.command != 'B' &&
    data.command != 'L' &&
    data.command != 'R' &&
    data.command != 'S'
  ) {
    return;
  }

  currentCommand = data.command;
  lastTilt = data.tilt;

  lastReceiveTime = millis();
}

// =================================================
// ACTIVE MOTOR BRAKING
// =================================================

void brakeCar() {

  // Both motor terminals LOW
  // L298N active braking mode

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);
}

// =================================================
// NORMAL MOTOR STOP (COAST)
// =================================================

void stopCar() {

  rightMotor(0);
  leftMotor(0);
}

// =================================================
// RIGHT MOTOR
// =================================================

void rightMotor(int speed) {

  if (speed > 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

  }
  else if (speed < 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

  }
  else {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

  }

  ledcWrite(ENA, constrain(abs(speed), 0, 255));
}

// =================================================
// LEFT MOTOR
// =================================================

void leftMotor(int speed) {

  if (speed > 0) {

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

  }
  else if (speed < 0) {

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

  }
  else {

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

  }

  ledcWrite(ENB, constrain(abs(speed), 0, 255));
}

// =================================================
// DRIVE
// =================================================

void drive(int rightSpeed, int leftSpeed) {

  rightMotor(rightSpeed);
  leftMotor(leftSpeed);
}

// =================================================
// SETUP
// =================================================

void setup() {

  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ESP32 Arduino Core 3.x PWM
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  stopCar();

  // IR sensors
  pinMode(FRONT_IR_PIN, INPUT_PULLUP);
  pinMode(REAR_IR_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW initialization FAILED!");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("==============================");
  Serial.println("ESP-NOW RECEIVER READY");
  Serial.println("FRONT IR READY");
  Serial.println("REAR IR READY");
  Serial.println("DIRECTIONAL SAFETY ENABLED");
  Serial.println("==============================");
}

// =================================================
// MAIN LOOP
// =================================================

void loop() {

  // Read obstacle sensors
  bool frontBlocked = obstacleDetected(FRONT_IR_PIN);
  bool rearBlocked = obstacleDetected(REAR_IR_PIN);

  // Read received command
  bool signalLost =
    (millis() - lastReceiveTime > TIMEOUT);

  char cmd = signalLost ? 'S' : currentCommand;
  uint8_t tilt = lastTilt;

  // Forward includes F, L and R
  bool forwardRequest =
    (cmd == 'F' || cmd == 'L' || cmd == 'R');

  bool reverseRequest = (cmd == 'B');

  // Check whether requested direction is blocked
  bool directionBlocked =
    (frontBlocked && forwardRequest) ||
    (rearBlocked && reverseRequest);

  // =================================================
  // START OBSTACLE BRAKING
  // =================================================

  if (directionBlocked && !brakeHoldActive) {

    brakeHoldActive = true;

    blockedForward = forwardRequest;

    brakeStartedAt = millis();

    Serial.println("OBSTACLE DETECTED - BRAKING!");
  }

  // =================================================
  // BRAKING / ESCAPE CONTROL
  // =================================================

  if (brakeHoldActive) {

    targetSpeed = 0;
    motorSpeed = 0;
    turnSpeed = 0;

    activeCmd = 'S';

    unsigned long elapsed =
      millis() - brakeStartedAt;

    // Apply active braking for 200 ms
    if (elapsed < BRAKE_PULSE_MS) {

      brakeCar();

    }
    else {

      // Keep motors stopped after braking pulse
      stopCar();

    }

    // Determine whether the opposite direction is safe
    bool safeEscapeDirection;

    if (blockedForward) {

      // Front obstacle: allow reverse if rear is clear
      safeEscapeDirection =
        reverseRequest && !rearBlocked;

    }
    else {

      // Rear obstacle: allow forward if front is clear
      safeEscapeDirection =
        forwardRequest && !frontBlocked;

    }

    // Require a neutral command or safe escape direction.
    // Never automatically resume the blocked direction.
    bool neutralCommand = (cmd == 'S');

    if (
      elapsed >= BRAKE_HOLD_MS &&
      (neutralCommand || safeEscapeDirection)
    ) {

      brakeHoldActive = false;

      Serial.println("Safety delay complete");
    }
  }

  // =================================================
  // NORMAL MOTOR CONTROL
  // =================================================

  else {

    bool moving =
      (forwardRequest || reverseRequest);

    if (moving) {

      activeCmd = cmd;

      updateSpeedFromTilt(tilt);

      if (motorSpeed < MIN_SPEED) {
        motorSpeed = MIN_SPEED;
      }

    }
    else {

      targetSpeed = 0;

    }

    rampSpeed();

    // Drive motors
    if (motorSpeed <= 0) {

      stopCar();
      activeCmd = 'S';

    }
    else if (activeCmd == 'F') {

      drive(motorSpeed, motorSpeed);

    }
    else if (activeCmd == 'B') {

      drive(-motorSpeed, -motorSpeed);

    }
    else if (activeCmd == 'L') {

      drive(motorSpeed, turnSpeed);

    }
    else if (activeCmd == 'R') {

      drive(turnSpeed, motorSpeed);

    }
    else {

      stopCar();

    }
  }

  // =================================================
  // SERIAL MONITOR
  // =================================================

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 200) {

    lastPrint = millis();

    Serial.print("Command: ");
    Serial.print(cmd);

    Serial.print(" | Front: ");
    Serial.print(frontBlocked ? "OBSTACLE" : "CLEAR");

    Serial.print(" | Rear: ");
    Serial.print(rearBlocked ? "OBSTACLE" : "CLEAR");

    Serial.print(" | Speed: ");
    Serial.print(motorSpeed);

    Serial.print(" | Safety: ");
    Serial.println(brakeHoldActive ? "BRAKING/LOCKED" : "READY");
  }

  delay(20);
}
