/*
 * Autonomous Self-Driving Car - Arduino Motor & Sensor Controller
 * Author: Nivesh Kumar Meena
 *
 * Capabilities:
 * - Dual DC Motor H-Bridge PWM Driver (TB6612FNG / L298N)
 * - Front Wheel Steering Servo Control (PWM pin 9)
 * - Interrupt-based Optical Wheel Encoder Tachometer (INT1 - Pin 3)
 * - Ultrasonic HC-SR04 Trigger & Echo Rangefinder (Pin 4 Trigger, Pin 2 Echo INT0)
 * - Watchdog Safety Failsafe: Stops motors if communication lost for >350ms
 * - Binary Framed Serial Protocol with XOR Checksum (115200 baud)
 */

#include <Servo.h>

// ================= Pin Assignments =================
// Steering Servo
const int PIN_SERVO = 9;

// DC Motor Driver (L298N / TB6612FNG)
const int PIN_MOTOR_PWM = 5;  // Motor Speed (Timer 0/3)
const int PIN_MOTOR_DIR_A = 7;
const int PIN_MOTOR_DIR_B = 8;
const int PIN_MOTOR_ENABLE = 6;

// Sensors
const int PIN_ENCODER = 3;       // External Interrupt 1
const int PIN_ECHO = 2;          // Ultrasonic Echo (Interrupt 0)
const int PIN_TRIG = 4;          // Ultrasonic Trigger
const int PIN_BATTERY_SENSE = A0;// Voltage Divider Analog In

// ================= Protocol Constants =================
const uint8_t SYNC_1 = 0xAA;
const uint8_t SYNC_2 = 0x55;
const uint8_t MSG_DRIVE_CMD = 0x01;
const uint8_t MSG_TELEMETRY = 0x02;

// ================= Global State =================
Servo steeringServo;
volatile unsigned long encoderTicks = 0;
volatile unsigned long echoStartTime = 0;
volatile unsigned long echoDuration = 0;
volatile bool newDistanceReady = false;

unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 350;
const unsigned long TELEMETRY_INTERVAL_MS = 50; // 20 Hz

float targetSteerDeg = 0.0;
float targetThrottle = 0.0;
float targetBrake = 0.0;
bool emergencyStop = false;

// ================= Interrupt Service Routines =================
void isrEncoder() {
  encoderTicks++;
}

void isrEcho() {
  if (digitalRead(PIN_ECHO) == HIGH) {
    echoStartTime = micros();
  } else {
    echoDuration = micros() - echoStartTime;
    newDistanceReady = true;
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);

  // Pin modes
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_DIR_A, OUTPUT);
  pinMode(PIN_MOTOR_DIR_B, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  digitalWrite(PIN_MOTOR_ENABLE, HIGH);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_ENCODER, INPUT_PULLUP);

  // Attach Steering Servo
  steeringServo.attach(PIN_SERVO);
  steeringServo.write(90); // Center steering (90 deg)

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), isrEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), isrEcho, CHANGE);

  lastCommandTime = millis();
}

// ================= Motor Control =================
void applyActuation(float steerDeg, float throttle, float brake, bool estop) {
  if (estop) {
    analogWrite(PIN_MOTOR_PWM, 0);
    digitalWrite(PIN_MOTOR_DIR_A, LOW);
    digitalWrite(PIN_MOTOR_DIR_B, LOW);
    return;
  }

  // Servo: map [-30 deg, +30 deg] to [60 deg, 120 deg] servo angle
  int servoAngle = constrain(90 + (int)steerDeg, 55, 125);
  steeringServo.write(servoAngle);

  // Motor drive
  if (brake > 0.05) {
    // Dynamic braking (short motor terminals)
    digitalWrite(PIN_MOTOR_DIR_A, HIGH);
    digitalWrite(PIN_MOTOR_DIR_B, HIGH);
    analogWrite(PIN_MOTOR_PWM, (int)(brake * 255.0));
  } else if (throttle > 0.01) {
    // Forward drive
    digitalWrite(PIN_MOTOR_DIR_A, HIGH);
    digitalWrite(PIN_MOTOR_DIR_B, LOW);
    int pwmVal = constrain((int)(throttle * 255.0), 0, 255);
    analogWrite(PIN_MOTOR_PWM, pwmVal);
  } else {
    // Coast
    analogWrite(PIN_MOTOR_PWM, 0);
    digitalWrite(PIN_MOTOR_DIR_A, LOW);
    digitalWrite(PIN_MOTOR_DIR_B, LOW);
  }
}

// ================= Ultrasonic Trigger =================
void triggerUltrasonic() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
}

// ================= Packet Handling =================
void processSerial() {
  while (Serial.available() >= 18) {
    if (Serial.read() != SYNC_1) continue;
    if (Serial.read() != SYNC_2) continue;

    uint8_t msgId = Serial.read();
    uint8_t length = Serial.read();

    if (msgId == MSG_DRIVE_CMD && length == 13) {
      uint8_t payload[13];
      for (int i = 0; i < 13; i++) {
        payload[i] = Serial.read();
      }
      uint8_t receivedChk = Serial.read();

      // Compute XOR checksum
      uint8_t calcChk = 0;
      for (int i = 0; i < 13; i++) calcChk ^= payload[i];

      if (calcChk == receivedChk) {
        memcpy(&targetSteerDeg, &payload[0], 4);
        memcpy(&targetThrottle, &payload[4], 4);
        memcpy(&targetBrake, &payload[8], 4);
        emergencyStop = (payload[12] == 1);
        lastCommandTime = millis();
      }
    }
  }
}

void sendTelemetry() {
  // Speed calculation from encoder ticks:
  // circumference = 2 * pi * 0.033 = 0.207m; 20 ticks per rev
  static unsigned long lastTicks = 0;
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dtSec = (now - lastTime) / 1000.0;
  if (dtSec <= 0.001) return;

  unsigned long currentTicks = encoderTicks;
  unsigned long deltaTicks = currentTicks - lastTicks;
  float speedMps = ((float)deltaTicks / 20.0 * 0.2073) / dtSec;

  lastTicks = currentTicks;
  lastTime = now;

  // Ultrasonic distance (duration in microseconds / 58 = cm, / 5800 = meters)
  float distM = (echoDuration > 0) ? ((float)echoDuration / 5800.0) : 3.0;

  // Battery voltage: 10k / 4.7k divider
  int rawAdc = analogRead(PIN_BATTERY_SENSE);
  float batteryVolts = (rawAdc * 5.0 / 1023.0) * ((10.0 + 4.7) / 4.7);

  float dummyYaw = 0.0;

  // Frame telemetry packet
  uint8_t payload[16];
  memcpy(&payload[0], &speedMps, 4);
  memcpy(&payload[4], &distM, 4);
  memcpy(&payload[8], &dummyYaw, 4);
  memcpy(&payload[12], &batteryVolts, 4);

  uint8_t chk = 0;
  for (int i = 0; i < 16; i++) chk ^= payload[i];

  Serial.write(SYNC_1);
  Serial.write(SYNC_2);
  Serial.write(MSG_TELEMETRY);
  Serial.write(16);
  Serial.write(payload, 16);
  Serial.write(chk);
}

// ================= Main Loop =================
void loop() {
  processSerial();

  // Safety Watchdog
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS) {
    // Communication lost: stop vehicle safely
    applyActuation(0.0, 0.0, 1.0, true);
  } else {
    applyActuation(targetSteerDeg, targetThrottle, targetBrake, emergencyStop);
  }

  // Periodic Telemetry & Range Triggering
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    triggerUltrasonic();
    sendTelemetry();
  }
}
