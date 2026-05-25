/*
 * Autonomous Self-Driving Car - ESP32 Dual-Core RTOS Controller
 * Author: Nivesh Kumar Meena
 *
 * Uses FreeRTOS on ESP32:
 * - Core 0: High-speed Serial Packet Decoding & Watchdog Task
 * - Core 1: Hardware LEDC High-frequency PWM Actuation & Sensor ISRs
 */

#include <Arduino.h>

// Pins
#define PIN_SERVO_PWM       18
#define PIN_MOTOR_PWM_A     19
#define PIN_MOTOR_PWM_B     21
#define PIN_ENCODER_A       34
#define PIN_ULTRASONIC_TRIG 22
#define PIN_ULTRASONIC_ECHO 23

// LEDC Channels
#define LEDC_SERVO_CH       0
#define LEDC_MOTOR_A_CH     1
#define LEDC_MOTOR_B_CH     2
#define LEDC_FREQ_SERVO     50     // 50 Hz for standard servos
#define LEDC_FREQ_MOTOR     20000  // 20 kHz silent motor drive
#define LEDC_RES_BITS       14     // 14-bit resolution

// Command Structure
struct ActuationCommand {
  float steer_deg;
  float throttle;
  float brake;
  bool estop;
};

QueueHandle_t actQueue;
volatile unsigned long lastHeartbeat = 0;

void IRAM_ATTR encoderISR() {
  // Encoder tick handling
}

// ================= Task Core 0: Communication =================
void taskCommunication(void *pvParams) {
  while (true) {
    if (Serial.available() >= 18) {
      if (Serial.read() == 0xAA && Serial.read() == 0x55) {
        uint8_t id = Serial.read();
        uint8_t len = Serial.read();
        if (id == 0x01 && len == 13) {
          uint8_t buf[13];
          Serial.readBytes(buf, 13);
          uint8_t chk = Serial.read();

          uint8_t calc = 0;
          for (int i = 0; i < 13; i++) calc ^= buf[i];

          if (calc == chk) {
            ActuationCommand cmd;
            memcpy(&cmd.steer_deg, &buf[0], 4);
            memcpy(&cmd.throttle, &buf[4], 4);
            memcpy(&cmd.brake, &buf[8], 4);
            cmd.estop = (buf[12] == 1);

            xQueueOverwrite(actQueue, &cmd);
            lastHeartbeat = millis();
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================= Task Core 1: Actuation & Watchdog =================
void taskActuation(void *pvParams) {
  ActuationCommand currentCmd = {0.0, 0.0, 0.0, false};

  while (true) {
    if (xQueueReceive(actQueue, &currentCmd, 0) == pdTRUE) {
      // New command received
    }

    // Safety watchdog: cut motors if no command for 350ms
    if (millis() - lastHeartbeat > 350 || currentCmd.estop) {
      ledcWrite(LEDC_MOTOR_A_CH, 0);
      ledcWrite(LEDC_MOTOR_B_CH, 0);
    } else {
      // Map steering angle (-30 to +30 deg) to servo duty cycle
      float clampedSteer = constrain(currentCmd.steer_deg, -30.0, 30.0);
      // 50 Hz: 1ms = 819 counts, 1.5ms = 1228 counts, 2.0ms = 1638 counts
      uint32_t servoDuty = map((long)(clampedSteer * 10), -300, 300, 950, 1500);
      ledcWrite(LEDC_SERVO_CH, servoDuty);

      // Motor PWM
      if (currentCmd.throttle > 0.01) {
        uint32_t motorDuty = (uint32_t)(currentCmd.throttle * 16383.0);
        ledcWrite(LEDC_MOTOR_A_CH, motorDuty);
        ledcWrite(LEDC_MOTOR_B_CH, 0);
      } else if (currentCmd.brake > 0.01) {
        uint32_t brakeDuty = (uint32_t)(currentCmd.brake * 16383.0);
        ledcWrite(LEDC_MOTOR_A_CH, brakeDuty);
        ledcWrite(LEDC_MOTOR_B_CH, brakeDuty); // Dynamic brake
      } else {
        ledcWrite(LEDC_MOTOR_A_CH, 0);
        ledcWrite(LEDC_MOTOR_B_CH, 0);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz control loop
  }
}

void setup() {
  Serial.begin(115200);

  actQueue = xQueueCreate(1, sizeof(ActuationCommand));

  // Configure LEDC PWM
  ledcAttach(PIN_SERVO_PWM, LEDC_FREQ_SERVO, LEDC_RES_BITS);
  ledcAttach(PIN_MOTOR_PWM_A, LEDC_FREQ_MOTOR, LEDC_RES_BITS);
  ledcAttach(PIN_MOTOR_PWM_B, LEDC_FREQ_MOTOR, LEDC_RES_BITS);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, RISING);

  lastHeartbeat = millis();

  // Create tasks pinned to specific cores
  xTaskCreatePinnedToCore(taskCommunication, "TaskComm", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskActuation, "TaskAct", 4096, NULL, 2, NULL, 1);
}

void loop() {
  // Empty - FreeRTOS tasks manage execution
  vTaskDelete(NULL);
}
