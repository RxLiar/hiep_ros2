#include <Arduino.h>

// ============================================================
// Hardware pins - copied from the legacy robot
// ============================================================
#define ENCODER_L_A 14
#define ENCODER_L_B 27
#define ENCODER_R_A 34
#define ENCODER_R_B 35

#define IN1 26
#define IN2 25
#define IN3 33
#define IN4 32
#define EN_A 13
#define EN_B 12

// Change these to -1 if a motor/encoder direction is reversed physically.
constexpr int LEFT_MOTOR_SIGN = 1;
constexpr int RIGHT_MOTOR_SIGN = 1;
constexpr int LEFT_ENCODER_SIGN = 1;
constexpr int RIGHT_ENCODER_SIGN = 1;

// ============================================================
// Robot constants - starter values from the legacy firmware
// ============================================================
constexpr float PULSES_PER_REV = 2970.0f;
constexpr float WHEEL_DIAMETER_M = 0.048f;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float MAX_WHEEL_SPEED_MPS = 0.60f;

constexpr uint32_t CONTROL_PERIOD_MS = 20;     // 50 Hz PID + telemetry
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;  // HARDWARE FAIL-SAFE

// PID values are starter values only. Re-tune on the real robot.
constexpr float KP_L = 0.96f;
constexpr float KI_L = 0.05f;
constexpr float KD_L = 0.05f;
constexpr float KP_R = 1.00f;
constexpr float KI_R = 0.05f;
constexpr float KD_R = 0.05f;
constexpr float I_MAX = 1.0f;

volatile int32_t encoder_left = 0;
volatile int32_t encoder_right = 0;

float target_speed_left = 0.0f;
float target_speed_right = 0.0f;
float actual_speed_left = 0.0f;
float actual_speed_right = 0.0f;

float integral_left = 0.0f;
float integral_right = 0.0f;
float prev_error_left = 0.0f;
float prev_error_right = 0.0f;

uint32_t last_valid_command_ms = 0;
uint32_t telemetry_sequence = 0;
bool command_seen = false;

// ============================================================
// Encoder ISRs
// ============================================================
void IRAM_ATTR encoderLeftISR() {
  const int direction = (digitalRead(ENCODER_L_B) == HIGH) ? 1 : -1;
  encoder_left += LEFT_ENCODER_SIGN * direction;
}

void IRAM_ATTR encoderRightISR() {
  const int direction = (digitalRead(ENCODER_R_B) == HIGH) ? 1 : -1;
  encoder_right += RIGHT_ENCODER_SIGN * direction;
}

// ============================================================
// Motor driver
// ============================================================
void setMotorRaw(int in1, int in2, int pwm_pin, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(pwm_pin, abs(pwm));
}

void setLeftMotor(int pwm) {
  setMotorRaw(IN1, IN2, EN_A, LEFT_MOTOR_SIGN * pwm);
}

void setRightMotor(int pwm) {
  setMotorRaw(IN3, IN4, EN_B, RIGHT_MOTOR_SIGN * pwm);
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  target_speed_left = 0.0f;
  target_speed_right = 0.0f;
  integral_left = 0.0f;
  integral_right = 0.0f;
  prev_error_left = 0.0f;
  prev_error_right = 0.0f;
}

// ============================================================
// Serial protocol
// ROS -> ESP32: CMD,<seq>,<left_mm_s>,<right_mm_s>
// ============================================================
void handleLine(char *line) {
  // Device identity. Useful for diagnostics; ROS auto-detection itself is
  // passive and does not need to send PING to unknown serial devices.
  if (strcmp(line, "PING") == 0) {
    Serial.println("ID,AGV_ESP32,0.4.0");
    return;
  }

  long sequence = 0;
  long left_mm_s = 0;
  long right_mm_s = 0;

  if (sscanf(line, "CMD,%ld,%ld,%ld", &sequence, &left_mm_s, &right_mm_s) == 3) {
    (void)sequence;  // Reserved for diagnostics/ACK later.

    target_speed_left = constrain(left_mm_s / 1000.0f, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    target_speed_right = constrain(right_mm_s / 1000.0f, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);

    last_valid_command_ms = millis();
    command_seen = true;
  }
}

// Non-blocking line assembler. The old readBytesUntil() version returned a
// partial line whenever two bytes were >2 ms apart, silently dropping that
// command, and also blocked the control loop while waiting.
void parseSerialInput() {
  static char buffer[96];
  static size_t len = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;

    if (c == '\n' || c == '\r') {
      if (len > 0 && !overflow) {
        buffer[len] = '\0';
        handleLine(buffer);
      }
      len = 0;
      overflow = false;
      continue;
    }

    if (len < sizeof(buffer) - 1) {
      buffer[len++] = static_cast<char>(c);
    } else {
      overflow = true;  // discard the rest of this over-long line
    }
  }
}

// ============================================================
// PID
// ============================================================
float encoderDeltaToSpeed(int32_t delta_ticks, float dt) {
  const float revolutions = delta_ticks / PULSES_PER_REV;
  const float distance_m = revolutions * PI_F * WHEEL_DIAMETER_M;
  return distance_m / dt;
}

float computePid(
    float target,
    float actual,
    float &integral,
    float &prev_error,
    float kp,
    float ki,
    float kd,
    float dt) {

  const float error = target - actual;
  integral += error * dt;
  integral = constrain(integral, -I_MAX, I_MAX);
  const float derivative = (error - prev_error) / dt;
  prev_error = error;

  return kp * error + ki * integral + kd * derivative;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);

  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderRightISR, RISING);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  stopMotors();

  // Announce identity once at boot. The host can also identify this firmware
  // from the continuously emitted TEL frames below.
  delay(50);
  Serial.println("ID,AGV_ESP32,0.4.0");
}

void loop() {
  static uint32_t last_control_ms = millis();
  static int32_t last_encoder_left = 0;
  static int32_t last_encoder_right = 0;

  parseSerialInput();

  const uint32_t now_ms = millis();

  // HARD FAIL-SAFE: if ROS/USB/PC disappears, stop locally on the ESP32.
  if (!command_seen || (now_ms - last_valid_command_ms) > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }

  const uint32_t elapsed_ms = now_ms - last_control_ms;
  if (elapsed_ms < CONTROL_PERIOD_MS) {
    return;
  }
  last_control_ms = now_ms;
  const float dt = elapsed_ms / 1000.0f;

  noInterrupts();
  const int32_t current_left = encoder_left;
  const int32_t current_right = encoder_right;
  interrupts();

  const int32_t delta_left = current_left - last_encoder_left;
  const int32_t delta_right = current_right - last_encoder_right;
  last_encoder_left = current_left;
  last_encoder_right = current_right;

  actual_speed_left = encoderDeltaToSpeed(delta_left, dt);
  actual_speed_right = encoderDeltaToSpeed(delta_right, dt);

  const float control_left = computePid(
      target_speed_left, actual_speed_left,
      integral_left, prev_error_left,
      KP_L, KI_L, KD_L, dt);

  const float control_right = computePid(
      target_speed_right, actual_speed_right,
      integral_right, prev_error_right,
      KP_R, KI_R, KD_R, dt);

  // PID output is interpreted as requested speed correction, then scaled to PWM.
  int pwm_left = static_cast<int>(255.0f * control_left / MAX_WHEEL_SPEED_MPS);
  int pwm_right = static_cast<int>(255.0f * control_right / MAX_WHEEL_SPEED_MPS);
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  // Keep motors physically stopped while watchdog is active.
  if (!command_seen || (now_ms - last_valid_command_ms) > COMMAND_TIMEOUT_MS) {
    pwm_left = 0;
    pwm_right = 0;
  }

  setLeftMotor(pwm_left);
  setRightMotor(pwm_right);

  ++telemetry_sequence;
  const long vel_left_mm_s = lroundf(actual_speed_left * 1000.0f);
  const long vel_right_mm_s = lroundf(actual_speed_right * 1000.0f);

  Serial.printf(
      "TEL,%lu,%lu,%ld,%ld,%ld,%ld\n",
      static_cast<unsigned long>(telemetry_sequence),
      static_cast<unsigned long>(now_ms),
      static_cast<long>(current_left),
      static_cast<long>(current_right),
      vel_left_mm_s,
      vel_right_mm_s);
}
