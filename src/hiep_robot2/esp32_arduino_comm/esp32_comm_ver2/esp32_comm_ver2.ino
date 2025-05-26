#include <Arduino.h>

// --- ENCODER PINS ---
#define ENCODER_L_A 14
#define ENCODER_L_B 27
#define ENCODER_R_A 34
#define ENCODER_R_B 35

// --- MOTOR PINS (L298N) ---
#define IN1 26
#define IN2 25
#define IN3 33
#define IN4 32
#define EN_A 13  // PWM Left
#define EN_B 12  // PWM Right

// --- ENCODER STATE ---
volatile long encoder_left = 0;
volatile long encoder_right = 0;

// --- PID SETPOINT ---
float target_speed_L = 0;
float target_speed_R = 0;

// --- PID GAINS ---
const float Kp_L = 0.96, Ki_L = 0.05, Kd_L = 0.05;
const float Kp_R = 1.0, Ki_R = 0.05, Kd_R = 0.05;

// --- PID STATE ---
float error_L = 0, prev_error_L = 0, integral_L = 0;
float error_R = 0, prev_error_R = 0, integral_R = 0;

// --- Speed Calculation ---
float actual_speed_L = 0;
float actual_speed_R = 0;

constexpr float pulses_per_rev = 2970.0;
constexpr float wheel_diameter = 0.048;
constexpr float pi = 3.1415926;
constexpr float I_MAX = 1.0;
constexpr float MAX_SPEED = 0.5;  // m/s

// --- INTERRUPT HANDLERS ---
void IRAM_ATTR encoderL_ISR() {
  encoder_left += (digitalRead(ENCODER_L_B) == HIGH) ? 1 : -1;
}

void IRAM_ATTR encoderR_ISR() {
  encoder_right += (digitalRead(ENCODER_R_B) == HIGH) ? 1 : -1;
}

// --- MOTOR CONTROL ---
void setMotor(int in1, int in2, int pwm_pin, float pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  digitalWrite(in1, pwm_val > 0);
  digitalWrite(in2, pwm_val < 0);
  analogWrite(pwm_pin, abs((int)pwm_val));
}

// --- SPEED CALCULATION ---
float calculate_speed(long delta_encoder, float dt) {
  float revs = delta_encoder / pulses_per_rev;
  float distance = revs * (pi * wheel_diameter);
  return distance / dt;  // m/s
}

// --- PARSE SERIAL INPUT ---
void parseSerialInput() {
  static char buffer[32];  // đủ lớn cho chuỗi như "123,-120\n"
  if (Serial.available()) {
    size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';
    int v_l = 0, v_r = 0;
    if (sscanf(buffer, "%d,%d", &v_l, &v_r) == 2) {
      target_speed_L = v_l / 255.0 * MAX_SPEED;
      target_speed_R = v_r / 255.0 * MAX_SPEED;
    }
  }
}

// --- PID CONTROL ---
float computePID(float target, float actual, float& integral, float& prev_error, float Kp, float Ki, float Kd, float dt) {
  float error = target - actual;
  integral += error * dt;
  integral = constrain(integral, -I_MAX, I_MAX);
  float derivative = (error - prev_error) / dt;
  prev_error = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderR_ISR, RISING);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(EN_A, OUTPUT); pinMode(EN_B, OUTPUT);

  setMotor(IN1, IN2, EN_A, 0);
  setMotor(IN3, IN4, EN_B, 0);
}

// --- LOOP ---
void loop() {
  static unsigned long last_time = millis();
  static long last_encoder_L = 0;
  static long last_encoder_R = 0;

  parseSerialInput();

  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  if (dt >= 0.05) {
    last_time = now;

    noInterrupts();
    long curr_encoder_L = encoder_left;
    long curr_encoder_R = encoder_right;
    interrupts();

    long delta_L = curr_encoder_L - last_encoder_L;
    long delta_R = curr_encoder_R - last_encoder_R;
    last_encoder_L = curr_encoder_L;
    last_encoder_R = curr_encoder_R;

    actual_speed_L = calculate_speed(delta_L, dt);
    actual_speed_R = calculate_speed(delta_R, dt);

    float output_L = computePID(target_speed_L, actual_speed_L, integral_L, prev_error_L, Kp_L, Ki_L, Kd_L, dt);
    float output_R = computePID(target_speed_R, actual_speed_R, integral_R, prev_error_R, Kp_R, Ki_R, Kd_R, dt);

    int16_t pwm_L = output_L / MAX_SPEED * 255.0;
    int16_t pwm_R = output_R / MAX_SPEED * 255.0;

    setMotor(IN1, IN2, EN_A, pwm_L);
    setMotor(IN3, IN4, EN_B, pwm_R);

    Serial.printf("%ld,%ld\n", curr_encoder_L, curr_encoder_R);
  }
}
