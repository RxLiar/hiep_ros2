
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

// --- PID GAINS (separated) ---
float Kp_L = 0.964, Ki_L = 0.05, Kd_L = 0.05;
float Kp_R = 1.0, Ki_R = 0.05, Kd_R = 0.05;

// --- PID State ---
float error_L = 0, prev_error_L = 0, integral_L = 0;
float error_R = 0, prev_error_R = 0, integral_R = 0;

// --- Speed Calculation ---
float actual_speed_L = 0;
float actual_speed_R = 0;

const float pulses_per_rev = 2970;
const float wheel_diameter = 0.048;
const float pi = 3.1415926;
const float I_MAX = 1.0;  // Max integral term

// --- INTERRUPT HANDLERS ---
void IRAM_ATTR encoderL_ISR() {
  int b = digitalRead(ENCODER_L_B);
  encoder_left += (b == HIGH) ? 1 : -1;
}

void IRAM_ATTR encoderR_ISR() {
  int b = digitalRead(ENCODER_R_B);
  encoder_right += (b == HIGH) ? 1 : -1;
}

// --- MOTOR CONTROL ---
void setMotor(int in1, int in2, int pwm_pin, float pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm_val < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm_pin, abs((int)pwm_val));
}

// --- CALCULATE SPEED ---
float calculate_speed(long delta_encoder, float dt) {
  float revs = delta_encoder / pulses_per_rev;
  float distance = revs * (pi * wheel_diameter);
  return distance / dt; // m/s
}

void setup() {
  Serial.begin(115200);

  // --- Encoder Setup ---
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderR_ISR, RISING);

  // --- Motor Setup ---
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(EN_A, OUTPUT); pinMode(EN_B, OUTPUT);

  setMotor(IN1, IN2, EN_A, 0);
  setMotor(IN3, IN4, EN_B, 0);
}

void loop() {
  static unsigned long last_time = millis();
  static long last_encoder_L = 0;
  static long last_encoder_R = 0;

  // --- Receive Command from ROS ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int v_l = 0, v_r = 0;
    if (sscanf(input.c_str(), "%d,%d", &v_l, &v_r) == 2) {
      target_speed_L = v_l / 255.0 * 0.5;  // scale to m/s
      target_speed_R = v_r / 255.0 * 0.5;
    }
  }

  // --- PID Loop (every 50ms) ---
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  if (dt >= 0.05) {
    last_time = now;

    // Read encoder safely
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

    // --- PID Left ---
    error_L = target_speed_L - actual_speed_L;
    integral_L += error_L * dt;
    integral_L = constrain(integral_L, -I_MAX, I_MAX);
    float derivative_L = (error_L - prev_error_L) / dt;
    float output_L = Kp_L * error_L + Ki_L * integral_L + Kd_L * derivative_L;
    prev_error_L = error_L;

    // --- PID Right ---
    error_R = target_speed_R - actual_speed_R;
    integral_R += error_R * dt;
    integral_R = constrain(integral_R, -I_MAX, I_MAX);
    float derivative_R = (error_R - prev_error_R) / dt;
    float output_R = Kp_R * error_R + Ki_R * integral_R + Kd_R * derivative_R;
    prev_error_R = error_R;

    // Convert output speed to PWM
    int16_t pwm_L = output_L / 0.5 * 255.0;
    int16_t pwm_R = output_R / 0.5 * 255.0;

    setMotor(IN1, IN2, EN_A, pwm_L);
    setMotor(IN3, IN4, EN_B, pwm_R);

    // --- Send Encoder to ROS ---
    Serial.printf("%ld,%ld\n", curr_encoder_L, curr_encoder_R);
  }
}
