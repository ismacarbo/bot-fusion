// Low-level controller for Arduino UNO + TB6612FNG.
// Receives PWM commands from Raspberry Pi and streams IMU + safety sensors.
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define PIN_Motor_PWMA  5
#define PIN_Motor_PWMB  6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY  3

#define PIN_TRIG  13
#define PIN_ECHO  12
#define PIN_IR_LEFT  A2
#define PIN_IR_RIGHT A3

const long SERIAL_BAUD = 115200;
const float GYRO_DPS_TO_RAD = 0.017453292519943295f;
const float GYRO_RAD_TO_DEG = 57.29577951308232f;

const int LEFT_SIGN = 1;   // set to -1 if left wheel direction is inverted
const int RIGHT_SIGN = 1;  // set to -1 if right wheel direction is inverted
const int SLEW_STEP = 16;

const unsigned long CMD_TIMEOUT_MS = 450;
const unsigned long TELEMETRY_MS = 50;
const unsigned long DIST_MS = 90;
const unsigned long ULTRA_TIMEOUT_US = 25000UL;

MPU6050 mpu;
float gyro_bias_z = 0.0f;
float yaw_rad = 0.0f;
float yaw_rate_dps = 0.0f;

int target_pwm_l = 0;
int target_pwm_r = 0;
int current_pwm_l = 0;
int current_pwm_r = 0;

int dist_c_cm = 999;
int ir_l_raw = 0;
int ir_r_raw = 0;

unsigned long last_cmd_ms = 0;
unsigned long last_imu_ms = 0;
unsigned long last_telemetry_ms = 0;
unsigned long last_dist_ms = 0;

char serial_buf[48];
uint8_t serial_idx = 0;

int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float wrap_pi(float a) {
  while (a > 3.14159265f) a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

int step_towards(int current, int target, int step) {
  if (target > current) return current + min(step, target - current);
  if (target < current) return current - min(step, current - target);
  return current;
}

void set_motor_hw(int pwm_l, int pwm_r) {
  int hw_l = clampi(LEFT_SIGN * pwm_l, -255, 255);
  int hw_r = clampi(RIGHT_SIGN * pwm_r, -255, 255);

  digitalWrite(PIN_Motor_AIN_1, hw_l >= 0);
  digitalWrite(PIN_Motor_BIN_1, hw_r >= 0);
  analogWrite(PIN_Motor_PWMA, abs(hw_l));
  analogWrite(PIN_Motor_PWMB, abs(hw_r));
}

void set_targets(int pwm_l, int pwm_r) {
  target_pwm_l = clampi(pwm_l, -255, 255);
  target_pwm_r = clampi(pwm_r, -255, 255);
  last_cmd_ms = millis();
}

void stop_motors() {
  target_pwm_l = 0;
  target_pwm_r = 0;
}

void calibrate_gyro_bias(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(4);
  }
  gyro_bias_z = sum / (float)samples;
}

void update_imu() {
  unsigned long now = millis();
  float dt = (now - last_imu_ms) / 1000.0f;
  if (dt <= 0.0f) {
    last_imu_ms = now;
    return;
  }
  last_imu_ms = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  yaw_rate_dps = (gz - gyro_bias_z) / 131.0f;
  float wz_rad = yaw_rate_dps * GYRO_DPS_TO_RAD;
  yaw_rad = wrap_pi(yaw_rad + wz_rad * dt);
}

long measure_pulse_us() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, ULTRA_TIMEOUT_US);
}

int read_distance_cm() {
  long us = measure_pulse_us();
  if (us == 0) return 999;
  int cm = (int)(us / 58);
  if (cm < 1) cm = 1;
  if (cm > 500) cm = 500;
  return cm;
}

void update_sensors() {
  unsigned long now = millis();
  if (now - last_dist_ms >= DIST_MS) {
    last_dist_ms = now;
    dist_c_cm = read_distance_cm();
  }
  ir_l_raw = analogRead(PIN_IR_LEFT);
  ir_r_raw = analogRead(PIN_IR_RIGHT);
}

void parse_line(char* line) {
  if (line[0] == '\0') return;

  if (strncmp(line, "CMD,", 4) == 0) {
    int l = 0, r = 0;
    if (sscanf(line, "CMD,%d,%d", &l, &r) == 2) {
      set_targets(l, r);
    }
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    stop_motors();
    return;
  }

  if (strcmp(line, "GYRO_ZERO") == 0) {
    stop_motors();
    set_motor_hw(0, 0);
    calibrate_gyro_bias(120);
    yaw_rad = 0.0f;
    Serial.println(F("ACK,GYRO_ZERO"));
    return;
  }

  if (strcmp(line, "PING") == 0) {
    Serial.println(F("ACK,PONG"));
    return;
  }
}

void poll_serial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serial_buf[serial_idx] = '\0';
      parse_line(serial_buf);
      serial_idx = 0;
      continue;
    }

    if (serial_idx < sizeof(serial_buf) - 1) {
      serial_buf[serial_idx++] = c;
    } else {
      serial_idx = 0;
    }
  }
}

void update_motors() {
  unsigned long now = millis();
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    target_pwm_l = 0;
    target_pwm_r = 0;
  }

  current_pwm_l = step_towards(current_pwm_l, target_pwm_l, SLEW_STEP);
  current_pwm_r = step_towards(current_pwm_r, target_pwm_r, SLEW_STEP);
  set_motor_hw(current_pwm_l, current_pwm_r);
}

void send_telemetry() {
  unsigned long now = millis();
  if (now - last_telemetry_ms < TELEMETRY_MS) return;
  last_telemetry_ms = now;

  Serial.print(F("STAT,"));
  Serial.print(now);
  Serial.print(F(","));
  Serial.print(yaw_rad * GYRO_RAD_TO_DEG, 2);
  Serial.print(F(","));
  Serial.print(yaw_rate_dps, 2);
  Serial.print(F(","));
  Serial.print(current_pwm_l);
  Serial.print(F(","));
  Serial.print(current_pwm_r);
  Serial.print(F(","));
  Serial.print(dist_c_cm);
  Serial.print(F(","));
  Serial.print(ir_l_raw);
  Serial.print(F(","));
  Serial.print(ir_r_raw);
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  Wire.begin();
  mpu.initialize();
  delay(900);
  calibrate_gyro_bias(200);

  last_cmd_ms = millis();
  last_imu_ms = millis();
  last_telemetry_ms = millis();
  last_dist_ms = millis();

  set_motor_hw(0, 0);
  Serial.println(F("READY, motors+imu+sensors"));
}

void loop() {
  poll_serial();
  update_imu();
  update_sensors();
  update_motors();
  send_telemetry();
}
