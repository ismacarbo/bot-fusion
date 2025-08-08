#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// Motor driver pins (TB6612FNG)
#define PIN_Motor_PWMA   5
#define PIN_Motor_PWMB   6
#define PIN_Motor_AIN_1  7
#define PIN_Motor_BIN_1  8
#define PIN_Motor_STBY   3

// Encoder pins (single-channel, rising edge)
#define ENCODER_LEFT_PIN  2  // interrupt 0
#define ENCODER_RIGHT_PIN 3  // interrupt 1

// Constants for odometry
const float WHEEL_RADIUS = 0.03f;    // meters (adjust to your wheel)
const float WHEEL_BASE = 0.15f;      // distance between left/right wheels (meters)
const int TICKS_PER_REV = 360;        // placeholder: you must measure this
const float ALPHA = 0.98f;           // complementary filter weight (gyro high-pass)

// Conversion
#define DEG_TO_RAD 0.017453292519943295f

// globals for encoders
volatile long encoder_left = 0;
volatile long encoder_right = 0;

// odometry state
float x = 0.0f, y = 0.0f;            // position in meters
float theta_enc = 0.0f;              // orientation estimate from encoders
float theta_fused = 0.0f;            // fused orientation
float theta_gyro = 0.0f;             // integrated gyro orientation

unsigned long last_odom_time = 0;
float last_gyro_z = 0.0f;             // for optional smoothing

// MPU6050
MPU6050 mpu;
float gyro_z_offset = 0.0f; // to calibrate bias

// Utility to read encoder counts safely
void safeReadEncoders(long &left, long &right) {
  noInterrupts();
  left = encoder_left;
  right = encoder_right;
  interrupts();
}

// Interrupt handlers
void encoderLeftISR() {
  encoder_left++;
}

void encoderRightISR() {
  encoder_right++;
}

// Motor control
void move(int pwmA, int pwmB) {
  digitalWrite(PIN_Motor_AIN_1, pwmA >= 0);
  digitalWrite(PIN_Motor_BIN_1, pwmB >= 0);
  analogWrite(PIN_Motor_PWMA, abs(pwmA));
  analogWrite(PIN_Motor_PWMB, abs(pwmB));
}

void stopMotors() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // Motor pins
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);

  // Encoder pins
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightISR, RISING);

  randomSeed(analogRead(A0));

  // Small delay to stabilize MPU
  delay(1000);

  // Calibrate gyro z bias (take average of few samples)
  const int CAL_SAMPLES = 200;
  long sum = 0;
  for (int i = 0; i < CAL_SAMPLES; ++i) {
    int16_t gx, gy, gz, ax, ay, az;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(5);
  }
  gyro_z_offset = sum / (float)CAL_SAMPLES;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyro_z_offset);

  last_odom_time = millis();
}

void updateIMUandOdometry(float &x, float &y, float &theta_enc, float &theta_fused, float &theta_gyro) {
  unsigned long now = millis();
  float dt = (now - last_odom_time) / 1000.0f;
  if (dt <= 0.0f) return; // safety
  last_odom_time = now;

  // --- Read MPU6050 ---
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // Convert gyro Z to rad/s, subtract offset
  float gyro_z_dps = (gz_raw - gyro_z_offset) / 131.0f; // deg/s
  float gyro_z_rad = gyro_z_dps * DEG_TO_RAD;           // rad/s

  // Integrate gyro orientation
  theta_gyro += gyro_z_rad * dt;

  // --- Read encoders ---
  static long last_left_ticks = 0;
  static long last_right_ticks = 0;
  long left_ticks, right_ticks;
  safeReadEncoders(left_ticks, right_ticks);

  long delta_left = left_ticks - last_left_ticks;
  long delta_right = right_ticks - last_right_ticks;
  last_left_ticks = left_ticks;
  last_right_ticks = right_ticks;

  // Convert ticks to distance
  float dist_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float dL = delta_left * dist_per_tick;
  float dR = delta_right * dist_per_tick;

  float ds = (dR + dL) / 2.0f;
  float dtheta_enc = (dR - dL) / WHEEL_BASE;

  // Update encoder-based theta
  theta_enc += dtheta_enc;

  // Complementary filter: fuse gyro integration (fast) with encoder (slow drift correction)
  float predicted_theta = theta_fused + gyro_z_rad * dt;
  theta_fused = ALPHA * predicted_theta + (1.0f - ALPHA) * theta_enc;

  // Normalize theta_fused between -pi..pi
  if (theta_fused > M_PI) theta_fused -= 2.0f * M_PI;
  if (theta_fused < -M_PI) theta_fused += 2.0f * M_PI;

  // Update position using fused orientation
  x += ds * cosf(theta_fused);
  y += ds * sinf(theta_fused);

  // --- Debug output ---
  Serial.print("EncL:");
  Serial.print(delta_left);
  Serial.print(" EncR:");
  Serial.print(delta_right);
  Serial.print(" dL:");
  Serial.print(dL, 4);
  Serial.print(" dR:");
  Serial.print(dR, 4);
  Serial.print(" theta_enc:");
  Serial.print(theta_enc, 4);
  Serial.print(" theta_gyro:");
  Serial.print(theta_gyro, 4);
  Serial.print(" fused:");
  Serial.print(theta_fused, 4);
  Serial.print(" x:");
  Serial.print(x, 4);
  Serial.print(" y:");
  Serial.print(y, 4);
  Serial.print(" | GyroZ(rad/s):");
  Serial.print(gyro_z_rad, 4);
  Serial.print(" Accel:");
  Serial.print(ax_raw);
  Serial.print(",");
  Serial.print(ay_raw);
  Serial.print(",");
  Serial.print(az_raw);
  Serial.println();
}

void loop() {
  // Random motion test (2s move, 1s stop)
  int dir = random(0, 4);
  int speed = random(150, 255);

  switch (dir) {
    case 0: move(speed, speed);    Serial.println("Command: forward"); break;
    case 1: move(-speed, -speed);  Serial.println("Command: backward"); break;
    case 2: move(-speed, speed);   Serial.println("Command: rotate left"); break;
    case 3: move(speed, -speed);   Serial.println("Command: rotate right"); break;
  }

  unsigned long move_start = millis();
  while (millis() - move_start < 2000) {
    updateIMUandOdometry(x, y, theta_enc, theta_fused, theta_gyro);
    delay(50); // ~20Hz update
  }

  stopMotors();
  Serial.println("Stopped");
  unsigned long stop_start = millis();
  while (millis() - stop_start < 1000) {
    updateIMUandOdometry(x, y, theta_enc, theta_fused, theta_gyro);
    delay(50);
  }
}
