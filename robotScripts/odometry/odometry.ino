
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>


#define PIN_Motor_PWMA   5  
#define PIN_Motor_PWMB   6  
#define PIN_Motor_AIN_1  7  
#define PIN_Motor_BIN_1  8  
#define PIN_Motor_STBY   3  


const float WHEEL_RADIUS = 0.03f;  
const float WHEEL_BASE   = 0.15f;  


float KV_LEFT  = 0.0035f;  
float KV_RIGHT = 0.0035f;  
int   DEADZONE = 20;       


const float KV_MIN = 0.0008f, KV_MAX = 0.02f;
const float KV_LR_LAMBDA = 0.12f;        
const float DZ_GAMMA     = 0.05f;        
const int   DZ_MIN = 0, DZ_MAX = 140;


const float OMEGA_THRESH = 0.25f;        
const int   PWM_MIN_FOR_CAL = 60;        
const int   PWM_RAMP_STEP   = 6;         
const unsigned long RAMP_STEP_MS = 150;  


MPU6050 mpu;
float gyro_bias_z = 0.0f;                


float x = 0.0f, y = 0.0f;                
float theta_model = 0.0f;                
float theta_fused = 0.0f;                
unsigned long last_ms = 0;
const float ALPHA = 0.98f;               


int pwmA_cmd = 0, pwmB_cmd = 0;          


enum Mode {
  CAL_DZ_LEFT_RAMP,   // ramp PWM until gyro detects rotation (left-turn pattern)
  CAL_DZ_RIGHT_RAMP,  // same for right-turn pattern
  CAL_KV_LEFT,        // hold PWM and learn KV while rotating left
  CAL_KV_RIGHT,       // hold PWM and learn KV while rotating right
  RUN                 // normal run mode
};
Mode mode = CAL_DZ_LEFT_RAMP;

int dz_left_found = -1, dz_right_found = -1;  // discovered PWM thresholds
unsigned long state_t0 = 0, last_ramp_ms = 0;
int ramp_pwm = 0;


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int   clampi(int v, int a, int b)   { return v < a ? a : (v > b ? b : v); }
float clampf(float v, float a, float b){ return v < a ? a : (v > b ? b : v); }


void setMotors(int pwmA, int pwmB){
  pwmA_cmd = clampi(pwmA, -255, 255);
  pwmB_cmd = clampi(pwmB, -255, 255);
  digitalWrite(PIN_Motor_AIN_1, pwmA_cmd >= 0);
  digitalWrite(PIN_Motor_BIN_1, pwmB_cmd >= 0);
  analogWrite(PIN_Motor_PWMA, abs(pwmA_cmd));
  analogWrite(PIN_Motor_PWMB, abs(pwmB_cmd));
}

void stopMotors(){ setMotors(0, 0); }

// Convert PWM to wheel linear velocity [m/s] using current model
float pwm_to_v(int pwm, float KV){
  int ap = abs(pwm);
  if (ap <= DEADZONE) return 0.0f;
  float v = KV * (ap - DEADZONE);
  return (pwm >= 0) ? v : -v;
}

// Read IMU yaw rate [rad/s]
float read_omega_rad(){
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float dps = (gz - gyro_bias_z) / 131.0f;   // deg/s (MPU6050 scale for ±250 dps)
  return dps * (M_PI / 180.0f);
}

// Online adaptation of KV and DEADZONE while rotating in place
void adaptive_identification(float omega){
  // Require opposite signs (in-place turn) and similar magnitudes
  if ((pwmA_cmd * pwmB_cmd) >= 0) return;
  int a = abs(pwmA_cmd), b = abs(pwmB_cmd);
  if (a < PWM_MIN_FOR_CAL || b < PWM_MIN_FOR_CAL) return;
  if (abs(a - b) > 10) return; // make sure magnitudes are similar

  float abs_pwm = 0.5f * (a + b);

  // If yaw rate is too small, likely under the deadzone: push DEADZONE upward
  if (fabsf(omega) < OMEGA_THRESH){
    int dz_target = clampi((int)abs_pwm, DZ_MIN, DZ_MAX);
    DEADZONE = clampi((int)roundf((1.0f - DZ_GAMMA) * DEADZONE + DZ_GAMMA * dz_target), DZ_MIN, DZ_MAX);
    return;
  }

  // For a symmetric in-place turn: |vL| = |vR| = |omega| * L / 2
  float v_abs = fabsf(omega) * (WHEEL_BASE * 0.5f);
  float denom = abs_pwm - (float)DEADZONE;
  if (denom < 5.0f) return; // avoid unstable division

  // Instantaneous KV estimate
  float k_inst = v_abs / denom;
  k_inst = clampf(k_inst, KV_MIN, KV_MAX);

  // Update both sides towards k_inst (without encoders we can’t distinguish sides well)
  KV_LEFT  = (1.0f - KV_LR_LAMBDA) * KV_LEFT  + KV_LR_LAMBDA * k_inst;
  KV_RIGHT = (1.0f - KV_LR_LAMBDA) * KV_RIGHT + KV_LR_LAMBDA * k_inst;
}

// -------------------------------- Setup --------------------------------
void setup(){
  Serial.begin(115200);
  Wire.begin();

  // TB6612 pins
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);   // enable driver

  // IMU init
  mpu.initialize();
  delay(800);

  // Gyro Z bias calibration (simple average at rest)
  const int N = 200; 
  long sum = 0;
  for (int i = 0; i < N; i++){
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(5);
  }
  gyro_bias_z = sum / (float)N;

  last_ms      = millis();
  state_t0     = millis();
  last_ramp_ms = millis();

  Serial.println(F("Auto-calibration starting (DEADZONE + KV). Keep area clear."));
}

// ------------------------------- Loop ----------------------------------
void loop(){
  // Time step
  unsigned long now = millis();
  float dt = (now - last_ms) / 1000.0f;
  if (dt <= 0) dt = 1e-3f;
  last_ms = now;

  // IMU yaw rate
  float omega = read_omega_rad();

  
  switch (mode){
    case CAL_DZ_LEFT_RAMP: {
      // Ramp PWM to find DEADZONE for left-turn pattern (A back, B forward)
      if (now - last_ramp_ms > RAMP_STEP_MS){
        last_ramp_ms = now;
        ramp_pwm = clampi(ramp_pwm + PWM_RAMP_STEP, 0, 255);
        setMotors(-ramp_pwm, +ramp_pwm);
      }
      // Once yaw exceeds threshold, we found the effective PWM
      if (fabsf(omega) > OMEGA_THRESH){
        dz_left_found = ramp_pwm;
        Serial.print(F("[DZ] Left threshold PWM = ")); Serial.println(dz_left_found);
        stopMotors(); delay(300);
        ramp_pwm = 0;
        state_t0 = now;
        last_ramp_ms = now;
        mode = CAL_DZ_RIGHT_RAMP;
      }
    } break;

    case CAL_DZ_RIGHT_RAMP: {
      // Ramp PWM to find DEADZONE for right-turn pattern (A forward, B back)
      if (now - last_ramp_ms > RAMP_STEP_MS){
        last_ramp_ms = now;
        ramp_pwm = clampi(ramp_pwm + PWM_RAMP_STEP, 0, 255);
        setMotors(+ramp_pwm, -ramp_pwm);
      }
      if (fabsf(omega) > OMEGA_THRESH){
        dz_right_found = ramp_pwm;
        Serial.print(F("[DZ] Right threshold PWM = ")); Serial.println(dz_right_found);
        stopMotors(); delay(300);

        // Set DEADZONE as the average (minus a small margin)
        if (dz_left_found < 0) dz_left_found = dz_right_found;
        DEADZONE = clampi(((dz_left_found + dz_right_found)/2) - 4, DZ_MIN, DZ_MAX);
        Serial.print(F("[DZ] DEADZONE set to ")); Serial.println(DEADZONE);

        // Prepare KV calibration at a constant PWM
        ramp_pwm = clampi(DEADZONE + 70, 60, 220);  // constant PWM for KV
        state_t0 = now;
        mode = CAL_KV_LEFT;
        Serial.print(F("[KV] Calibrating LEFT at PWM=")); Serial.println(ramp_pwm);
      }
    } break;

    case CAL_KV_LEFT: {
      // Spin left at constant PWM ~4 s, adapt KV & DEADZONE online
      setMotors(-ramp_pwm, +ramp_pwm);
      adaptive_identification(omega);
      if (now - state_t0 > 4000){
        stopMotors(); delay(300);
        state_t0 = now;
        mode = CAL_KV_RIGHT;
        Serial.print(F("[KV] Calibrating RIGHT at PWM=")); Serial.println(ramp_pwm);
      }
    } break;

    case CAL_KV_RIGHT: {
      // Spin right at constant PWM ~4 s, adapt KV & DEADZONE online
      setMotors(+ramp_pwm, -ramp_pwm);
      adaptive_identification(omega);
      if (now - state_t0 > 4000){
        stopMotors(); delay(200);
        mode = RUN;
        Serial.print(F("[KV] Final KV_L=")); Serial.print(KV_LEFT, 5);
        Serial.print(F(" KV_R=")); Serial.print(KV_RIGHT, 5);
        Serial.print(F(" DEADZONE=")); Serial.println(DEADZONE);
        Serial.println(F("[RUN] Starting forward motion."));
      }
    } break;

    case RUN: {
      // Example "run" behavior: drive forward with a moderate PWM
      // TODO: replace with navigation
      int pwm_run = clampi(DEADZONE + 60, 80, 200);
      setMotors(pwm_run, pwm_run);

      // Optional: continue tiny online adaptation if you detect in-place turns
      if ((pwmA_cmd * pwmB_cmd) < 0) adaptive_identification(omega);
    } break;
  }

  // ---------------------- Odometry computation --------------------------
  // Convert current PWMs to wheel speeds
  float vL = pwm_to_v(pwmA_cmd, KV_LEFT);
  float vR = pwm_to_v(pwmB_cmd, KV_RIGHT);

  // Differential-drive model
  float ds     = 0.5f * (vR + vL) * dt;
  float dtheta = (vR - vL) / WHEEL_BASE * dt;

  // Complementary filter for yaw: gyro integration + model correction
  float theta_pred = theta_fused + omega * dt;  // gyro-integrated yaw
  theta_model     += dtheta;                    // model-integrated yaw
  theta_fused      = ALPHA * theta_pred + (1.0f - ALPHA) * theta_model;

  // Normalize yaw to [-pi, pi]
  if (theta_fused > M_PI)  theta_fused -= 2.0f * M_PI;
  if (theta_fused < -M_PI) theta_fused += 2.0f * M_PI;

  // Update (x, y) using midpoint heading
  float theta_mid = theta_fused - 0.5f * dtheta;
  x += ds * cosf(theta_mid);
  y += ds * sinf(theta_mid);

  // --------------------------- Periodic debug ---------------------------
  static unsigned long last_dbg = 0;
  if (now - last_dbg > 300){
    last_dbg = now;
    Serial.print(F("mode=")); Serial.print((int)mode);
    Serial.print(F(" x:")); Serial.print(x,3);
    Serial.print(F(" y:")); Serial.print(y,3);
    Serial.print(F(" th(deg):")); Serial.print(theta_fused * 180.0f / M_PI, 1);
    Serial.print(F(" | KV_L=")); Serial.print(KV_LEFT,4);
    Serial.print(F(" KV_R=")); Serial.print(KV_RIGHT,4);
    Serial.print(F(" DZ=")); Serial.print(DEADZONE);
    Serial.print(F(" pwm=(")); Serial.print(pwmA_cmd);
    Serial.print(",");         Serial.print(pwmB_cmd);
    Serial.println(F(")"));
}
