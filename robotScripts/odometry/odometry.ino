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


volatile long virt_left_ticks  = 0;
volatile long virt_right_ticks = 0;
const long  TPR_VIRTUAL = 360;     



const float KV_LEFT   = 0.0035f;   
const float KV_RIGHT  = 0.0032f;   
const int   DEADZONE  = 20;        


MPU6050 mpu;
float gyro_bias_z = 0.0f;


float x = 0.0f, y = 0.0f;
float theta_enc = 0.0f;     
float theta_fused = 0.0f;   
unsigned long last_ms = 0;
const float ALPHA = 0.98f;  


int pwmA_cmd = 0, pwmB_cmd = 0;


float clampf(float v, float a, float b){ return v < a ? a : (v > b ? b : v); }

void setMotors(int pwmA, int pwmB) {
  pwmA_cmd = clampf(pwmA, -255, 255);
  pwmB_cmd = clampf(pwmB, -255, 255);

  digitalWrite(PIN_Motor_AIN_1, pwmA_cmd >= 0);
  digitalWrite(PIN_Motor_BIN_1, pwmB_cmd >= 0);
  analogWrite(PIN_Motor_PWMA, abs(pwmA_cmd));
  analogWrite(PIN_Motor_PWMB, abs(pwmB_cmd));
}

void stopMotors(){ setMotors(0,0); }

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);

  mpu.initialize();
  delay(1000);

  
  const int N=200; long sum=0;
  for(int i=0;i<N;i++){
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    sum += gz;
    delay(5);
  }
  gyro_bias_z = sum / (float)N;

  last_ms = millis();

  Serial.println(F("Ready. Serial cmds: f/b/l/r/s (move), p=print, z=zero."));
  Serial.println(F("NOTE: KV_LEFT/KV_RIGHT must be calibrated!"));
}

void updateOdomSim() {
  unsigned long now = millis();
  float dt = (now - last_ms) / 1000.0f;
  if (dt <= 0) return;
  last_ms = now;

  
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float gyro_z_dps = (gz - gyro_bias_z) / 131.0f;      
  float gyro_z_rad = gyro_z_dps * (M_PI / 180.0f);     

  
  auto pwm2v = [&](int pwm, float KV)->float{
    int ap = abs(pwm);
    if (ap <= DEADZONE) return 0.0f;
    float v = KV * (ap - DEADZONE);
    return (pwm >= 0) ? v : -v;
  };
  float vL = pwm2v(pwmA_cmd, KV_LEFT);
  float vR = pwm2v(pwmB_cmd, KV_RIGHT);

  
  float omegaL = vL / WHEEL_RADIUS;             
  float omegaR = vR / WHEEL_RADIUS;             
  float revL   = (omegaL * dt) / (2.0f * M_PI); 
  float revR   = (omegaR * dt) / (2.0f * M_PI);

  long dTicksL = lround(revL * TPR_VIRTUAL);
  long dTicksR = lround(revR * TPR_VIRTUAL);
  virt_left_ticks  += dTicksL;
  virt_right_ticks += dTicksR;

  
  float dL = vL * dt;
  float dR = vR * dt;
  float ds = (dR + dL) / 2.0f;
  float dtheta_enc = (dR - dL) / WHEEL_BASE;
  theta_enc += dtheta_enc;

  
  float theta_pred = theta_fused + gyro_z_rad * dt;
  theta_fused = ALPHA * theta_pred + (1.0f - ALPHA) * theta_enc;

  
  if (theta_fused > M_PI)  theta_fused -= 2.0f*M_PI;
  if (theta_fused < -M_PI) theta_fused += 2.0f*M_PI;

  
  float theta_mid = theta_fused - dtheta_enc/2.0f;
  x += ds * cosf(theta_mid);
  y += ds * sinf(theta_mid);

  
  static unsigned long last_dbg=0;
  if (now - last_dbg > 200) {
    last_dbg = now;
    Serial.print(F("ticksL:")); Serial.print(virt_left_ticks);
    Serial.print(F(" ticksR:")); Serial.print(virt_right_ticks);
    Serial.print(F(" | x:")); Serial.print(x,3);
    Serial.print(F(" y:")); Serial.print(y,3);
    Serial.print(F(" th(deg):")); Serial.print(theta_fused*180.0f/M_PI,1);
    Serial.print(F(" vL:")); Serial.print(vL,3);
    Serial.print(F(" vR:")); Serial.print(vR,3);
    Serial.println();
  }
}

void handleSerial(){
  if(!Serial.available()) return;
  char c = (char)Serial.read();
  switch(c){
    case 'f': setMotors(+200,+200); Serial.println(F("forward")); break;
    case 'b': setMotors(-200,-200); Serial.println(F("backward")); break;
    case 'l': setMotors(-180,+180); Serial.println(F("rotate left")); break;
    case 'r': setMotors(+180,-180); Serial.println(F("rotate right")); break;
    case 's': stopMotors(); Serial.println(F("stop")); break;
    case 'p':
      Serial.print(F("virt TPR=")); Serial.print(TPR_VIRTUAL);
      Serial.print(F("  ticksL=")); Serial.print(virt_left_ticks);
      Serial.print(F("  ticksR=")); Serial.print(virt_right_ticks);
      Serial.print(F("  x=")); Serial.print(x,3);
      Serial.print(F("  y=")); Serial.print(y,3);
      Serial.print(F("  th(deg)=")); Serial.println(theta_fused*180.0f/M_PI,1);
      break;
    case 'z':
      x=y=0; theta_enc=theta_fused=0;
      virt_left_ticks=virt_right_ticks=0;
      Serial.println(F("odom reset"));
      break;
  }
}

void loop(){
  handleSerial();
  updateOdomSim();
}
