#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>


#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 3

#define PIN_TRIG  13
#define PIN_ECHO  12
#define PIN_SERVO 10
Servo head;

#define PIN_IR_LEFT  A2
#define PIN_IR_RIGHT A3


bool IR_ACTIVE_HIGH = true;          
int  irThreshL = 600, irThreshR = 600;


const bool IR_TRIGGERS_EMERG = true; 
const unsigned long IR_STICKY_MS = 120; 


const int PWM_FWD  = 50;   
const int PWM_BACK = 45;   
const int PWM_TURN = 90;  
const int DEADZONE_PWM = 0;


const int SLEW_MAX_DELTA = 20; 


const int MIN_DIST_CM = 25;   
const int BACK_MS     = 300;
const int TURN_MS     = 400;

const int MAX_SCORE_DIST_CM = 90;
const int SLOWDOWN_DIST_CM  = 80;  
const int MIN_CRUISE_PWM    = 60;  
const int TURN_GAIN         = 90;  
const int IR_WEIGHT         = 1;   
const int IR_SLOW_BASE      = 55;  


int       SERVO_CMD_CENTER = 30;
int       SERVO_SWING      = 55;
const int SERVO_SETTLE_MS  = 120;
const int SERVO_DWELL_MS   = 140;


const unsigned long PULSE_TIMEOUT_US = 30000UL; 


MPU6050 mpu;
float gyro_z_offset = 0.0f;
unsigned long last_imu_ms = 0;
float yaw_rad = 0.0f;
float yaw_rate_dps = 0.0f;
const float DPS_TO_RAD = 0.017453292519943295f;
const float YAW_DAMP_GAIN = 1.2f;


enum State { RUN, EMERG_BACK, EMERG_TURN };
State state = RUN;
unsigned long state_t0 = 0;
int bestTurn = 0;


unsigned long last_telemetry = 0;
const unsigned long TELEMETRY_MS = 100;


int distL = -1, distC = -1, distR = -1;
int currentServoDeg = 0;
unsigned long servo_t0 = 0;
int servoPhase = 0;


int last_pwmL = 0, last_pwmR = 0;


unsigned long irL_last_ms = 0, irR_last_ms = 0;


inline int clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
inline int clampServo(int v){ return clampi(v, 0, 180); }
inline int capDist(int cm,int cap){ return cm>cap?cap:(cm<0?0:cm); }

void servoGo(int deg){ deg = clampServo(deg); head.write(deg); currentServoDeg = deg; }
void servoCenter(){ servoGo(SERVO_CMD_CENTER); }
void servoLeft(){   servoGo(SERVO_CMD_CENTER + SERVO_SWING); }
void servoRight(){  servoGo(SERVO_CMD_CENTER - SERVO_SWING); }

int applyDeadzone(int pwm){
  if (pwm == 0) return 0;
  int s = (pwm>0) ? 1 : -1;
  int ap = abs(pwm);
  if (ap < DEADZONE_PWM) ap = DEADZONE_PWM;
  return s*clampi(ap,0,255);
}


static inline int approachWithSlew(int target, int current, int step){
  if (target > current) return current + min(step, target - current);
  if (target < current) return current - min(step, current - target);
  return current;
}

void setMotorsRaw(int pwmL, int pwmR){
  
  pwmL = applyDeadzone(clampi(pwmL, -255, 255));
  pwmR = applyDeadzone(clampi(pwmR, -255, 255));
  
  int nextL = approachWithSlew(pwmL, last_pwmL, SLEW_MAX_DELTA);
  int nextR = approachWithSlew(pwmR, last_pwmR, SLEW_MAX_DELTA);
  
  digitalWrite(PIN_Motor_AIN_1, nextL >= 0);
  digitalWrite(PIN_Motor_BIN_1, nextR >= 0);
  analogWrite(PIN_Motor_PWMA, clampi(abs(nextL), 0, 255));
  analogWrite(PIN_Motor_PWMB, clampi(abs(nextR), 0, 255));
  last_pwmL = nextL;
  last_pwmR = nextR;
}

void setMotorsForwardDifferential(int base, int turn){
  
  turn = clampi(turn, -PWM_TURN, PWM_TURN);
  int l = clampi(base - turn, -255, 255);
  int r = clampi(base + turn, -255, 255);
  setMotorsRaw(l, r);
}
void setMotorsBackward(int pwm){ setMotorsRaw(-clampi(pwm,0,255), -clampi(pwm,0,255)); }
void stopMotors(){ setMotorsRaw(0,0); }


long measurePulse(){
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, PULSE_TIMEOUT_US);
}
int readDistanceCmOnce(){
  long us = measurePulse();
  if (us == 0) return 999;
  int cm = (int)(us / 58);
  if (cm < 1) cm = 1;
  if (cm > 500) cm = 500;
  return cm;
}
int median3(int a, int b, int c){
  if (a > b) { int t=a; a=b; b=t; }
  if (b > c) { int t=b; b=c; c=t; }
  if (a > b) { int t=a; a=b; b=t; }
  return b;
}
int readDistanceCm(){
  int a = readDistanceCmOnce(); delay(5);
  int b = readDistanceCmOnce(); delay(5);
  int c = readDistanceCmOnce();
  return median3(a,b,c);
}


int readIRRaw(int pin){ return analogRead(pin); }

bool irTriggeredRaw(int raw, int thresh){
  return IR_ACTIVE_HIGH ? (raw > thresh) : (raw < thresh);
}


bool irDebounced(int raw, int thresh, unsigned long &t_last){
  unsigned long now = millis();
  bool trig = irTriggeredRaw(raw, thresh);
  if (trig) t_last = now;
  if ((now - t_last) <= IR_STICKY_MS) return true;
  return false;
}


char lastCmd = 0;
void pollSerialCmd(){
  if (Serial.available()){
    char c = Serial.read();
    lastCmd = c;
    if      (c=='S'){ stopMotors(); state=RUN; }
    else if (c=='F'){ state=RUN; }
    else if (c=='B'){ state=EMERG_BACK; state_t0=millis(); }
    else if (c=='L'){ bestTurn=+1; state=EMERG_TURN; state_t0=millis(); }
    else if (c=='R'){ bestTurn=-1; state=EMERG_TURN; state_t0=millis(); }
  }
}


void autoCalibrateIR(){
  long sumL=0, sumR=0;
  const int N=50;
  for (int i=0;i<N;i++){
    sumL += analogRead(PIN_IR_LEFT);
    sumR += analogRead(PIN_IR_RIGHT);
    delay(5);
  }
  int baseL=sumL/N, baseR=sumR/N;
  int margin=120;
  irThreshL = clampi(baseL + (IR_ACTIVE_HIGH?margin:-margin), 0, 1023);
  irThreshR = clampi(baseR + (IR_ACTIVE_HIGH?margin:-margin), 0, 1023);
}


void imuInit(){
  mpu.initialize();
  delay(800);
  const int N=200;
  long sum=0;
  for(int i=0;i<N;i++){
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    sum += gz;
    delay(5);
  }
  gyro_z_offset = sum / (float)N;
  last_imu_ms = millis();
}
void imuUpdate(){
  unsigned long now = millis();
  float dt = (now - last_imu_ms) / 1000.0f;
  if (dt <= 0.0f) { last_imu_ms = now; return; }
  last_imu_ms = now;
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float gz_dps = (gz - gyro_z_offset) / 131.0f;
  yaw_rate_dps = gz_dps;
  float gz_rad = gz_dps * DPS_TO_RAD;
  yaw_rad += gz_rad * dt;
  if (yaw_rad > 3.14159265f)  yaw_rad -= 6.28318531f;
  if (yaw_rad < -3.14159265f) yaw_rad += 6.28318531f;
}


void updateServoScan(){
  unsigned long now = millis();
  if (now - servo_t0 < (unsigned long)SERVO_DWELL_MS) return;
  servo_t0 = now;
  if (servoPhase == 0){
    servoLeft();  delay(SERVO_SETTLE_MS);
    imuUpdate();
    distL = readDistanceCm();
    servoPhase = 1;
  }
  else if (servoPhase == 1){
    servoCenter(); delay(SERVO_SETTLE_MS);
    imuUpdate();
    distC = readDistanceCm();
    servoPhase = 2;
  }
  else if (servoPhase == 2){
    servoRight(); delay(SERVO_SETTLE_MS);
    imuUpdate();
    distR = readDistanceCm();
    servoPhase = 3;
  }
  else{
    servoCenter(); delay(SERVO_SETTLE_MS/2);
    imuUpdate();
    distC = readDistanceCm();
    servoPhase = 0;
  }
}


void decideSteeringAndDrive(int irLraw, int irRraw){
  bool irL = irTriggeredRaw(irLraw, irThreshL);
  bool irR = irTriggeredRaw(irRraw, irThreshR);

  int dL = distL<0?0:distL, dC = distC<0?0:distC, dR = distR<0?0:distR;
  int cL = capDist(dL, MAX_SCORE_DIST_CM);
  int cC = capDist(dC, MAX_SCORE_DIST_CM);
  int cR = capDist(dR, MAX_SCORE_DIST_CM);

  int numL = cL, numR = cR;
  if (irL) numL -= (MAX_SCORE_DIST_CM * IR_WEIGHT);
  if (irR) numR -= (MAX_SCORE_DIST_CM * IR_WEIGHT);

  int steerNum = clampi(numL - numR, -MAX_SCORE_DIST_CM, MAX_SCORE_DIST_CM);
  int turnPWM = (steerNum * TURN_GAIN) / MAX_SCORE_DIST_CM;

  
  turnPWM -= (int)(YAW_DAMP_GAIN * yaw_rate_dps);

  
  int base = PWM_FWD;
  if (cC < SLOWDOWN_DIST_CM){
    int scale = 30 + (70 * cC) / SLOWDOWN_DIST_CM; 
    base = clampi((PWM_FWD * scale)/100, MIN_CRUISE_PWM, PWM_FWD);
  }

  
  if (irL || irR){
    base = min(base, IR_SLOW_BASE);
  }

  setMotorsForwardDifferential(base, turnPWM);
}


void setup(){
  Serial.begin(115200);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  Wire.begin();
  imuInit();

  head.attach(PIN_SERVO);
  servoCenter();
  delay(300);

  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
  autoCalibrateIR();

  stopMotors();
  state = RUN;
  state_t0 = millis();
  servo_t0 = millis();
  servoPhase = 0;

  Serial.println(F("READY, obstacle avoidance + IMU + IR enhanced"));
}

void loop(){
  pollSerialCmd();
  imuUpdate();
  updateServoScan();

  int irLraw = readIRRaw(PIN_IR_LEFT);
  int irRraw = readIRRaw(PIN_IR_RIGHT);

  
  bool irL = irDebounced(irLraw, irThreshL, irL_last_ms);
  bool irR = irDebounced(irRraw, irThreshR, irR_last_ms);

  int dC = distC<0?999:distC;

  
  bool ir_emerg = IR_TRIGGERS_EMERG ? (irL || irR) : (irL && irR);
  bool nearObstacle = (dC <= MIN_DIST_CM) || ir_emerg;

  unsigned long now = millis();

  switch (state){
    case RUN:
      if (nearObstacle){
        stopMotors();
        state = EMERG_BACK;
        state_t0 = now;
      } else if (irL && !irR){
        
        servoCenter();
        setMotorsRaw(+PWM_TURN, -PWM_TURN);
      } else if (irR && !irL){
        
        servoCenter();
        setMotorsRaw(-PWM_TURN, +PWM_TURN);
      } else {
        decideSteeringAndDrive(irLraw, irRraw);
      }
      break;

    case EMERG_BACK:
      servoCenter();
      setMotorsBackward(PWM_BACK);
      if (now - state_t0 >= BACK_MS){
        stopMotors();

        
        int l = distL<0?0:distL, r = distR<0?0:distR;
        if (irL && !irR)      bestTurn = +1; 
        else if (irR && !irL) bestTurn = -1; 
        else                  bestTurn = (l > r) ? +1 : -1;

        state = EMERG_TURN;
        state_t0 = now;
      }
      break;

    case EMERG_TURN:
      servoCenter();
      if (bestTurn > 0) setMotorsRaw(-PWM_TURN, +PWM_TURN);
      else              setMotorsRaw(+PWM_TURN, -PWM_TURN);
      if (now - state_t0 >= TURN_MS){
        stopMotors();
        state = RUN;
        state_t0 = now;
      }
      break;
  }

  
  if (now - last_telemetry >= TELEMETRY_MS){
    last_telemetry = now;
    float yaw_deg = yaw_rad * (180.0f/3.14159265f);
    Serial.print(F("STAT,"));
    Serial.print(now);               Serial.print(F(","));
    Serial.print(distC);             Serial.print(F(","));
    Serial.print(distL);             Serial.print(F(","));
    Serial.print(distR);             Serial.print(F(","));
    Serial.print(irLraw);            Serial.print(F(","));
    Serial.print(irRraw);            Serial.print(F(","));
    Serial.print(currentServoDeg);   Serial.print(F(","));
    Serial.print(last_pwmL);         Serial.print(F(","));
    Serial.print(last_pwmR);         Serial.print(F(","));
    Serial.print((int)state);        Serial.print(F(","));
    Serial.print(yaw_deg, 2);        Serial.print(F(","));
    Serial.print(yaw_rate_dps, 2);
    Serial.println();
  }
}
