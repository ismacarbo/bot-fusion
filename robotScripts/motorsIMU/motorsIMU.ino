#define PIN_Motor_PWMA   5
#define PIN_Motor_PWMB   6
#define PIN_Motor_AIN_1  7
#define PIN_Motor_BIN_1  8
#define PIN_Motor_STBY   3
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);

  digitalWrite(PIN_Motor_STBY, HIGH);
  randomSeed(analogRead(A0));
}

void loop(){
  int dir = random(0, 4)
  int speed = random(150, 255);

  switch(dir){
    case 0: move(speed, speed);    Serial.println("up"); break;
    case 1: move(-speed, -speed);  Serial.println("back"); break;
    case 2: move(-speed, speed);   Serial.println("left"); break;
    case 3: move(speed, -speed);   Serial.println("right"); break;
  }

  accelDatas();

  delay(2000);
  stopMotors();
  delay(1000);
}

void accelDatas(){
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("Accel: ");
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" | Gyro: ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.println(gz);

  delay(500);
}

void move(int pwmA, int pwmB){
  digitalWrite(PIN_Motor_AIN_1, pwmA >= 0);
  digitalWrite(PIN_Motor_BIN_1, pwmB >= 0);
  analogWrite(PIN_Motor_PWMA, abs(pwmA));
  analogWrite(PIN_Motor_PWMB, abs(pwmB));
}

void stopMotors(){
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
  Serial.println("Stop");
}
