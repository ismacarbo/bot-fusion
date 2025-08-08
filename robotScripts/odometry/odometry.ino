volatile long encoder_left = 0;
volatile long encoder_right = 0;

#define ENCODER_LEFT_PIN  2
#define ENCODER_RIGHT_PIN 3

const float WHEEL_RADIUS = 0.03f; 
const float WHEEL_BASE = 0.15f; 
const int TICKS_PER_REV = 360;        // still measuring

float x = 0, y = 0, theta = 0;
unsigned long last_time = 0;
long last_left = 0, last_right = 0;

void encoderLeftISR() { encoder_left++; }
void encoderRightISR() { encoder_right++; }

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightISR, RISING);
  last_time = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  if (dt < 0.01f) return; 

  noInterrupts();
  long left = encoder_left;
  long right = encoder_right;
  interrupts();

  long delta_left = left - last_left;
  long delta_right = right - last_right;
  last_left = left;
  last_right = right;

  float dist_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float dL = delta_left * dist_per_tick;
  float dR = delta_right * dist_per_tick;

  float ds = (dR + dL) / 2.0f;
  float dtheta = (dR - dL) / WHEEL_BASE;

  float theta_mid = theta + dtheta / 2.0f;
  x += ds * cosf(theta_mid);
  y += ds * sinf(theta_mid);
  theta += dtheta;

  if (theta > M_PI) theta -= 2.0f * M_PI;
  if (theta < -M_PI) theta += 2.0f * M_PI;

  Serial.print("x:"); Serial.print(x, 4);
  Serial.print(" y:"); Serial.print(y, 4);
  Serial.print(" theta(deg):"); Serial.print(theta * 180.0f / M_PI, 2);
  Serial.print(" ds:"); Serial.print(ds, 4);
  Serial.print(" dtheta:"); Serial.println(dtheta, 4);

  last_time = now;
}
