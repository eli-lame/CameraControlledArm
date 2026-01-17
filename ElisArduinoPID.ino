#include <util/atomic.h>

// ================== Pin Assignments ==================
#define ENCA 2
#define ENCB 3
#define PWM  6
#define IN1  4
#define IN2  5

// ================== Encoder ==================
volatile long encoderTicks = 0;
volatile unsigned long prevT_ISR = 0;

// ================== Control ==================
long motorPosTarget = 0;     // <-- motorpos from PC
long encoderPrev = 0;

unsigned long prevT = 0;

// ================== Velocity Filter ==================
float vFilt = 0;
float vPrev = 0;

// ================== PID Gains ==================
float Kp_pos = 0.05;   // position → velocity
float kp_vel = 5.0;    // velocity PID
float ki_vel = 10.0;

float eintegral = 0;

// ================== Limits ==================
float maxRPM = 150;
const long LOWER_LIMIT = -1000;
const long UPPER_LIMIT = 1000;

// ================== Debug ==================
unsigned long lastPrint = 0;
const unsigned long printInterval = 200;

// ================== Encoder ISR ==================
void readEncoder() {
  int b = digitalRead(ENCB);
  encoderTicks += (b > 0) ? 1 : -1;
}

// ================== Motor ==================
void setMotor(int dir, int pwmVal) {
  pwmVal = constrain(pwmVal, 0, 255);
  analogWrite(PWM, pwmVal);

  if (dir > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  prevT = micros();
  Serial.println("Position Control Ready");
}

// ================== Loop ==================
void loop() {

  // -------- Receive target position --------
  if (Serial.available()) {
    motorPosTarget = Serial.parseInt();
  }

  motorPosTarget = constrain(motorPosTarget, LOWER_LIMIT, UPPER_LIMIT);

  // -------- Read encoder safely --------
  long pos;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = encoderTicks;
  }

  // -------- Timing --------
  unsigned long currT = micros();
  float deltaT = (currT - prevT) * 1e-6;
  if (deltaT <= 0) return;
  prevT = currT;

  // -------- Velocity estimate --------
  float velocity = (pos - encoderPrev) / deltaT; // counts/sec
  encoderPrev = pos;

  // -------- Convert to RPM --------
  float vRPM = velocity / 600.0 * 60.0; // adjust 600 if needed

  // -------- Low-pass filter --------
  vFilt = 0.854 * vFilt + 0.0728 * vRPM + 0.0728 * vPrev;
  vPrev = vRPM;

  // -------- Position → Velocity --------
  float posError = motorPosTarget - pos;
  float vTarget = Kp_pos * posError;
  vTarget = constrain(vTarget, -maxRPM, maxRPM);

  // -------- Velocity PID --------
  float e = vTarget - vFilt;
  eintegral += e * deltaT;

  float u = kp_vel * e + ki_vel * eintegral;

  // -------- Motor output --------
  int dir = (u > 0) ? 1 : -1;
  int pwm = abs(u);
  pwm = constrain(pwm, 0, 255);

  setMotor(dir, pwm);

  // -------- Debug --------
  unsigned long now = millis();
  if (now - lastPrint >= printInterval) {
    lastPrint = now;
    Serial.print("Target: ");
    Serial.print(motorPosTarget);
    Serial.print(" | Pos: ");
    Serial.print(pos);
    Serial.print(" | RPM: ");
    Serial.println(vFilt);
  }
}
