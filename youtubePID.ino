#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 6
#define IN1 4
#define IN2 5

// globals
long prevT = 0;
int posPrev = 0;

// Encoder ISR variables
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

// Velocity filtering
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

// Velocity PID
float eintegral = 0;

// ===== Position control =====
long target = 0;          // Encoder ticks target
float Kp_pos = 0.05;      // Position → velocity gain
float maxRPM = 150;      // Speed limit (tune for your arm)

void setup() {
  Serial.begin(115200);

  // Immediately print initial target
  Serial.print("Target,Position\n");
  Serial.print(target);
  Serial.print(",");
  Serial.println(pos_i);


  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {

  // Read position & velocity atomically
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Velocity method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM (adjust 600 if your encoder CPR is different)
  float v1 = velocity1 / 600.0 * 60.0;
  float v2 = velocity2 / 600.0 * 60.0;

  // Low-pass filter (25Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // ===== Position → Velocity target =====
  float posError = target - pos;
  float vt = Kp_pos * posError;

  if (vt > maxRPM) vt = maxRPM;
  if (vt < -maxRPM) vt = -maxRPM;

  // ===== Velocity PID (unchanged) =====
  float kp = 5;
  float ki = 10;

  float e = vt - v1Filt;
  eintegral += e * deltaT;
  
  float u = kp * e + ki * eintegral;

  // Motor drive
  int dir = 1;
  if (u < 0) dir = -1;

  int pwr = (int) fabs(u);
  if (pwr > 255) pwr = 255;

  setMotor(dir, pwr, PWM, IN1, IN2);

  // ===== Serial position input =====
  if (Serial.available()) {
    long newTarget = Serial.parseInt();
    if (newTarget != 0 || Serial.peek() == '\n') {
      target = newTarget;
    }
  }

  Serial.print(target);
  Serial.print(",");
  Serial.println(pos);

  delay(1);
}

// ================= Motor =================
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// ================= Encoder ISR =================
void readEncoder(){
  int b = digitalRead(ENCB);
  int increment = (b > 0) ? 1 : -1;
  pos_i += increment;

  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}
