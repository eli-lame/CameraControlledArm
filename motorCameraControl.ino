// ----- Pin Assignments -----
const int ENA = 6;   // PWM pin that controls motor speed
const int IN1 = 4;   // Motor direction pin 1
const int IN2 = 5;   // Motor direction pin 2
const int ENC_A = 2; // Encoder channel A (supports hardware interrupt)
const int ENC_B = 3; // Encoder channel B (regular digital read)

const long LOWER_LIMIT = -1000;  // adjust these values
const long UPPER_LIMIT = 1000;   // adjust these values

volatile long encoderCount = 0;  // Tracks encoder ticks (updated inside interrupt)

// ---- Timed-print variables ----
unsigned long lastPrint = 0;             // store last print time
const unsigned long printInterval = 200; // print every 200 ms (5 times per second)


// ----- Encoder Interrupt Routine -----
// Triggered whenever ENC_A changes state
void encoderA_ISR() {
  bool A = digitalRead(ENC_A);  // current state of encoder channel A
  bool B = digitalRead(ENC_B);  // state of channel B used to determine direction

  // If A and B match → CW rotation, else CCW rotation
  if (A == B) {
    encoderCount++;   // clockwise
  } else {
    encoderCount--;   // counterclockwise
  }
}


void setup() {
  Serial.begin(9600); // Start serial communication with PC/Python

  // Motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Encoder input pins
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  // Attach interrupt on channel A (fires on any change: HIGH→LOW or LOW→HIGH)
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderA_ISR, CHANGE);

  Serial.println("Hand-Controlled Motor + Encoder Ready");
}


void loop() {

  // ---- Read Serial Commands from Python ----
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'L') {
      // Move LEFT only if above lower limit
      if (encoderCount > LOWER_LIMIT) {
        moveMotor(-150);
      } else {
        moveMotor(0);  // hit limit → stop
      }
    }
    else if (cmd == 'R') {
      // Move RIGHT only if below upper limit
      if (encoderCount < UPPER_LIMIT) {
        moveMotor(150);
      } else {
        moveMotor(0);  // hit limit → stop
      }
    }
    else if (cmd == 'S') {
      moveMotor(0);
    }
}
 


  // ---- Print encoder values less often to prevent freezing ----
  unsigned long now = millis();  // current time in ms since Arduino started
  if (now - lastPrint >= printInterval) {
    lastPrint = now;  // update timestamp

    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);  // print current encoder value
  }
}


// ----- Motor Movement Function -----
// Positive → forward
// Negative → reverse
// Zero → stop
void moveMotor(int speedVal) {

  if (speedVal > 0) {
    // Forward direction
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speedVal);  // speed 0–255
  }
  else if (speedVal < 0) {
    // Reverse direction
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speedVal); // use absolute value for PWM
  }
  else {
    // Stop motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}
