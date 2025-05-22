#include <PID_v1.h>

#define REV_MOTOR_PWM 10      // PWM pin for revolute motor
#define REV_MOTOR_DIR 9       // Direction pin for revolute motor
#define PRISM_MOTOR_PWM 11    // PWM pin for prismatic motor
#define REV_ENCODER_A 2       // Encoder channel A (interrupt)
#define REV_ENCODER_B 3       // Encoder channel B

// PID Variables 
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 0.5, Kd = 0.1;

// PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Encoder counter
volatile long rev_encoder_count = 0;

// Timer variables
unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 50; // ms (20 Hz)

// Encoder ISR 
void readRevEncoder() {
  if (digitalRead(REV_ENCODER_B) == digitalRead(REV_ENCODER_A)) {
    rev_encoder_count++;
  } else {
    rev_encoder_count--;
  }
}

void setup() {
  Serial.begin(9600);

  // Motor control pins
  pinMode(REV_MOTOR_PWM, OUTPUT);
  pinMode(REV_MOTOR_DIR, OUTPUT);
  pinMode(PRISM_MOTOR_PWM, OUTPUT);

  // Encoder pins
  pinMode(REV_ENCODER_A, INPUT);
  pinMode(REV_ENCODER_B, INPUT);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(REV_ENCODER_A), readRevEncoder, CHANGE);

  // Initialize PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Serial Command Handling
  if (Serial.available()) {
    char key = Serial.read();

    if (key == 'a') Setpoint -= 10;  // rotate left
    if (key == 'd') Setpoint += 10;  // rotate right
    Setpoint = constrain(Setpoint, -1000, 1000); // optional bounds

    if (key == 'w') analogWrite(PRISM_MOTOR_PWM, 255); // extend
    if (key == 's') analogWrite(PRISM_MOTOR_PWM, 0);   // retract
  }

  // PID Control Loop (non-blocking)
  unsigned long currentTime = millis();
  if (currentTime - lastPIDTime >= PID_INTERVAL) {
    lastPIDTime = currentTime;

    Input = (double)rev_encoder_count;
    myPID.Compute();

    // Determine motor direction
    bool dir = Output >= 0;
    double pwm = constrain(abs(Output), 0, 255);

    digitalWrite(REV_MOTOR_DIR, dir);  // HIGH = forward, LOW = reverse
    analogWrite(REV_MOTOR_PWM, pwm);

    // Debugging output
    Serial.print("Encoder: "); Serial.print(rev_encoder_count);
    Serial.print(" | Setpoint: "); Serial.print(Setpoint);
    Serial.print(" | Output: "); Serial.println(Output);
  }
}