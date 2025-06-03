#include <PID_v1.h>

#define REV_MOTOR_PWM 10      // PWM pin for revolute motor
#define REV_MOTOR_DIR 9       // Direction pin for revolute motor
#define REV_ENCODER_A 2       // Encoder channel A (interrupt)
#define REV_ENCODER_B 3       // Encoder channel B

#define PRISM_MOTOR_PWM 11    // PWM pin for prismatic motor
#define PRISM_ENCODER_A 18    // Encoder channel A
#define PRISM_ENCODER_B 19    // Encoder channel B
#define PRISM_MOTOR_DIR 8     // Direction pin for prismatic motor


// PID Variables - Revolute Joint 
double Setpoint_theta = 0, Input_theta, Output_theta;
double Kp_theta = 2.0, Ki_theta = 0.5, Kd_theta = 0.1;
PID pid_theta(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta, Ki_theta, Kd_theta, DIRECT);
volatile long rev_encoder_count = 0; //volatile prevents caching of value in register

// Prismatic Joint
double Setpoint_linear = 0, Input_linear, Output_linear;
double Kp_linear = 1.5, Ki_linear = 0.4, Kd_linear = 0.08;
PID pid_linear(&Input_linear, &Output_linear, &Setpoint_linear, Kp_linear, Ki_linear, Kd_linear, DIRECT);
volatile long prism_encoder_count = 0;


// Timer variables
unsigned long lastPIDTime = 0; // Unsigned as time cannot be negative
const unsigned long PID_INTERVAL = 50; // ms (20 Hz)

// --- Interrupt Service Routines ---
void readRevEncoder() {
  if (digitalRead(REV_ENCODER_B) == digitalRead(REV_ENCODER_A)) {
    rev_encoder_count++;
  } else {
    rev_encoder_count--;
  }
}

void readPrismEncoder() {
  if (digitalRead(PRISM_ENCODER_B) == digitalRead(PRISM_ENCODER_A)) {
    prism_encoder_count++;
  } else {
    prism_encoder_count--;
  }
}

void setup() {
  Serial.begin(9600);

  // Motor control pins
  pinMode(REV_MOTOR_PWM, OUTPUT);
  pinMode(REV_MOTOR_DIR, OUTPUT);
  pinMode(PRISM_MOTOR_PWM, OUTPUT);
  pinMode(PRISM_MOTOR_DIR, OUTPUT);

  // Encoder pins
  pinMode(REV_ENCODER_A, INPUT);
  pinMode(REV_ENCODER_B, INPUT);
  pinMode(PRISM_ENCODER_A, INPUT);
  pinMode(PRISM_ENCODER_B, INPUT);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(REV_ENCODER_A), readRevEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PRISM_ENCODER_A), readPrismEncoder, CHANGE);
  // Initialize PIDs
  pid_theta.SetMode(AUTOMATIC);
  pid_linear.SetMode(AUTOMATIC); 
}

void loop() {
  // Serial Command Handling
  if (Serial.available()) {
    char key = Serial.read();
    
    if (key == 'a') Setpoint_theta -= 10;
    if (key == 'd') Setpoint_theta += 10;
    if (key == 'w') Setpoint_linear += 10;
    if (key == 's') Setpoint_linear -= 10;
    
    Setpoint_theta = constrain(Setpoint_theta, -1000, 1000);
    Setpoint_linear = constrain(Setpoint_linear, 0, 3000);
  }

  // PID Control Loop (update)
  if (millis() - lastPIDTime >= PID_INTERVAL) {
    lastPIDTime = millis();

    Input_theta = (double)rev_encoder_count;
    pid_theta.Compute();
    controlMotor(Output_theta, REV_MOTOR_DIR, REV_MOTOR_PWM);

    Input_linear = (double)prism_encoder_count;
    pid_linear.Compute();
    controlMotor(Output_linear, PRISM_MOTOR_DIR, PRISM_MOTOR_PWM);
  

    // Debugging output
    Serial.print("θ Count: "); Serial.print(rev_encoder_count);
    Serial.print(" | θ Setpoint: "); Serial.print(Setpoint_theta);
    Serial.print(" | θ PWM: "); Serial.print(Output_theta);
    Serial.print(" || Δd Count: "); Serial.print(prism_encoder_count);
    Serial.print(" | Δd Setpoint: "); Serial.print(Setpoint_linear);
    Serial.print(" | Δd PWM: "); Serial.println(Output_linear);
  }
}

// Motor control function
void controlMotor(double speed, int dirPin, int pwmPin) {
  int pwm = constrain(abs(speed), 0, 255);
  bool direction = speed >= 0;

  digitalWrite(dirPin, direction ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}
