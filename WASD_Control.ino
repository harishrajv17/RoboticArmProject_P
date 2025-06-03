#include <PID_v1.h>

#define REV_ENCODER_A 2
#define REV_ENCODER_B 3
#define PRISM_ENCODER_A 18
#define PRISM_ENCODER_B 19

#define REV_MOTOR_IN1 8
#define REV_MOTOR_IN2 9
#define REV_MOTOR_PWM 10

#define PRISM_MOTOR_IN3 11
#define PRISM_MOTOR_IN4 12
#define PRISM_MOTOR_PWM 13

volatile long rev_encoder_count = 0; //volatile prevents caching of value in register
volatile long prism_encoder_count = 0;

// Kinematics & Conversion
const double encoder_CPR = 48.0;
const double gear_ratio = 46.85;
const double total_counts_per_rev = encoder_CPR * gear_ratio; // ≈ 2248.86
const double rad_per_count = 2 * 3.1416 / total_counts_per_rev;

const double pinion_diameter = 32.9; // mm
const double pinion_circumference = 3.1416 * pinion_diameter;
const double mm_per_count = pinion_circumference / total_counts_per_rev;

// Control Variables
double theta_actual = 0.0, theta_target = 0.0;
double delta_d_actual = 0.0, delta_d_target = 0.0;
double rev_motor_output = 0.0, prism_motor_output = 0.0;

// PID Setup
double Kp_theta = 2.0, Ki_theta = 0.5, Kd_theta = 0.1;
double Kp_linear = 1.5, Ki_linear = 0.4, Kd_linear = 0.05;

PID pid_theta(&theta_actual, &rev_motor_output, &theta_target, Kp_theta, Ki_theta, Kd_theta, DIRECT);
PID pid_linear(&delta_d_actual, &prism_motor_output, &delta_d_target, Kp_linear, Ki_linear, Kd_linear, DIRECT);

// Interrupts
void updateRevEncoder() {
  if (digitalRead(REV_ENCODER_B) == digitalRead(REV_ENCODER_A)) {
    rev_encoder_count++;
  } else {
    rev_encoder_count--;
  }
  theta_actual = rev_encoder_count * rad_per_count;
}

void updatePrismEncoder() {
  if (digitalRead(PRISM_ENCODER_B) == digitalRead(PRISM_ENCODER_A)) {
    prism_encoder_count++;
  } else {
    prism_encoder_count--;
  }
  delta_d_actual = prism_encoder_count * mm_per_count;
}


void setup() {
  Serial.begin(9600);

  // Motor control pins
  pinMode(REV_MOTOR_IN1, OUTPUT);
  pinMode(REV_MOTOR_IN2, OUTPUT);
  pinMode(REV_MOTOR_PWM, OUTPUT);
  pinMode(PRISM_MOTOR_IN3, OUTPUT);
  pinMode(PRISM_MOTOR_IN4, OUTPUT);
  pinMode(PRISM_MOTOR_PWM, OUTPUT);

  // Encoder pins
  pinMode(REV_ENCODER_A, INPUT);
  pinMode(REV_ENCODER_B, INPUT);
  pinMode(PRISM_ENCODER_A, INPUT);
  pinMode(PRISM_ENCODER_B, INPUT);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(REV_ENCODER_A), updateRevEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PRISM_ENCODER_A), updatePrismEncoder, CHANGE);
  
  // Initialize PIDs
  pid_theta.SetMode(AUTOMATIC);
  pid_linear.SetMode(AUTOMATIC); 
}

void loop() {
  // Serial Command Handling
  if (Serial.available()) {
    char key = Serial.read();
    
    if (key == 'a') theta_target -= 0.05; // radians
    if (key == 'd') theta_target += 0.05;
    if (key == 'w') delta_d_target += 1.0; // mm
    if (key == 's') delta_d_target -= 1.0;
    
    // Limit ranges
    theta_target = constrain(theta_target, -3.14, 3.14); // Revolute joint limit
    delta_d_target = constrain(delta_d_target, 0, 100.0); // Prismatic joint range (e.g. 0 to 100 mm)
  }

   // Compute PID outputs
  pid_theta.Compute();
  pid_linear.Compute();

  // Apply to motors
  controlMotor(-rev_motor_output, REV_MOTOR_IN1, REV_MOTOR_IN2, REV_MOTOR_PWM);
  controlMotor(prism_motor_output, PRISM_MOTOR_IN3, PRISM_MOTOR_IN4, PRISM_MOTOR_PWM);

  // Debug info
  Serial.print("θ Target: "); Serial.print(theta_target, 2);
  Serial.print(" | θ Actual: "); Serial.print(theta_actual, 2);
  Serial.print(" | Δd Target: "); Serial.print(delta_d_target, 1);
  Serial.print(" | Δd Actual: "); Serial.println(delta_d_actual, 1);

  delay(50);
}

// Motor control function
void controlMotor(double speed, int in1, int in2, int pwmPin) {
  int pwm = constrain(abs(speed), 0, 255);
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(pwmPin, pwm);
}
