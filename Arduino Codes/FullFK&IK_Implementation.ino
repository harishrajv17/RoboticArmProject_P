#include <PID_v1.h>
#include <math.h>

// Encoder Pins
#define REV_ENCODER_A 2
#define REV_ENCODER_B 3
#define PRISM_ENCODER_A 18
#define PRISM_ENCODER_B 19

// Motor Control Pins (L298N) 
#define REV_MOTOR_IN1 8
#define REV_MOTOR_IN2 9
#define REV_MOTOR_PWM 10

#define PRISM_MOTOR_IN3 11
#define PRISM_MOTOR_IN4 12
#define PRISM_MOTOR_PWM 13

// Encoder Variables 
volatile long rev_encoderCount = 0;
volatile long prism_encoderCount = 0;
double theta_actual = 0.0;
double delta_d_actual = 0.0;

// Kinematic Parameters
const double L1 = 0.25;
const double L2 = 0.125;
const double L3 = 0.1;
const double h = sqrt(L3 * L3 + L2 * L2);
const double gamma = ((180.0 - atan(L2 / L3)) / 2.0) * (3.1416 / 180.0); // radians

// Encoder & Gear Constants 
const double encoderCountsPerRev = 48;
const double gearReduction = 46.85;
const double encoderCountsPerJointRev = encoderCountsPerRev * gearReduction;

const double pinionDiameter = 32.9; // mm
const double pinionCircumference = 3.1416 * pinionDiameter;
const double mmPerCount = pinionCircumference / encoderCountsPerJointRev;

// Targets & Setpoints 
double x_target = 0.3;
double z_target = 0.4;
double theta_target = 0.0;
double delta_d_target = 0.0;

// PID Variables 
double rev_motor_speed, prism_motor_speed;
double Kp = 2.0, Ki = 0.5, Kd = 0.1;

PID pid_theta(&theta_actual, &rev_motor_speed, &theta_target, Kp, Ki, Kd, DIRECT);
PID pid_linear(&delta_d_actual, &prism_motor_speed, &delta_d_target, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(9600);

    // Motor Pins
    pinMode(REV_MOTOR_IN1, OUTPUT);
    pinMode(REV_MOTOR_IN2, OUTPUT);
    pinMode(REV_MOTOR_PWM, OUTPUT);
    pinMode(PRISM_MOTOR_IN3, OUTPUT);
    pinMode(PRISM_MOTOR_IN4, OUTPUT);
    pinMode(PRISM_MOTOR_PWM, OUTPUT);

    // Encoder Pins
    pinMode(REV_ENCODER_A, INPUT);
    pinMode(REV_ENCODER_B, INPUT);
    pinMode(PRISM_ENCODER_A, INPUT);
    pinMode(PRISM_ENCODER_B, INPUT);

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(REV_ENCODER_A), updateRevEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PRISM_ENCODER_A), updatePrismEncoder, CHANGE);

    // Enable PID Controllers
    pid_theta.SetMode(AUTOMATIC);
    pid_linear.SetMode(AUTOMATIC);
    pid_theta.SetSampleTime(50);
    pid_linear.SetSampleTime(50);
}

// Encoder ISRs 
void updateRevEncoder() {
    int stateB = digitalRead(REV_ENCODER_B);
    rev_encoderCount += (stateB == HIGH) ? 1 : -1;
    theta_actual = (rev_encoderCount * 2.0 * 3.1416) / encoderCountsPerJointRev;
}

void updatePrismEncoder() {
    int stateB = digitalRead(PRISM_ENCODER_B);
    prism_encoderCount += (stateB == HIGH) ? 1 : -1;
    delta_d_actual = prism_encoderCount * mmPerCount;
}

// Inverse Kinematics 
void computeInverseKinematics() {
    double cosTheta = (x_target - h * cos(gamma)) / L1;
    cosTheta = constrain(cosTheta, -1.0, 1.0); // safety clamp
    theta_target = acos(cosTheta);
    delta_d_target = (z_target - (L1 * sin(theta_target) + h * sin(gamma))) * 1000.0;
}

// Forward Kinematics 
void computeForwardKinematics(double &x_fk, double &z_fk) {
    x_fk = L1 * cos(theta_actual) + h * cos(gamma);
    z_fk = L1 * sin(theta_actual) + h * sin(gamma) + delta_d_actual;
}

// Motor Control 
void controlMotor(double speed, int in1, int in2, int pwmPin) {
    int pwmValue = constrain(abs(speed), 0, 255);
    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    analogWrite(pwmPin, pwmValue);
}

// Main Loop
void loop() {
    computeInverseKinematics();

    pid_theta.Compute();
    pid_linear.Compute();

    controlMotor(rev_motor_speed, REV_MOTOR_IN1, REV_MOTOR_IN2, REV_MOTOR_PWM);
    controlMotor(prism_motor_speed, PRISM_MOTOR_IN3, PRISM_MOTOR_IN4, PRISM_MOTOR_PWM);

    double x_fk, z_fk;
    computeForwardKinematics(x_fk, z_fk);

    Serial.print("Target: X="); Serial.print(x_target, 3);
    Serial.print(" Z="); Serial.print(z_target, 3);
    Serial.print(" | Actual: X="); Serial.print(x_fk, 3);
    Serial.print(" Z="); Serial.println(z_fk, 3);

    delay(50);
}