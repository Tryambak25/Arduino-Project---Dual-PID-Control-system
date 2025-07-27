#include "Wire.h"       
#include "MPU6050.h"    

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MAX_DISTANCE 200

double angle = 0;
double gyroRate = 0;
double dt = 0.01;

double setpoint = 0;
double error = 0;
double previousError = 0;
double integral = 0;
double kp = 4.0, ki = 0.2, kd = 1.0;

double distanceSetpoint = 20;
double distanceError = 0;
double previousDistanceError = 0;
double distanceIntegral = 0;
double kp_distance = 1.0, ki_distance = 0.0, kd_distance = 0.2;

int motorPin1A = 9;
int motorPin2A = 8;
int motorPWMA = 10;

int motorPin1B = 7;
int motorPin2B = 6;
int motorPWMB = 5;

void setup() {
    Wire.begin();
    mpu.initialize();

    pinMode(motorPin1A, OUTPUT);
    pinMode(motorPin2A, OUTPUT);
    pinMode(motorPWMA, OUTPUT);

    pinMode(motorPin1B, OUTPUT);
    pinMode(motorPin2B, OUTPUT);
    pinMode(motorPWMB, OUTPUT);

    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Serial.begin(9600);
}

void loop() {
    double error = getError();
    double distanceError = getDistanceError();
    double balanceOutput = pidControl(error);
    double distanceOutput = pidDistanceControl(distanceError);
    double combinedOutput = balanceOutput + distanceOutput;
    
    // Print angle and distance values
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" Distance: "); Serial.println(getDistance());

    // Print PID control outputs
    Serial.print("Balance Output: "); Serial.print(balanceOutput);
    Serial.print(" Distance Output: "); Serial.print(distanceOutput);
    Serial.print(" Combined Output: "); Serial.println(combinedOutput);

    controlMotors(combinedOutput);
    delay(10);
}

double getError() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double accelAngle = atan2(ay, az) * 180 / PI;
    gyroRate = gx / 131.0;
    angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accelAngle;

    // Print gyroscope and angle values for debugging
    Serial.print("Accel Angle: "); Serial.print(accelAngle);
    Serial.print(" Gyro Rate: "); Serial.print(gyroRate);
    Serial.print(" Calculated Angle: "); Serial.println(angle);

    double currentError = setpoint - angle;
    return currentError;
}

double pidControl(double currentError) {
    double proportional = currentError;
    integral += currentError * dt;
    double derivative = (currentError - previousError) / dt;
    previousError = currentError;
    double output = (kp * proportional) + (ki * integral) + (kd * derivative);
    return output;
}

double getDistance() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    double distance = (duration / 2.0) * 0.0343;
    if (distance > MAX_DISTANCE) {
        distance = MAX_DISTANCE;
    }
    return distance;
}

double getDistanceError() {
    double distance = getDistance();
    double currentDistanceError = distanceSetpoint - distance;
    return currentDistanceError;
}

double pidDistanceControl(double currentDistanceError) {
    double proportional = currentDistanceError;
    distanceIntegral += currentDistanceError * dt;
    double derivative = (currentDistanceError - previousDistanceError) / dt;
    previousDistanceError = currentDistanceError;
    double output = (kp_distance * proportional) + (ki_distance * distanceIntegral) + (kd_distance * derivative);
    return output;
}

void controlMotors(double combinedOutput) {
    int pwmValue = abs(combinedOutput);
    pwmValue = constrain(pwmValue, 150, 255); // Ensure PWM is at least 50
    if (combinedOutput > 0) {
        analogWrite(motorPWMA, pwmValue);
        analogWrite(motorPWMB, pwmValue);
        digitalWrite(motorPin1A, HIGH);
        digitalWrite(motorPin2A, LOW);
        digitalWrite(motorPin1B, HIGH);
        digitalWrite(motorPin2B, LOW);
    } else {
        analogWrite(motorPWMA, pwmValue);
        analogWrite(motorPWMB, pwmValue);
        digitalWrite(motorPin1A, LOW);
        digitalWrite(motorPin2A, HIGH);
        digitalWrite(motorPin1B, LOW);
        digitalWrite(motorPin2B, HIGH);
    }
}
