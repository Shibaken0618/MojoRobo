#include <PID_v2.h>
#include <Wire.h>
#include <Arduino.h>

const int MPU_addr = 0x68;  // I2C address of the MPU6050

// Global variables for raw sensor data.
double AccelX, AccelY, AccelZ, Tmp, GyroX, GyroY, GyroZ;

// Timer for loop interval calculation.
uint32_t timer;

// Angle estimation variables.
double Angle_X;  
#define degconvert 57.29577951  // Conversion from radians to degrees

// Motor pin definitions:
// Motor 1 (Left Motor)
#define m1_left   7
#define m1_right  5
#define m1_en     10
// Motor 2 (Right Motor) -- this motor will be inverted in code.
#define m2_left   8
#define m2_right  6
#define m2_en     9

// PID parameters – these are starting values. Fine-tuning is likely necessary.
double Kp = 50;      // Reduced proportional gain for smoother response
double Ki = 0.5;     // Small integral term to correct steady-state error
double Kd = 20;      // Derivative gain to damp oscillations

double targetAngle = 0;   // Level (desired angle)
double input, output;     // PID input and output
volatile float currentAngle;  // Current angle after filtering
volatile float errorVal;      // Error (difference from target)

// PID controller instance.
PID pid(&input, &output, &targetAngle, Kp, Ki, Kd, DIRECT);

// Deadband threshold for small errors (in degrees)
const float DEADZONE = 1.0;

// For output smoothing (low-pass filter on PID output)
float smoothOutput = 0;
const float SMOOTHING_ALPHA = 0.1; // Lower alpha means more smoothing

// Calibration offset determined at startup.
float calibrationOffset = 0;

// Number of samples to average during calibration.
const int CALIBRATION_SAMPLES = 200;

void calibrateSensor() {
  // Wait a moment for the sensor to settle
  delay(500);
  float angleSum = 0;
  
  // Take several samples to compute the average starting angle.
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    // Request sensor data
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // Starting register for ACCEL data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);  // Only need the accelerometer data

    // Read accelerometer values for X, Y, Z.
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();

    // Calculate roll angle from accelerometer.
    float roll = atan2(ay, az) * degconvert;
    angleSum += roll;
    delay(5); // small delay between samples
  }
  
  calibrationOffset = angleSum / CALIBRATION_SAMPLES;
}

void setup() {
  // Setup motor pins.
  pinMode(m1_left, OUTPUT);
  pinMode(m1_right, OUTPUT);
  pinMode(m1_en, OUTPUT);
  pinMode(m2_left, OUTPUT);
  pinMode(m2_right, OUTPUT);
  pinMode(m2_en, OUTPUT);
  
  // Initialize I2C and MPU6050.
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2;
  #endif
  
  // Wake up the MPU6050.
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register.
  Wire.write(0);     // Set to zero (wakes up the MPU6050).
  Wire.endTransmission(true);
  
  Serial.begin(57600);
  delay(100);

  // Calibrate sensor to determine zero angle.
  calibrateSensor();

  // Set the initial angle using the calibration offset.
  Angle_X = calibrationOffset;
  input = Angle_X;  // Set PID input to starting angle

  timer = micros();
  
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(5);  // 5ms sample time (adjust if needed)
  pid.SetOutputLimits(-255, 255);
}

void loop() {
  // Read sensor data.
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  
  // Read accelerometer values.
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  // Skip temperature.
  Wire.read(); Wire.read();
  // Read gyro values.
  int16_t gx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Skip remaining gyro values for now

  // Compute time delta.
  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();

  // Calculate roll angle from accelerometer.
  float roll = atan2(ay, az) * degconvert;
  
  // Read gyro rate (for roll axis) and convert.
  float gyroRate = gx / 131.0;  // The conversion factor may vary.
  
  // Complementary filter: blend gyro integration with accelerometer.
  // Using a higher weight for gyro over short dt.
  Angle_X = 0.95 * (Angle_X + gyroRate * dt) + 0.05 * roll;
  
  // Subtract the calibration offset so that level = 0.
  currentAngle = Angle_X - calibrationOffset;
  input = currentAngle;  // Update PID input.

  // Safety: if tilt exceeds safe bounds, stop motors.
  if (currentAngle > 40.0 || currentAngle < -40.0) {
    stop();
    return;
  }
  
  // Deadband: if error is very small, do nothing.
  if (abs(currentAngle - targetAngle) < DEADZONE) {
    stop();
    return;
  }
  
  // Compute the PID output.
  pid.Compute();
  
  // Apply smoothing to the PID output.
  smoothOutput = (1 - SMOOTHING_ALPHA) * smoothOutput + SMOOTHING_ALPHA * output;
  
  // Debug printout.
  Serial.print("Angle: ");
  Serial.print(currentAngle);
  Serial.print(" | PID out: ");
  Serial.println(smoothOutput);

  // Command motors:
  // Assume left motor runs normally and right motor is inverted.
  if (smoothOutput > 0) {
    // Robot tilts forward => drive forward.
    // Left motor forward.
    digitalWrite(m1_left, HIGH);
    digitalWrite(m1_right, LOW);
    analogWrite(m1_en, abs(smoothOutput));
    
    // Right motor (inverted) forward.
    digitalWrite(m2_left, LOW);
    digitalWrite(m2_right, HIGH);
    analogWrite(m2_en, abs(smoothOutput));
  } else {
    // Robot tilts backward => drive backward.
    // Left motor reverse.
    digitalWrite(m1_left, LOW);
    digitalWrite(m1_right, HIGH);
    analogWrite(m1_en, abs(smoothOutput));
    
    // Right motor (inverted) reverse.
    digitalWrite(m2_left, HIGH);
    digitalWrite(m2_right, LOW);
    analogWrite(m2_en, abs(smoothOutput));
  }
}

void stop() {
  // Stop all motor outputs.
  digitalWrite(m1_left, LOW);
  digitalWrite(m1_right, LOW);
  digitalWrite(m2_left, LOW);
  digitalWrite(m2_right, LOW);
  analogWrite(m1_en, 0);
  analogWrite(m2_en, 0);
}
