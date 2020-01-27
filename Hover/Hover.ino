#include <Servo.h>
#include "btModule.h"
#include "MPU.h"
#include "PID.h"

// create servo object to control the motors
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Some variables used with the IMU
extern float gForceX, gForceY, gForceZ;
extern float rotX, rotY, rotZ;
extern float calibX, calibY, calibZ;
//extern float gyrAngX, gyrAngY, gyrAngZ;
extern float accPitch, accRoll;
extern float pitch, roll, yaw;
extern float mpuTemp;
extern long loopTimer;
extern bool firstLoop;

// PID
extern int anti_windup;
extern float pitch_prev;
extern float roll_prev;
extern float pitch_ref;
extern float roll_ref;

// Motor thrusts
extern const float motor_saturation;
float m1Th, m2Th, m3Th, m4Th;
float maxThrust = 120.0;
float m2Prev; // DEBUG

void logg(float a, float b) {
  static int count = 0;
  //Serial.println(m2);
  count++;
  if (count == 100) {
    count = 0;
    Serial.print(a);
    Serial.print(" ");
    Serial.println(b);
    //Serial.println(roll);
  }
}


void setup() {
  loopTimer = 0;
  Wire.begin(); // Start wire-library
  Serial.begin(9600);
  setupBT();
  Serial.println("BT setup complete");

  // Setup STATUS_PIN (LED)
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);
  
  // Attach all motors: Digital pins 7-10, and pwm signal 1000-2000
  m1Th = m2Th = m3Th = m4Th = 0.0;
  motor1.attach(7,1000,2000);
  motor1.write(m1Th);
  motor2.attach(8,1000,2000);
  motor2.write(m2Th);
  motor3.attach(9,1000,2000);
  motor3.write(m3Th);
  motor4.attach(10,1000,2000);
  motor4.write(m4Th);

  // Setup and calibrate gyro
  setupMPU();
  Serial.println("Wait...");
  //delay(2000);
  calibrateGyro();
  setupPID();
  Serial.println("Setup Complete!");
  // Do not start the main code before we start from the app
  while (getThrust() == 0) {
    if (readMPU()) {
      while (micros() - loopTimer < 4000);
      loopTimer = micros();
    }
  }
  m2Prev = 0;
}

void loop() {
  // Read data from the MPU
  readMPU();
  /*
  if (readMPU()) {
    while (micros() - loopTimer < 4000);
    loopTimer = micros();
  }
  */

  int thrust = getThrust(); // Receive 1/0 (enable/disable motors)

  //Make this check global
  anti_windup = 1;
  // If any motor saturated then enable the antiwindup
  if (m1Th >= motor_saturation || m2Th >= motor_saturation || m3Th >= motor_saturation || m4Th >= motor_saturation) anti_windup = 0;
  // Initialize variables for motorthrust from Pitch, Roll and Yaw PID controllers
  float m1P = 0.0,    m2P = 0.0,    m3P = 0.0,    m4P = 0.0;    // Thrust from Pitch-PID
  float m1R = 0.0,    m2R = 0.0,    m3R = 0.0,    m4R = 0.0;    // Thrust from Roll-PID
  float m1Y = 0.0,    m2Y = 0.0,    m3Y = 0.0,    m4Y = 0.0;    // Thrust from Yaw-PID
  float m1Base = thrust, m2Base = thrust, m3Base = thrust, m4Base = thrust; // Base thrust (from phone)

  // Calculate Pitch, Roll and Yaw thrust from the PID's
  PID_Pitch(m1P, m2P, m3P, m4P);
  PID_Roll(m1R, m2R, m3R, m4R);
  PID_Yaw(m1Y, m2Y, m3Y, m4Y);
  // Calculate tot thrust to motors
  m1Th = constrain(m1Base + m1P + m1R + m1Y, 0.0, maxThrust); //
  m2Th = constrain(m2Base + m2P + m2R + m2Y, 0.0, maxThrust); //
  m3Th = constrain(m3Base + m3P + m3R + m3Y, 0.0, maxThrust); //
  m4Th = constrain(m4Base + m4P + m4R + m4Y, 0.0, maxThrust); //
  //btPrint(pitch);

  //float sumPIDaccX = constrain(PID_accX(), -10, 10);
  //float sumPIDaccY = constrain(PID_accY(), -10, 10);
  
  //pitch_ref = sumPIDaccX;
  //roll_ref = sumPIDaccY;
  
  //logg(m2Th, m4Th);
  //logg(m4Th);
  m2Prev = m2Th;
  /*
  Serial.println(pitch);
  Serial.print(" ");
  Serial.print(m1Th);
  Serial.print(" ");
  Serial.print(m2Th);
  Serial.print(" ");
  Serial.print(m3Th);
  Serial.print(" ");
  Serial.println(m4Th);
  */
  
  // If mototrs enabled
  if (thrust) {
    // Write thrust to motors
    motor1.write(m1Th);
    motor2.write(m2Th);
    motor3.write(m3Th);
    motor4.write(m4Th);
  }
  else {
    // Set motorthrust to zero
    motor1.write(0);
    motor2.write(0);
    motor3.write(0);
    motor4.write(0);
  }

  // Wait until 4 milliseconds to ensure 250Hz
  while (micros() - loopTimer < 4000) {
    //Serial.print("Waiting...\t");  // Debugging
    //Serial.println(micros() - loopTimer);
  }
  loopTimer = micros();
}
