#include <Servo.h>
#include "btModule.h"
#include "MPU.h"
#include "PID.h"
#include "Quadcopter.h"

/* create servo object to control the motors
 *  Servos are controlled by a pwm signal, just as the ESC, so we
 *  reuse the library so we can control the ESCs by sending them
 *  a pwm signal of 1000-2000 us.
 */
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Timer for constant loop-time
extern long loopTimer;

// Declear a global quadobject containing a lot of parameters
Quadcopter* quad;

void setup() {
  quad = new Quadcopter();
  loopTimer = 0;
  Wire.begin(); // Start wire-library
  Serial.begin(9600);
  setupBT();
  Serial.println("BT setup complete");

  // Setup STATUS_PIN (LED)
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);
  
  // Attach all motors: Digital pins 7-10, and pwm signal 1000-2000 us
  motor1.attach(7,1000,2000);
  motor1.write(0);
  motor2.attach(8,1000,2000);
  motor2.write(0);
  motor3.attach(9,1000,2000);
  motor3.write(0);
  motor4.attach(10,1000,2000);
  motor4.write(0);

  // Setup and calibrate gyro
  setupMPU();
  Serial.println("Wait...");
  //delay(2000);
  calibrateGyro();
  Serial.println("Setup Complete!");
  // Do not start the main code before we start from the app
  while (getThrust() == 0) {
    if (readMPU()) {
      while (micros() - loopTimer < 4000);
      loopTimer = micros();
    }
  }
}

void loop() {
  // Read data from the MPU
  readMPU();
  
  // Declearing thrust for the motors
  float m1Th = 0.0;
  float m2Th = 0.0;
  float m3Th = 0.0;
  float m4Th = 0.0;

  // Passed by reference as to update all the values for the motors
  quad->getMotorThrust(m1Th, m2Th, m3Th, m4Th);
  // Write thrust to motors
  motor1.write(m1Th);
  motor2.write(m2Th);
  motor3.write(m3Th);
  motor4.write(m4Th);

  if (micros()-loopTimer > 1050) {
    Serial.println("error");
  }
  // Wait until 4 milliseconds to ensure 250Hz
  while (micros() - loopTimer < 4000) {
  }
  loopTimer = micros();
}
