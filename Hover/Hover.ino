#include <Servo.h>
#include "btModule.h"
#include "MPU.h"
#include "PID.h"
#include "Quadcopter.h"
#include "Receiver.h"

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
  Receiver* receiver = new Receiver();
  quad->setReceiver(receiver);
  
  loopTimer = 0;
  Wire.begin(); // Start wire-library
  Serial.begin(9600);
  setupBT();
  Serial.println("BT setup complete");

  // Setup STATUS_PIN (LED)
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);

  // Setup receiver
  pinMode(RECEIVE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RECEIVE_PIN), ppm_interrupt, METHOD);

  
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
  while (quad->receiver->getThrust() < 10/*getThrust() == 0*/) {
    Serial.println("Waiting...");
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

  /*
  static int aaa = 0;
  if ( aaa++ % 50 == 0) {
    Serial.print(quad->receiver->getPitchRef());
    Serial.print("\t");
    Serial.print(quad->receiver->getRollRef());
    Serial.print("\t");
    Serial.print(quad->receiver->getYawRef());
    Serial.print("\t");
    Serial.print(quad->receiver->getThrust());
    Serial.print("\t");
    Serial.print(quad->receiver->getSwA());
    Serial.print("\t");
    Serial.print(quad->receiver->getSwC());
    Serial.println("\t");
    aaa = 1;
  }*/

  /*if (micros()-loopTimer > 3500) {
    Serial.println("error");
  }*/
  // Wait until 4 milliseconds to ensure 250Hz
  while (micros() - loopTimer < 4000) {
  }
  loopTimer = micros();
}


void ppm_interrupt() {
  static byte i;
  static unsigned long int t_old;
  unsigned long int t = micros(); //store time value a when pin value falling/rising
  unsigned long int dt = t - t_old; //calculating time inbetween two peaks
  t_old = t;

  if ((dt > DETECTION_SPACE) || (i > CHANNEL_AMOUNT)) {
    i = 0;
    quad->receiver->calcValues();
  }
  quad->receiver->setChannel(i, dt);
  i++;
}
