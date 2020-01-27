#pragma once

#include "IMU_Defines.h"
#include <Wire.h>
#include "Arduino.h"

extern float gForceX, gForceY, gForceZ;
extern float rotX, rotY, rotZ;
extern float calibX, calibY, calibZ;
//extern float gyrAngX, gyrAngY, gyrAngZ;
extern float accPitch, accRoll;
extern float pitch, roll, yaw;
extern float mpuTemp;
extern long loopTimer;
extern bool firstLoop;


void setupMPU();
void calibrateGyro();

bool readMPU();
bool MPUReadAccel();
bool MPUReadGyro();

void MPUPrintData();
void calcAngles();