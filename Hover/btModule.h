#pragma once

#include <Wire.h>
#include "Arduino.h"
#include <SoftwareSerial.h>

// Setup the HC-05 bluetooth module
void setupBT();

// Get value from android app
int getThrust();

// Print to the app
void btPrint(float val);