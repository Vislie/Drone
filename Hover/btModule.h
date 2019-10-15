#pragma once

#include <Wire.h>
#include "Arduino.h"
#include <SoftwareSerial.h>

void setupBT();

int getThrust();

void btPrint(float val);