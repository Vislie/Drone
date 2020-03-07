#pragma once
#include "Arduino.h"

#define RECEIVE_PIN 2
#define CHANNEL_AMOUNT 8
#define RECEIVER_ARRAY_SIZE (CHANNEL_AMOUNT + 1)
#define DETECTION_SPACE 2500
#define METHOD FALLING

enum SWITCH {DOWN = -1, MIDDLE, UP, UNDEFINED};

class Receiver
{
private:
	int ch[RECEIVER_ARRAY_SIZE];
	double values[5];	// Unused, roll, pitch, thrust, yaw
	SWITCH switches[4];	// SwA, SwC, unused, unused
public:
	Receiver() { for (int i = 0; i < RECEIVER_ARRAY_SIZE; i++) ch[i] = 0; }
	
	void calcValues();
	void setChannel(int i, int val);

	double getPitchRef() { return values[2]; }
	double getRollRef() { return values[1]; }
	double getYawRef() { return values[4]; }
	double getThrust() { return values[3]; }
	double getSwA() { return (int)switches[0]; }
	double getSwC() { return (int)switches[1]; }
};