#pragma once
#include "Receiver.h"

enum class flightState { manualHeightControl };

class Quadcopter
{
	flightState state = flightState::manualHeightControl;
	bool firstLoop = true;

public:
	Receiver* receiver;		// RC-controller

	float gForceX, gForceY, gForceZ;
	float rotX, rotY, rotZ;
	float pitch, roll, yaw;
	float pitchRef, rollRef, yawRef;
	/*
	Since the MPU in not level on the dronebody, we have to adjust the accelerometer data.
	These values are found from placing the drone on a level surface, and looking at the
	accelerometer data. We subtract them from the real value to "simulate" a level MPU.
	*/
	float accOffX = 0.058;
	float accOffY = 0.0201;
	float accOffZ = 0.02;
	float calibX, calibY, calibZ;	// Gyro offset values
	int maxThrust = 180;
	float maxIntegralThrust = (float)maxThrust / 3.0;
	int integralAntiWindupPitch = 1;	// 1 = Integral is activated
	int integralAntiWindupRoll = 1;		// 1 = Integral is activated

	Quadcopter();
	~Quadcopter();
	void getMotorThrust(float& m1Thrust, float& m2Thrust, float& m3Thrust, float& m4Thrust);
	void calcAngles();
	void setGyroCalibration(float x, float y, float z) { calibX = x; calibY = y; calibZ = z; }

	void setReceiver(Receiver* _receiver) { receiver = _receiver; }
};

