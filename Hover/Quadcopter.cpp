#include "Quadcopter.h"
#include "btModule.h"
#include "PID.h"

Quadcopter::Quadcopter()
{
	pitchRef = 0.0;
	rollRef = 0.0;
	yawRef = 0.0;
}

Quadcopter::~Quadcopter()
{
}

void Quadcopter::getMotorThrust(float& m1Thrust, float& m2Thrust, float& m3Thrust, float& m4Thrust)
{
	//int thrust = getThrust();		// From Bluetooth
	int thrust = receiver->getThrust();
	pitchRef = receiver->getPitchRef();
	rollRef = receiver->getRollRef();
	yawRef = receiver->getYawRef();
	float m1P = 0.0, m2P = 0.0, m3P = 0.0, m4P = 0.0;    // Thrust from Pitch-PID
	float m1R = 0.0, m2R = 0.0, m3R = 0.0, m4R = 0.0;    // Thrust from Roll-PID
	float m1Y = 0.0, m2Y = 0.0, m3Y = 0.0, m4Y = 0.0;    // Thrust from Yaw-PID
	PID_Pitch(m1P, m2P, m3P, m4P);
	PID_Roll(m1R, m2R, m3R, m4R);
	PID_Yaw(m1Y, m2Y, m2Y, m2Y);

	if (state == flightState::manualHeightControl) {
		if (thrust > 0) {
			m1Thrust = constrain(thrust + m1P + m1R + m1Y, 0.0, maxThrust);
			m2Thrust = constrain(thrust + m2P + m2R + m2Y, 0.0, maxThrust);
			m3Thrust = constrain(thrust + m3P + m3R + m3Y, 0.0, maxThrust);
			m4Thrust = constrain(thrust + m4P + m4R + m4Y, 0.0, maxThrust);
		}
		else {
			m1Thrust = 0;
			m2Thrust = 0;
			m3Thrust = 0;
			m4Thrust = 0;
		}
	}
}

void Quadcopter::calcAngles()
{
	// Calculating the pitch and roll from accelerometer
	if (gForceZ == 0) {
		gForceZ = 0.00000001;		// Making sure we dont divide by zero
	}
	float accPitch = (atan(gForceX / gForceZ) * RADTODEG) - 4;	// Some measured offset because gyro is not flat in pitch direction
	float accRoll = atan(gForceY / gForceZ) * RADTODEG;

	// If first loop, then set the angles based on the accelerometer
	if (firstLoop) {
		pitch = accPitch;
		roll = accRoll;
		firstLoop = false;
	}

	// Calculating pitch, roll, yaw from gyro
	pitch += (rotY - calibY) * 0.004;
	roll += (rotX - calibX) * 0.004;
	yaw += (rotZ - calibZ) * 0.004;

	// Correcting roll&pitch depending on yaw in a slope (See video :) )
	float tempPitch = pitch;
	pitch -= roll * sin(0.004 * (rotZ - calibZ) * DEGTORAD);	// Must convert to radians
	roll += tempPitch * sin(0.004 * (rotZ - calibZ) * DEGTORAD);

	// Combining acc and gyro angles to avoid drift
	float combConst = 0.9996;
	pitch = pitch * combConst + accPitch * (1 - combConst);
	roll = roll * combConst + accRoll * (1 - combConst);
}
