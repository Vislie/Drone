
#include "PID.h"
#include "Arduino.h"
#include "Quadcopter.h"



int anti_windup;
//extern float pitch, roll, yaw;
//extern float gForceX, gForceY, gForceZ;
//float pitch_ref, roll_ref, yaw_ref;
//float accX_ref, accY_ref, accZ_ref;

extern Quadcopter* quad;


void setupPID() {
	//-----------Pitch, Roll, Yaw refs------------//
	//pitch_ref = 0.0;
	//roll_ref = 0.0;
	//yaw_ref = 0.0;

	//-----------accX------------//
	//accX_ref = 0.0;

	//-----------accY------------//
	//accX_ref = 0.0;

	//-----------accZ------------//
	//accX_ref = 0.0;


	// Anti-windup used to prevent intregal windup
	anti_windup = 1;
}


void PID_Pitch(float &motor1, float &motor2, float &motor3, float &motor4){
	// calculate error
	float errorPitch = quad->pitchRef - quad->pitch;
	static float errorPrevPitch = errorPitch;
	static float error2xPrevPitch = errorPitch;
    
	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;
    
	// Calculate gains
	P = Kp_pitch * errorPitch;
	I += Ki_pitch * errorPitch * anti_windup * 0.004;	// (1 / 250) Hz
	D = Kd_pitch * (errorPitch - ((errorPrevPitch + error2xPrevPitch) / 2));

	error2xPrevPitch = errorPrevPitch;
	errorPrevPitch = errorPitch;

  // ADD PROPORTIONAL PART TO THRUST
	motor1 = P + I + D;
	motor2 = P + I + D;
	motor3 = -(P + I + D);
	motor4 = -(P + I + D);

	// ADD INTEGRAL PART TO THRUST
	//motor1 +=  I;
	//motor2 +=  I;
	//motor3 += -I;
	//motor4 += -I;

	// ADD DERIVATIVE PART TO THRUST
	//motor1 +=  D;
	//motor2 +=  D;
	//motor3 += -D;
	//motor4 += -D;
}


void PID_Roll(float &motor1, float &motor2, float &motor3, float &motor4){
	// calculate error
	float errorRoll = quad->rollRef - quad->roll;
	static float errorPrevRoll = errorRoll;
	static float error2xPrevRoll = errorRoll;
    
	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;
    
	// Calculate gains
	P = Kp_roll * errorRoll;
	I += Ki_roll * errorRoll * anti_windup * 0.004;	// (1 / 250) Hz
	D = Kd_roll * (errorRoll - ((errorPrevRoll + error2xPrevRoll) / 2));
    
	error2xPrevRoll = errorPrevRoll;
	errorPrevRoll = errorRoll;

	// ADD PR0PORTIONAL PART TO THRUST
	motor1 = P + I + D;
	motor2 = -(P + I + D);
	motor3 = -(P + I + D);
	motor4 = P + I + D;

	// ADD INTEGRAL PART TO THRUST
	//motor1 +=  I;
	//motor2 += -I;
	//motor3 += -I;
	//motor4 +=  I;

	// ADD DERIVATIVE PART TO THRUST
	//motor1 +=  D;
	//motor2 += -D;
	//motor3 += -D;
	//motor4 +=  D;
}


void PID_Yaw(float &motor1, float &motor2, float &motor3, float &motor4) {
	// calculate error
	float errorYaw = quad->yawRef - quad->yaw;
	static float errorPrevYaw = errorYaw;
	static float error2xPrevYaw = errorYaw;

	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;

	// Calculate gains
	P = Kp_yaw * errorYaw;
	I += Ki_yaw * errorYaw * anti_windup * 0.004;	// (1 / 250) Hz
	D = Kd_yaw * (errorYaw - ((errorPrevYaw + error2xPrevYaw) / 2));

	error2xPrevYaw = errorPrevYaw;
	errorPrevYaw = errorYaw;

	motor1 = -(P + I + D);
	motor2 = P + I + D;
	motor3 = -(P + I + D);
	motor4 = P + I + D;

	//motor1 += -I;
	//motor2 +=  I;
	//motor3 += -I;
	//motor4 +=  I;

	//motor1 += -D;
	//motor2 +=  D;
	//motor3 += -D;
	//motor4 +=  D;
}

/*
float PID_accX() {
	// calculate error
	float error = accX_ref - ((-1) * gForceX * 9.81 * cos(pitch * DEGTORAD) + gForceZ * 9.81 * sin(pitch * DEGTORAD) + 0.67);	// 0.67 is some measuret offset that is needed
	static float error_prev = error;
	static float error_2xprev = error_prev;

	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;

	// Calculate gains
	P = Kp_accX * error;
	I += Ki_accX * error * 0.004;	// (1 / 250) Hz
	D = Kd_accX * (error - ((error_prev + error_2xprev) / 2));

	error_2xprev = error_prev;
	error_prev= error;

	pitch_ref = P + I + D;
	return (P + I + D);
}


float PID_accY() {
	// calculate error
	float error = accY_ref - (gForceY * 9.81 * cos(roll * DEGTORAD) - gForceZ * 9.81 * sin(roll * DEGTORAD) + 0.01);
	static float error_prev = error;
	static float error_2xprev = error_prev;

	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;

	// Calculate gains
	P = Kp_accY * error;
	I += Ki_accY * error * 0.004;	// (1 / 250) Hz
	D = Kd_accY * (error - ((error_prev + error_2xprev) / 2));

	error_2xprev = error_prev;
	error_prev = error;

	roll_ref = P + I + D;
	return (P + I + D);
}


// NOT YET IMPLEMENTED...
void PID_accZ() {
	return;
}
*/