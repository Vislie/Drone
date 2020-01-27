
#include "PID.h"
#include "Arduino.h"



int anti_windup;
extern float pitch, roll, yaw;
extern float gForceX, gForceY, gForceZ;
float pitch_ref, pitch_prev, roll_ref, roll_prev, yaw_ref, yaw_prev;
float accX_ref, accY_ref, accZ_ref;


void setupPID() {
	//-----------Pitch------------//
	pitch_ref = 0.0;
	pitch_prev = 0.0;

	//-----------Roll------------//
	roll_ref = 0.0;
	roll_prev = 0.0;

	//-----------Yaw------------//
	yaw_ref = 0.0;
	yaw_prev = 0.0;

	//-----------accX------------//
	accX_ref = 0.0;

	//-----------accY------------//
	accX_ref = 0.0;

	//-----------accZ------------//
	accX_ref = 0.0;


	// Anti-windup used to prevent intregal windup
	anti_windup = 1;
}


void PID_Pitch(float &motor1, float &motor2, float &motor3, float &motor4){
	// calculate error
	float error_pitch = pitch_ref - pitch;
	static float error_prev_pitch = error_pitch;
	static float error_2xprev_pitch = error_pitch;
    
	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;
    
	// Calculate gains
	P = Kp_pitch * error_pitch;
	I += Ki_pitch * error_pitch * anti_windup * 0.004;	// (1 / 250) Hz
	D = Kd_pitch * (error_pitch - ((error_prev_pitch + error_2xprev_pitch) / 2));

	error_2xprev_pitch = error_prev_pitch;
	error_prev_pitch = error_pitch;
	pitch_prev = pitch;

  // ADD PROPORTIONAL PART TO THRUST
	motor1 =  P;
	motor2 =  P;
	motor3 = -P;
	motor4 = -P;

	// ADD INTEGRAL PART TO THRUST
	motor1 +=  I;
	motor2 +=  I;
	motor3 += -I;
	motor4 += -I;

  // ADD DERIVATIVE PART TO THRUST
	motor1 +=  D;
	motor2 +=  D;
	motor3 += -D;
	motor4 += -D;
}


void PID_Roll(float &motor1, float &motor2, float &motor3, float &motor4){
	// calculate error
	float error_roll = roll_ref - roll;
	static float error_prev_roll = error_roll;
	static float error_2xprev_roll = error_roll;
    
	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;
    
	// Calculate gains
	P = Kp_roll * error_roll;
	I += Ki_roll * error_roll * anti_windup * 0.004;	// (1 / 250) Hz
	D = Kd_roll * (error_roll - ((error_prev_roll + error_2xprev_roll) / 2));
    
	error_2xprev_roll = error_prev_roll;
	error_prev_roll = error_roll;
	roll_prev = roll;

	// ADD PR0PORTIONAL PART TO THRUST
	motor1 =  P;
	motor2 = -P;
	motor3 = -P;
	motor4 =  P;

	// ADD INTEGRAL PART TO THRUST
	motor1 +=  I;
	motor2 += -I;
	motor3 += -I;
	motor4 +=  I;

	// ADD DERIVATIVE PART TO THRUST
	motor1 +=  D;
	motor2 += -D;
	motor3 += -D;
	motor4 +=  D;
}


void PID_Yaw(float &motor1, float &motor2, float &motor3, float &motor4) {
	// calculate error
	float error_yaw = yaw_ref - yaw;
	static float error_prev_yaw = error_yaw;
	static float error_2xprev_yaw = error_yaw;

	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;

	// Calculate gains
	P = Kp_yaw * error_yaw;
	I += Ki_yaw * error_yaw * anti_windup * 0.004;	// (1 / 250) Hz
	D = Kd_yaw * (error_yaw - ((error_prev_yaw + error_2xprev_yaw) / 2));

	error_2xprev_yaw = error_prev_yaw;
	error_prev_yaw = error_yaw;
	yaw_prev = yaw;

	motor1 = -P;
	motor2 =  P;
	motor3 = -P;
	motor4 =  P;

	motor1 += -I;
	motor2 +=  I;
	motor3 += -I;
	motor4 +=  I;

	motor1 += -D;
	motor2 +=  D;
	motor3 += -D;
	motor4 +=  D;
}


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
