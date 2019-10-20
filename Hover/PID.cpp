
#include "PID.h"
#include "Arduino.h"


//-----------Pitch------------//
float pitch_ref = 0.0;
//float pitch = 0.0;
float pitch_prev = 0.0;

//-----------Roll------------//
float roll_ref = 0.0;
//float roll = 0.0;
float roll_prev = 0.0;

//-----------Yaw------------//
float yaw_ref = 0.0;
//float yaw = 0.0;
float yaw_prev = 0.0;

extern float pitch, roll, yaw;
int anti_windup = 1;


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
    
    float sum_thrustPI = fabs(P + I);
    if (error_pitch >= 0.0 ){
        motor1 = sum_thrustPI;
        motor2 = sum_thrustPI;
        motor3 = -sum_thrustPI;
        motor4 = -sum_thrustPI;
    }else{
        motor1 = -sum_thrustPI;
        motor2 = -sum_thrustPI;
        motor3 = sum_thrustPI;
        motor4 = sum_thrustPI;
    }

	float sum_thrustD = fabs(D);
	if (D >= 0.0) {
		motor1 +=  sum_thrustD;
		motor2 +=  sum_thrustD;
		motor3 += -sum_thrustD;
		motor4 += -sum_thrustD;
	}
	else {
		motor1 += -sum_thrustD;
		motor2 += -sum_thrustD;
		motor3 +=  sum_thrustD;
		motor4 +=  sum_thrustD;
	}
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
    
    float sum_thrustPI = fabs(P + I);
    if (error_roll >= 0.0 ) {
        motor1 =  sum_thrustPI;
		motor2 = -sum_thrustPI;
		motor3 = -sum_thrustPI;
        motor4 =  sum_thrustPI;
    }else{
        motor1 = -sum_thrustPI;
		motor2 =  sum_thrustPI;
		motor3 =  sum_thrustPI;
        motor4 = -sum_thrustPI;
    }

	float sum_thrustD = fabs(D);
	if (D >= 0.0) {
		motor1 +=  sum_thrustD;
		motor2 += -sum_thrustD;
		motor3 += -sum_thrustD;
		motor4 +=  sum_thrustD;
	}
	else {
		motor1 += -sum_thrustD;
		motor2 +=  sum_thrustD;
		motor3 +=  sum_thrustD;
		motor4 += -sum_thrustD;
	}
}


void PID_Yaw(float &motor1, float &motor2, float &motor3, float &motor4) {
	// calculate error
	float error_yaw = yaw_ref - yaw;
	static float error_prev_yaw = error_yaw;

	// Decleare regulator variables
	float P = 0;
	static float I = 0;
	float D = 0;

	// Calculate gains
	P = Kp_yaw * error_yaw;
	I += Ki_yaw * error_yaw * anti_windup * 0.004;	// (1 / 250) Hz
	D = Kd_yaw * (error_yaw - error_prev_yaw);

	error_prev_yaw = error_yaw;
	yaw_prev = yaw;

	float sum_thrust = fabs(P + I + D);

	// Check theese +/- values
	if (error_yaw >= 0.0) {
		motor1 = -sum_thrust;
		motor2 =  sum_thrust;
		motor3 = -sum_thrust;
		motor4 =  sum_thrust;
	}
	else {
		motor1 =  sum_thrust;
		motor2 = -sum_thrust;
		motor3 =  sum_thrust;
		motor4 = -sum_thrust;
	}
}
