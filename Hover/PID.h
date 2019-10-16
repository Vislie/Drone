
#ifndef PID_Pitch_hpp
#define PID_Pitch_hpp

constexpr float motor_saturation = 120.0;

//-----------Pitch------------//
extern float pitch_ref; // See CPP for definition
//float pitch = 0.0;
extern float pitch_prev; // See CPP for definition

constexpr float Kp_pitch = 0.35;
constexpr float Ki_pitch = 0.1;
constexpr float Kd_pitch = 10.0;

//-----------Roll------------//
extern float roll_ref; // See CPP for definition
//float roll = 0.0;
extern float roll_prev; // See CPP for definition

constexpr float Kp_roll = 1.0;
constexpr float Ki_roll = 1.0;
constexpr float Kd_roll = 1.0;

//-----------Yaw------------//
extern float yaw_ref; // See CPP for definition
//float roll = 0.0;
extern float yaw_prev; // See CPP for definition

constexpr float Kp_yaw = 1.0;
constexpr float Ki_yaw = 1.0;
constexpr float Kd_yaw = 0.0;	// Probably dont need I for yaw ( Ref Haavard )






void PID_Pitch(float &motor1, float &motor2, float &motor3, float &motor4);

void PID_Roll(float &motor1, float &motor2, float &motor3, float &motor4);

void PID_Yaw(float &motor1, float &motor2, float &motor3, float &motor4);



#endif /* PID_Pitch_hpp */
