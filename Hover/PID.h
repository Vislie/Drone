
#ifndef PID_Pitch_hpp
#define PID_Pitch_hpp

#define PI							  3.14159
#define RADTODEG					  180 / PI
#define DEGTORAD					  PI / 180

constexpr float motor_saturation = 160.0;

//-----------Pitch------------//
extern float pitch_ref; // See CPP for definition

// Ku = 0.05, Tu = 3.09, ZN-method
constexpr float Kp_pitch = 0.1;
constexpr float Ki_pitch = 0.012;	//tidl 0.007
constexpr float Kd_pitch = 14.0;	//tidl 10.0, 12

//-----------Roll------------//
extern float roll_ref;

constexpr float Kp_roll = 0.1;
constexpr float Ki_roll = 0.012;
constexpr float Kd_roll = 14.0;

//-----------Yaw------------//
extern float yaw_ref;

constexpr float Kp_yaw = 0.0;// 0.2;
constexpr float Ki_yaw = 0.0;// 0.03;
constexpr float Kd_yaw = 0.0;// 25.0;

//-----------accX------------//
extern float accX_ref;

constexpr float Kp_accX = 0.0;// 0.2;
constexpr float Ki_accX = 0.0;
constexpr float Kd_accX = 0.0;

//-----------accY------------//
extern float accY_ref;

constexpr float Kp_accY = 0.0;//0.2;
constexpr float Ki_accY = 0.0;
constexpr float Kd_accY = 0.0;

//-----------accZ------------//
extern float accZ_ref;

constexpr float Kp_accZ = 0.00;
constexpr float Ki_accZ = 0.00;
constexpr float Kd_accZ = 0.00;






void setupPID();

void PID_Pitch(float &motor1, float &motor2, float &motor3, float &motor4);

void PID_Roll(float &motor1, float &motor2, float &motor3, float &motor4);

void PID_Yaw(float &motor1, float &motor2, float &motor3, float &motor4);

float PID_accX();

float PID_accY();

void PID_accZ();



#endif /* PID_Pitch_hpp */
