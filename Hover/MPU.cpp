#include "MPU.h"
#include <math.h>

float gForceX, gForceY, gForceZ;
float rotX, rotY, rotZ;
float calibX, calibY, calibZ;
//float gyrAngX, gyrAngY, gyrAngZ;
float accPitch, accRoll;
float pitch, roll, yaw;
float mpuTemp;
long loopTimer;
bool firstLoop;


void setupMPU() {
	roll = pitch = yaw = 0;
	firstLoop = true;
	// Set up MPU
	Wire.beginTransmission(MPU1_I2C_ADDRESS);
	Wire.write(MPU_POWER_REG);
	Wire.write(MPU_POWER_CYCLE);
	Wire.endTransmission();
	// Set up gyro
	Wire.beginTransmission(MPU1_I2C_ADDRESS);
	Wire.write(MPU_GYRO_CFG_REG);
	Wire.write(MPU_GYRO_CFG_500DEG);        // Might have to increaso to 1000DEG
	Wire.endTransmission();
	// Set up Accelerometer
	Wire.beginTransmission(MPU1_I2C_ADDRESS);
	Wire.write(MPU_ACCEL_CFG_REG);
	Wire.write(MPU_ACCEL_CFG_4G);           // Might have to increase this as well
	Wire.endTransmission();
	// Activate low-pass filter
	Wire.beginTransmission(MPU1_I2C_ADDRESS);
	Wire.write(MPU_DLPF_CFG_REG);
	Wire.write(MPU_DLPF_CFG_BW);
	Wire.endTransmission();
}


void calibrateGyro() {
	digitalWrite(STATUS_PIN, HIGH);
	Serial.println("Calibrating Gyro...");
	
	calibX = calibY = calibZ = 0;

	// Add N readings and divide by N to get average offset
	for (int i = 0; i < MPU_CALIBRATE_READING_NUM; i++) {
		if (MPUReadGyro()) {
			calibX += rotX;
			calibY += rotY;
			calibZ += rotZ;

			// wait for the next sample cycle
			while (micros() - loopTimer < 4000);
			loopTimer = micros();
		}
		else {
			// If a reading is missed then subtract one from loop, to read again
			i--;
		}
	}

	calibX = calibX / MPU_CALIBRATE_READING_NUM;
	calibY = calibY / MPU_CALIBRATE_READING_NUM;
	calibZ = calibZ / MPU_CALIBRATE_READING_NUM;

	Serial.println("Calibration done");
	Serial.print("xoff: ");
	Serial.print(calibX);
	Serial.print("\tyoff: ");
	Serial.print(calibY);
	Serial.print("\tzoff: ");
	Serial.println(calibZ);
	
	digitalWrite(STATUS_PIN, LOW);
}


bool readMPU() {
	// If able to read both accel and gyro then print data and calculate angles
	if (MPUReadAccel() && MPUReadGyro()) {
		calcAngles();
		MPUPrintData();
		return true;
	}
	return false;
}


bool MPUReadAccel() {
	Wire.beginTransmission(MPU1_I2C_ADDRESS);
	Wire.write(MPU_ACCEL_READ_REG);
	Wire.endTransmission();
	Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);
	// If no response within TIMEOUT then ignore waiting and return false
	long timeout = millis() + MPU_READ_TIMEOUT;
	while (Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
	if (timeout <= millis())
		return false;
	// Calculate the g-forces from the accel
	gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G;
	gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G;
	gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G - 0.02;	// Some random offset
	return true;
}


bool MPUReadGyro() {
	Wire.beginTransmission(MPU1_I2C_ADDRESS);
	Wire.write(MPU_GYRO_READ_REG);
	Wire.endTransmission();
	Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_GYRO_READ_REG_SIZE);
	// If no response within TIMEOUT then ignore waiting and return false
	long timeout = millis() + MPU_READ_TIMEOUT;
	while (Wire.available() < MPU_GYRO_READ_REG_SIZE && timeout < millis());
	if (timeout <= millis())
		return false;
	// Calculate the angular speed (in deg) from the gyro
	rotX = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
	rotY = -(long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
	rotZ = -(long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
	return true;
}


void calcAngles() {

	// Calculating the pitch and roll from accelerometer
	if (gForceZ == 0) {
		gForceZ = 0.00000001;		// Making sure we dont divide by zero
	}
	accPitch = atan(gForceX / gForceZ) * 180 / PI;
	accRoll = atan(gForceY / gForceZ) * 180 / PI;

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
	pitch -= roll * sin(0.004 * (rotZ - calibZ) * PI / 180);	// Must convert to radians
	roll += tempPitch * sin(0.004 * (rotZ-calibZ) * PI / 180);
	
	// Combining acc and gyro angles to avoid drift
	float combConst = 0.9996;
	pitch = pitch * combConst + accPitch * (1 - combConst);
	roll = roll * combConst + accRoll * (1 - combConst);
}


void MPUPrintData() {
	static int count = 0;

	count++;
	if (count % 100 == 0) {
		count = 1;
		//Serial.print(pitch);
		//Serial.print(" ");
		//Serial.println(roll);
		/*
		Serial.print("Temp (deg c) ");
		Serial.print(mpuTemp);
		Serial.print(" Gyro (deg/s)");
		Serial.print(" X=");
		Serial.print(rotX - calibX);
		Serial.print(" Y=");
		Serial.print(rotY - calibY);
		Serial.print(" Z=");
		Serial.print(rotZ - calibZ);
		Serial.print(" Accel (g)");
		Serial.print(" X=");
		Serial.print(gForceX);
		Serial.print(" Y=");
		Serial.print(gForceY);
		Serial.print(" Z=");
		Serial.println(gForceZ);
		Serial.print(" RT:");
		Serial.print(" Pitch:");
		Serial.print(pitch);
		Serial.print(" Roll:");
		Serial.print(roll);
		Serial.print(" Yaw:");
		Serial.print(yaw);
		Serial.print(" AccPitch:");
		Serial.print(accPitch);
		Serial.print(" AccRoll:");
		Serial.print(accRoll);
		Serial.println();
		*/
	}

}