#include "MPU.h"
#include <math.h>
#include "Quadcopter.h"

//float gForceX, gForceY, gForceZ;
//float rotX, rotY, rotZ;
//float calibX, calibY, calibZ;
//float gyrAngX, gyrAngY, gyrAngZ;
//float accPitch, accRoll;
//float pitch, roll, yaw;
//float posX, posY, posZ;	// Distance travelled in all directions, relative to startposition.
float mpuTemp;
long loopTimer;

//float accOffX;	// Offset from not having the gyro plane with the drone
//float accOffY;	// Offset from not having the gyro plane with the drone
//float accOffZ;	// Offset from not having the gyro plane with the drone

extern Quadcopter * quad;


void setupMPU() {
	// Initialize pitch, roll, yaw, and all positions to zero
	//posX = posY = posZ = 0;
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
  // While calibrating, StatusLED will light up
	digitalWrite(STATUS_PIN, HIGH);
	Serial.println("Calibrating Gyro...");
	
	float tempCalibX = 0.0;
	float tempCalibY = 0.0;
	float tempCalibZ = 0.0;

	// Add N readings and divide by N to get average offset
	for (int i = 0; i < MPU_CALIBRATE_READING_NUM; i++) {
		if (MPUReadGyro()) {
			tempCalibX += quad->rotX;
			tempCalibY += quad->rotY;
			tempCalibZ += quad->rotZ;

			// wait for the next sample cycle
			while (micros() - loopTimer < 4000);
			loopTimer = micros();
		}
		else {
			// If a reading is missed then subtract one from loop, to read again
			i--;
		}
	}

	tempCalibX = tempCalibX / MPU_CALIBRATE_READING_NUM;
	tempCalibY = tempCalibY / MPU_CALIBRATE_READING_NUM;
	tempCalibZ = tempCalibZ / MPU_CALIBRATE_READING_NUM;

	quad->setGyroCalibration(tempCalibX, tempCalibY, tempCalibZ);

	Serial.println("Calibration done");
	Serial.print("xoff: ");
	Serial.print(tempCalibX);
	Serial.print("\tyoff: ");
	Serial.print(tempCalibY);
	Serial.print("\tzoff: ");
	Serial.println(tempCalibZ);
	
	digitalWrite(STATUS_PIN, LOW);
}


bool readMPU() {
	// If able to read both accel and gyro then print data and calculate angles
	if (MPUReadAccel() && MPUReadGyro()) {
		quad->calcAngles();
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
	quad->gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G - quad->accOffX;	// Offsets from not having the gyro level with the drone-body
	quad->gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G - quad->accOffY;
	quad->gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G - quad->accOffZ;
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
	quad->rotX = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
	quad->rotY = -(long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
	quad->rotZ = -(long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_500DEG;
	return true;
}


void MPUPrintData() {
	static int count = 0;

	// Dont want to print every cycle...
	count++;
	if (count % 100 == 0) {
		count = 1;
		//Serial.print(posX);
		//Serial.print("\t");
		//Serial.print(quad->gForceX);
		//Serial.print("\t");
		//Serial.println(quad->gForceZ);
		//Serial.print("\t");
		//Serial.println(quad->pitch);
		//Serial.print(" ");
		//Serial.print(roll);
		//Serial.print(" ");
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
		Serial.println(yaw);
		Serial.print(" AccPitch:");
		Serial.print(accPitch);
		Serial.print(" AccRoll:");
		Serial.print(accRoll);
		Serial.println();
		*/
	}

}
