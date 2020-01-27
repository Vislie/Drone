#include "MPU.h"
#include <math.h>

float gForceX, gForceY, gForceZ;
float rotX, rotY, rotZ;
float calibX, calibY, calibZ;
//float gyrAngX, gyrAngY, gyrAngZ;
float accPitch, accRoll;
float pitch, roll, yaw;
float posX, posY, posZ;	// Distance travelled in all directions, relative to startposition.
float mpuTemp;
long loopTimer;
bool firstLoop;

float accOffX;	// Offset from not having the gyro plane with the drone
float accOffY;	// Offset from not having the gyro plane with the drone
float accOffZ;	// Offset from not having the gyro plane with the drone


void setupMPU() {
	// Initialize pitch, roll, yaw, and all positions to zero
	roll = pitch = yaw = 0;
	posX = posY = posZ = 0;
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
	/*
	Since the MPU in not level on the dronebody, we have to adjust the accelerometer data.
	These values are found from placing the drone on a level surface, and looking at the
	accelerometer data. We subtract them from the real value to "simulate" a level MPU.
	*/
	accOffX = 0.058;
	accOffY = 0.0201;
	accOffZ = 0.02;

}


void calibrateGyro() {
  // While calibrating, StatusLED will light up
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
	gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G -accOffX;	// Offsets from not having the gyro level with the drone-body
	gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G -accOffY;
	gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_4G -accOffZ;
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
	accPitch = (atan(gForceX / gForceZ) * RADTODEG) - 4;	// Some measured offset because gyro is not flat in pitch direction
	accRoll = atan(gForceY / gForceZ) * RADTODEG;

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


void MPUPrintData() {
	static int count = 0;

	// Dont want to print every cycle...
	count++;
	if (count % 100 == 0) {
		count = 1;
		//Serial.print(posX);
		//Serial.print("\t");
		//Serial.print(gForceX);
		//Serial.print("\t");
		//Serial.println(gForceZ);
		//Serial.print("\t");
		//Serial.println(pitch);
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
		Serial.print(" Yaw:");*/
		Serial.println(yaw);/*
		Serial.print(" AccPitch:");
		Serial.print(accPitch);
		Serial.print(" AccRoll:");
		Serial.print(accRoll);
		Serial.println();
		*/
	}

}
