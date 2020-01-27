#pragma once

//==================== GENERAL ====================
#define MPU_POWER_REG                 0x6B        // Register for enabling the MPU6050
#define MPU_POWER_CYCLE               0b00000000  //
#define MPU_READ_TIMEOUT              2000        // If no response from IMU in 2s then ignore
#define MPU_SAMP_FREQ                 250         // Sample 250Hz
#define MPU_DLPF_CFG_REG			  0x1A		  // The config register for Digital Low-Pass Filter
#define MPU_DLPF_CFG_BW				  0x04		  // The Bandwith of the low-passfilter (see datasheet for other configs, max filtration at 0x06)

//==================== GYRO ====================
#define MPU_GYRO_CFG_REG              0x1B        // Gyro config register
#define MPU_GYRO_READ_REG             0x43        // Register for reading gyro data
#define MPU_GYRO_READ_REG_SIZE        6           // Number of bytes to read from gyro-register
/*
The following are settings for the gyro. Choose 250, 500, 1000 or 2000
in the main code. The number is the maximum degrees per second the gyro
can measure, but higher numbers give less accuracy
*/
#define MPU_GYRO_CFG_250DEG           0b00000000
#define MPU_GYRO_READINGSCALE_250DEG  131.0
#define MPU_GYRO_CFG_500DEG           0b00001000
#define MPU_GYRO_READINGSCALE_500DEG  65.5
#define MPU_GYRO_CFG_1000DEG          0b00010000
#define MPU_GYRO_READINGSCALE_1000DEG 32.8
#define MPU_GYRO_CFG_2000DEG          0b00011000
#define MPU_GYRO_READINGSCALE_2000DEG 16.4
#define MPU_CALIBRATE_READING_NUM     2000        // Number of reads in calibration

//==================== TEMP ====================
#define MPU_TEMP_READ_REG             0x41        // Register for reading temp data
#define MPU_TEMP_READ_REG_SIZE        2           // Number of bytes to read from temp-register

//==================== ACCEL ====================
#define MPU_ACCEL_CFG_REG             0x1C        // Accel config register
#define MPU_ACCEL_READ_REG            0x3B        // Register for reading accel data
#define MPU_ACCEL_READ_REG_SIZE       6           // Number of bytes to read from accel-register
/*
The following are settings for the gyro. Choose 2g, 4g, 8g or 16g
in the main code. The number is the maximum accel gyro can measure,
but higher numbers give less accuracy
*/
#define MPU_ACCEL_CFG_2G              0b00000000
#define MPU_ACCEL_READINGSCALE_2G     16384.0
#define MPU_ACCEL_CFG_4G              0b00001000
#define MPU_ACCEL_READINGSCALE_4G     8192.0
#define MPU_ACCEL_CFG_8G              0b00010000
#define MPU_ACCEL_READINGSCALE_8G     4096.0
#define MPU_ACCEL_CFG_16G             0b00011000
#define MPU_ACCEL_READINGSCALE_16G    2048.0

//==================== MPU - I2C ====================
#define MPU1_I2C_ADDRESS              0b1101000     // The adress used to access the gyro if only one gyro connected
#define MPU2_I2C_ADDRESS              0b1101001     // If 2 gyros connected the second will have to be pulled up and this is then the address


//==================== DIV ====================
#define STATUS_PIN					  13
#define PI							  3.14159
#define RADTODEG					  180 / PI
#define DEGTORAD					  PI / 180