#include "btModule.h"

SoftwareSerial BTserial(4, 5); // RX | TX
// Connect the HC-05 TX to Arduino pin 4 RX. 
// Connect the HC-05 RX to Arduino pin 5 TX through a voltage divider.
// Connect the HC-05 STATE pin to Arduino pin 6. (We haven't done this)

int thrustBT;     // Thrust received from BT module
const byte BTpin = 6;     // Read-Pin STATE from HC-05 (we dont use this)


void setupBT() {
	// set the BTpin for input
	pinMode(BTpin, INPUT);
	Serial.println("Connect the HC-05 to an arduino device to continue");
	// Start serial communication with the bluetooth module
	// HC-05 default serial speed for communication mode is 9600 but can be different
	BTserial.begin(9600);
	thrustBT = 0;

}


int getThrust() {
	// Keep reading from the HC-05 and send to Arduino Serial Monitor
	if (BTserial.available())
	{
		thrustBT = BTserial.read();
		Serial.println(thrustBT);
	}
	return thrustBT;

	/*
	// Keep reading from Arduino Serial Monitor input field and send to HC-05
	if (Serial.available())
	{
		thrustBT = Serial.read();
		BTserial.write(thrustBT);
	}
	*/
}


void btPrint(float val) {
	//BTSerial.print(val);
	//char str[8];
	static int counter = 0;
	counter++;
	if (counter % 100 == 0) {
		counter = 0;
		int8_t a = round(val);
		BTserial.write(a);
	}
	//snprintf(str, sizeof(str), "%.2f", val);
	//BTserial.write('a');
	//BTserial.write(sizeof(str));
	//Serial.write(sizeof(str));
	//Serial.print(BTserial.write(str, 8));
	//Serial.write(str);
	//Serial.print(val);
}