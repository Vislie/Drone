#include "Receiver.h"


void Receiver::calcValues() {
	// Unused
	values[0] = ch[0];
	// Roll
	values[1] = constrain(ch[1], 1000, 2000);
	values[1] = map(values[1], 1000, 2000, -100, 100);
	values[1] = values[1] / 10.0;
	// Pitch
	values[2] = constrain(ch[2], 1000, 2000);
	values[2] = map(values[2], 1000, 2000, -100, 100);
	values[2] = - values[2] / 10.0;
	// Thrust
	values[3] = constrain(ch[3], 1000, 2000);
	values[3] = map(values[3], 1000, 2000, 0, 1800);
	values[3] = values[3] / 10.0;
	// Yaw
	values[4] = constrain(ch[4], 1000, 2000);
	values[4] = map(values[4], 1000, 2000, -900, 900);
	values[4] = values[4] / 10.0;

	// Switch A
	if (ch[5] > 1500) {
		switches[0] = SWITCH::DOWN;
	}
	else {
		switches[0] = SWITCH::UP;
	}
	// Switch C
	if (ch[6] > 1750) {
		switches[1] = SWITCH::DOWN;
	}
	else if (ch[6] > 1250) {
		switches[1] = SWITCH::MIDDLE;
	}
	else {
		switches[1] = SWITCH::UP;
	}

	switches[2] = SWITCH::UNDEFINED;
	switches[3] = SWITCH::UNDEFINED;
}


void Receiver::setChannel(int i, int val) {
	ch[i] = val;
}