#include "CKobuki.h"
#include <iostream>
#include <cmath>

int main() {
	unsigned char * null_ptr(0);
	CKobuki robot;

	robot.startCommunication("/dev/cu.usbserial-kobuki_AI02MVQM", true, null_ptr);
	usleep(1*1000*1000);

	robot.goStraight(0.2);

	robot.doRotation(-PI);
	// goToXy(robot,0,3);
	usleep(25*1000);
}
