#include "CKobuki.h"
#include <iostream>

long demoCallback(void *user_data, TKobukiData &Kobuki_data) {
	std::cout << "mam kokt!" << Kobuki_data.EncoderLeft << std::endl;
	
	return 0;
}


int main() {
	unsigned char * null_ptr(0);
	CKobuki robot;
	robot.startCommunication("/dev/ttyUSB0", true, &demoCallback, null_ptr);
	//robot.setPower(1);
	
	//usleep(1*1000*1000);
	robot.setLed(1,1);
	//usleep(25*1000);

	/*
	robot.setTranslationSpeed(100);
	usleep(1*1000*1000);
	robot.setRotationSpeed(3.14159/4.0);
	usleep(2*1000*1000);
        robot.setTranslationSpeed(100);
        usleep(1*1000*1000);
        robot.setRotationSpeed(3.14159/4.0);
        usleep(2*1000*1000);
        robot.setTranslationSpeed(100);
        usleep(1*1000*1000);
        robot.setRotationSpeed(3.14159/4.0);
        usleep(2*1000*1000);
        robot.setTranslationSpeed(100);
        usleep(1*1000*1000);
        robot.setRotationSpeed(3.14159/4.0);
        usleep(2*1000*1000);
	*/

	for (int i = 0; i < 16; i++) {
		robot.setRotationSpeed(3.14159/2.0*1.11);
		usleep(1*1000*1000);
		std::cout << "otoceu som sa " << i << "-ty raz"  << std::endl;
	}

	robot.setTranslationSpeed(0);
	std::cout << "kokoty za tri kompoty" << std::endl;

}
