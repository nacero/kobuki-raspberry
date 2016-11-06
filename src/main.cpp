#include "CKobuki.h"
#include <iostream>

long demoCallback(void *user_data, TKobukiData &Kobuki_data) {
	std::cout << "mam kokt!" << std::endl;
	return 0;
}


int main() {
	
	CKobuki robot;
	// void startCommunication(char *portname,bool CommandsEnabled,src_callback_kobuki_data userCallback,void *userDataL);
	robot.startCommunication("/dev/ttyUSB0", true, &demoCallback, nullptr);
    // robot.dataProcess(this,&demoCallback);

	std::cout << "kokoty za tri kompoty" << std::endl;

}