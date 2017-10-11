#include "CKobuki.h"
#include <iostream>
#include <cmath>


int main() {
	unsigned char * null_ptr(0);
	CKobuki robot;


// tu treba doimplementovat shared pointer na data z robota RobotPosition, ktore by mali byt nasledovny struct:
// {
// 	absX: ,
// 	absY: ,
// 	absGyroTheta: 
// } RoboPosition
// 
// Tento struct si nasledne bude vycitavat nadradeny thread ako polohu robota on-demand




	robot.startCommunication("/dev/cu.usbserial-kobuki_AI02MVQM", true, null_ptr);
	usleep(1*1000*1000);



// instrukcie do robota by sa mohli predavat ako program v textovej podobe z textaku:
// ROTATE: PI/2 [rad, PI parsed and substituted via https://github.com/codeplea/tinyexpr]
// GO: 0.2 [m]




	// robot.goStraight(0.2);
	// usleep(25*1000);
	// robot.doRotation(PI/3);
	// robot.doRotation(PI/3);
	// robot.doRotation(PI/3);
	// robot.doRotation(PI/3);
	// robot.doRotation(PI/3);
	// robot.doRotation(PI/3);
	// robot.goStraight(0.2);

	// robot.doRotation(-PI);
	// robot.goToXy(0.1,0.1);
	
	// robot.goStraight(3);
	// robot.doRotation(PI/2);
	
	// robot.goStraight(3);
	// robot.doRotation(PI/2);
	
	// robot.goStraight(3);
	// robot.doRotation(PI/2);
	
	// robot.goStraight(3);

	robot.goStraight(0.5);
	robot.doRotation(PI/2);

	robot.goStraight(0.5);
	robot.doRotation(PI/2);

	robot.goStraight(0.5);
	robot.doRotation(PI/2);

	robot.goStraight(0.5);
	robot.doRotation(PI/2);



	// robot.goStraight(3);
	// robot.doRotation(PI/2);

	// robot.goStraight(3);
	// robot.doRotation(PI/2);

	// robot.goStraight(3);
	// robot.doRotation(PI/2);

	// robot.goStraight(3);
	// robot.doRotation(PI/2);


	// robot.doRotation(-PI/2);
	// robot.doRotation(PI/2);
	// robot.doRotation(-PI/2);
	// robot.doRotation(PI/2);
	// robot.doRotation(-PI/2);
	// robot.doRotation(PI/2);
	// robot.doRotation(-PI/2);
	// robot.doRotation(PI/2);
	// robot.doRotation(-PI/2);
	// robot.doRotation(PI/2);
	// robot.doRotation(-PI/2);
	// robot.doRotation(PI/2);
	// robot.doRotation(-PI/2);
	// robot.doRotation(PI/2);
	// robot.doRotation(-PI/2);

	usleep(25*1000);
}
