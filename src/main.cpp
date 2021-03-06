#include "CKobuki.h"
#include <iostream>
#include <cmath>

#include "graph.h"

using namespace std;

int main() {
	unsigned char * null_ptr(0);
	CKobuki robot;

//    plot p;
//    for(int a=0;a<100;a++) {
//        vector<float> x, y;
//        for (int k = a; k < a + 200; k++) {
//            x.push_back(k);
//            y.push_back(k * k);
//        }
//        p.plot_data(x, y);
//    }

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



	robot.goStraight(0.3);
	robot.doRotation(PI/2);
	
	robot.goStraight(0.3);
	robot.doRotation(PI/2);

	robot.goStraight(0.10);
	robot.doRotation(PI/2);

	robot.goStraight(0.3);
	robot.doRotation(-PI/2);

	robot.goStraight(0.10);
	robot.doRotation(PI/2);

	robot.goStraight(0.10);
	robot.doRotation(PI/2);



	usleep(30*1000*1000);
}
