#include "CKobuki.h"
#include <iostream>
#include <cmath>

int prevLeftEncoder, prevRightEncoder; // [ticks]
uint16_t prevTimestamp;// [ms]
long totalLeft, totalRight = 0;
int directionL = 0; // 1 = forward, 0 = undefined, -1 = backwards
int directionR = 0;
int iterationCount = 0;
long double tickToMeter = 0.000085292090497737556558; // [m/tick]

long double x = 0; // [m]
long double y = 0;
long double theta = 0; // [rad]
long double b = 0.23;

long demoCallback(void *user_data, TKobukiData &Kobuki_data) {
	/*std::cout << "Encoder left: " << Kobuki_data.EncoderLeft << std::endl
	<< "Encoder right: " << Kobuki_data.EncoderRight << std::endl
	<< "Direction left: " << directionL << std::endl
	<< "Direction right: " << directionR << std::endl;
	*/
	if (iterationCount == 0) {
		prevLeftEncoder = Kobuki_data.EncoderLeft;
                prevRightEncoder = Kobuki_data.EncoderRight;
		prevTimestamp = Kobuki_data.timestamp;
		iterationCount++;
		return 0;
	}
	
	int dLeft;
	if (abs(Kobuki_data.EncoderLeft - prevLeftEncoder) > 32000) {
		dLeft = Kobuki_data.EncoderLeft - prevLeftEncoder + (Kobuki_data.EncoderLeft > prevLeftEncoder ? -65536 : +65536);	
		//std::cout << "Hybem sa rovno, hovno!" << std::endl;
	}
	else {
		dLeft = Kobuki_data.EncoderLeft - prevLeftEncoder;
	}

        int dRight;
        if (abs(Kobuki_data.EncoderRight - prevRightEncoder) > 32000) {
                dRight = Kobuki_data.EncoderRight - prevRightEncoder + (Kobuki_data.EncoderRight > prevRightEncoder ? -65536 : +65536);
        }
        else {
                dRight = Kobuki_data.EncoderRight - prevRightEncoder;
        }



	//int dRight =  Kobuki_data.EncoderRight - prevRightEncoder;
	uint16_t dTimestamp = Kobuki_data.timestamp - prevTimestamp; 


	//std::cout << "dLeft:" << dLeft << " dRight:" << dRight << std::endl;

	long double mLeft = dLeft*tickToMeter;
	long double mRight = dRight*tickToMeter;

	if (mLeft == mRight) {
		x = x + mRight;
	} else {
		theta = (mRight-mLeft)/b + theta;
		x = x + (b*(mRight+mLeft))/(2*(mRight-mLeft))*(sin((mRight-mLeft)/b + theta) - sin(theta));
		y = y + (b*(mRight+mLeft))/(2*(mRight-mLeft))*(cos((mRight-mLeft)/b + theta) - cos(theta));

		std::cout << "X: " << x << " Y:"<< y << " theta:" << theta << std::endl;
	}


	//double speedL = dLeft*tickToMeter/dTimestamp; // [m/ms]
	//double speedR = dRight*tickToMeter/dTimestamp;

	//std::cout /*<< "Current speeds are:" << std::endl*/ << "left:" << speedL << "right: " << speedR << std::endl;

	//std::cout << "dLeft:" << dLeft << " dRight:" << dRight << " dTimestamp:" << dTimestamp << std::endl;

	totalLeft +=dLeft;
	totalRight +=dRight;

//	std::cout << "totalLeft:" << totalLeft << " totalRight:" << totalRight << std::endl;
	// ak je suma novej a predchadzajucej vacsia ako 65536 tak to pretieklo?
	directionL = (prevLeftEncoder < Kobuki_data.EncoderLeft ? 1 : -1);
	directionR = (prevRightEncoder < Kobuki_data.EncoderRight ? 1 : -1);
	dTimestamp = (Kobuki_data.timestamp < prevTimestamp ? prevTimestamp - Kobuki_data.timestamp + 65536 : dTimestamp);
	
//	std::cout << "Smery otocenia: left:" << directionL << " right:" << directionR << std::endl;

	prevLeftEncoder = Kobuki_data.EncoderLeft;
	prevRightEncoder = Kobuki_data.EncoderRight;
	prevTimestamp = Kobuki_data.timestamp;

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
    sleep(2*1000*1000);
	*/

	/*for (int i = 0; i < 16; i++) {
		// negative = CW, positive = CCW
		robot.setRotationSpeed(3.14159/2.0*1.11);
		usleep(1*1000*1000);
		std::cout << "otoceu som sa " << i << "-ty raz"  << std::endl;
	}*/

	// positive = FORWARD, negative = BACKWARD, max = 700
	
	//double distance = totalLeft*tickToMeter;
	int speed = 20;
	
	while (fabs(x) < 1 ||  fabs(y) < 1) {
		//robot.setTranslationSpeed(speed);
		robot.setArcSpeed(speed, 1200);
		usleep(25*1000);
		
		//distance = totalLeft*tickToMeter;
//		std::cout << "Left wheen actual distance:" << distance << std::endl;
		if (speed < 500) {
			speed += 20;
		//	std::cout << "Current speeed:" << speed << std::endl;
		}
	}
	
	/*std::cout<< "Kolesa presli realne v metroch:" << std::endl
	<< "lave: " << totalLeft*tickToMeter << std::endl
	<< "prave: " << totalRight*tickToMeter << std::endl;
	*/
	robot.setTranslationSpeed(0);

}
