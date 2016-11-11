#define PI          3.141592653589793238462643383279502884L /* pi */
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

// forward declarations
void rotateByTheta(CKobuki &robot, long double th);




long demoCallback(void *user_data, TKobukiData &Kobuki_data) {

	//std::cout << "callback!" << std::endl;
	std::cout <<  Kobuki_data.EncoderLeft << " " << Kobuki_data.EncoderRight << " " << Kobuki_data.EncoderLeft - Kobuki_data.EncoderRight << " " << theta<< std::endl;
	//<< "Direction left: " << directionL << std::endl
	//<< "Direction right: " << directionR << std::endl;
	
	if (iterationCount == 0) {
		prevLeftEncoder = Kobuki_data.EncoderLeft;
                prevRightEncoder = Kobuki_data.EncoderRight;
		prevTimestamp = Kobuki_data.timestamp;
		iterationCount++;
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

	//	std::cout << "X: " << x << " Y:"<< y << " theta:" << theta << std::endl;
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

// in meters
void goStraight(CKobuki &robot, long double distance){
	long double u_translation = 0; // riadena velicina, rychlost robota pri pohybe
	long double w_translation = distance; // pozadovana hodnota
	long double Kp_translation = 1600; 
	long double e_translation = 0;	
	int upper_thresh_translation = 500;
	int lower_thresh_translation = 40;
	int translation_start_gain = 20;

	x = 0;
	y = 0;
	theta = 0;

	long i = 1;

	while (fabs(x - w_translation) > 0.005 && x<w_translation) {
		e_translation = w_translation - x;
		u_translation = Kp_translation * e_translation;

		// limit translation speed
		if (u_translation > upper_thresh_translation) 
			u_translation = upper_thresh_translation;
		if (u_translation < lower_thresh_translation) 
			u_translation = lower_thresh_translation;
		
		// rewrite starting speed with line
		if (i < u_translation) {
			u_translation = i;
		}

		robot.setTranslationSpeed(u_translation);
		
		usleep(25*1000);
		// increment starting speed
		i = i + translation_start_gain;
	}		
	robot.setTranslationSpeed(0);
}



// in meters
void goToXy(CKobuki &robot,long double xx, long double yy) {
	long double th;
	
	yy = yy*-1;
	
	th = atan2(yy,xx);
	rotateByTheta(robot, th);

	long double s = sqrt(pow(xx,2)+pow(yy,2));

	// resetnem suradnicovu sustavu robota
	x = 0;
	y = 0;
	iterationCount = 0;
	theta = 0;	

	//std::cout << "mam prejst: " << s << "[m]" << std::endl;
	
	goStraight(robot,s);

	usleep(25*1000);
	return;
}


void doRotation(CKobuki &robot, long double th) {
 	long double u = 0; // riadena velicina, uhlova rychlost robota pri pohybe
        long double w = th; // pozadovana hodnota v uhloch
        long double Kp = PI/4;
        long double e = 0;
        int thresh = PI;

        theta = 0;
	x = 0;
	y = 0;

        long double i = 0;

	if (th > 0) {
		while (theta < w) {
		//	std::cout << "pozadovana: " << th << "aktualna: " << theta << std::endl;
			e = w - theta;
			u = Kp*e;

			if (u > thresh) u = thresh;
			if (u < 0.3) u = 0.3;

			if (i < u) {
				u = i;
			}

			std::cout << "Theta: " << theta << std::endl;

			robot.setRotationSpeed(u);
			usleep(25*1000);
			i = i + 0.1;
		}
	} 
	else {
		while (theta > w) {
		//      std::cout << "pozadovana: " << th << "aktualna: " << theta << std::endl;
			e = w - theta;
			u = Kp*e;

			if (u < -1*thresh) u = thresh*-1;
			if (u > -1*0.3) u = -1*0.3;

			if (i > u) {
				u = i;
			}

			std::cout << "Theta: " << theta << std::endl;

			robot.setRotationSpeed(u);
			usleep(25*1000);
			i = i - 0.1;
		}


	}
	robot.setRotationSpeed(0);
}



// in radians
void rotateByTheta(CKobuki &robot, long double th) {
	// resetnem rotacie robota
	theta = 0;
	if (th > 0) {
		std::cout << "Rotating CCW" << std::endl;
		while (th > theta) {
			robot.setRotationSpeed(1);
			usleep(25*1000);
		}	
		robot.setRotationSpeed(0);	

	} else if (th < 0) {
		std::cout << "Rotating CW" << std::endl;
		while (th < theta) {
			robot.setRotationSpeed(-1);
			usleep(25*1000);
		}
		robot.setRotationSpeed(0);
	}
	theta = 0;	
	usleep(25*1000);
	return;
}


int main() {
	unsigned char * null_ptr(0);
	CKobuki robot;

	robot.startCommunication("/dev/ttyUSB0", true, &demoCallback, null_ptr);

	robot.setLed(1,1);

	// positive = FORWARD, negative = BACKWARD, max = 700



/*	int speed = 20;
	while (fabs(x) < 1 ||  fabs(y) < 1) {
		robot.setArcSpeed(speed, 1200);
		usleep(25*1000);
		
		if (speed < 500) {
			speed += 20;
		}
	}
	
	robot.setTranslationSpeed(0);
*/


	//goStraight(robot,0.2);


	// CCW ked je kladne
	goStraight(robot,1);
	//doRotation(robot,-PI/2);
	//goStraight(robot,1);
	//doRotation(robot,-PI/2);
	//goStraight(robot,1);
	//doRotation(robot,-PI/2);
	//goStraight(robot,1);
	//doRotation(robot,-PI/2);


//	rotateByTheta(robot, 2*-PI);
//	goToXy(robot,1,0);
//	goToXy(robot,0,3);
//	goToXy(robot,0,3);
//	goToXy(robot,0,3);
	//usleep(1*1000*1000);

	//goToXy(robot,0.3,0);
	//rotateByTheta(robot, PI);



}
