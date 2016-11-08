//#pragma once
////*************************************************************************************
////*************************************************************************************
//// autor Martin Dekan  mail: dekdekan@gmail.com
////-------------------------------------------------------------------------------------
//// co to je:
//// trieda na pracu s robotom kobuki. mala by mat implementovane citanie dat
//// a posielanie prikazov...
//// neobsahuje ziadnu logiku co s datami robit, to je na userovi aby spravil v callback funkcii
////*************************************************************************************
////*************************************************************************************
#ifndef KOBUKI_CLASS_123456789
#define KOBUKI_CLASS_123456789

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "pthread.h"
#include "unistd.h"
#include "fcntl.h"
#include "string.h"
#include <math.h>
#include <stdint.h>
#include <iostream>

typedef struct
{
	
	unsigned short x;
	unsigned short y;
	unsigned short z;
}TRawGyroData;
typedef struct
{
	//Hardware Version
	unsigned char HardwareVersionMajor;
	unsigned char HardwareVersionMinor;
	unsigned char HardwareVersionPatch;
	//Firmware Version
	unsigned char FirmwareVersionMajor;
	unsigned char FirmwareVersionMinor;
	unsigned char FirmwareVersionPatch;

	//Unique Device IDentifier(UDID)
	unsigned int UDID0;
	unsigned int UDID1;
	unsigned int UDID2;
	//Controller Info
	unsigned char PIDtype;
	unsigned int PIDgainP;
	unsigned int PIDgainI;
	unsigned int PIDgainD;
}TExtraRequestData;

typedef struct
{
	//---zakladny balik
	unsigned short timestamp;
	//narazniky
	bool BumperLeft;
	bool BumperCenter;
	bool BumperRight;
	//cliff
	bool CliffLeft;
	bool CliffCenter;
	bool CliffRight;
	// padnutie kolies
	bool WheelDropLeft;  
	bool WheelDropRight; 
	//tocenie kolies
	unsigned short EncoderRight;
	unsigned short EncoderLeft;
	unsigned char PWMright;
	unsigned char PWMleft;
	//gombiky
	unsigned char ButtonPress;// 0 nie, 1 2 4 pre button 0 1 2 (7 je ze vsetky tri)
	//napajanie
	unsigned char Charger;
	unsigned char Battery;
	unsigned char overCurrent;
	//---docking ir
	unsigned char IRSensorRight;
	unsigned char IRSensorCenter;
	unsigned char IRSensorLeft;
	//---Inertial Sensor Data
	unsigned short GyroAngle;
	unsigned short GyroAngleRate;
	//---Cliff Sensor Data
	unsigned short CliffSensorRight;
	unsigned short CliffSensorCenter;
	unsigned short CliffSensorLeft;
	//---Current
	unsigned char wheelCurrentLeft;
	unsigned char wheelCurrentRight;
	//---Raw Data Of 3D Gyro
	unsigned char frameId;
	std::vector<TRawGyroData> gyroData;
	//---General Purpose Input
	unsigned short digitalInput;
	unsigned short analogInputCh0;
	unsigned short analogInputCh1;
	unsigned short analogInputCh2;
	unsigned short analogInputCh3;
	//---struktura s datami ktore sa nam tam objavia iba na poziadanie
	TExtraRequestData extraInfo;
}TKobukiData;


typedef long(*src_callback_kobuki_data) (void *user_data, TKobukiData &Kobuki_data);

class CKobuki
{
public:
	CKobuki() { stopVlakno = 0; };
	 virtual ~CKobuki() { 
		stopVlakno = 1; 
	 	close(HCom);
		pthread_cancel(threadHandle); 
	};
	
	void enableCommands(bool commands) {
		enabledCommands = commands;
	};
	void startCommunication(char *portname,bool CommandsEnabled,src_callback_kobuki_data userCallback,void *userDataL);
	int measure(); //vlaknova funkcia, ma v sebe nekonecne vlakno a vycitava udaje
	void setLed(int led1 = 0, int led2 = 0); //led1 zelena/cervena 2/1, //led2 zelena/cervena 2/1
	void setTranslationSpeed(int mmpersec);
	void setRotationSpeed(double radpersec);
	void setArcSpeed(int mmpersec,int radius);
	void setSound(int noteinHz, int duration);
	void setPower(int value);
private:
	int HCom;
	pthread_t threadHandle; // handle na vlakno
	int threadID;  // id vlakna
	int stopVlakno;
	TKobukiData data;
	src_callback_kobuki_data callbackFunction;
	void *userData;
	bool enabledCommands;
	int parseKobukiMessage(TKobukiData &output, unsigned char *data );
	int connect(char *portname);
	unsigned char *readKobukiMessage();
	int checkChecksum(unsigned char *data);
	
	//--spustenie merania v novom vlakne (vycitavanie bezi v novom vlakne. treba ho stopnut ak chceme poslat request)
	static void * KobukiProcess(void *param)
	{
		//std::cout << "Nase vlakno Kobuki process nastartovalo" << std::endl;
		CKobuki *hoku = (CKobuki*)param;
		int vystup = hoku->measure();
		
		return param;
	}


};

#endif
