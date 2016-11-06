#include "CKobuki.h"
#include "termios.h"
#include "errno.h"

// obsluha tty pod unixom
int set_interface_attribs2 (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking2 (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}






int CKobuki::connect(char * comportT)
{
	HCom= open(comportT,O_RDWR|O_NOCTTY|O_NONBLOCK);

    if ( HCom== -1 )
    {
        printf("Kobuki nepripojeny\n");
        //  m_status="Chyba:  Port sa neda otvorit.";
        // potom nasleduje    Closeint(hCom);  a potom asi exit...
        return HCom;

    }
    else
    {
        set_interface_attribs2 (HCom, B57600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking2 (HCom, 0);                // set no blocking
      /*  struct termios settings;
        tcgetattr(HCom, &settings);

        cfsetospeed(&settings, B57600); // baud rate
        settings.c_cflag &= ~PARENB; // no parity
        settings.c_cflag &= ~CSTOPB; // 1 stop bit
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= CS8 | CLOCAL; // 8 bits
        settings.c_lflag &= ~ICANON; // canonical mode
        settings.c_cc[VTIME]=1;
        settings.c_oflag &= ~OPOST; // raw output

        tcsetattr(HCom, TCSANOW, &settings); // apply the settings*/
        tcflush(HCom, TCOFLUSH);


    /*	DCB PortDCB;
        PortDCB.DCBlength = sizeof(DCB);  // Inicializuj položku DCBlength
        GetCommState(HCom,&PortDCB);       // Naèítaj aktuálne nastavenia
        PortDCB.BaudRate=57600;
        PortDCB.ByteSize=8;
        PortDCB.Parity=0;
        SetCommState(HCom,&PortDCB);
        PurgeComm(HCom,PURGE_TXCLEAR | PURGE_RXCLEAR);
        COMMTIMEOUTS timeouts;

        timeouts.ReadIntervalTimeout         = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier  = 0;
        timeouts.ReadTotalTimeoutConstant    = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant    = 0;

        SetCommTimeouts(HCom,&timeouts);*/
        printf("Kobuki pripojeny\n");
         // usleep(100*1000);
        // SentToCreate(OI_START);
         // usleep(100*1000);
         // SentToCreate(OI_FULL);
         // usleep(100*1000);
        return HCom;
    }
}

unsigned char * CKobuki::readKobukiMessage()
{
	unsigned char buffer[1];
	uint32_t Pocet;
	buffer[0] = 0;
	//citame kym nezachytime zaciatok spravy
	do {
		Pocet=read(HCom,buffer,1);
		// ReadFile(hCom, buffer, 1, &Pocet, NULL);
	} while (buffer[0] != 0xAA);
	//mame zaciatok spravy (asi)
	if (Pocet == 1 && buffer[0] == 0xAA)
	{
		//citame dalsi byte
		do {
			Pocet=read(HCom,buffer,1);
			// ReadFile(hCom, buffer, 1, &Pocet, NULL);
		} while (Pocet == 0);
		//a ak je to druhy byte hlavicky
		if (Pocet == 1 && buffer[0] == 0x55)
		{
			// precitame dlzku
			Pocet=read(HCom,buffer,1);
			// ReadFile(hCom, buffer, 1, &Pocet, NULL);
			if (Pocet == 1)
			{			
				//mame dlzku.. nastavime vektor a precitame ho cely
				int readLenght = buffer[0];
				unsigned char *outputBuffer = (unsigned char*)calloc(readLenght+2,sizeof(char));
				outputBuffer[0] = buffer[0];
				int pct = 0;
				
				do
				{
					Pocet = 0;
					int readpoc = (readLenght + 1 - pct);
					Pocet=read(HCom,outputBuffer+1+pct,readpoc);
					// ReadFile(hCom, outputBuffer+1+pct, readpoc, &Pocet, NULL);
					pct = pct + Pocet;
				} while (pct != (readLenght + 1));
				return outputBuffer;
			}
		}
	}

	return nullptr;
}

int CKobuki::checkChecksum(unsigned char * data)
{//najprv hlavicku
	unsigned char chckSum = 0;
	for (int i = 0; i < data[0]+2; i++)
	{
		chckSum ^= data[i];
	}
	return chckSum;//0 ak je vsetko v poriadku,inak nejake cislo
}

void CKobuki::setLed(int led1, int led2)
{
	char message[8] = {0xaa,0x55,0x04,0x0c,0x02,0x00,(led1+led2*4)%256,0x00};
	message[7] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6];
	uint32_t pocet;
	pocet=write(HCom,&message,8);
	// WriteFile(hCom, message, 8, &pocet, NULL);
}

void CKobuki::setTranslationSpeed(int mmpersec)
{
	char message[10] = { 0xaa,0x55,0x06,0x01,0x04,mmpersec%256,mmpersec>>8,0x00,0x00,  0x00 };
	message[9] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8];
	
	uint32_t pocet;
	pocet=write(HCom,&message,10);
	// WriteFile(hCom, message, 10, &pocet, NULL);
}

void CKobuki::setRotationSpeed(double radpersec)
{
	int speedvalue = floor(radpersec * 230.0 / 2.0  +0.5);
	char message[10] = { 0xaa,0x55,0x06,0x01,0x04,speedvalue % 256,speedvalue >>8,0x01,0x00,  0x00 };
	message[9] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8];
	uint32_t pocet;
	pocet=write(HCom,&message,10);
	// WriteFile(hCom, message, 10, &pocet, NULL);
}

void CKobuki::setArcSpeed(int mmpersec, int radius)
{
	int speedvalue = mmpersec * ((radius + (radius>0? 230:-230) )/ 2 ) / radius;
	char message[10] = { 0xaa,0x55,0x06,0x01,0x04,speedvalue % 256,speedvalue >>8,radius % 256,radius >>8,  0x00 };
	message[9] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8];
	uint32_t pocet;
	pocet=write(HCom,&message,10);
	// WriteFile(hCom, message, 10, &pocet, NULL);
}

void CKobuki::setSound(int noteinHz, int duration)
{
	int notevalue = floor((double)1.0 / ((double)noteinHz*0.00000275) + 0.5);
	char message[9] = { 0xaa,0x55,0x05,0x03,0x03,notevalue%256,notevalue>>8,duration%256,0x00 };
	message[8] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7];
	//char message[] = { 0xaa,0x55,0x19,0x03,0x03,notevalue % 256,notevalue / 256,duration % 256,0x03,0x03,(notevalue+5) % 256,(notevalue+5) / 256,duration % 256,0x03,0x03,(notevalue + 10) % 256,(notevalue + 10) / 256,duration % 256,0x03,0x03,(notevalue + 15) % 256,(notevalue +15) / 256,duration % 256,0x03,0x03,(notevalue + 20) % 256,(notevalue + 20) / 256,duration % 256,0x00 };

	uint32_t pocet;
	pocet=write(HCom,&message,28);
	// WriteFile(hCom, message, 28, &pocet, NULL);
}




void CKobuki::startCommunication(char * portname, bool CommandsEnabled, src_callback_kobuki_data userCallback,  void *userDataL)
{
	connect(portname);
	enableCommands(CommandsEnabled);
	callbackFunction = userCallback;
	userData = userDataL;
	// threadHandle = CreateThread(NULL, 0, KobukiProcess, (void *)this, 0, &threadID);



    threadID = pthread_create(&threadHandle,NULL,&KobukiProcess,(void*)this);

}


int CKobuki::measure()
{

	while (stopVlakno==0)
	{
		
		
		unsigned char *message = readKobukiMessage();
		if (message == NULL)
		{
			printf("vratil null message\n");
			continue;
		}
		int ok=parseKobukiMessage(data,message);
		
		//maximalne moze trvat callback funkcia 20 ms, ak by trvala viac, nestihame citat
	
		if (ok == 0 && callbackFunction!=NULL)
		{
			callbackFunction(userData, data);
		}
		free(message);
	}
	return 0;
}

int CKobuki::parseKobukiMessage(TKobukiData &output, unsigned char * data)
{
	int rtrnvalue = checkChecksum(data);
	//ak je zly checksum,tak kaslat na to
	if (rtrnvalue != 0)
		return -1;

	int checkedValue = 1;
	//kym neprejdeme celu dlzku
	while (checkedValue < data[0])
	{
		//basic data subload
		if (data[checkedValue] == 0x01)
		{
			checkedValue++;
			if (data[checkedValue ] != 0x0F)
				return -1;
			checkedValue++;
			output.timestamp = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.BumperCenter = data[checkedValue ] && 0x02;
			output.BumperLeft = data[checkedValue] && 0x04;
			output.BumperRight = data[checkedValue] && 0x01;
			checkedValue++;
			output.WheelDropLeft= data[checkedValue] && 0x02;
			output.WheelDropRight = data[checkedValue] && 0x01;
			checkedValue++;
			output.CliffCenter = data[checkedValue] && 0x02;
			output.CliffLeft = data[checkedValue] && 0x04;
			output.CliffRight = data[checkedValue] && 0x01;
			checkedValue++;
			output.EncoderLeft = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.EncoderRight = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.PWMleft = data[checkedValue] ;
			checkedValue++;
			output.PWMright = data[checkedValue] ;
			checkedValue++;
			output.ButtonPress = data[checkedValue];
			checkedValue++;
			output.Charger = data[checkedValue];
			checkedValue++;
			output.Battery = data[checkedValue];
			checkedValue++;
			output.overCurrent = data[checkedValue];
			checkedValue++;
		}
		else if (data[checkedValue] == 0x03)
		{
			checkedValue++;
			if (data[checkedValue] != 0x03)
				return -1;
			checkedValue++;
			output.IRSensorRight = data[checkedValue];
			checkedValue++;
			output.IRSensorCenter = data[checkedValue];
			checkedValue++;
			output.IRSensorLeft = data[checkedValue];
			checkedValue++;
		}
		else if (data[checkedValue] == 0x04)
		{
			checkedValue++;
			if (data[checkedValue] != 0x07)
				return -1;
			checkedValue++;
			output.GyroAngle = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.GyroAngleRate = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 5;//3 unsued
		}
		else if (data[checkedValue] == 0x05)
		{
			checkedValue++;
			if (data[checkedValue] != 0x06)
				return -1;
			checkedValue++;
			output.CliffSensorRight = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.CliffSensorCenter = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.CliffSensorLeft = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
		}
		else if (data[checkedValue] == 0x06)
		{
			checkedValue++;
			if (data[checkedValue] != 0x02)
				return -1;
			checkedValue++;
			output.wheelCurrentLeft =  data[checkedValue];
			checkedValue ++;
			output.wheelCurrentRight =data[checkedValue];
			checkedValue ++;
			
		}
		else if (data[checkedValue] == 0x0A)
		{
			checkedValue++;
			if (data[checkedValue] != 0x04)
				return -1;
			checkedValue++;
			output.extraInfo.HardwareVersionPatch = data[checkedValue];
			checkedValue++;
			output.extraInfo.HardwareVersionMinor = data[checkedValue];
			checkedValue++;
			output.extraInfo.HardwareVersionMajor = data[checkedValue];
			checkedValue += 2;

		}
		else if (data[checkedValue] == 0x0B)
		{
			checkedValue++;
			if (data[checkedValue] != 0x04)
				return -1;
			checkedValue++;
			output.extraInfo.FirmwareVersionPatch = data[checkedValue];
			checkedValue++;
			output.extraInfo.FirmwareVersionMinor = data[checkedValue];
			checkedValue++;
			output.extraInfo.FirmwareVersionMajor = data[checkedValue];
			checkedValue += 2;

		}
		else if (data[checkedValue] == 0x0D)
		{
			checkedValue++;
			if (data[checkedValue]%2 !=0)
				return -1;
			checkedValue++;
			output.frameId = data[checkedValue];
			checkedValue++;
			int howmanyFrames = data[checkedValue]/3;
			checkedValue++;
			output.gyroData.reserve(howmanyFrames);
			output.gyroData.clear();
			for (int hk = 0; hk < howmanyFrames; hk++)
			{
				TRawGyroData temp;
				temp.x = data[checkedValue + 1] * 256 + data[checkedValue];
				checkedValue += 2;
				temp.y = data[checkedValue + 1] * 256 + data[checkedValue];
				checkedValue += 2;
				temp.z = data[checkedValue + 1] * 256 + data[checkedValue];
				checkedValue += 2;
				output.gyroData.push_back(temp);
			}
		}
		else if (data[checkedValue] == 0x10)
		{
			checkedValue++;
			if (data[checkedValue] != 0x10)
				return -1;
			checkedValue++;
			output.digitalInput = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.analogInputCh0 = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.analogInputCh1 = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.analogInputCh2 = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 2;
			output.analogInputCh3 = data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 8;//2+6 
			

		}
		else if (data[checkedValue] == 0x13)
		{
			checkedValue++;
			if (data[checkedValue] != 0x0C)
				return -1;
			checkedValue++;
			output.extraInfo.UDID0 = data[checkedValue + 3] * 256*256*256+ data[checkedValue + 2] * 256*256+ data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 4;
			output.extraInfo.UDID1 = data[checkedValue + 3] * 256 * 256 * 256 + data[checkedValue + 2] * 256 * 256 +data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 4;
			output.extraInfo.UDID2 = data[checkedValue + 3] * 256 * 256 * 256 + data[checkedValue + 2] * 256 * 256 +data[checkedValue + 1] * 256 + data[checkedValue];
			checkedValue += 4;


		}
		else
		{
			checkedValue++;
			checkedValue += data[checkedValue] + 1;
		}
	}
	return 0;
}
