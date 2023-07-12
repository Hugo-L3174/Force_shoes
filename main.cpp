#include <stdio.h>		// Needed for printf etc
#include <objbase.h>	// Needed for COM functionality
#include "cmt3.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include <conio.h>		// included for _getch and _kbhit
#include "main.h"


// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

using namespace xsens;

// used to signal that the user initiated the exit, so we do not wait for an extra keypress-
int userQuit = 0;
CmtOutputMode mode;
CmtOutputSettings settings;
unsigned long mtCount = 0;
int screenSensorOffset = 0;
int temperatureOffset = 0;
CmtDeviceId deviceIds[256];
xsens::Cmt3 cmt3;



int main(void)
{

	XsensResultValue res = XRV_OK;
	short screenSkipFactor = 10;
	short screenSkipFactorCnt = screenSkipFactor;
	
	// Set exit function
	atexit(exitFunc);

	// Perform hardware scan
	doHardwareScan();

	// Give user a (short) chance to see hardware scan results
	Sleep(2000);
	
	//clear screen present & get the user output mode selection.
	clrscr();
	getUserInputs();

	// Set device to user input settings
	doMtSettings();
	
	// Wait for first data item(s) to arrive. In production code, you would use a callback function instead (see cmtRegisterCallback function)
	Sleep(20);

	//get the placement offsets, clear the screen and write the fixed headers.
	calcScreenOffset();
	clrscr();
	writeHeaders();

	// vars for sample counter & temp.
	unsigned short sdata = NULL;
	double tdata;
	
	//structs to hold data.
	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;
	CmtwRaw raw_force;
	Message message;

	// Initialize packet for data
	Packet* packet = new Packet((unsigned short)mtCount,cmt3.isXm());
	

	while(!userQuit && res == XRV_OK && sdata < 20)
	{
		
		cmt3.waitForDataMessage(packet);

		//get sample count, goto position & display.
		sdata = packet->getSampleCounter();
		//gotoxy(0,0);
		printf("Sample Counter %05hu\n", sdata);
		printf("number of data items in the message: %d\n", packet->m_itemCount);
		printf("individual packet data size (according to outputmode) is : %d\n", packet->getDataSize());

		printf("sample counter data (2bytes) is at position: %d\n", packet->getInfoList().m_sc);
		//printf("raw inertial data (20bytes) position in individual packet 1 is : %d\n", packet->getInfoList().m_rawData);
		//printf("auxiliary data (4bytes) position in individual packet 1 is : %d\n", packet->getInfoList().m_analogIn1);
		//printf("raw force 1 : G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d \n", 
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 20),
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 22), 
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 24),
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 26), 
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 28), 
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 30),
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 32),
		//	packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 34));
		////packet->m_msg.getDataShort();
		//printf("raw byte data: %x %x\n", packet->m_msg.getDataByte(packet->getInfoList(0).m_rawData + 20), packet->m_msg.getDataByte(packet->getInfoList(0).m_rawData + 21));
		//printf("raw temp data is: %d \n", packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 18));
		//printf("real raw temp data is: %d \n", packet->m_msg.getDataShort(packet->getInfoList(0).m_rawTemp));
		//printf("raw temp data from function is: %d \n", packet->getRawTemp());

		//printf("raw mag z data is: %d \n", packet->m_msg.getDataShort(packet->getInfoList(0).m_rawData + 16));
		//printf("raw mag z data is: %x %x \n", packet->m_msg.getDataByte(packet->getInfoList(0).m_rawData + 16), packet->m_msg.getDataByte(packet->getInfoList(0).m_rawData + 17));
		//


		//printf("raw inertial data (20bytes) position in individual packet 2 is : %d\n", packet->getInfoList(1).m_rawData);
		//printf("auxiliary data (4bytes) position in individual packet 2 is : %d\n", packet->getInfoList(1).m_analogIn1);
		//printf("raw force 2 : G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d", packet->m_msg.getDataShort(packet->getInfoList(1).m_rawData + 20));

		//printf("raw inertial data (20bytes) position in individual packet 3 is : %d\n", packet->getInfoList(2).m_rawData);
		//printf("auxiliary data (4bytes) position in individual packet 3 is : %d\n", packet->getInfoList(2).m_analogIn1);
		//printf("raw force 3 : G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d", packet->m_msg.getDataShort(packet->getInfoList(2).m_rawData + 20));

		//printf("raw inertial data (20bytes) position in individual packet 4 is : %d\n", packet->getInfoList(3).m_rawData);
		//printf("auxiliary data (4bytes) position in individual packet 4 is : %d\n", packet->getInfoList(3).m_analogIn1);
		//printf("raw force 4 : G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d", packet->m_msg.getDataShort(packet->getInfoList(3).m_rawData + 20));
		
		
		printf("calibrated data acc (3 floats: 4*3=12bytes) in packet 1 is at: %d\n", packet->getInfoList(0).m_calAcc);
		//printf("worth %f, %f, %f\n", packet->getCalAcc().m_data[0], packet->getCalAcc().m_data[1], packet->getCalAcc().m_data[2]);
		printf("calibrated data gyr (3 floats: 4*3=12bytes) in packet 1 is at: %d\n", packet->getInfoList(0).m_calGyr);
		printf("calibrated data mag (3 floats: 4*3=12bytes) in packet 1 is at: %d\n", packet->getInfoList(0).m_calMag);


		printf("raw force 1 should be at: %d and is worth G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d\n", 
			packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16, 
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 0),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 2),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 4),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 6),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 8),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 10),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 12),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 1 * 3 * 12 + 0 * 16 + 14));
		printf("raw force 2 should be at: %d and is worth G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d\n", 
			packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16,
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 0),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 2),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 4),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 6),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 8),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 10),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 12),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 2 * 3 * 12 + 1 * 16 + 14));
			
		printf("raw force 3 should be at: %d and is worth G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d\n", 
			packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16,
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 0),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 2),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 4),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 6),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 8),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 10),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 12),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 3 * 3 * 12 + 2 * 16 + 14));
		printf("raw force 4 should be at: %d and is worth G0: %d, G1: %d, G2: %d, G3: %d, G4: %d, G5: %d, G6: %d, ref: %d\n", 
			packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16,
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 0),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 2),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 4),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 6),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 8),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 10),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 12),
				packet->m_msg.getDataShort(packet->getInfoList(0).m_calAcc + 4 * 3 * 12 + 3 * 16 + 14));
		//printf("after force is a new cal acc packet? at %d and is worth %f\n", packet->getInfoList(0).m_calMag + 12 + 16, packet->m_msg.getDataFloat(packet->getInfoList(0).m_calMag + 12 + 16));
		//printf("worth %f, %f, %f\n", packet->getCalAcc(1).m_data[0], packet->getCalAcc(1).m_data[1], packet->getCalAcc(1).m_data[2]);

		printf("message size without headers is : %d\n", packet->m_msg.getDataSize());
		printf("real individual packet size is : %d\n", packet->m_msg.getDataSize()/4);
		printf("total message size is : %d\n\n", packet->m_msg.getTotalMessageSize());
		

		if (screenSkipFactorCnt++ == screenSkipFactor) {
			screenSkipFactorCnt = 0;

			for (unsigned int i = 0; i < mtCount; i++) {	
				


			}	
		}				

		if (_kbhit())
			userQuit = 1;
	}

	delete packet;

	//clrscr();
	cmt3.closePort();

	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as an int16_t (16 bits).
int16_t getDataSignShort(const uint16_t offset)
{
	int16_t ret;
	int8_t* dest = (int8_t*)&ret;
	int8_t* src = 0/*&(getDataStart()[offset]) */ ;
	dest[0] = src[1];
	dest[1] = src[0];

	return ret;
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
void doHardwareScan()
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;

	printf("Scanning for connected Xsens devices...");
	xsens::cmtScanPorts(portInfo);
	portCount = portInfo.length();
	printf("done\n");

	if (portCount == 0) {
		printf("No Xsens devices found\n\n");
		exit(0);
	}

	for(int i = 0; i < (int)portCount; i++) {	
		printf("Using COM port %d at %d baud\n\n",
			(long) portInfo[i].m_portNr, portInfo[i].m_baudrate);	
	}

	printf("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmt3.openPort(portInfo[p].m_portNr, portInfo[p].m_baudrate);
		EXIT_ON_ERROR(res,"cmtOpenPort");  
	}
	printf("done\n\n");

	//get the Mt sensor count.
	printf("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	mtCount = cmt3.getMtCount();
	mtCount = mtCount;
	printf("MotionTracker count: %i\n\n",mtCount);

	// retrieve the device IDs 
	printf("Retrieving MotionTrackers device ID(s)\n");
	for(unsigned int j = 0; j < mtCount; j++ ){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		printf("Device ID at busId %i: %08x\n",j+1,(long) deviceIds[j]);
	}	
}

//////////////////////////////////////////////////////////////////////////
// getUserInputs
//
// Request user for output data
void getUserInputs()
{
	mode = 7;
	//do{
	//	printf("Select desired output:\n");
	//	printf("1 - Calibrated data\n");
	//	printf("2 - Orientation data\n");
	//	printf("3 - Both Calibrated and Orientation data\n");
	//	printf("4 - Temperature and Calibrated data\n");
	//	printf("5 - Temperature and Orientation data\n");
	//	printf("6 - Temperature, Calibrated and Orientation data\n");
	//	printf("Enter your choice: ");
	//	scanf_s("%d", &mode);
	//	// flush stdin
	//	while (getchar() != '\n') continue;

	//	if (mode < 1 || mode > 6) {
	//		printf("\n\nPlease enter a valid output mode\n");
	//	}
	//}while(mode < 1 || mode > 6);
	//clrscr();

	switch(mode)
	{
	case 1:
		mode = CMT_OUTPUTMODE_CALIB;
		break;
	case 2:
		mode = CMT_OUTPUTMODE_ORIENT;
		break;
	case 3:
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		break;
	case 4:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB;
		break;
	case 5:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_ORIENT;
		break;
	case 6:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		break;
	case 7:
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_AUXILIARY /* | CMT_OUTPUTMODE_CALIB*/;
		break;
	}

	if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
		do{
			printf("Select desired output format\n");
			printf("1 - Quaternions\n");
			printf("2 - Euler angles\n");
			printf("3 - Matrix\n");
			printf("Enter your choice: ");
			scanf_s("%d", &settings);
			// flush stdin
			while (getchar() != '\n') continue;

			if (settings < 1  || settings > 3) {
				printf("\n\nPlease enter a valid choice\n");
			}
		}while(settings < 1 || settings > 3);

		// Update outputSettings to match data specs of SetOutputSettings
		switch(settings) {
		case 1:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
			break;
		case 2:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
			break;
		case 3:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX;
			break;
		}
	}
	else if ((mode & CMT_OUTPUTMODE_AUXILIARY) != 0)
	{
		settings = CMT_OUTPUTSETTINGS_AUXILIARYMODE_FORCE;
		settings = 0x00001c00;
		//settings = CMT_OUTPUTSETTINGS_AUXILIARYMODE_MASK;
		//settings = 0x00000000;
		printf("Auxiliary mode, sending force info \n", settings);
	}
	else{
		settings = 0;
	}
	settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;

}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized cmt3 class with open COM port
void doMtSettings(void) 
{
	XsensResultValue res;

	//res = cmt3.getProcessingFlags();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	EXIT_ON_ERROR(res,"gotoConfig");

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	printf("Configuring your mode selectio\n");
	CmtDeviceMode deviceMode(mode, settings, sampleFreq);
	for(unsigned int i = 0; i < mtCount; i++){
		res = cmt3.setDeviceMode(deviceMode,true, deviceIds[i]);
		EXIT_ON_ERROR(res,"setDeviceMode");
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	EXIT_ON_ERROR(res,"gotoMeasurement");
}

//////////////////////////////////////////////////////////////////////////
// writeHeaders
//
// Write appropriate headers to screen
void writeHeaders()
{
	for (unsigned int i = 0; i < mtCount; i++) {	
		gotoxy(0, 2 + i * screenSensorOffset);
		printf("MotionTracker %d\n", i + 1);

		if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {
			temperatureOffset = 3;
			gotoxy(0,3 + i * screenSensorOffset);
			printf("Temperature");
			gotoxy(7,4 + i * screenSensorOffset);
			printf("degrees celcius");
			gotoxy(0,6 + i * screenSensorOffset);
		}

		if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {
			gotoxy(0,3 + temperatureOffset + i * screenSensorOffset);
			printf("Calibrated sensor data");
			gotoxy(0,4 + temperatureOffset + i * screenSensorOffset);
			printf(" Acc X\t Acc Y\t Acc Z");
			gotoxy(23, 5 + temperatureOffset + i * screenSensorOffset);
			printf("(m/s^2)");
			gotoxy(0,6 + temperatureOffset + i * screenSensorOffset);
			printf(" Gyr X\t Gyr Y\t Gyr Z");
			gotoxy(23, 7 + temperatureOffset + i * screenSensorOffset);
			printf("(rad/s)");
			gotoxy(0,8 + temperatureOffset + i * screenSensorOffset);
			printf(" Mag X\t Mag Y\t Mag Z");
			gotoxy(23, 9 + temperatureOffset + i * screenSensorOffset);
			printf("(a.u.)");
			gotoxy(0,11 + temperatureOffset + i * screenSensorOffset);
		}

		if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
			printf("Orientation data\n");
			switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
			case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				printf("    q0\t    q1\t    q2\t    q3\n");
				break;
			case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				printf("  Roll\t Pitch\t   Yaw\n");
				printf("                       degrees\n");
				break;
			case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				printf(" Matrix\n");
				break;
			default:
				;
			}			
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// calcScreenOffset
//
// Calculates offset for screen data with multiple sensors.
void calcScreenOffset()
{
	// 1 line for "Sensor ..."
	screenSensorOffset += 1;
	if ((mode & CMT_OUTPUTMODE_TEMP) != 0)
		screenSensorOffset += 3;
	if ((mode & CMT_OUTPUTMODE_CALIB) != 0)
		screenSensorOffset += 8;
	if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
		switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
		case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
			screenSensorOffset += 4;
			break;
		case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
			screenSensorOffset += 4;
			break;
		case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
			screenSensorOffset += 6;
			break;
		default:
			;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// clrscr
//
// Clear console screen
void clrscr() 
{
#ifdef WIN32
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coord = {0, 0};
	DWORD count;

	GetConsoleScreenBufferInfo(hStdOut, &csbi);
	FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y, coord, &count);
	SetConsoleCursorPosition(hStdOut, coord);
#else
	int i;

	for (i = 0; i < 100; i++)
		// Insert new lines to create a blank screen
		putchar('\n');
	gotoxy(0,0);
#endif
}

//////////////////////////////////////////////////////////////////////////
// gotoxy
//
// Sets the cursor position at the specified console position
//
// Input
//	 x	: New horizontal cursor position
//   y	: New vertical cursor position
void gotoxy(int x, int y)
{
#ifdef WIN32
	COORD coord;
	coord.X = x;
	coord.Y = y;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
#else
	char essq[100];		// String variable to hold the escape sequence
	char xstr[100];		// Strings to hold the x and y coordinates
	char ystr[100];		// Escape sequences must be built with characters

	/*
	** Convert the screen coordinates to strings
	*/
	sprintf(xstr, "%d", x);
	sprintf(ystr, "%d", y);

	/*
	** Build the escape sequence (vertical move)
	*/
	essq[0] = '\0';
	strcat(essq, "\033[");
	strcat(essq, ystr);

	/*
	** Described in man terminfo as vpa=\E[%p1%dd
	** Vertical position absolute
	*/
	strcat(essq, "d");

	/*
	** Horizontal move
	** Horizontal position absolute
	*/
	strcat(essq, "\033[");
	strcat(essq, xstr);
	// Described in man terminfo as hpa=\E[%p1%dG
	strcat(essq, "G");

	/*
	** Execute the escape sequence
	** This will move the cursor to x, y
	*/
	printf("%s", essq);
#endif
}

//////////////////////////////////////////////////////////////////////////
// exitFunc
//
// Closes cmt nicely
void exitFunc(void)
{
	// Close any open COM ports
	cmt3.closePort();
	
	// get rid of keystrokes before we post our message
	while (_kbhit()) _getch();

	// wait for a keypress
	if (!userQuit)
	{
		printf("Press a key to exit\n");
		_getch();
	}
}
