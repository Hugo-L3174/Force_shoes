void doHardwareScan();
void doMtSettings(void);
void getUserInputs(void);
void calcScreenOffset(void);
void writeHeaders(void);
void clrscr(void);
void gotoxy(int x, int y);
void exitFunc(void);
int16_t getDataSignShort(const uint16_t offset = 0);

struct CmtwRaw
{
	uint16_t G0;
	uint16_t G1;
	uint16_t G2;
	uint16_t G3;
	uint16_t G4;
	uint16_t G5;
	uint16_t G6;
	uint16_t ref;

};