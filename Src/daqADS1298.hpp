#ifndef DAQADS1298_H
#define DAQADS1298_H
#include "daq.hpp"

//Global
const int ADS1298_Nbytes = 9; //3 header bytes + 2 channel
const int ADS1298_Nchannels = 3;

// register commands
static uint8_t ADS1298_WREG = 0x40;
static uint8_t ADS1298_RREG  = 0x20;
static uint8_t ADS1298_RDATAC = 0x10;
static uint8_t ADS1298_SDATAC = 0x11;
static uint8_t ADS1298_RDATA = 0x12;


static int32_t bufferADS1298[ADS1298_Nchannels];

class DaqADS1298: public Daq
{
public:
    DaqADS1298();
    void setup();
		void initGPIO();
		void getData();
    int getDrdyPin(void);
    int getMisoPin();
    int getMosiPin();
    int getSclkPin();
    int getFclk(void);
    void setFsFromCfg();
    int getNbytes();
    int getNchans();

private:
    void writeReg(uint8_t address, uint8_t data);
		void sendCmd(uint8_t cmd);
		int32_t uint32Toint32(uint32_t in);
    uint8_t readReg(uint8_t address);
    void printRegs();
		myGPIO DRDY;
		myGPIO nRESET;
		myGPIO START;
		myGPIO nCS;
    uint8_t cfg[25];
		int countSamples;

    int fclk; //2.048MHz (max 20MHz)
};

#endif // DAQADS1298_H
