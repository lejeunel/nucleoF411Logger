#ifndef DAQ_H
#define DAQ_H

#include <stdint.h>
#include <vector>
#include "stm32f4xx_hal.h"

struct myGPIO {
  GPIO_TypeDef * GPIOx;
  uint16_t GPIO_Pin;
} ;



class Daq
{

public:
    virtual void setup()=0;
	virtual void initGPIO()=0;
		virtual void getData() = 0;
    int getChan();
    int getFs();
void setSpiHandle(SPI_HandleTypeDef * hspi);
    virtual int getNbytes() = 0;
    virtual int getNchans() = 0;
		SPI_HandleTypeDef * spiHandle;
		uint32_t timeout;
		HAL_StatusTypeDef halStatus;

protected:
    int chan;
    void run();
    bool quit;
    void loadCfg();
    int fs;

};





#endif // DAQ_H
