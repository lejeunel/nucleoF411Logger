#include "daqADS1298.hpp"

//static uint8_t tmp[nSerialBytes];

DaqADS1298::DaqADS1298(){
	
		cfg[0] = 0x86; //CONFIG1 
		cfg[1] = 0x00; //CONFIG2 
		cfg[2] = 0x10; //CONFIG3 
		cfg[3] = 0x00; //LOFF 
		cfg[4] = 0x80; //CH1set 
		cfg[5] = 0x60; //
		cfg[6] = 0x60; //
		cfg[7] = 0x10; //
		cfg[8] = 0x10; //
		cfg[10] = 0x10; //
		cfg[11] = 0x10; //
		cfg[11] = 0x80; //CH8set 
		cfg[12] = 0x06; //RLD_SENSP 
		cfg[13] = 0x06; //RLD_SENSN
		cfg[14] = 0x00; //LOFF_SENSP
		cfg[15] = 0x00; //LOFF_SENSN
		cfg[16] = 0x00; //LOFF_FLIP
		cfg[17] = 0x00; //LOFF_STATP
		cfg[18] = 0x00; //LOFF_STATN
		cfg[19] = 0x00; //GPIO 
		cfg[20] = 0x00; //PACE 
		cfg[21] = 0x00; //RESP 
		cfg[22] = 0x00; //CONFIG4 
		cfg[23] = 0xEE; //WCT1 
		cfg[24] = 0xF6; //WCT2 
		
		countSamples = 0;
	
}

void DaqADS1298::writeReg(uint8_t address, uint8_t data)
{
    // ADS1298_SDATAC (stop read data continuous mode) default mode
		HAL_Delay(5); // milliseconds
		sendCmd (ADS1298_SDATAC);

    uint8_t spiDataWrite[3];
    spiDataWrite[0] = ADS1298_WREG+address;
    spiDataWrite[1] = 0x00;
    spiDataWrite[2] = data;
		HAL_SPI_Transmit (spiHandle, spiDataWrite, 3, 0x1000);
    HAL_Delay(5);
}


uint8_t DaqADS1298::readReg(uint8_t address)
{
		uint8_t dataIn[1];
		uint8_t dataOut[3] = {0};
		dataIn[0] = ADS1298_RREG+address;
		HAL_SPI_TransmitReceive (spiHandle, dataIn, dataOut, 3, 0x1000);
		return dataOut[2];

}

void DaqADS1298::sendCmd(uint8_t cmd){
		uint8_t tmp[1] = {0};
		// ADS1298_SDATAC (stop read data continuous mode) default mode
		HAL_SPI_TransmitReceive (spiHandle, &cmd,tmp, 1, 0x1000);
}

void DaqADS1298::initGPIO(){
	GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	START.GPIOx = GPIOC;
	START.GPIO_Pin = GPIO_PIN_6;

  /*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	DRDY.GPIOx = GPIOC;
	DRDY.GPIO_Pin = GPIO_PIN_7;
	HAL_NVIC_SetPriority (EXTI9_5_IRQn, 0, 0); //Set priority
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); //Enable interrupt

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	nRESET.GPIOx = GPIOB;
	nRESET.GPIO_Pin = GPIO_PIN_6;

}

void DaqADS1298::setup()
{
		
			/* Read ID register (testing) */
		
		//myHalStatus = HAL_SPI_TransmitReceive (spiHandle, dataIn, dataOut, 2, 0x10000);
		//myHalStatus = HAL_SPI_Transmit (spiHandle, dataIn, 1, 0x1000);
		//myHalStatus =  HAL_SPI_Receive(spiHandle, dataOut, 2, 0x1000);

		/* Power-up sequence */
		
		//pull nCS low permanently (single slave)
		//HAL_GPIO_WritePin (nCS.GPIOx, nCS.GPIO_Pin, GPIO_PIN_RESET);
		
    //pull START low
		HAL_GPIO_WritePin (START.GPIOx, START.GPIO_Pin, GPIO_PIN_RESET);
		
		//Toggle nRESET pin		
		HAL_GPIO_WritePin (nRESET.GPIOx, nRESET.GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(100); // milliseconds
		HAL_GPIO_WritePin (nRESET.GPIOx, nRESET.GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(1); // milliseconds
		HAL_GPIO_WritePin (nRESET.GPIOx, nRESET.GPIO_Pin, GPIO_PIN_SET);

		uint8_t tmp[1] = {0};
		// ADS1298_SDATAC (stop read data continuous mode) default mode
		sendCmd (ADS1298_SDATAC);
		
		/* Write config registers */
		int nRegs = sizeof(cfg)/sizeof(uint8_t);
		
		for(int i = 0;i<nRegs-1;i++){
			writeReg(i+1, cfg[i]);
		}
		
		/* Read ID register (testing) */
		uint8_t dataOut;
		dataOut = readReg(0);
		
		/* Read register (testing) */
		dataOut = readReg(3);
		
		/* start continuous acquisition */
		HAL_GPIO_WritePin (START.GPIOx, START.GPIO_Pin, GPIO_PIN_SET);
		sendCmd (ADS1298_RDATAC);
		HAL_Delay(3);
		
		/* stop continuous acquisition */
		HAL_GPIO_WritePin (START.GPIOx, START.GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(3);
			
}


void DaqADS1298::getData(){
	  //qDebug() << "getData ADS1298";
    //int chan = daqs[0]
    uint8_t tmp[ADS1298_Nbytes] = {0};
		uint8_t dataWrite[ADS1298_Nbytes] = {0};
    //    getWriteData(&(daqs[0]->myFile),8, 0, 27);
    HAL_SPI_TransmitReceive(spiHandle, tmp, dataWrite, ADS1298_Nbytes, 0x1000);

    // 3 bytes per channel
    for( int i=0; i < ADS1298_Nbytes/3; ++i ){
       bufferADS1298[i] =  uint32Toint32((dataWrite[3*i]<<16) + (dataWrite[3*i+1]<<8) + (dataWrite[3*i+2]));
    }

		countSamples++;
		// write samples
}

int32_t DaqADS1298::uint32Toint32(uint32_t in){
    if(in & 0x800000){
        in |= ~0xffffff;
    }

    return (int32_t)in;
}

void DaqADS1298::printRegs(){
    
}

int DaqADS1298::getDrdyPin(){
    return 0;
}

int DaqADS1298::getFclk(){
    return fclk;
}

int DaqADS1298::getMisoPin(){
    return 0;
}

int DaqADS1298::getMosiPin(){
    return 0;
}

int DaqADS1298::getSclkPin(){
    return 0;
}

int DaqADS1298::getNbytes(){
    return ADS1298_Nbytes;
}

void DaqADS1298::setFsFromCfg(){
    
}

int DaqADS1298::getNchans(){
   return ADS1298_Nchannels;
}
