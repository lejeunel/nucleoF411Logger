#include "daq.hpp"


int Daq::getChan(){return chan;}


int Daq::getFs(){
   return fs;
}

void Daq::setSpiHandle(SPI_HandleTypeDef * hspi){
	spiHandle = hspi;
}

