#include "generic_driver.hpp"

GenericDriver::GenericDriver(PinData pinData, uint16_t interval, char name[], uint8_t name_len){
	if(deviceManager.checkIfPinFree(pinData)){
		this->pinData = pinData;
	}
	else {
		this->pinData = deviceManager.getFreePin();
	}
	this->timer = deviceManager.getNewTimerHandle();

	this->interval = interval;
	for(int i=0; i<name_len; i++){
		this->name[i] = name[i];
	}
	for(int i=name_len; i<MAX_NAME_LEN; i++){
		this->name[i] = '\0';
	}
};
