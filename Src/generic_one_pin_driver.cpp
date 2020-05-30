#include <generic_one_pin_driver.hpp>

GenericOnePinDriver::GenericOnePinDriver(PinData pinData){
	if(deviceManager.checkIfPinFree(pinData)){
		this->pinData = pinData;
	}
	else {
		this->pinData = deviceManager.getFreePin();
	}
};
