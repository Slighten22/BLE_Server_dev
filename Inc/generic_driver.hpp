#ifndef GENERIC_DRIVER_HPP
#define GENERIC_DRIVER_HPP

#include "pin_data.hpp"
#include "timer.hpp"
#include "device_manager.hpp" //musimy miec obiekt DMa zeby dostac pin i timer
#include <functional>

extern DeviceManager deviceManager;

class GenericDriver;
typedef void (GenericDriver::*StateHandler)(void);

class GenericDriver { //generyczna, abstrakcyjna klasa drivera
protected:
//	PinData *pinData;
	Timer *timer; //WSKAZNIK, a nie obiekt timera

	PinData pinData;
//	Timer timer;

	StateHandler stateHandler;
public:
//	GenericDriver(PinData *pinData) {
//		if(deviceManager.checkIfPinFree(pinData)){
//			this->pinData = pinData;
//		}
//		else {
//			this->pinData = deviceManager.getFreePin();
//		}
//		this->timer = deviceManager.getNewTimerHandle();
//	};


	GenericDriver(PinData pinData){  //TODO: check
		this->pinData = pinData; //TODO: sprawdzanie czy wolny pin
		this->timer = deviceManager.getNewTimerHandle();
		//this->timer = deviceManager.getNewTimer();
	}; //dotad jest ok


	virtual void driverStartReadout(void) = 0;
	virtual void executeState(void) = 0;
};

#endif
