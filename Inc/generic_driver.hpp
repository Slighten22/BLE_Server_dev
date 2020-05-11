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
	PinData *pinData;
	Timer *timer;
	StateHandler stateHandler;
	virtual void firstStateHandler(void) = 0;
	virtual void secondStateHandler(void) = 0;
	virtual void thirdStateHandler(void) = 0;
public:
	GenericDriver(PinData *pinData) {
		if(deviceManager.checkIfPinFree(pinData)){
			this->pinData = pinData;
		}
		else {
			this->pinData = deviceManager.getFreePin();
		}
		this->timer = deviceManager.getNewTimerHandle();
	};
	virtual void driverStartReadout(void) = 0;
	virtual void executeState(void) = 0;
};

#endif
