#ifndef GENERIC_DRIVER_HPP
#define GENERIC_DRIVER_HPP

#include "pin_data.hpp"
#include "timer.hpp"
#include "device_manager.hpp" //musimy miec obiekt DMa zeby dostac pin i timer

extern DeviceManager deviceManager;

//class GenericDriver;
//typedef void (GenericDriver::*StateHandler)(void);

class GenericDriver { //klasa abstrakcyjna?
protected:
	PinData *pinData;
	Timer *timer;
//	StateHandler stateHandler; //TODO: ?stateHandler powinien nalezec do klady bazowej
public:
	GenericDriver(){}//TODO: NIE! a potrzebna dla generycznego sensora
	GenericDriver(PinData *pinData) {
		if(deviceManager.checkIfPinFree(pinData)){
			this->pinData = pinData;
		}
		else {
			this->pinData = deviceManager.getFreePin();
		}
		this->timer = deviceManager.getNewTimerHandle();
	};
	virtual void driverStartReadout(void) {};//= 0; //TODO: chyba nie abstrakcyjna
	virtual void executeState(void) {};//= 0;
};

#endif
