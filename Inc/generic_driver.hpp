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
//protected:
public:
	Timer *timer; //wskaznik na timera od DeviceManagera
	PinData pinData;
	StateHandler stateHandler;
	char name[MAX_NAME_LEN];
	uint16_t interval;
	uint8_t bytesToRead; //?
public:
	GenericDriver(PinData pinData, uint16_t interval, char name[], uint8_t name_len);
	virtual void driverStartReadout(void) = 0;
	virtual void executeState(void) = 0;
};

#endif
