#ifndef GENERIC_DRIVER_HPP
#define GENERIC_DRIVER_HPP

#include "pin_data.hpp"
#include "timer.hpp"
#include "device_manager.hpp" //musimy miec obiekt DMa zeby dostac pin i timer
#include <functional>

extern DeviceManager deviceManager;

class GenericOnePinDriver;
typedef void (GenericOnePinDriver::*StateHandler)(void);

class GenericOnePinDriver { //generyczna, abstrakcyjna klasa drivera dla sensorow wykorzystujacych 1 pin do komunikacji
protected:
	PinData pinData;
	StateHandler stateHandler;
public:
	GenericOnePinDriver(PinData pinData);
	virtual void startReadout(std::function<void(uint8_t *)> readoutFinishedHandler) = 0;
	virtual void executeState(void) = 0;
};

#endif
