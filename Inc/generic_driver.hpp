#ifndef GENERIC_DRIVER_HPP
#define GENERIC_DRIVER_HPP

#include "pin_data.hpp"
#include "timer.hpp"

//class GenericDriver;
//typedef void (GenericDriver::*StateHandler)(void);

class GenericDriver { //klasa abstrakcyjna
protected:
	PinData *pinData;
	Timer *timer;
//	StateHandler stateHandler; //TODO: stateHandler powinien nalezec do klady bazowej
public:
	//konstruktor?
	virtual void driverStartReadout(void) {};
	virtual void executeState(void) {};
};

#endif
