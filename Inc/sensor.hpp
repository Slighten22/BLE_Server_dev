/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "generic_driver.hpp"
#include "pin_data.hpp"

class Sensor { //generyczna klasa sensora, ?abstrakcyjna?
private:
	GenericDriver genericDriver;
//	GenericDriver *concreteDriver;
public:
//	Sensor(PinData *pinData);
//	virtual void startNewReadout(void) {}; //TODO: wirtualna?
};

#endif
