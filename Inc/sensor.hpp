/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "generic_driver.hpp"
#include "pin_data.hpp"

class Sensor { //generyczna, abstrakcyjna klasa sensora
protected:
	GenericDriver *concreteDriver;
public:
	Sensor() {};
	virtual void startNewReadout(void) = 0; //czysto wirtualna
};

#endif
