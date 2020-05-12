/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <memory>
#include "generic_driver.hpp"
#include "pin_data.hpp"

class Sensor { //generyczna, abstrakcyjna klasa sensora
protected:
	std::unique_ptr<GenericDriver> driver;
public:
	Sensor() {};
	virtual void startNewReadout(void) = 0;
};

#endif
