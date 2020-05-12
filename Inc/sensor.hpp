/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <memory>
#include "generic_driver.hpp"
#include "pin_data.hpp"

class Sensor { //generyczna klasa sensora
protected:
	std::unique_ptr<GenericDriver> driver;
public:
	void startNewReadout(void){
		this->driver->driverStartReadout();
	}
};

#endif
