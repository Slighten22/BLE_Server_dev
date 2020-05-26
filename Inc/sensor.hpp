/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <memory>
#include "generic_driver.hpp"
#include "pin_data.hpp"

class Sensor { //generyczna klasa sensora
protected:
	std::shared_ptr<GenericDriver> driver;
//	std::unique_ptr<GenericDriver> driver;
	std::string name; //TODO: "kuchnia"
	uint16_t interval; //co ile czytac
public:
	//Sensor(PinData *pinData){} //TODO: konstruktor ustawiajacy drivera
	void startNewReadout(void){
		this->driver->driverStartReadout();
	}
};

#endif
