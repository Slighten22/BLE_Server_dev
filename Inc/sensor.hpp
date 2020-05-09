/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "generic_driver.hpp"
#include "one_wire_driver.hpp"
#include "pin_data.hpp"

class Sensor { //albo OneWireDevice
private:
	OneWireDriver oneWireDriver; //TODO
//	GenericDriver genericDriver;
	float lastTemperatureValue;
	float lastHumidityValue;
public:
	Sensor(PinData *pinData); //i tutaj tez przydzielenie pierwszego wolnego timera
	void startNewReadout(void);
	float getLastTempVal(void);
	float getLastHumidVal(void);
};

#endif
