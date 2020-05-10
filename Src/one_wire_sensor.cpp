#include "one_wire_sensor.hpp"

#include <functional>

OneWireSensor::OneWireSensor(PinData *pinData) : /*Sensor(pinData),*/ oneWireDriver(OneWireDriver(pinData)) {}

void OneWireSensor::startNewReadout(void){
	this->oneWireDriver.driverStartReadout();
}

float OneWireSensor::getLastTempVal(void){
	return this->lastTemperatureValue;
}

float OneWireSensor::getLastHumidVal(void){
	return this->lastHumidityValue;
}

#ifdef __cplusplus
template <typename T>
inline T max(T a, T b) {
    return a > b ? a : b;
}
#endif
