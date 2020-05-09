#include "one_wire_sensor.hpp"

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
