#include "one_wire_sensor.hpp"

OneWireSensor::OneWireSensor(PinData *pinData) : /*Sensor(pinData),2 */oneWireDriver(OneWireDriver(pinData)) {
//	OneWireDriver oneWireDriver(pinData);
	this->concreteDriver = &oneWireDriver;
}

void OneWireSensor::startNewReadout(void){
//	this->oneWireDriver.driverStartReadout();
	this->concreteDriver->driverStartReadout();
}

float OneWireSensor::getLastTempVal(void){
	return this->lastTemperatureValue;
}

float OneWireSensor::getLastHumidVal(void){
	return this->lastHumidityValue;
}
