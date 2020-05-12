#include "one_wire_sensor.hpp"

OneWireSensor::OneWireSensor(PinData *pinData) {
	auto oneWireDriver = std::make_unique<OneWireDriver>(pinData);
	this->driver = std::move(oneWireDriver);
}

void OneWireSensor::startNewReadout(void){
	this->driver->driverStartReadout();
}

float OneWireSensor::getLastTempVal(void){
	return this->lastTemperatureValue;
}

float OneWireSensor::getLastHumidVal(void){
	return this->lastHumidityValue;
}
