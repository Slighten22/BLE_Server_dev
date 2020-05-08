#include <sensor.hpp>

Sensor::Sensor(PinData *pinData) : oneWireDriver( OneWireDriver(pinData) ) {}

void Sensor::startNewReadout(void){
	this->oneWireDriver.driverStartReadout();
}

float Sensor::getLastTempVal(void){
	return this->lastTemperatureValue;
}

float Sensor::getLastHumidVal(void){
	return this->lastHumidityValue;
}
