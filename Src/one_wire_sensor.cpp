#include "one_wire_sensor.hpp"

OneWireSensor::OneWireSensor(PinData *pinData) : lastTemperatureValue(0.0F), lastHumidityValue(0.0F) {
	std::unique_ptr<OneWireDriver> oneWireDriver(new OneWireDriver(pinData));
	this->driver = std::move(oneWireDriver); //TODO: klasa bazowa ustawia! lista inicjalizacyjna
}

float OneWireSensor::getLastTempVal(void){
	return this->lastTemperatureValue;
}

float OneWireSensor::getLastHumidVal(void){
	return this->lastHumidityValue;
}
