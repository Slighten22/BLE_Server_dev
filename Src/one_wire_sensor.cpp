#include "one_wire_sensor.hpp"

//OneWireSensor::OneWireSensor(PinData *pinData) : lastTemperatureValue(0.0F), lastHumidityValue(0.0F) {
////	std::shared_ptr<OneWireDriver> oneWireDriver(new OneWireDriver(pinData));
//	std::unique_ptr<OneWireDriver> oneWireDriver(new OneWireDriver(pinData));
////	OneWireDriver oneWireDriver(pinData);
//	this->driver = std::move(oneWireDriver); //TODO: klasa bazowa ustawia! lista inicjalizacyjna
////	this->driver = &oneWireDriver;
//}

OneWireSensor::OneWireSensor(PinData pinData) {
//	std::unique_ptr<OneWireDriver> oneWireDriver(new OneWireDriver(pinData));
	//driver globalny, statycznie tworzony?
	std::shared_ptr<OneWireDriver> oneWireDriver(new OneWireDriver(pinData));
//	this->driver = std::move(oneWireDriver);
	this->driver = oneWireDriver;
}


//OneWireSensor::OneWireSensor(PinData pinData){
//	std::shared_ptr<OneWireDriver> oneWireDriver(new OneWireDriver(pinData));
//	this->driver = std::move(oneWireDriver); //TODO: klasa bazowa ustawia! lista inicjalizacyjna
//}
//
//
//
//OneWireSensor::OneWireSensor(PinData *pinData, std::string name, uint16_t interval) : lastTemperatureValue(0.0F), lastHumidityValue(0.0F){
//	std::shared_ptr<OneWireDriver> oneWireDriver(new OneWireDriver(pinData));
//	this->driver = std::move(oneWireDriver);
//	this->name = name;
//	this->interval = interval;
//}

//OneWireSensor::OneWireSensor(const OneWireSensor &sensor){
//	this->driver = std::move(sensor.driver);
//}

float OneWireSensor::getLastTempVal(void){
	return this->lastTemperatureValue;
}

float OneWireSensor::getLastHumidVal(void){
	return this->lastHumidityValue;
}
