#include "temperature_sensor.hpp"

TemperatureSensor::TemperatureSensor(PinData pinData, uint16_t interval, std::string name)
							: GenericOnePinDriver(pinData) {
	this->interval = interval;
	this->name = name;
	this->stateHandler = static_cast<StateHandler>(&TemperatureSensor::firstStateHandler);
	this->timer = deviceManager.getNewTimerHandle();
	this->timer->registerCallback(std::bind(&TemperatureSensor::executeState, this));
};

void TemperatureSensor::driverStartReadout(void){
	this->stateHandler = static_cast<StateHandler>(&TemperatureSensor::firstStateHandler);
	this->executeState();
}

void TemperatureSensor::executeState(void){
	(this->*stateHandler)();
}

void TemperatureSensor::changePinMode(oneWireMode mode){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = this->pinData.GPIO_Pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	if(mode == ONE_WIRE_OUTPUT){
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	}
	else if(mode == ONE_WIRE_INPUT){
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	}
	HAL_GPIO_Init(this->pinData.GPIO_Port, &GPIO_InitStruct);
};

void TemperatureSensor::writePin(bool state){
	HAL_GPIO_WritePin(this->pinData.GPIO_Port,
					  this->pinData.GPIO_Pin,
					  state == true ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool TemperatureSensor::readPin(void){
	return (1&HAL_GPIO_ReadPin(this->pinData.GPIO_Port, this->pinData.GPIO_Pin));
}
