#include "temperature_sensor.hpp"

TemperatureSensor::TemperatureSensor(PinData pinData, uint16_t interval, std::string name, std::function<void(void)> readoutFinishedHandler)
							: GenericOnePinDriver(pinData) {
	this->interval = interval;
	this->name = name;
	this->stateHandler = static_cast<StateHandler>(&TemperatureSensor::firstStateHandler);

	this->readoutFinishedHandler = readoutFinishedHandler;

	this->timer = deviceManager.getNewTimerHandle();
	this->timer->registerCallback(std::bind(&TemperatureSensor::executeState, this));
	this->timer->setARR_Register(12500*interval-1); //aby dostac przerwanie co <interval> sekund
	this->timer->startGeneratingInterrupts(); //czujnik zaczyna zyc wlasnym zyciem
};

void TemperatureSensor::startReadout(std::function<void(void)> readoutFinishedHandler){
	this->stateHandler = static_cast<StateHandler>(&TemperatureSensor::firstStateHandler);
	this->readoutFinishedHandler = readoutFinishedHandler;
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
