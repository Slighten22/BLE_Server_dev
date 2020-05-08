#include "one_wire_driver.hpp"

OneWireDriver::OneWireDriver(PinData *inputPinData) {
	if(deviceManager.checkIfPinFree(inputPinData)){
		this->pinData = inputPinData;
	}
	else {
		this->pinData = deviceManager.getFreePin();
	}
	this->timer = deviceManager.getNewTimerHandle();
	this->timer->setDriver(this); //zamiast podczepiania callbacka - podczepanie obiektu drivera (TODO: podczepic obiekt generycznego drivera)
	this->stateHandler = &OneWireDriver::firstStateHandler;
};

void OneWireDriver::driverStartReadout(void){
	this->stateHandler = &OneWireDriver::firstStateHandler;
	this->executeState();
}

void OneWireDriver::executeState(void){
	(this->*stateHandler)();
}

void OneWireDriver::changePinMode(oneWireMode mode){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = this->pinData->GPIO_Pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	if(mode == ONE_WIRE_OUTPUT){
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	}
	else if(mode == ONE_WIRE_INPUT){
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	}
	HAL_GPIO_Init(this->pinData->GPIO_Port, &GPIO_InitStruct);
};

void OneWireDriver::writePin(bool state){
	HAL_GPIO_WritePin(this->pinData->GPIO_Port,
					  this->pinData->GPIO_Pin,
					  state == true ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool OneWireDriver::readPin(void){
	return (1&HAL_GPIO_ReadPin(this->pinData->GPIO_Port, this->pinData->GPIO_Pin));
}

//void OneWireDriver::registerTimerCallback(void){ //to nie dziala (i chyba nigdy nie dzialalo, skoro mam metody z roznych klas)
//	//check nie powinno byc  timer->registerCallback(OneWireDriver::&executeState());
//	this->timer->registerCallback(&OneWireDriver::executeState);
//	//? moze bedzie trzeba zmienic typ timer->callback na OneWireDriver::*TimerCallback
//
//	//this->timer->callback = [this]{ executeState(); }; //lambda - nie
//};
