#include "timer.hpp"

Timer::Timer(TIM_HandleTypeDef *htim){
	this->handle = htim;
}

void Timer::registerCallback(std::function<void(void)> callbackFunction){
	this->callback = callbackFunction;
}

void Timer::executeCallback(void){
	callback();
}

TIM_HandleTypeDef* Timer::getHandle(void){
	return this->handle;
}

void Timer::wakeMeUpAfterMicroseconds(uint16_t us){
	//bazowa czestotliwosc taktowania 80MHz, prescaler 6400-1 => tick co 80 mikrosekund
	this->clearCNT_Register();
	this->setARR_Register(us/80-1);
	this->startCounter();
}

void Timer::wakeMeUpAfterSeconds(uint16_t sec){
	//bazowa czestotliwosc taktowania 80MHz, prescaler 6400-1 => tick co 80 mikrosekund => 12,5k tickow na sek.
	this->clearCNT_Register();
	this->setARR_Register(12500*sec-1);


	//TODO: zmiany w tempSensor.cpp i main.cpp


	this->startCounter();
}

void Timer::stopCounter(void){
	this->handle->Instance->CR1 &= (~(TIM_CR1_CEN)); //bit Enable na 0
}

void Timer::startCounter(void){
	this->handle->Instance->CR1 |= TIM_CR1_CEN;//bit Enable na 1
}

void Timer::clearCNT_Register(void){
	this->handle->Instance->CNT = 0;
}

void Timer::setARR_Register(uint16_t value){
	this->handle->Instance->ARR = value;
}

void Timer::startGeneratingInterrupts(void){
	HAL_TIM_Base_Start_IT(this->handle);
}
