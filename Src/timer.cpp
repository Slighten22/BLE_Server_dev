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
	//bazowa czestotliwosc taktowania 80MHz => 80 taktow na us
	this->handle->Init.Prescaler = 79;
	this->handle->Init.Period = (us > 0) ? (us - 1) : 0;
	if (HAL_TIM_Base_Init(this->handle) == HAL_OK){
		HAL_TIM_Base_Start_IT(this->handle); //TODO: TIM6, TIM4 fix!
		HAL_TIM_Base_Stop(this->handle);
	}
}
