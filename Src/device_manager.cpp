#include "device_manager.hpp"

//Timery
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim4;
//TIM_HandleTypeDef htim5;
//TIM_HandleTypeDef htim2;

//... kolejne TIM_HandleTypedefy
Timer tim7(&htim7);
Timer tim6(&htim6);
Timer tim4(&htim4);
//Timer tim5(&htim5); //TODO
//Timer tim2(&htim2);

Timer *timers[] = {/*&tim5, &tim2,*/ &tim4, &tim6, &tim7};

DeviceManager::DeviceManager(){
	occupiedPinsCount = 0;
	usedTimersCount = 0;
	hardwarePinsList[0].GPIO_Port = GPIOA;		 //((GPIO_TypeDef *) GPIOA_BASE);
	hardwarePinsList[0].GPIO_Pin  = GPIO_PIN_4;  //((uint16_t)0x0010);
	hardwarePinsList[1].GPIO_Port = GPIOA;		 //((GPIO_TypeDef *) GPIOA_BASE);
	hardwarePinsList[1].GPIO_Pin  = GPIO_PIN_9;  //((uint16_t)0x0200);
	hardwarePinsList[2].GPIO_Port = GPIOA;		 //((GPIO_TypeDef *) GPIOA_BASE);
	hardwarePinsList[2].GPIO_Pin  = GPIO_PIN_10; //((uint16_t)0x0400);
	hardwarePinsList[3].GPIO_Port = GPIOA;		 //((GPIO_TypeDef *) GPIOA_BASE);
	hardwarePinsList[3].GPIO_Pin  =	GPIO_PIN_11; //((uint16_t)0x0800);
	hardwarePinsList[4].GPIO_Port = GPIOA;		 //((GPIO_TypeDef *) GPIOA_BASE);
	hardwarePinsList[4].GPIO_Pin  =	GPIO_PIN_12; //((uint16_t)0x1000);
	hardwarePinsList[0].occupied  = false;
	hardwarePinsList[1].occupied  = false;
	hardwarePinsList[2].occupied  = false;
	hardwarePinsList[3].occupied  = false;
	hardwarePinsList[4].occupied  = false;
}

bool DeviceManager::checkIfPinFree(PinData pinData){
	GPIO_TypeDef* reqPort = pinData.GPIO_Port;
	uint16_t reqPin = pinData.GPIO_Pin;
	for(uint8_t i=0; i<(sizeof(hardwarePinsList)/sizeof(hardwarePinsList[0])); i++){
			if(reqPort == hardwarePinsList[i].GPIO_Port
			   && reqPin == hardwarePinsList[i].GPIO_Pin
			   && hardwarePinsList[i].occupied == false){
					hardwarePinsList[i].occupied = true;
					return true;
			}
	}
	return false;
}

PinData DeviceManager::getFreePin(void){
	for(uint8_t i=0; i<(sizeof(hardwarePinsList)/sizeof(hardwarePinsList[0])); i++){
		if(hardwarePinsList[i].occupied == false)
			return hardwarePinsList[i];
	}
	return hardwarePinsList[0]; //TODO
}

Timer* DeviceManager::getNewTimerHandle(void){
	if(usedTimersCount < (sizeof(timers)/sizeof(timers[0]))){
		return timers[usedTimersCount++];
	}
	else{
		return timers[1]; //TODO
	}
}

uint8_t DeviceManager::getOccupiedPinsCount(void){
	return this->occupiedPinsCount;
};

uint8_t DeviceManager::getUsedTimersCount(void){
	return this->usedTimersCount;
}

int DeviceManager::getTimerIndex(TIM_HandleTypeDef *htim){
	int timersElemNumber = (sizeof(timers)/sizeof(timers[0]));
	for(int i=0; i<timersElemNumber; i++){
		if(htim == timers[i]->getHandle())
			return i;
	}
	return 0; //TODO: zabezpieczyc to!
}

