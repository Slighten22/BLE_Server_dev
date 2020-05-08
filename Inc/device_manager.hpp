/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

#include "timer.hpp"
#include "pin_data.hpp"
#include <stm32l476xx.h>

//Timery
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim4; //TODO: inicjalizacja w HALu
//... kolejne TIM_HandleTypedefy
extern Timer tim7;
extern Timer tim6;
extern Timer tim4;
//... kolejne obiekty timerow
extern Timer *timers[];

class DeviceManager{
private:
	/*
	 * PA_x pins to use
	 * Their corresponding numbers (defined in "stm32l4xx_hal_gpio.h") stored in hardwarePinsList[]
	 * Nucleo-L476RG board pinout scheme: https://os.mbed.com/platforms/ST-Nucleo-L476RG/
	 */
	uint8_t occupiedPinsCount;
	uint8_t usedTimersCount;
	PinData hardwarePinsList[5];
public:
	DeviceManager(); //inicjalizuje managera i jego tablice zasobow do rozdania
	bool checkIfPinFree(PinData *data); //w konstruktorze sensora bedzie sprawdzane czy pin ktorego chcemy uzyc jest z listy i czy wolny
	uint8_t getOccupiedPinsCount(void);
	uint8_t getUsedTimersCount(void);
	Timer* getNewTimerHandle(void);
	PinData* getFreePin(void);
	int getTimerIndex(TIM_HandleTypeDef *htim);
};

#endif
