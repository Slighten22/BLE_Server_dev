/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMER_HPP
#define TIMER_HPP

#include <stm32l4xx_hal.h>
#include <functional>

class Timer{
private:
	TIM_HandleTypeDef *handle; //htim7, htim8, ...
	std::function<void(void)> callback; //timer nie musi wiedziec dla jakiego drivera pracuje!
public:
	Timer(TIM_HandleTypeDef *htim); //tworzenie timerow - globalne obiekty w deviceManagerze
	void executeCallback(void); //w HAL_TIM_PeriodElapsedCallback
	void registerCallback(std::function<void(void)> callbackFunction); //u drivera
	TIM_HandleTypeDef* getHandle(void);
	void wakeMeUpAfterMicroseconds(uint16_t us);
};

#endif
