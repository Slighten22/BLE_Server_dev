/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMER_HPP
#define TIMER_HPP

#include <stm32l4xx_hal.h>

//class Timer; //?
//typedef void (Timer::*TimerCallback)(void);
class OneWireDriver;
typedef void (OneWireDriver::*TimerCallback)(void);

class Timer{
private:
	TIM_HandleTypeDef *handle; //htim7, htim8, ...
//	TimerCallback callback;    //funkcja do obsl. przerwania od danego timera - wskaznik na metode drivera!
	OneWireDriver *myDriver;   //zeby pamietal swojego "rodzica"???
public:
	Timer(TIM_HandleTypeDef *htim); //tworzenie timerow - globalne obiekty w deviceManagerze
//	void executeCallback(void); //w HAL_TIM_PeriodElapsedCallback
//	void registerCallback(TimerCallback callbackFunction); //u drivera: timer.RegisterCallback(this->ExecuteState);
	TIM_HandleTypeDef* getHandle(void);
	void setDriver(OneWireDriver *oneWireDriver);
	OneWireDriver* getDriver(void);
	void wakeMeUpAfterMicroseconds(uint16_t us);
};

#endif
