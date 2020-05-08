/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PIN_DATA_HPP
#define PIN_DATA_HPP

#include <stm32l476xx.h>

typedef struct{
	GPIO_TypeDef* GPIO_Port;
	uint16_t	  GPIO_Pin;
	bool		  occupied;
} PinData;

#endif
