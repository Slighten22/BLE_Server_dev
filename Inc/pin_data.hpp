/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PIN_DATA_HPP
#define PIN_DATA_HPP

#include <stm32l476xx.h>

#define MAX_NAME_LEN 15 //nazwa sensora

typedef struct{
	GPIO_TypeDef* GPIO_Port;
	uint16_t	  GPIO_Pin;
	bool		  occupied;
} PinData;

#endif
