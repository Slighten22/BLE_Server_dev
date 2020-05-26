#ifndef SERVER_CONFIGURATION_HPP
#define SERVER_CONFIGURATION_HPP

#include "pin_data.hpp"

typedef enum {
	DHT22
	//... inne typy sensorow
} SensorType;

typedef struct SensorInfo {
	SensorType sensorType;
	uint16_t interval; //co ile czytac
	char name[MAX_NAME_LEN]; //"kuchnia"
	PinData pinData;
} SensorInfo;

#endif /* SERVER_CONFIGURATION_HPP */
