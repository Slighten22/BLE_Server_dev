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
	char charName[MAX_NAME_LEN]; //"kuchnia"
	std::string name;
	PinData pinData;
} SensorInfo;

#endif /* SERVER_CONFIGURATION_HPP */
