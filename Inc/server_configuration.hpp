#ifndef SERVER_CONFIGURATION_HPP
#define SERVER_CONFIGURATION_HPP

#include "pin_data.hpp"

typedef enum {
	DHT22
	//... inne typy sensorow
} SensorType;

typedef struct SensorInfo {
	SensorType sensorType;
	std::string name;
	PinData pinData;
	uint16_t interval;
} SensorInfo;

#endif /* SERVER_CONFIGURATION_HPP */
