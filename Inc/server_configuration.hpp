#ifndef SERVER_CONFIGURATION_HPP
#define SERVER_CONFIGURATION_HPP

#include "pin_data.hpp"

typedef enum {
	OneWireSensorType //DHT22
	//... inne typy sensorow
} SensorType;

typedef struct SensorInfo {
	SensorType sensorType;
	uint8_t interval; //co ile czytac
	std::string name; //"kuchnia"
	PinData *pinData; //PRZYDZIELONE PRZEZ deVmAN
} SensorInfo;

#endif /* SERVER_CONFIGURATION_HPP */
