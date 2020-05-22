#ifndef SERVER_CONFIGURATION_HPP
#define SERVER_CONFIGURATION_HPP

#include "pin_data.hpp"

#define MAX_SENSORS_NUMBER 16

typedef enum {
	OneWireSensorType
	//... inne typy sensorow
} SensorType;

typedef struct ServerConfiguration {
	uint8_t sensorsCount;
	SensorType sensorTypes[MAX_SENSORS_NUMBER];
	PinData pinAddresses[MAX_SENSORS_NUMBER];
	uint8_t bytesAwaitedCount;
	//... inne dane, np. interwal co ile robic odczyty
} ServerConfiguration;


#endif /* SERVER_CONFIGURATION_HPP */
