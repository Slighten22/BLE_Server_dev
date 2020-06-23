#ifndef TEMPERATURE_SENSOR_HPP
#define TEMPERATURE_SENSOR_HPP

#include "pin_data.hpp"
#include "timer.hpp"
#include "generic_one_pin_driver.hpp"
#include "device_manager.hpp" //musimy miec obiekt DMa zeby dostac pin i timer
#include <string.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <functional>

typedef enum {
	ONE_WIRE_OUTPUT = 0,
	ONE_WIRE_INPUT  = 1,
} oneWireMode;

extern DeviceManager deviceManager;
extern uint8_t readData[];
extern void delayMicroseconds(uint32_t us);

class TemperatureSensor : public GenericOnePinDriver {
private:
	void firstStateHandler(void);
	void secondStateHandler(void);
	void thirdStateHandler(void); //.. moga byc kolejne stany wewnatrz odczytu
	std::function<void(uint32_t, uint8_t, uint32_t, std::string)> readoutFinishedHandler;
	Timer *timer;
	std::string name;
	uint16_t interval;
	float lastTempValue;
	float lastHumidValue;
	uint32_t lastDataBits;
	static SemaphoreHandle_t singleReadoutSem;
public:
	TemperatureSensor(PinData pinData, uint16_t interval, std::string name,
					  std::function<void(uint32_t, uint8_t, uint32_t, std::string)> readoutFinishedHandler);
	void startReadout(void);
	void executeState(void);
	void changePinMode(oneWireMode mode);
	void writePin(bool state);
	bool readPin(void);
	void performDataReadout(uint32_t &dataBits, uint8_t &checksumBits);
};

bool checkIfTempSensorReadoutCorrect(uint32_t dataBits, uint8_t checksumBits);
float calculateTempValue(uint32_t dataBits);
float calculateHumidValue(uint32_t dataBits);

#endif
