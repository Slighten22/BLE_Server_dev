#ifndef TEMPERATURE_SENSOR_HPP
#define TEMPERATURE_SENSOR_HPP

#include "pin_data.hpp"
#include "timer.hpp"
#include "generic_one_pin_driver.hpp"
#include "device_manager.hpp" //musimy miec obiekt DMa zeby dostac pin i timer
#include <functional>

typedef enum {
	ONE_WIRE_OUTPUT = 0,
	ONE_WIRE_INPUT  = 1,
} oneWireMode;

extern DeviceManager deviceManager;

class TemperatureSensor : public GenericOnePinDriver {
private:
	void firstStateHandler(void);
	void secondStateHandler(void);
	void thirdStateHandler(void); //.. moga byc kolejne stany wewnatrz odczytu
	std::function<void(void)> readoutFinishedHandler;
	Timer *timer;
	std::string name;
	uint16_t interval;
public:
	TemperatureSensor(PinData pinData, uint16_t interval, std::string name);
	void startReadout(std::function<void(void)> readoutFinishedHandler);
	void executeState(void);
	void changePinMode(oneWireMode mode);
	void writePin(bool state);
	bool readPin(void);
};

#endif
