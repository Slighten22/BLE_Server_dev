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
extern uint8_t readData[];
extern void delayMicroseconds(uint32_t us);
#define MSG_LEN 20

class TemperatureSensor : public GenericOnePinDriver {
private:
	void firstStateHandler(void);
	void secondStateHandler(void);
	void thirdStateHandler(void); //.. moga byc kolejne stany wewnatrz odczytu
	std::function<void(void)> readoutFinishedHandler;
	Timer *timer;
	std::string name;
	uint16_t interval;
	float lastTempValue;
	float lastHumidValue;
public:
	TemperatureSensor(PinData pinData, uint16_t interval, std::string name, std::function<void(void)> readoutFinishedHandler);
	void startReadout(std::function<void(void)> readoutFinishedHandler);
	void executeState(void);
	void changePinMode(oneWireMode mode);
	void writePin(bool state);
	bool readPin(void);
	bool hasTempOrHumidChanged(uint32_t dataBits, uint8_t checksumBits);
	void performDataReadout(uint32_t &dataBits, uint8_t &checksumBits);
};

#endif
