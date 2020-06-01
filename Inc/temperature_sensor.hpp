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
	Timer *timer; //wskaznik na timera od DeviceManagera;
	std::string name;
	uint16_t interval;
	uint8_t bytesToRead;
	float lastTempReadout;
	float lastHumidReadout;
	//29: w ostatnim stanie nadpisuje wynik i wysyla powiadomienie, ze mozna wyslac jego nowe dane
public:
	TemperatureSensor(PinData pinData, uint16_t interval, std::string name);
	void driverStartReadout(void);
	void executeState(void);
	void changePinMode(oneWireMode mode);
	void writePin(bool state);
	bool readPin(void);
};

#endif
