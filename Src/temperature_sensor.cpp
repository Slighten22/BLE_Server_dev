#include "temperature_sensor.hpp"

SemaphoreHandle_t TemperatureSensor::singleReadoutSem = xSemaphoreCreateBinary();

TemperatureSensor::TemperatureSensor(PinData pinData, uint16_t interval, std::string name,
							std::function<void(uint32_t, uint8_t, uint32_t, std::string)> readoutFinishedHandler)
							: GenericOnePinDriver(pinData) {
	this->interval = interval;
	this->name = name;
	this->stateHandler = static_cast<StateHandler>(&TemperatureSensor::firstStateHandler);
	this->readoutFinishedHandler = readoutFinishedHandler;
	this->timer = deviceManager.getNewTimerHandle();
	this->timer->registerCallback(std::bind(&TemperatureSensor::executeState, this));
	this->timer->setARR_Register(12500*interval-1); //aby dostac przerwanie co <interval> sekund
	this->timer->startGeneratingInterrupts(); //czujnik zaczyna zyc wlasnym zyciem
};

void TemperatureSensor::startReadout(void){
	this->stateHandler = static_cast<StateHandler>(&TemperatureSensor::firstStateHandler);
	this->readoutFinishedHandler = readoutFinishedHandler;
	this->executeState();
}

void TemperatureSensor::executeState(void){
	(this->*stateHandler)();
}

void TemperatureSensor::changePinMode(oneWireMode mode){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = this->pinData.GPIO_Pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	if(mode == ONE_WIRE_OUTPUT){
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	}
	else if(mode == ONE_WIRE_INPUT){
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	}
	HAL_GPIO_Init(this->pinData.GPIO_Port, &GPIO_InitStruct);
};

void TemperatureSensor::writePin(bool state){
	HAL_GPIO_WritePin(this->pinData.GPIO_Port,
					  this->pinData.GPIO_Pin,
					  state == true ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool TemperatureSensor::readPin(void){
	return (1&HAL_GPIO_ReadPin(this->pinData.GPIO_Port, this->pinData.GPIO_Pin));
}

void TemperatureSensor::performDataReadout(uint32_t &dataBits, uint8_t &checksumBits){
	while(this->readPin());
	while(!this->readPin());
	while(this->readPin());
	for (int8_t i = 31; i >= 0; i--){	//Read 32 bits of temp.&humidity data
		/*
		 * Bit data "0" signal: the level is LOW for 50ms and HIGH for 26-28ms;
		 * Bit data "1" signal: the level is LOW for 50ms and HIGH for 70ms;
		 * MAX FREQUENCY ON STM32L476RG = 80MHz
		 * SO IT TAKES 12,5 ns FOR ONE INSTRUCTION TO EXECUTE
		 * A DELAY OF 1 SECOND (x TICKS): 80 MILLION NOP INSTRUCTIONS TO EXECUTE
		 */
		while (!this->readPin());
		delayMicroseconds(50);
		if (this->readPin()) {
			dataBits |= (1UL << i);
		}
		while (this->readPin());
	}
	for (int8_t i = 7; i >= 0; i--){
		while (!this->readPin());
		delayMicroseconds(50);
		if (this->readPin()) {
			checksumBits |= (1UL << i);
		}
		while (this->readPin());
	}
}

bool TemperatureSensor::hasTempOrHumidChanged(uint32_t dataBits, uint8_t checksumBits){
	uint8_t len;
	for(len=0; len<MSG_LEN; len++){
		readData[len] = '\0';
	} //memset
	for(len=0; this->name[len] != '\0' && len<MAX_NAME_LEN; len++){
		readData[len] = (uint8_t)this->name[len];
	}
	readData[len] = '\0';
	readData[len+1] = (dataBits >> 24) & 0xFF;
	readData[len+2] = (dataBits >> 16) & 0xFF;
	readData[len+3] = (dataBits >> 8) & 0xFF;
	readData[len+4] = (dataBits >> 0) & 0xFF;
	readData[len+5] = (checksumBits) & 0xFF;
	uint16_t humidTimesTen = (readData[len+1] << 8) | (readData[len+2]);
	uint16_t tempTimesTen  = (readData[len+3] << 8) | (readData[len+4]);
	float newHumidVal = (float)(humidTimesTen / 10.0F);
	float newTempVal  = (float)(tempTimesTen / 10.0F);
	//byla zmiana temp./wilgotnosci => wyslij nowa wartosc
	if(newTempVal != this->lastTempValue || newHumidVal != this->lastHumidValue){
		this->lastTempValue = newTempVal;
		this->lastHumidValue = newHumidVal;
		return true;
	}
	return false;
}

bool checkIfTempSensorReadoutCorrect(uint32_t dataBits, uint8_t checksumBits){
	uint8_t value = ((dataBits >> 24) & 0xFF) + ((dataBits >> 16) & 0xFF) + ((dataBits >> 8) & 0xFF) + (dataBits & 0xFF);
	if(value == checksumBits)
		return true;
	return false;
}

float calculateTempValue(uint32_t dataBits){
	uint16_t tempTimesTen = (((dataBits >> 8) & 0xFF) << 8) | (dataBits & 0xFF);
	float tempValue = (float)(tempTimesTen / 10.0F);
	return tempValue;
}

float calculateHumidValue(uint32_t dataBits){
	uint16_t humidTimesTen = (((dataBits >> 24) & 0xFF) << 8) | ((dataBits >> 16) & 0xFF);
	float humidValue = (float)(humidTimesTen / 10.0F);
	return humidValue;
}

