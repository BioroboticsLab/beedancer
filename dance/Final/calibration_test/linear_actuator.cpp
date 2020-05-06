#include "linear_actuator.h"

Linear_Actuator::Linear_Actuator(int potentiometerPin, int speedPin, int directionPin)
{
	_potentiometerPin = potentiometerPin;
	_speedPin = speedPin;
	_directionPin = directionPin;
	_is_extracted = true;
}

void Linear_Actuator::init()
{
	// setting PWM properties
	const int freq = 96;
	const int resolution = 8;
	// PQ12_directionPin forward/Backward pin
	pinMode(_directionPin, OUTPUT);
	// configure Motor PWM functionalitites
	ledcSetup(_PWMChannel, freq, resolution);
	// attach the channel to the GPIO to be controlled
	ledcAttachPin(_speedPin, _PWMChannel);

	if(getPot() < _low + 100){_is_extracted = false;}
	else{_is_extracted = true;}
}

int Linear_Actuator::getPot()
{
	return analogRead(_potentiometerPin);
} // 10bits (0-4096)

void Linear_Actuator::extract()
{
	digitalWrite(_directionPin, LOW);
	ledcWrite(_PWMChannel, _speedValue);
	for(int i = 0 ; i < 10 ; i++){
		while(getPot() < _high){delay(1);}
		delay(10);
	}
	this->stop();
	_is_extracted = true;
}

void Linear_Actuator::retract()
{  
	digitalWrite(_directionPin, HIGH);
	ledcWrite(_PWMChannel, _speedValue);
	for(int i = 0 ; i < 10 ; i++){
		while(getPot() > _low){delay(1);}
		delay(10);
	}
	this->stop();
	_is_extracted = false;
}

void Linear_Actuator::stop()
{
	digitalWrite(_directionPin, LOW);
	ledcWrite(_PWMChannel, 0);
}

bool Linear_Actuator::is_extracted()
{
	return _is_extracted;
}
