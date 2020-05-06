#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H
#include <Arduino.h>


class Linear_Actuator
{
public:
	Linear_Actuator(int potentiometerPin, int speedPin, int directionPin);
	void init();
	int getPot(); // 10bits (0-4096)
	void extract();
	void retract();
	void stop();
	bool is_extracted();

private:
	const int _speedValue = 250;
	const int _high = 4060;
	const int _low = 30;
	const int _PWMChannel = 4;
	int _potentiometerPin;
	int _speedPin;
	int _directionPin;
	bool _is_extracted;
};

#endif