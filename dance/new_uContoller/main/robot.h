#ifndef ROBOT_H
#define ROBOT_H
#include "linear_actuator.h"
#include "linear_stepper.h"
#include <DebounceInput.h>

typedef struct
{
	Linear_Stepper* StepperX;
	Linear_Stepper* StepperY;
	Linear_Stepper* StepperT;
	Linear_Stepper* StepperDF;
	Linear_Actuator* PQ12;
	DebouncedInput* xSwitch;
	DebouncedInput* ySwitch;
	DebouncedInput* dfSwitch;
	int motorCerrorPin;
} robot;

#endif