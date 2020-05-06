#ifndef ROBOT_H
#define ROBOT_H
#include "linear_actuator.h"
#include "stepper.h"
#include <DebounceInput.h>

typedef struct
{
	Stepper* StepperX;
	Stepper* StepperY;
	Stepper* StepperT;
	Stepper* StepperDF;
	Linear_Actuator* PQ12;
	DebouncedInput* xSwitch;
	DebouncedInput* ySwitch;
	DebouncedInput* dfSwitch;
} robot;

#endif