#ifndef ROBOT_H
#define ROBOT_H
#include "linear_actuator.h"
#include "linear_stepper.h"
#include "angular_stepper.h"
#include <DebounceInput.h>

typedef struct
{
	Linear_Stepper* StepperX;
	Linear_Stepper* StepperY;
	Angular_Stepper* StepperT;
	Angular_Stepper* StepperDF;
	Linear_Actuator* PQ12;
	DebouncedInput* xSwitch;
	DebouncedInput* ySwitch;
	DebouncedInput* dfSwitch;
	hw_timer_t* syncTimerMotion;
	int photo_res;
	int led_pin;
	int motorCerrorPin;
	float dancefloor_angle;
} robot;

#endif