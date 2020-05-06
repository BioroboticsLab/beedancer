#include "motion_control.h"
#include <Arduino.h>
#include "stepper.h"
#include "esp_timer.h"
#include "robot.h"
#include "calibration.h"

Motion_control::Motion_control(robot* beedancer)
{
	_beedancer = beedancer;
	_is_calibrated = false;
}

void Motion_control::goto_target(float x, float y, float z)
{
	if(_is_calibrated){
		_beedancer->StepperX->setPosition(x, false);
		_beedancer->StepperY->setPosition(y, false);
		_beedancer->StepperT->setPosition(z, false);
	}
	else{
		Serial.println("The robot is still not calibrated, impossible to move.");
	}
}

void Motion_control::calibrate()
{	
	vTaskDelay(10);
	calibrationXY(_beedancer);
	vTaskDelay(10);

	vTaskDelay(10);
	//calibrationDF(_beedancer);
	vTaskDelay(10);
}

bool Motion_control::is_calibrate()
{
	return _is_calibrated;
}

void Motion_control::extract()
{
	_beedancer->PQ12->extract();
}

void Motion_control::retract()
{
	_beedancer->PQ12->retract();
}

void Motion_control::_controller()
{

}

void Motion_control::resetTimeout()
{
	_beedancer->StepperX->resetCommandTimeout();
	_beedancer->StepperY->resetCommandTimeout();
	_beedancer->StepperT->resetCommandTimeout();
	_beedancer->StepperDF->resetCommandTimeout();
}	

void Motion_control::init()
{
	_beedancer->StepperX->init();
	_beedancer->StepperY->init();
	_beedancer->StepperT->init();
	_beedancer->StepperDF->init();
	_beedancer->PQ12->init();
}