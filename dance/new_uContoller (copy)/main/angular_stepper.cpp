#include "angular_stepper.h"
#include <AccelStepper.h>

#define PI 3.14159265

Angular_Stepper::Angular_Stepper(int step_pin, int dir_pin, int enable_pin, int micro_step, float reduction_ration) : AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin)
{
	_step_pin = step_pin;
	_dir_pin = dir_pin;
	_enable_pin = enable_pin; 
	_micro_step = micro_step;
	_reduction_ratio = reduction_ration;
	setEnablePin(_enable_pin);
	setPinsInverted(false, false, true);
}

 int Angular_Stepper::rad2step(float rad)
{
	return (int)(rad * _step_rad_ratio * _micro_step * _reduction_ratio);
}

float Angular_Stepper::step2rad(int steps)
{
	return (float)(steps / (_micro_step * _step_rad_ratio * _reduction_ratio));
}

float Angular_Stepper::get_pos_rad(){
	_current_position_step = currentPosition();
	_current_position_rad = step2rad(_current_position_step);
	return _current_position_rad;
}

void Angular_Stepper::set_max_speed(float radps){
	_max_speed_steps = (int32_t)rad2step(radps);
	setMaxSpeed(_max_speed_steps);
}

void Angular_Stepper::set_max_speed(){
	setMaxSpeed(_max_speed_steps);
}

void Angular_Stepper::set_acceleration(float radpss){
	_max_speed_steps = (int32_t)rad2step(radpss);
	setAcceleration(_max_speed_steps);
}
void Angular_Stepper::set_acceleration(){
	setAcceleration(_max_speed_steps);
}

void Angular_Stepper::set_speed_rad(float radps)
{
	_state_of_operation = 2;
	if(radps != 0.){
		enableOutputs();
		// target in m.s-1
		_speed_target = (int32_t)rad2step(radps);
		// Vs = Step speed
		// Vt = target speed
		// Ls = Length of step
		// Us = micro_step (1/16;1/32)
		// TIC : speed measure in step per 10000s
		if(_speed_target >= 0){_is_direct = 1;}
		else{_is_direct = -1;}
		/*
		Serial.print("Speed rad :");
		Serial.println(_speed_target);*/
		setSpeed(_speed_target);
	}
	else{
		_speed_target = 0;
		stop();
		disableOutputs();
		_state_of_operation = 0;
	}
}

void Angular_Stepper::set_position_rad(float target, bool blocking)
{
	
	float remapped_target = 0.;
	float shortestArc = 0.;
	float new_position = 0.;

	remapped_target = getPrincipaleAngle(target);
	_current_position_step = currentPosition();
	_current_position_rad = step2rad(_current_position_step);
	shortestArc = getShortestArc(_current_position_rad, remapped_target);
	new_position = _current_position_rad;
	new_position = new_position + shortestArc;
	_position_target = rad2step(new_position);

	if(shortestArc>=0){_is_direct = 1;}
	else{_is_direct = -1;}

	enableOutputs();
	_state_of_operation = 1;

	moveTo(_position_target);

	if(blocking){
		while(distanceToGo()!=0){run();}
		_state_of_operation = 0;
		disableOutputs();
	}
}

long Angular_Stepper::set_position_rad_overshoot(float target, bool blocking)
{
	
	float remapped_target = 0.;
	float shortestArc = 0.;
	float new_position = 0.;

	remapped_target = getPrincipaleAngle(target);
	_current_position_step = currentPosition();
	_current_position_rad =step2rad(_current_position_step);
	shortestArc = getShortestArc(_current_position_rad, remapped_target);
	new_position = _current_position_rad + shortestArc;
	_position_target = rad2step(new_position);

	if(shortestArc>=0){_is_direct = 1;}
	else{_is_direct = -1;}

	enableOutputs();
	_state_of_operation = 1;

	_acceleration_ramp_length = (long)((_maxSpeed * _maxSpeed) / (2.0 * _acceleration));

	moveTo(_position_target + (_is_direct * _acceleration_ramp_length));

	if(blocking){
		while(distanceToGo()!=0){run();}
		_state_of_operation = 0;
		disableOutputs();
	}
	return _position_target;
}

bool Angular_Stepper::is_idle()
{
	switch(_state_of_operation)
	{
		case 0: 
			disableOutputs();
			return true;
			break;
		case 1: 
			if(distanceToGo() == 0){
				_state_of_operation == 0;
				disableOutputs();
				return true;
			} 
			else{
				return false;
			}
			break;
		case 2: 
			return false;
			break;
	}
}


float Angular_Stepper::getPrincipaleAngle(float angleRad)
{
	//Convert an angle between 0 and 2pi in an angle between -pi and pi
	return fmod((angleRad  - PI ), ( 2. * PI )) - PI;
}

float Angular_Stepper::getShortestArc(float current, float target)
{
	// Search which arc is the smallest ie. which arc is the shortest between indirect (CW) rotation and direct (CCW) rotation
	float directArc = 0.;
	float indirectArc = 0.;
	if(target < current){
		directArc = 2 * PI - (current - target);
		indirectArc = - (2 * PI - directArc); 
	}
	else{
		directArc = (target - current);
		indirectArc = - (2 * PI - directArc);
	}

	if(abs(directArc) < abs(indirectArc)){
		return directArc;
	}
	else{
		return indirectArc;
	}
}
