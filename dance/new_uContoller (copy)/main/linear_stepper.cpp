#include "linear_stepper.h"
#include <AccelStepper.h>

Linear_Stepper::Linear_Stepper(int step_pin, int dir_pin, int enable_pin, int micro_step) : AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin)
{
	_step_pin = step_pin;
	_dir_pin = dir_pin;
	_enable_pin = enable_pin; 
	_micro_step = micro_step;
	setEnablePin(_enable_pin);
	setPinsInverted(false, false, true);
}

 int Linear_Stepper::m2step(float m)
{
	return (int)(m / _fullstep_m_ratio * _micro_step);
}

float Linear_Stepper::step2m(int steps)
{
	return (float)steps / _micro_step * _fullstep_m_ratio;
}

float Linear_Stepper::get_pos_meter(){
	_current_position_step = currentPosition();
	_current_position_meter = step2m(_current_position_step);
	return _current_position_meter;
}

void Linear_Stepper::set_max_speed(float mps){
	_max_speed_steps = (int32_t)m2step(mps);
	setMaxSpeed(_max_speed_steps);
}

void Linear_Stepper::set_max_speed(){
	setMaxSpeed(_max_speed_steps);
}

void Linear_Stepper::set_acceleration(float mpss){
	_max_accel_steps = (int32_t)m2step(mpss);
	setAcceleration(_max_accel_steps);
}

void Linear_Stepper::set_acceleration(){
	setAcceleration(_max_accel_steps);
}

void Linear_Stepper::set_speed_meter(float meterps)
{
	_state_of_operation = 2;
	if(meterps != 0.){
		enableOutputs();
		// target in m.s-1
		_speed_target = (int32_t)m2step(meterps); //Vs = Vt/(Ls.Us)*step
		// Vs = Step speed
		// Vt = target speed
		// Ls = Length of step
		// Us = micro_step (1/16;1/32)
		setSpeed(_speed_target);
	}
	else{
		_speed_target = 0;
		stop();
		disableOutputs();
		_state_of_operation = 0;
	}
}

void Linear_Stepper::set_position_meter(float target, bool blocking)
{
	_state_of_operation = 1;
	_position_target = (int32_t)m2step(target);
	enableOutputs();

	moveTo(_position_target);
	//Serial.print("Stepper ");
	//Serial.print(_enable_pin);
	//Serial.print(" position : ");
	//Serial.println(_position_target);

	if(blocking){
		while(isRunning()){run();}
		_state_of_operation = 0;
		disableOutputs();
	}
}

long Linear_Stepper::set_position_meter_overshoot(float target, bool blocking)
{
	long position_target_noovershoot;
	_state_of_operation = 1;
	position_target_noovershoot = (int32_t)m2step(target);
	enableOutputs();
	_acceleration_ramp_length = (long)((_maxSpeed * _maxSpeed) / (2.0 * _acceleration));
	if(currentPosition() < position_target_noovershoot){
		_position_target = position_target_noovershoot + _acceleration_ramp_length;
	}
	if(currentPosition() > position_target_noovershoot){
		_position_target = position_target_noovershoot - _acceleration_ramp_length;
	}
	moveTo(_position_target);

	if(blocking){
		while(isRunning()){run();}
		_state_of_operation = 0;
		disableOutputs();
	}

	return position_target_noovershoot;
}

long Linear_Stepper::distance_to_go_overshoot()
{
	if(distanceToGo() > 0){
		return distanceToGo() - _acceleration_ramp_length;
	}
	else{
		return - distanceToGo() - _acceleration_ramp_length;
	}
}

bool Linear_Stepper::is_idle()
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
