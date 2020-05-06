#include "linear_stepper.h"
#include <Tic.h>

Linear_Stepper::Linear_Stepper(int addr, int current_limit) : TicI2C(addr), Printable()
{
	_addr = addr;
	_current_limit = (uint16_t)current_limit;
}

void Linear_Stepper::init()
{
	setProduct(TicProduct::T249);
	exitSafeStart();
	setAgcMode(TicAgcMode::On);
	get_micro_step();
	set_moving_current(false);
}

 int Linear_Stepper::m2step(float m)
{
	return (int)(m / _fullstep_m_ratio * _micro_step);
}

float Linear_Stepper::step2m(int steps)
{
	return steps / _micro_step * _fullstep_m_ratio;
}

float Linear_Stepper::get_pos_meter(){
	_current_position_step = getCurrentPosition();
	_current_position_meter = step2m(_current_position_step);
	return step2m(getCurrentPosition());
}

void Linear_Stepper::get_micro_step()
{
	switch(getStepMode())
	{
		case TicStepMode::Microstep1:   _micro_step = 1;
			break;
		case TicStepMode::Microstep2:   _micro_step = 2;
			break;
		case TicStepMode::Microstep4:   _micro_step = 4;
			break;
		case TicStepMode::Microstep8:   _micro_step = 8;
			break;
		case TicStepMode::Microstep16:   _micro_step = 16;
			break;
		case TicStepMode::Microstep32:   _micro_step = 32;
			break;
		default : _micro_step = -1;
			break;
	}
}

void Linear_Stepper::set_moving_current(bool is_moving)
{
	if(is_moving){
		setCurrentLimit(_current_limit);
	}
	else{
		setCurrentLimit(0);
	}
}

void Linear_Stepper::set_speed_meter(float meterps)
{
	_state_of_operation = 2;
	if(meterps != 0.){
		set_moving_current(true);
		// target in m.s-1
		_speed_target = (int32_t)(meterps/(_fullstep_m_ratio*(1./_micro_step))*10000); //Vs = Vt/(Ls.Us)*step10000
		// Vs = Step speed
		// Vt = target speed
		// Ls = Length of step
		// Us = micro_step (1/16;1/32)
		// TIC : speed measure in step per 10000s
		set_speed_and_acknowledge(_speed_target);
	}
	else{
		_speed_target = 0;
		haltAndHold();
		set_moving_current(false);
		_state_of_operation = 0;
	}
}

void Linear_Stepper::set_position_meter(float target, bool blocking)
{
	_state_of_operation = 1;
	_position_target = (int32_t)m2step(target);

	if(getCurrentLimit() == 0){
		Serial.print("Stepper addr: ");
		Serial.print(_addr);
		Serial.print(" Current is set to 0, unable to move please correct your code by with set_moving_current(true)");
	}
	set_position_and_acknowledge(_position_target);
	if(blocking){
		while(!is_idle()){}
	}
}

bool Linear_Stepper::is_idle()
{
	switch(_state_of_operation)
	{
		case 0: 
			return true;
			break;
		case 1: 
			if(getCurrentPosition() == _position_target){
				_state_of_operation == 0;
				set_moving_current(false);
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

size_t Linear_Stepper::printTo(Print& p) const 
{
	size_t r = 0;

	r += p.print("Stepper  : addr = ");
	r += p.print(_addr);
	r += p.print(" ; Micro_step = ");
	r += p.print(_micro_step);
	r += p.print(" ; Current Limit = ");
	r += p.print(_current_limit);
	return r;
 }

void Linear_Stepper::check_errors(){
	uint32_t errors = getErrorsOccurred();
	if (errors & (1 << (uint8_t)TicError::CommandTimeout))
	{
		Serial.print("ErrorOccured (TimeOut) on Stepper : ");
		Serial.println(_addr);
		resetCommandTimeout();
	}
	if (errors & (1 << (uint8_t)TicError::SerialError))
	{
		Serial.print("ErrorOccured (Serial) on Stepper :");
		Serial.println(_addr);
		if(_state_of_operation == 1){
			set_position_and_acknowledge(_position_target);
		}
		else if(_state_of_operation == 2){
			set_speed_and_acknowledge(_speed_target);
		}

	}
}

void Linear_Stepper::set_position_and_acknowledge(int32_t target)
{
	setTargetPosition(target);
	if(getTargetPosition() != target){set_position_and_acknowledge(target);}
	else{}
}

void Linear_Stepper::set_speed_and_acknowledge(int32_t target)
{
	setTargetVelocity(target);
	if(getTargetVelocity() != target){set_speed_and_acknowledge(target);}
	else{}
}
