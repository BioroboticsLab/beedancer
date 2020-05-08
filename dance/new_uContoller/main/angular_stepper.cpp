#include "angular_stepper.h"


Angular_Stepper::Angular_Stepper(int addr, int current_limit) : TicI2C(addr), Printable()
{
	_addr = addr;
	_current_limit = (uint16_t)current_limit;
}

void Angular_Stepper::init()
{
	setProduct(TicProduct::T249);
	exitSafeStart();
	setAgcMode(TicAgcMode::On);
	get_micro_step();
	set_moving_current(false);
}

 int Angular_Stepper::rad2step(float rad)
{
	return (int)(rad / _step_rad_ratio * _micro_step / _reduction_ratio);
}

float Angular_Stepper::step2rad(int steps)
{
	return steps / _micro_step * _step_rad_ratio * _reduction_ratio;
}

float Angular_Stepper::get_pos_rad(){
	_current_position_step = getCurrentPosition();
	_current_position_rad = step2rad(_current_position_step);
	return _current_position_rad;
}

void Angular_Stepper::get_micro_step()
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

void Angular_Stepper::set_moving_current(bool is_moving)
{
	if(is_moving){
		setCurrentLimit(_current_limit);
	}
	else{
		setCurrentLimit(0);
	}
}

void Angular_Stepper::set_speed_rad(float radps)
{
	_state_of_operation = 2;
	if(radps != 0.){
		set_moving_current(true);
		// target in m.s-1
		_speed_target = (int32_t)((radps / _reduction_ratio) / (_step_rad_ratio*(1./_micro_step))*10000);
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

void Angular_Stepper::set_position_meter(float target, bool blocking)
{
	_state_of_operation = 1;
	
	float remapped_target = 0.;
	float shortestArc = 0.;
	float new_position = 0.;
	int computed_target = 0;

	remapped_target = getPrincipaleAngle(target);
	_position_step = getCurrentPosition();
	_position = step2rad(_position_step);
	shortestArc = getShortestArc(_position, remapped_target);
	new_position = _position;
	new_position = new_position + shortestArc;
	computed_target = rad2step(new_position);


	if(getCurrentLimit() == 0){
		set_moving_current(true);
	}
	set_position_and_acknowledge(computed);

	if(blocking){
		while(!is_idle()){vTaskDelay(1);}
	}
}

bool Angular_Stepper::is_idle()
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

size_t Angular_Stepper::printTo(Print& p) const 
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

void Angular_Stepper::check_errors(){
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

void Angular_Stepper::set_position_and_acknowledge(int32_t target)
{
	setTargetPosition(target);
	if(getTargetPosition() != target){set_position_and_acknowledge(target);}
	else{}
}

void Angular_Stepper::set_speed_and_acknowledge(int32_t target)
{
	setTargetVelocity(target);
	if(getTargetVelocity() != target){set_speed_and_acknowledge(target);}
	else{}
}

float Angular_Stepper::getPrincipaleAngle(float angleRad)
{
	return fmod((angleRad  - PI ), ( 2. * PI )) + PI;
}

float Angular_Stepper::getShortestArc(float current, float target)
{
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
