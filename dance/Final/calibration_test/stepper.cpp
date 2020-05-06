#include "stepper.h"
#include <Arduino.h>
#include <math.h>
#include <Tic.h>

#define PI 3.14159265

Stepper::Stepper(int addr, int max_current, bool is_linear) : TicI2C(addr)
{
	_addr = addr;
	_position = 0.;	// Actual Position of the stepper in um
	_position_step = 0;	// Actual Position of the stepper in steps
	_is_linear = is_linear;
	_current_limit = max_current;
	_micro_step = 1;
}

void Stepper::init()
{
	setProduct(TicProduct::T249);
	exitSafeStart();
	setAgcMode(TicAgcMode::On);
	setTargetVelocity(0);
	delay(10);
	_micro_step = getMicro_step();
}

int Stepper::m2step(float m)
{
	Serial.print("Stepper div0  stmR : ");
	Serial.println(_step_m_ratio, 8);
	Serial.print("Stepper div0  m : ");
	Serial.println(m, 8);
	Serial.print("Stepper div0  _micro_step : ");
	Serial.println(_micro_step, 8);
	return (int)(m / _step_m_ratio * _micro_step);
}

float Stepper::step2m(int steps)
{
	return steps / _micro_step * _step_m_ratio;
}

int Stepper::rad2step(float deg)
{
	return (int)(deg / _step_rad_ratio * _micro_step / _reduction_ratio);
}

float Stepper::step2rad(int steps)
{
	return steps / _micro_step * _step_rad_ratio * _reduction_ratio;
}

float Stepper::getPos()
{
	if(_is_linear){
		_position_step = getCurrentPosition();
		_position = step2m(_position_step);
		return _position;
	}
	else{
		_position_step = getCurrentPosition();
		_position = step2rad(_position_step);
		return _position;
	}
}

int Stepper::getPos_step()
{
	return getCurrentPosition();
}

int Stepper::getMicro_step()
{
	switch(getStepMode())
	{
		case TicStepMode::Microstep1:   return 1;
		case TicStepMode::Microstep2:   return 2;
		case TicStepMode::Microstep4:   return 4;
		case TicStepMode::Microstep8:   return 8;
		case TicStepMode::Microstep16:   return 16;
		case TicStepMode::Microstep32:   return 32;
		default : return -1;
	}
}

void Stepper::setVelocity(float target)
{

	if(_is_linear){
		// target in m.s-1
		int Vstep = (int)(target/(_step_m_ratio*(1./_micro_step))*10000); //Vs = Vt/(Ls.Us)*step10000
		// Vs = Step speed
		// Vt = target speed
		// Ls = Length of step
		// Us = micro_step (1/16;1/32)
		// TIC : speed measure in step per 10000s
		setTargetVelocity(Vstep);
	}
	else{
		//target in rad s-1
		int Vstep = (int)((target / _reduction_ratio) / (_step_rad_ratio*(1./_micro_step))*10000);
		Serial.print("stepper;");
		Serial.print("Vstep : ");
		Serial.println(Vstep);
		setTargetVelocity(Vstep);
	}
}

void Stepper::handleError(uint32_t errors, int target){
	if (errors & (1 << (uint8_t)TicError::CommandTimeout))
	{
		Serial.println("ErrorOccured (TimeOut) : resetting Timeout.");
		resetCommandTimeout();
	}
	if (errors & (1 << (uint8_t)TicError::SerialError))
	{
		Serial.println("ErrorOccured (Serial) : resetting target.");
		setTargetPosition(target);
	}
}

void Stepper::setPosition(float target, bool blocking)
{
	float remapped_target = 0.;
	float shortestArc = 0.;
	float new_position = 0.;
	int computed_target = 0;
	Serial.println("stepper;beforeif");
	if(_is_linear){
		// target in m.s-1
		Serial.println("stepper;afterif");
		computed_target = m2step(target);
		setTargetPosition(computed_target);
		if(blocking)
		{
			Serial.println("stepper;beforeblocking");
			while(getCurrentPosition() != computed_target){
				Serial.print("inLoop");
				handleError(getErrorsOccurred(), computed_target);
				vTaskDelay(1);
			}
			Serial.println("stepper;afterblocking");
		}
	}
	else{
		remapped_target = getPrincipaleAngle(target);
		_position_step = getCurrentPosition();
		_position = step2rad(_position_step);
		shortestArc = getShortestArc(_position, remapped_target);
		new_position = _position;
		new_position = new_position + shortestArc;

		Serial.print(" ; target : ");
		Serial.print(target);
		Serial.print(" ; rmtarget : ");
		Serial.print(remapped_target);
		Serial.print(" ; posstep : ");
		Serial.print(_position_step);
		Serial.print(" ; pos : ");
		Serial.print(_position);
		Serial.print(" ; shortestArc : ");
		Serial.print(shortestArc);
		Serial.print(" ; new_position : ");
		Serial.print(new_position);
		Serial.print(" ; new_positioninstep : ");
		Serial.print(rad2step(new_position));
		computed_target = rad2step(new_position);

		setTargetPosition(computed_target);
		if(blocking)
		{
			while(getCurrentPosition() != computed_target){
				handleError(getErrorsOccurred(), computed_target);
				vTaskDelay(1);
			}
		}
	}
}

float Stepper::getPrincipaleAngle(float angleRad)
{
	return fmod((angleRad  - PI ), ( 2. * PI )) + PI;
}

float Stepper::getShortestArc(float current, float target)
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


