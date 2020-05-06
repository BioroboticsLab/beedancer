#include "motion_control.h"


Motion_Control::Motion_Control(robot* beedancer)
{
	_beedancer = beedancer;
	_is_calibrated = false;
	_calibration_step = 0;
}

void Motion_Control::goto_pos(float x, float y, float t, float df)
{

	if(_is_calibrated || _in_calibration){
		if(_target_pos_x == x && _target_pos_y == y && _target_pos_t == t && _target_pos_df == df){}
		else{
			_state_of_operation = 1;
			_target_pos_x = x;
			_target_pos_y = y;
			_target_pos_t = t;
			_target_pos_df = df;
			
			_beedancer->StepperX->set_position_meter(_target_pos_x);
			_beedancer->StepperY->set_position_meter(_target_pos_y);
			// _beedancer->StepperT->set_position_meter(z);
		}
		
	}
	else{
		Serial.println("The robot is still not calibrated, impossible to move.");
	}
}

void Motion_Control::goto_speed(float x, float y, float t, float df)
{
	_state_of_operation = 2;
	if(_target_speed_x == x && _target_speed_y == y && _target_speed_t == t && _target_speed_df == df){}
	else{
		_target_speed_x = x;
		_target_speed_y = y;
		_target_speed_t = t;
		_target_speed_df = df;
		
		_beedancer->StepperX->set_speed_meter(_target_speed_x);
		_beedancer->StepperY->set_speed_meter(_target_speed_y);
		_beedancer->StepperT->set_speed_meter(_target_speed_t);
		_beedancer->StepperDF->set_speed_meter(_target_speed_df);
	}
}


void Motion_Control::stop()
{
	_target_speed_x = 0.;
	_target_speed_y = 0.;
	_target_speed_t = 0.;
	_target_speed_df = 0.;

	_target_pos_x = 0.;
	_target_pos_y = 0.;
	_target_pos_t = 0.;
	_target_pos_df = 0.;

	_state_of_operation = 0;
}

void Motion_Control::set_moving_current()
{

}

void Motion_Control::step()
{
	if(_state_of_operation == 0){
		goto_speed(0, 0, 0, 0);
	}
	if(_state_of_operation == 1){
	}
	if(_state_of_operation ==)
}

void Motion_Control::check_error()
{
	if(digitalRead(_beedancer->motorCerrorPin)){
		_beedancer->StepperX->check_errors();
		_beedancer->StepperY->check_errors();
		_beedancer->StepperT->check_errors();
		_beedancer->StepperDF->check_errors();
	}
}

void Motion_Control::calibrate()
{	
	float theta = calibrationXY();
}

bool Motion_Control::is_calibrate()
{
	return _is_calibrated;
}

void Motion_Control::extract()
{
	_beedancer->PQ12->extract();
}

void Motion_Control::retract()
{
	_beedancer->PQ12->retract();
}

void Motion_Control::_controller()
{

}

void Motion_Control::resetTimeout()
{
	_beedancer->StepperX->resetCommandTimeout();
	_beedancer->StepperY->resetCommandTimeout();
	_beedancer->StepperT->resetCommandTimeout();
	_beedancer->StepperDF->resetCommandTimeout();
}	

void Motion_Control::init()
{
	_beedancer->StepperX->init();
	_beedancer->StepperY->init();
	_beedancer->StepperT->init();
	_beedancer->StepperDF->init();
	_beedancer->PQ12->init();
}

bool Motion_Control::is_on_target()
{
	return is_x_on_target && is_y_on_target && is_t_on_target && is_df_on_target;
}