#include "motion_control.h"
#include "angular_stepper.h"
#include <queue>

#define PI 3.14159265

Motion_Control::Motion_Control(robot* beedancer)
{
	_beedancer = beedancer;
	_is_calibrated = true;   //TO BE ON FALSE
	_calibration_step = 0;
}

void Motion_Control::goto_pos(float x, float y, float t, bool blocking)
{

	if(_is_calibrated || _in_calibration){
		if(_target_pos_x == x && _target_pos_y == y && _target_pos_t == t){}
		else{
			_state_of_operation = 1;
			_target_pos_x = x;
			_target_pos_y = y;
			_target_pos_t = t;
			
			_beedancer->StepperX->set_position_meter(_target_pos_x);
			_beedancer->StepperY->set_position_meter(_target_pos_y);
			_beedancer->StepperT->set_position_rad(_target_pos_t);

			while(!is_on_target()){
				vTaskDelay(1);
			}
		}
		
	}
	else{
		Serial.println("The robot is still not calibrated, impossible to move.");
	}
}

void Motion_Control::goto_speed(float x, float y, float t)
{
	_state_of_operation = 2;
	if(_target_speed_x == x && _target_speed_y == y && _target_speed_t == t){}
	else{
		_target_speed_x = x;
		_target_speed_y = y;
		_target_speed_t = t;
		
		_beedancer->StepperX->set_speed_meter(_target_speed_x);
		_beedancer->StepperY->set_speed_meter(_target_speed_y);
		_beedancer->StepperT->set_speed_rad(_target_speed_t);
	}
}

void Motion_Control::dance(float x, float y, float t, float interval_milliS){
	std::vector<float> instruction_vector(4);
	instruction_vector[0] = x;
	instruction_vector[1] = y;
	instruction_vector[2] = t;
	instruction_vector[3] = interval_milliS;

	_instruction_queue.push(instruction_vector);

	_state_of_operation = 3;
}

void Motion_Control::rotate_dancefloor(float target, float speed){
	// This function rotate the dancefloor, it's necessarely blocking
	_target_pos_df = target;
}


void Motion_Control::stop()
{
	_state_of_operation = 0;
}

void Motion_Control::set_moving_current()
{

}

void Motion_Control::step()
{
	switch(_state_of_operation){
		case 0:
			_beedancer->StepperX->set_speed_meter(0);
			_beedancer->StepperY->set_speed_meter(0);
			_beedancer->StepperT->set_speed_rad(0);
			_beedancer->StepperDF->set_speed_rad(0);
			break;
		case 1:
			if(is_on_target()){
				_state_of_operation = 0;
			}
			break;
		case 2:
			break;
		case 3:
			if((micros() - _time) > _instruction_vector[3]*1000){
				if(!_instruction_queue.empty()){
					_instruction_vector = _instruction_queue.front();
					_instruction_queue.pop();
					float time_coef = 1000. / _instruction_vector[3];
					float command_x_speed = 0.6 * (_instruction_vector[0] - _beedancer->StepperX->get_pos_meter()) * time_coef;
					float command_y_speed = 0.6 * (_instruction_vector[1] - _beedancer->StepperY->get_pos_meter()) * time_coef;
					float command_t_speed = 0.2 * (_instruction_vector[2] - _beedancer->StepperT->get_pos_rad()) * time_coef;
					/*
					Serial.print("xspeed : ");
					Serial.print(command_x_speed);
					Serial.print(" ; yspeed : ");
					Serial.print(command_y_speed);
					Serial.print(" ; tspeed : ");
					Serial.println(command_t_speed);*/
					_beedancer->StepperX->set_speed_meter(command_x_speed);
					_beedancer->StepperY->set_speed_meter(command_y_speed);
					_beedancer->StepperT->set_speed_rad(command_t_speed);
					_time = micros();
				}
				else{
					stop();
/*					Serial.print("Time : ");
					Serial.println(micros()-_time);
					Serial.print("xtarget: ");
					Serial.print(_instruction_vector[0], 6);
					Serial.print("/");
					Serial.print(_beedancer->StepperX->get_pos_meter(), 6);
					Serial.print("Ytarget: ");
					Serial.print(_instruction_vector[1], 6);
					Serial.print("/");
					Serial.print(_beedancer->StepperY->get_pos_meter(), 6);
					Serial.print("Ttarget: ");
					Serial.print(_instruction_vector[2], 6);
					Serial.print("/");
					Serial.print(_beedancer->StepperT->get_pos_rad(), 6);
					*/

					_counter = 0;
					_instruction_vector = {0.,0.,0.,0.};
				}
			}
			else{}
			break;
	}
}

void Motion_Control::check_error()
{
	_beedancer->StepperX->check_errors();
	_beedancer->StepperY->check_errors();
	_beedancer->StepperT->check_errors();
	_beedancer->StepperDF->check_errors();
}

void Motion_Control::calibrate()
{	
	float theta = calibrationXY(_beedancer);
	_is_calibrated = true;
	goto_pos(0.0001, 0.0001, 0.0, true);
	calibrationDF(_beedancer);
	_beedancer->StepperDF->set_position_rad(-((3 * PI)/4.), true);
	_beedancer->StepperX->haltAndSetPosition(0);
	calibrationT(_beedancer);
	_beedancer->StepperT->set_position_rad(-PI/2, true);
	_beedancer->StepperT->haltAndSetPosition(0);
	goto_pos(0.0292, 0.0273, 0.0, true);
	_beedancer->StepperX->haltAndSetPosition(0);
	_beedancer->StepperY->haltAndSetPosition(0);
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
	return _beedancer->StepperX->is_idle() && _beedancer->StepperY->is_idle() && _beedancer->StepperT->is_idle() && _beedancer->StepperDF->is_idle();
}