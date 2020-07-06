#include "motion_control.h"
#include "angular_stepper.h"
#include <math.h>
#include <queue>

#define PI 3.14159265

Motion_Control::Motion_Control(robot* beedancer)
{
	_beedancer = beedancer;
	_is_calibrated = true;   //SHALL BE ON FALSE
	_calibration_step = 0;
}

void Motion_Control::goto_pos(float x, float y, float t, float df, bool blocking)
{
	if(_is_calibrated || _in_calibration){
		if(_target_pos_x == x && _target_pos_y == y && _target_pos_t == t){}
		else{
			_state_of_operation = 1;
			_target_pos_x = x;
			_target_pos_y = y;
			_target_pos_t = t;
			_target_pos_df = df;
			
			_beedancer->StepperX->set_position_meter(x);
			_beedancer->StepperY->set_position_meter(y);
			_beedancer->StepperT->set_position_rad(t);
			_beedancer->StepperDF->set_position_rad(df);

			_target_pos_x_step = _beedancer->StepperX->targetPosition();
			_target_pos_y_step = _beedancer->StepperY->targetPosition();
			_target_pos_t_step = _beedancer->StepperT->targetPosition();
			_target_pos_df_step = _beedancer->StepperDF->targetPosition();

			if(blocking){
				//Serial.println(is_on_target());
				while(!is_on_target()){step();}
				_state_of_operation = 0;
				step();
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

	//Serial.print(x);
	//Serial.print(y);
	//Serial.println(t);
	if(x == y && y == t){
		_repeat = (int)x;
		_state_of_operation = 3;
	}
	else{
		//Serial.print("Size of the buffer : ");
		//Serial.println(_instruction_register.size());
		_instruction_register.push_back(instruction_vector);
	}
}


void Motion_Control::stop()
{
	_state_of_operation = 0;
}

void Motion_Control::step()
{
	switch(_state_of_operation){
		case 0:
			_beedancer->StepperX->stop();
			_beedancer->StepperY->stop();
			_beedancer->StepperT->stop();
			_beedancer->StepperDF->stop();
			_state_of_operation = 1;
			break;
		case 1:
			if(is_on_target()){
				_state_of_operation = 0;
			}
			_beedancer->StepperX->run();
			_beedancer->StepperY->run();
			_beedancer->StepperT->run();
			_beedancer->StepperDF->run();
			break;

		case 2:
			_beedancer->StepperX->runSpeed();
			_beedancer->StepperY->runSpeed();
			_beedancer->StepperT->runSpeed();
			_beedancer->StepperDF->runSpeed();
			break;

		case 3:
			// Serial.print("xtrue pos : ");
			// Serial.print(_beedancer->StepperX->currentPosition());
			// Serial.print(" ; xtarget pos : ");
			// Serial.print(_target_pos_x_step);
			// Serial.print(" ; ytrue pos : ");
			// Serial.print(_beedancer->StepperY->currentPosition());
			// Serial.print(" ; ytarget pos : ");
			// Serial.print(_target_pos_y_step);
			//Serial.print("XDover : ");
			//Serial.print(_beedancer->StepperX->distance_to_go_overshoot());
			//Serial.print("YDover : ");
			//Serial.println(_beedancer->StepperY->distance_to_go_overshoot());
			if(_beedancer->StepperX->distance_to_go_overshoot() <= 0 && _beedancer->StepperY->distance_to_go_overshoot() <= 0){ //|| _beedancer->StepperT->currentPosition() == _target_pos_t_step(_beedancer->StepperX->currentPosition() == _target_pos_x_step || _beedancer->StepperY->currentPosition() == _target_pos_y_step)
				if(_i < _instruction_register.size() - 1){
					// Serial.println("In if loop");
					_i = _i + 1;
					_instruction_vector = _instruction_register[_i];
					float time_ms = ((float)_instruction_vector[3]) / 1000.;
					float command_x_speed = (_instruction_vector[0] - _beedancer->StepperX->get_pos_meter()) / time_ms;
					float command_y_speed = (_instruction_vector[1] - _beedancer->StepperY->get_pos_meter()) / time_ms;
					float command_t_speed = (_instruction_vector[2] - _beedancer->StepperT->get_pos_rad()) / time_ms;
					
					// Serial.print("xspeed : ");
					// Serial.print(abs(_beedancer->StepperX->m2step(command_x_speed)));
					// Serial.print(" ; yspeed : ");
					// Serial.print(abs(_beedancer->StepperY->m2step(command_y_speed)));
					// Serial.print(" ; tspeed : ");
					// Serial.println(abs(_beedancer->StepperT->rad2step(command_t_speed)));
					_beedancer->StepperX->setMaxSpeed(abs(_beedancer->StepperX->m2step(command_x_speed)));
					_beedancer->StepperY->setMaxSpeed(abs(_beedancer->StepperY->m2step(command_y_speed)));
					_beedancer->StepperT->setMaxSpeed(abs(_beedancer->StepperT->rad2step(command_t_speed)));

					_target_pos_x = _instruction_vector[0];
					_target_pos_y = _instruction_vector[1];
					_target_pos_t = _instruction_vector[2];
			
					_target_pos_x_step = _beedancer->StepperX->set_position_meter_overshoot(_instruction_vector[0]);
					_target_pos_y_step = _beedancer->StepperY->set_position_meter_overshoot(_instruction_vector[1]);
					_target_pos_t_step = _beedancer->StepperT->set_position_rad_overshoot(_instruction_vector[2]);
				}
				else if(_i == _instruction_register.size() - 1 && _repeat == 1){

						// Serial.print("i= ");
						// Serial.println(i);
						// Serial.println("In else loop");
					_beedancer->StepperX->set_acceleration();
					_beedancer->StepperX->set_max_speed();
					_beedancer->StepperY->set_acceleration();
					_beedancer->StepperY->set_max_speed();
					_beedancer->StepperT->set_acceleration();
					_beedancer->StepperT->set_max_speed();
									// Serial.print("xtrue pos : ");
						// Serial.print(_beedancer->StepperX->currentPosition());
						// Serial.print(" ; xtarget pos : ");
						// Serial.print(_target_pos_x_step);
						// Serial.print(" ; ytrue pos : ");
						// Serial.print(_beedancer->StepperY->currentPosition());
						// Serial.print(" ; ytarget pos : ");
						// Serial.print(_target_pos_y_step);
						// Serial.print(" ; ttrue pos : ");
						// Serial.print(_beedancer->StepperT->currentPosition());
						// Serial.print(" ; ttarget pos : ");
						// Serial.println(_target_pos_t_step);
					goto_pos(_beedancer->StepperX->step2m(_target_pos_x_step), _beedancer->StepperY->step2m(_target_pos_y_step), _beedancer->StepperT->step2rad(_target_pos_t_step), _beedancer->StepperDF->step2rad(_beedancer->StepperDF->currentPosition()), false);
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
					_i = 0;
					_instruction_register.clear();
					_instruction_vector = {0.,0.,0.,0.};
				}
				else{
					_i = 0;
					_repeat = _repeat - 1; 
				}
			}
			else{
				_beedancer->StepperX->run();
				_beedancer->StepperY->run();
				_beedancer->StepperT->run();
				_beedancer->StepperDF->run();
			}
			break;
	}
}

void Motion_Control::calibrate()
{	
	_in_calibration = true;
	float theta = calibrationXY(_beedancer);
	_is_calibrated = true;
	goto_pos(0.0001, 0.0001, 0.0, 0.0, true);
	//calibrationDF(_beedancer);
	//_beedancer->StepperDF->set_position_rad(-((3 * PI)/4.), true);
	calibrationT(_beedancer);
	_beedancer->StepperT->set_position_rad(-PI/2, true);
	_beedancer->StepperT->setCurrentPosition(0);
	goto_pos(0.0285, 0.028, 0.0, 0.0, true);
	_beedancer->StepperX->setCurrentPosition(0);
	_beedancer->StepperY->setCurrentPosition(0);
	_in_calibration=false;
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

bool Motion_Control::is_on_target()
{
	return (_beedancer->StepperX->is_idle() && _beedancer->StepperY->is_idle() && _beedancer->StepperT->is_idle() && _beedancer->StepperDF->is_idle());
}
