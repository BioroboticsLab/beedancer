#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H
#include <Arduino.h>
#include "linear_stepper.h"
#include "robot.h"
#include "calibration.h"

class Motion_Control
{
public:
	Motion_Control(robot* beedancer);
	void goto_pos(float x, float y, float t, float df);
	void goto_speed(float x, float y, float t, float df);
	void calibrate();
	bool is_calibrate();
	bool is_calibrating();
	void resetTimeout();
	void init();
	void retract();
	void extract();
	bool is_on_target();
	void step();
	void stop();
	void check_error();
	void set_moving_current();

private:
	robot* _beedancer;
  	void _controller();
  	bool _is_calibrated;
  	bool _in_calibration;
  	int _calibration_step;

  	bool _speed_changed = false;
  	bool _pos_changed = false;

	bool is_x_on_target = true;
	bool is_y_on_target = true;
	bool is_t_on_target = true;
	bool is_df_on_target = true;
	int _state_of_operation = 0; //0:idle ; 1:position target ; 2:speed target ; 3:in calibration ; 4:move periodically

	float _target_pos_x = 0.;
	float _target_pos_y = 0.;
	float _target_pos_t = 0.;
	float _target_pos_df = 0.;

	float _target_speed_x = 0.;
	float _target_speed_y = 0.;
	float _target_speed_t = 0.;
	float _target_speed_df = 0.;

  	float _theta_robot_comb = 0.;
};
#endif