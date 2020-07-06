#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H
#include <Arduino.h>
#include "linear_stepper.h"
#include "robot.h"
#include "calibration.h"
#include <queue>

class Motion_Control
{
public:
	Motion_Control(robot* beedancer);
	void goto_pos(float x, float y, float t, float df, bool blocking = false);
	void goto_speed(float x, float y, float t);
	void dance(float x, float y, float t, float interval_milliS);
	void calibrate();
	bool is_calibrate();
	bool is_calibrating();
	void retract();
	void extract();
	bool is_on_target();
	void step();
	void stop();
	void check_error();

private:
	robot* _beedancer;
  	bool _is_calibrated;
  	bool _in_calibration;
  	int _calibration_step;

  	bool _speed_changed = false;
  	bool _pos_changed = false;

  	int _counter = 0;
  	int _time = 0.;

	bool is_x_on_target = true;
	bool is_y_on_target = true;
	bool is_t_on_target = true;
	bool is_df_on_target = true;
	int _state_of_operation = 0; //0:idle ; 1:position target ; 2:speed target ; 3:in calibration ; 4:move periodically

	float _target_pos_x = 0.;
	float _target_pos_y = 0.;
	float _target_pos_t = 0.;
	float _target_pos_df = 0.;

	long _target_pos_x_step = 0;
	long _target_pos_y_step = 0;
	long _target_pos_t_step = 0;
	long _target_pos_df_step = 0;

	float _target_posovershoot_x = 0.;
	float _target_posovershoot_y = 0.;
	float _target_posovershoot_t = 0.;
	float _target_posovershoot_df = 0.;

	float _target_speed_x = 0.;
	float _target_speed_y = 0.;
	float _target_speed_t = 0.;
	float _target_speed_df = 0.;

	long _quadratic_distance_to_go = 0;

  	float _theta_robot_comb = 0.;

  	float _dance_floor_angle = 0.;

  	int _i = 0;
  	int _repeat = 0;

  	std::vector<std::vector<float>> _instruction_register;
  	std::vector<float> _instruction_vector{std::vector<float>(4 ,0.)};
};
#endif