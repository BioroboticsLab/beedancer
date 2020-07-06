#ifndef LINEAR_STEPPER_H
#define LINEAR_STEPPER_H

#include <Arduino.h>
#include <AccelStepper.h>

class Linear_Stepper : public AccelStepper
{
public:
	Linear_Stepper(int step_pin, int dir_pin, int enable_pin, int _micro_step = 1);
	int m2step(float meter);
	float step2m(int step);
	float get_pos_meter();
	void set_speed_meter(float meterps);
	void set_max_speed(float mps);
	void set_max_speed();
	void set_acceleration(float mpss);
	void set_acceleration();
	void set_position_meter(float target, bool blocking = false);
	long set_position_meter_overshoot(float target, bool blocking = false);
	long distance_to_go_overshoot();
	bool is_idle();

private:
	float _current_position_meter = 0.;
	int32_t _current_position_step = 0;
	int _step_pin;
	int _dir_pin;
	int _enable_pin;
	int32_t _max_speed_steps;
	int32_t _max_accel_steps;
	int _state_of_operation = 0; //0:idle ; 1:position target ; 2:speed target
	int32_t _position_target = 0;
	int32_t _speed_target = 0;  
	const float _fullstep_m_ratio = 0.00001; //One step = 10um
	int _addr;
	int _micro_step;
	long _acceleration_ramp_length;
};

#endif