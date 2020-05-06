#ifndef LINEAR_STEPPER_H
#define LINEAR_STEPPER_H

#include <Arduino.h>
#include <Tic.h>

class Linear_Stepper : public TicI2C, public Printable
{
public:
	Linear_Stepper(int addr, int current_limit);
	int m2step(float meter);
	float step2m(int step);
	float get_pos_meter();
	void init();
	void get_micro_step();
	void set_moving_current(bool is_moving);
	size_t printTo(Print& p) const;
	void set_speed_meter(float meterps);
	void set_position_meter(float target);
	void check_errors();
	void set_position_and_acknowledge(int32_t target, bool blocking = false);
	void set_speed_and_acknowledge(int32_t target);
	bool is_idle();

private:
	float _current_position_meter = 0.;
	int32_t _current_position_step = 0;
	uint16_t _current_limit = 0;
	int _state_of_operation = 0; //0:idle ; 1:position target ; 2:speed target
	int32_t _position_target = 0;
	int32_t _speed_target = 0;  
	const float _fullstep_m_ratio = 0.00001; //One step = 10um
	int _addr;
	int _micro_step;
};

#endif