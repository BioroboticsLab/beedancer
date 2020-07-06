#ifndef ANGULAR_STEPPER_H
#define ANGULAR_STEPPER_H
#include <Arduino.h>
#include <Tic.h>

class Angular_Stepper : public TicI2C, public Printable
{
public:
	Angular_Stepper(int addr, int current_limit, float reduction_ratio);
	int rad2step(float rad);
	float step2rad(int steps);
	float get_pos_rad();
	void set_max_speed(float max_speed);
	void init();
	void get_micro_step();
	void set_moving_current(bool is_moving);
	size_t printTo(Print& p) const;
	void set_speed_rad(float radps);
	void set_position_rad(float target, bool blocking = false);
	void check_errors();
	void set_position_and_acknowledge(int32_t target);
	void set_speed_and_acknowledge(int32_t target);
	bool is_idle();
	float getPrincipaleAngle(float angleRad);
	float getShortestArc(float current, float target);

private:
	float _current_position_rad = 0.;
	int32_t _current_position_step = 0;
	uint16_t _current_limit = 0;
	int _state_of_operation = 0; //0:idle ; 1:position target ; 2:speed target
	int32_t _position_target = 0;
	int32_t _speed_target = 0;  
	const float _step_rad_ratio = (1.8 / 360.) * 2. * PI; //One step = 1.8°
	float _reduction_ratio = 1.; // The small gear is 12 teeth and the big is 72
	int _addr;
	int _micro_step;
};

#endif