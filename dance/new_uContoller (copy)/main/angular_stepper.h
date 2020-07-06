#ifndef ANGULAR_STEPPER_H
#define ANGULAR_STEPPER_H
#include <Arduino.h>
#include <AccelStepper.h>

class Angular_Stepper : public AccelStepper
{
public:
	Angular_Stepper(int step_pin, int dir_pin, int enable_pin, int micro_step, float reduction_ration);
	int rad2step(float rad);
	float step2rad(int steps);
	float get_pos_rad();
	void set_max_speed(float radps);
	void set_max_speed();
	void set_acceleration(float radpss);
	void set_acceleration();
	void set_moving_current(bool is_moving);
	void set_speed_rad(float radps);
	void set_position_rad(float target, bool blocking = false);
	long set_position_rad_overshoot(float target, bool blocking = false);
	bool is_idle();
	float getPrincipaleAngle(float angleRad);
	float getShortestArc(float current, float target);

private:
	float _current_position_rad = 0.;
	int32_t _current_position_step = 0;
	int _state_of_operation = 0; //0:idle ; 1:position target ; 2:speed target
	int32_t _position_target = 0;
	int32_t _speed_target = 0; 
	int32_t _max_speed_steps; 
	int32_t _max_accel_steps;
	const float _step_rad_ratio = 200. / (2. * PI); //One step = 1.8° so 200 step
	float _reduction_ratio = 1.; // The small gear is 12 teeth and the big is 72
	int _micro_step;
	int _step_pin;
	int _dir_pin;
	int _enable_pin;
	int _is_direct;
	long _acceleration_ramp_length;
};

#endif