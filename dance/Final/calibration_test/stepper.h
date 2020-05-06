#ifndef STEPPER_H
#define STEPPER_H
#include <Arduino.h>
#include <Tic.h>

#define PI 3.14159265

class Stepper: public TicI2C
{
public:
	Stepper(int addr, int max_current=0, bool is_linear=true);
	int m2step(float m);
	float step2m(int steps);
	float getPos();
	int getPos_step();
	int getMicro_step();
	void setVelocity(float target);
	void setPosition(float target, bool blocking = false);
	void init();
	void handleError(uint32_t errors, int target);
	float getShortestArc(float current, float target);
	float getPrincipaleAngle(float angleRad);
	float step2rad(int steps);
	int rad2step(float deg);

private:
	float _position;
	int _current_limit;
	int _position_step;
	bool _is_linear;
	const float _step_m_ratio = 0.00001; //One step = 10um
	const float _step_rad_ratio = (1.8 / 360.) * 2. * PI; //One step = 1.8°
	const float _reduction_ratio = 11./72.; // The small gear is 12 teeth and the big is 72
	int _addr;
	int _micro_step;
};

#endif