#ifndef STEPPER_H
#define STEPPER_H
#include <Arduino.h>
#include <Tic.h>

class Stepper
{
public:
	Stepper(TicI2C Stepper);
	int um2step(int um);
	int step2um(int steps);
	int getPos_um();
	int getPos_step();

private:
	int _position_um;
	int _position_step;
	const int _step_um_ratio = 10; //One step = 10um
	TicI2C _Stepper;
}

#endif