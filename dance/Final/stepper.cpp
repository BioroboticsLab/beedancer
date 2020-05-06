#include "stepper.h"
Stepper::Stepper(TicI2C Stepper)
{
	_Stepper = Stepper;
	_position_um = 0;
	_position_step = 0;
}

int Stepper::um2step(int um)
{
	return um / step_um_ratio;
}

int Stepper::step2um(int steps)

int Stepper::getPos_um()

int Stepper::getPos_step()