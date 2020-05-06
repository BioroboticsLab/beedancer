#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <Arduino.h>
#include <queue>
#include "robot.h"
#include "calibration.h"
#include "motion_control.h"

class State_Machine
{
public:
	State_Machine(Motion_control* beedancer);
	void handle_message(String* input);
	void step();
	bool runCalibration();
	bool extract();
	bool retract();
	bool speedMove();
	bool positionMove();
	String getValue(String data, char separator, int index);

private:
	enum state{unCalibrated, Calibration, Calibrated, Ready, SpeedMove, PositionMove};
	state _current_state;
	bool _is_calibrated = false;
	Motion_control* _beedancer;
	const int _buffer_size = 10;
	const int _instruction_size = 10;
	std::queue<std::vector<float>> _instruction_queue;
};

#endif