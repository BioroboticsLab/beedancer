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
	State_Machine(Motion_Control* beedancer);
	bool handle_message(String* input);
	void step();
	void send_ack();
	String getValue(String data, char separator, int index);

private:
	enum state{unCalibrated, Calibrated, Ready, Dancing};
	state _current_state;
	bool _is_calibrated = false;
	Motion_Control* _beedancer;
	const int _buffer_size = 10;
	const int _instruction_size = 10;
	std::queue<std::vector<float>> _instruction_queue;
};

#endif
