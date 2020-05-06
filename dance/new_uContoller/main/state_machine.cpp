#include <Arduino.h>
#include <queue>
#include "state_machine.h"
#include "robot.h"
#include "calibration.h"

/*
This is the "brain of the robot"
	SYCHRO : for connecting to the computer, we wait for it to send "who\n" the we send back
"beeboogie\n". this is a simple handshake.
	Instruction vector : composed of ten float in char ex : "0:1.42:8.65:7.658...x10" separated
by ":". The first number is the instruction the rest are the variable of this instruction.
*/

State_Machine::State_Machine(Motion_control* beedancer)
{
	_beedancer = beedancer;
	_current_state = state.unCalibrated;
}

void State_Machine::handle_message(String* input)
{	
	// First we sychronize the controller with the PC
	if(*input == "who\n"){
    	Serial.println("beeboogie\n");
    	}
	// If it's already sychronized we process the vector
	else{
		std::vector<float> instruction_vector (_instruction_size);
		for(int i = 0 ; i < _instruction_size ; i++){
			instruction_vector[i] = getValue(*input, ':', i).toFloat();
			Serial.println(instruction_vector[i]);
		}
		*input = "";
		_instruction_queue.push(instruction_vector);
		Serial.println(_instruction_queue.size());
	}
}

void State_Machine::step()
{
	std::vector<float> instruction_vector (_instruction_size);

	if(!_instruction_queue.empty()){
		instruction_vector = _instruction_queue.front();
		_instruction_queue.pop();
		switch(_current_state){
			case unCalibrated :
				if((int)instruction_vector[0] == 0){
					_current_state = state.Calibration;
				}
				else{
					Serial.println("Illegal instruction, still not calibrated, please send instruction 0")
				}
				break;

			case Calibration:
				_beedancer->calibrate();
				_current_state = state.Calibrated

			case Ready:
				
		}
		/*switch((int)instruction_vector[0]){
			case 0 : _beedancer->calibrate(); // case 0 = calibration
					Serial.println("calibrate");
					break;

			case 1 : _beedancer->extract(); // case 1 = extraction
					Serial.println("extract");
					break;
			case 2 : _beedancer->retract(); // case 2 = retraction
					Serial.println("retract");
					break;
			case 3 : _beedancer->goto_target(instruction_vector[1],
											 instruction_vector[2], 
											 instruction_vector[3]); // case 3 = move to blocking
					Serial.println("move to");
					break;

			case 4 : //; // case 4 = move in the rythm 
					Serial.println("retract");
					break;
			case 5 : //; // case 4 = 
					Serial.println("retract");
					break;
		}*/
	}
	else{}
}

String State_Machine::getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
