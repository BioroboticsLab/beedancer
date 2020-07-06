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

State_Machine::State_Machine(Motion_Control* beedancer)
{
	_beedancer = beedancer;
	_current_state = unCalibrated;
}

bool State_Machine::handle_message(String* input)
{	
	// First we sychronize the controller with the PC
	if(*input == "who\n"){
    	Serial.println("beeboogie");
    	*input = "";
    	return true;
    }
    //
    if(*input == "how\n"){
    	Serial.println(_current_state);
    	*input = "";
    	return true;
    }
	// If it's already sychronized we process the vector
	else{
		std::vector<float> instruction_vector (_instruction_size);
		for(int i = 0 ; i < _instruction_size ; i++){
			instruction_vector[i] = getValue(*input, ':', i).toFloat();
		}
		*input = "";
		_instruction_queue.push(instruction_vector);
		return true;
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
				if((int)instruction_vector[0] == 0){ //Calibration command received
					_beedancer->calibrate();
					_current_state = Calibrated;
					send_ack();
				}



				/*
				else if((int)instruction_vector[0] == 4){
					_beedancer->dance(instruction_vector[1], instruction_vector[2], instruction_vector[3], instruction_vector[4]);
				}
				else if((int)instruction_vector[0] == 3){
					_beedancer->goto_pos(instruction_vector[1], instruction_vector[2], instruction_vector[3], instruction_vector[4], true);
					send_ack();
				}*/



				else if((int)instruction_vector[0] == 2){ //Retract command received
					_beedancer->retract();
				}
				else{
					Serial.println("Illegal instruction, still not calibrated, please send instruction 0");
				}
				break;

			case Calibrated:
				if((int)instruction_vector[0] == 3){ //MoveTo command received
					_beedancer->goto_pos(instruction_vector[1], instruction_vector[2], instruction_vector[3], instruction_vector[4], true);
					send_ack();
				}
				else if((int)instruction_vector[0] == 1){ //Extract command received
					//_beedancer->goto_pos(0.01859, 0.00681, 4.972, 0., true);
					_beedancer->extract();
					_current_state = Ready;
					send_ack();
				}
				else{
					Serial.println("Illegal instruction, still not extracted");
				}

			case Ready:
				if((int)instruction_vector[0] == 3){ //MoveTo command received
					_beedancer->goto_pos(instruction_vector[1], instruction_vector[2], instruction_vector[3], instruction_vector[4], true);
					send_ack();
				}
				else if((int)instruction_vector[0] == 2){	//Retract command received
					_beedancer->retract();
					_current_state = Calibrated;
					send_ack();
				}
				else if((int)instruction_vector[0] == 4){	//Dance command received
					_beedancer->dance(instruction_vector[1], instruction_vector[2], instruction_vector[3], instruction_vector[4]);
					_current_state = Dancing;
					send_ack();
				}
				else{
					Serial.println("illegal instruction");
				}
				break;

			case Dancing:
				if((int)instruction_vector[0] == 4){
					_beedancer->dance(instruction_vector[1], instruction_vector[2], instruction_vector[3], instruction_vector[4]);
					send_ack();
				}
				if((int)instruction_vector[0] == 3){
					_beedancer->goto_pos(instruction_vector[1], instruction_vector[2], instruction_vector[3], instruction_vector[4], true);
					send_ack();
					_current_state = Ready;
				}
				break;

			default:
				Serial.println("Out of state machine, big problem");
					
		}
	}
	else{}
}

void State_Machine::send_ack(){
	Serial.println("ACK");
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
