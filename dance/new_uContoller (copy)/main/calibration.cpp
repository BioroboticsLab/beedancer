#include "linear_stepper.h"
#include "angular_stepper.h"
#include "motion_control.h"
#include <Tic.h>
#include "calibration.h"
#include "robot.h"
#include <math.h>

#define PI 3.14159265

float calibrationXY(robot* beedancer)
{
	float deltax = 0.;
	float deltay = 0.04;
	float theta = 0.;

	if(beedancer->PQ12->is_extracted()){
		Serial.println("Robot is in extracted position or extracting, please solve this problem first.");
		//return -1;
	}

	findOrigine(beedancer->StepperX, beedancer->xSwitch, true);
	beedancer->StepperX->set_position_meter(0.002, true);

	findOrigine(beedancer->StepperY, beedancer->ySwitch, true);
	beedancer->StepperY->set_position_meter(0.002, true);

	findOrigine(beedancer->StepperX, beedancer->xSwitch, true);
	beedancer->StepperX->set_position_meter(0.002, true);

	beedancer->StepperY->set_position_meter(deltay, true);

	findOrigine(beedancer->StepperX, beedancer->xSwitch, false);
	deltax = beedancer->StepperX->get_pos_meter();
	beedancer->StepperX->set_position_meter(0.002, true);

	theta = atan(deltax / deltay) * 180 / PI;

	return theta;
}



float calibrationT(robot* beedancer)
{
	delay(200);
	int nb_step = 500;
	int values[nb_step];
	int sensor_Value = 0;
	digitalWrite(beedancer->led_pin, HIGH);
	int max_v = INT_MIN;
	int max_i = 0;
	float first_middle = 0.;
	float final_middle1, final_middle2, final_middle3;

	first_middle = findOriginePhoto(beedancer->StepperT, beedancer->photo_res, 0, 2*PI, 500, 0);

	final_middle1 = findOriginePhoto(beedancer->StepperT, beedancer->photo_res, first_middle - PI / 6, first_middle + PI / 6, 200, 0); //first_middle - PI/6 + max_i * (PI/3) / nb_step;
	final_middle2 = findOriginePhoto(beedancer->StepperT, beedancer->photo_res, first_middle + PI / 6, first_middle - PI / 6, 200, 0);
	final_middle3 = findOriginePhoto(beedancer->StepperT, beedancer->photo_res, first_middle - PI / 6, first_middle + PI / 6, 200, 0);

	beedancer->StepperT->set_position_rad((final_middle1+final_middle2+final_middle3)/3, true);
	beedancer->StepperT->setCurrentPosition(0);

	digitalWrite(beedancer->led_pin,LOW);

}

float calibrationDF(robot* beedancer){
	findOrigineDF(beedancer);
}

float findOrigineDF(robot* beedancer){	
	const int repeat = 3;
	float switchPlus[repeat];
	float switchMinus[repeat];
	float switchAverage[repeat];
	float average_origine;
	//We do it repeat time for averging
	for(int i = 0 ; i < repeat ; i++){

		beedancer->StepperDF->set_speed_rad(PI);
		while(!beedancer->dfSwitch->falling()){beedancer->dfSwitch->read();beedancer->StepperDF->runSpeed();}
		while(!beedancer->dfSwitch->rising()){beedancer->dfSwitch->read();beedancer->StepperDF->runSpeed();}

		switchPlus[i] = beedancer->StepperDF->get_pos_rad();

		beedancer->StepperDF->set_speed_rad(-PI);
		while(!beedancer->dfSwitch->falling()){beedancer->dfSwitch->read();beedancer->StepperDF->runSpeed();}
		while(!beedancer->dfSwitch->rising()){beedancer->dfSwitch->read();beedancer->StepperDF->runSpeed();}

		switchMinus[i] = beedancer->StepperDF->get_pos_rad();

	}
	//Computing an average
	for(int i = 0 ; i < repeat ; i++)
	{
		switchAverage[i] = (switchPlus[i] + switchMinus[i]) / 2.;
	}
	for(int i = 0 ; i < repeat ; i++)
	{
		average_origine += switchAverage[i];
	}

	average_origine = average_origine / (float)repeat;

	beedancer->StepperDF->set_position_rad(average_origine, true);

	beedancer->StepperDF->setCurrentPosition(0);
}

float findOriginePhoto(Angular_Stepper* Stepper, int photo_res, float from, float to, int nb_points, int exclude_border){
	int values[nb_points];
	int max_v = INT_MIN;
	int max_i = 0;

	//Second slow round
	for(int i = 0 ; i < nb_points  ; i++){
		
		Stepper->set_position_rad(from + ((i * (to - from)) / nb_points), true);
		delay(10);

		int mean = 0;
		for(int j=0 ; j < 10 ; j++){
			mean = mean + analogRead(photo_res);
		}

		
		mean = (int)(mean / 10);
		//Serial.println(mean);
		values[i] = mean;
	}
	// Square convolution
	for(int i=1; i < sizeof(values)/sizeof(values[0])-1; i++ ){
		values[i] = (values[i-1] + values[i] + values[i+1])/3.;
	}

	//Compute index of max value
	for ( int i = exclude_border; i < sizeof(values)/sizeof(values[0]) - exclude_border; i++ )
	{
		if ( values[i] > max_v )
		{
			max_v = values[i];
			max_i = i;
		}
	}
	return (from + ((max_i * (to - from)) / nb_points));
}

float findOrigine(Linear_Stepper* Stepper, DebouncedInput* switchPin, bool setWhenFound)
{
	delay(200);
	const int repeat = 3;
	float switchPlus[3] = {0.,0.,0.};
	float switchMinus[3] = {0.,0.,0.};
	float switchAverage[3] = {0.,0.,0.};
	float current_position = 0.;
	float average_origine = 0.;

	//Find first the border
	Stepper->set_speed_meter(-0.01);

	//While we don't find it
	while(switchPin->low()){switchPin->read();Stepper->runSpeed();}

	//We do it repeat time for averging
	for(int i = 0 ; i < repeat ; i++)
	{
		Stepper->set_speed_meter(0.001);
		while(switchPin->high()){switchPin->read();Stepper->runSpeed();}
		switchPlus[i] = Stepper->get_pos_meter();

		Stepper->set_speed_meter(-0.001);
		while(switchPin->low()){switchPin->read();Stepper->runSpeed();}

		switchMinus[i] = Stepper->get_pos_meter();
	}

	//Computing an average
	for(int i = 0 ; i < repeat ; i++)
	{
		switchAverage[i] = (switchPlus[i] + switchMinus[i]) / 2.;
	}
	for(int i = 0 ; i < repeat ; i++)
	{
		average_origine += switchAverage[i];
	}

	average_origine = average_origine / (float)repeat;

	Stepper->set_position_meter(average_origine, true);

	if(setWhenFound)
	{
		Stepper->setCurrentPosition(0);
	}

	return average_origine;
}