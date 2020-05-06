#include "stepper.h"
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
		return -1;
	}

	findOrigine(beedancer->StepperX, beedancer->xSwitch, true);
	beedancer->StepperX->setPosition(0.002, true);

	findOrigine(beedancer->StepperY, beedancer->ySwitch, true);
	beedancer->StepperY->setPosition(0.002, true);

	findOrigine(beedancer->StepperX, beedancer->xSwitch, true);
	beedancer->StepperX->setPosition(0.002, true);
	beedancer->StepperY->setPosition(deltay, true);

	findOrigine(beedancer->StepperX, beedancer->xSwitch, false);
	deltax = beedancer->StepperX->getPos();
	beedancer->StepperX->setPosition(0.002, true);

	theta = atan(deltax / deltay) * 180 / PI;

	return theta;
}

float calibrationDF(robot* beedancer)
{	
	findOrigine(beedancer->StepperDF, beedancer->dfSwitch, true, true);	
	findOrigine(beedancer->StepperDF, beedancer->dfSwitch, false, true);
	Serial.println(beedancer->StepperDF->getCurrentPosition());
	beedancer->StepperDF->setPosition(-3*PI/4, true);
	beedancer->StepperDF->haltAndSetPosition(0);
}

float findOrigine(Stepper* Stepper, DebouncedInput* switchPin, bool setWhenFound, bool isCircular)
{
	delay(200);
	const int repeat = 3;
	float switchPlus[3] = {0.,0.,0.};
	float switchMinus[3] = {0.,0.,0.};
	float switchAverage[3] = {0.,0.,0.};
	float current_position = 0.;
	float average_origine = 0.;

	if(!isCircular){
		//Find first the border
		Stepper->setVelocity(-0.005);

		//While we don't find it
		while(!switchPin->low()){switchPin->read();vTaskDelay(1);}
		Serial.println("!0");
		Stepper->haltAndHold();
		vTaskDelay(100);

		//We do it repeat time for averging
		for(int i = 0 ; i < repeat ; i++)
		{
			Serial.println("!1");
			Stepper->setVelocity(0.001);
			Serial.println("!2");
			while(!switchPin->high()){switchPin->read();vTaskDelay(1);}
			Serial.println("!3");
			Stepper->haltAndHold();
			Serial.println("!4");
			vTaskDelay(100);
			switchPlus[i] = Stepper->getPos();
			Serial.println("!5");

			Stepper->setVelocity(-0.001);
			while(!switchPin->low()){switchPin->read();vTaskDelay(1);}
			Stepper->haltAndHold();
			vTaskDelay(100);

			switchMinus[i] = Stepper->getPos();
		}
	}
	else
	{
		//We do it repeat time for averging
		for(int i = 0 ; i < repeat ; i++){

			Stepper->setVelocity(PI/10.);

			while(!switchPin->falling()){switchPin->read();vTaskDelay(1);}
			while(!switchPin->rising()){switchPin->read();vTaskDelay(1);}

			switchPlus[i] = Stepper->getPos();

			vTaskDelay(100);
			Stepper->haltAndHold();
			vTaskDelay(100);

			Stepper->setVelocity(-PI/10.);
			while(!switchPin->falling()){switchPin->read();vTaskDelay(1);}
			while(!switchPin->rising()){switchPin->read();vTaskDelay(1);}

			switchMinus[i] = Stepper->getPos();

			vTaskDelay(100);
			Stepper->haltAndHold();
			vTaskDelay(100);

		}
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

	current_position = Stepper->getPos();

	Stepper->setPosition(average_origine, true);

	if(setWhenFound)
	{
		Stepper->haltAndSetPosition(0);
	}

	return current_position;
}