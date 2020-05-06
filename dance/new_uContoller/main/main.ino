#include "linear_stepper.h"
#include "robot.h"
#include "motion_control.h"
#include <Tic.h>
#include <DebounceInput.h>

#define ADDR_STEPPER_X 12
#define CURRENT_STEPPER_X 400

#define ADDR_STEPPER_Y 11
#define CURRENT_STEPPER_Y 400

#define ADDR_STEPPER_T 10
#define CURRENT_STEPPER_T 400

#define ADDR_STEPPER_DF 9
#define CURRENT_STEPPER_DF 600

#define PULLUP_PIN 1
#define PULLDOWN_PIN_1 6
#define PULLDOWN_PIN_2 7

#define MOTERR_PIN 18

#define INTERVAL_SM 10
#define INTERVAL_MC 1

unsigned long millisPrevSM = 0;
unsigned long millisPrevMC = 0;

Linear_Stepper StepperX(ADDR_STEPPER_X, CURRENT_STEPPER_X);
Linear_Stepper StepperY(ADDR_STEPPER_Y, CURRENT_STEPPER_Y);
Linear_Stepper StepperT(ADDR_STEPPER_T, CURRENT_STEPPER_T);
Linear_Stepper StepperDF(ADDR_STEPPER_DF, CURRENT_STEPPER_DF);

// The string used for each communication
String inputString = "";
bool stringComplete = false;  // whether the string is complete

robot beedancer;

Motion_Control Controller(&beedancer);

volatile SemaphoreHandle_t I2CMutex;

// Perform an action every 10 ticks.
void StepperErrorFunction( void * pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xDelay = 1;

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		//Things to do constantly
		Controller.check_error();
		if(xSemaphoreTake(I2CMutex, xDelay) == pdTRUE){

     	xSemaphoreGive(I2CMutex);
    	}
	}
}

void IRAM_ATTR StepperError() {
	Serial.println("ISR stepper error trigger");
	Controller.check_error();
	Controller.check_error();
}

// Perform an action every 10 ticks.
void TESTFunction( void * pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;
	const TickType_t xDelay = 100;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	StepperX.set_moving_current(true);
	StepperY.set_moving_current(true);

	for( ;; )
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		if(xSemaphoreTake(I2CMutex, xDelay) == pdTRUE){
			Controller.goto_speed(-0.002 , -0.002, 0, 0);
     	xSemaphoreGive(I2CMutex);
    	}
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		if(xSemaphoreTake(I2CMutex, xDelay) == pdTRUE){
			Controller.goto_speed(0.002 , 0.002, 0, 0);
     	xSemaphoreGive(I2CMutex);
    	}
	}

}

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	vTaskDelay(20);
	Wire.begin();
	vTaskDelay(20);

	StepperX.init();
	StepperY.init();
	StepperT.init();
	StepperDF.init();

	  // All the input outputs of the robot are in a data structure
	beedancer.StepperT = &StepperT;
	beedancer.StepperY = &StepperY;
	beedancer.StepperX = &StepperX;
	beedancer.StepperDF = &StepperDF;
	beedancer.motorCerrorPin = MOTERR_PIN;

	pinMode(PULLUP_PIN, OUTPUT);
	digitalWrite(PULLUP_PIN, HIGH);

	pinMode(MOTERR_PIN, INPUT);
	attachInterrupt(MOTERR_PIN, StepperError, RISING);

	I2CMutex = xSemaphoreCreateMutex(); 

	// reserve 200 bytes for the inputString:
	inputString.reserve(200);

	  // Creating the task who send a timeout flag to the stepper controller
	/*xTaskCreate(
		StepperErrorFunction, // Task function.
		"StepperError", // String with name of task.
		10000, // Stack size in words.
		NULL, // Parameter passed as input of the task
		1, // Priority of the task.
		NULL); // Task handle.*/
	// Creating the task who send a timeout flag to the stepper controller
	/*xTaskCreate(
		TESTFunction, // Task function.
		"Test", // String with name of task.
		10000, // Stack size in words.
		NULL, // Parameter passed as input of the task
		1, // Priority of the task.
		NULL); // Task handle.*/
	StepperX.set_moving_current(true);
	StepperY.set_moving_current(true);

	millisPrevSM = millis();
	millisPrevMC = millis();
}

void loop() {
	unsigned long currentMillis = millis();
	/*
	//State Machine actualisation
	if (currentMillis - millisPrevSM >= INTERVAL_SM) {
		millisPrevSM = currentMillis;

	}
	//Motion Control actualisation
	currentMillis = millis()
	if (currentMillis - millisPrevSM >= INTERVAL_MC) {
		millisPrevMC = currentMillis;

	}
	*/
	delay(1000);
	Controller.goto_pos(-0.002 , -0.002, 0, 0);

	delay(1000);
	Controller.goto_pos(0.002 , 0.002, 0, 0);

}

void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// add it to the inputString:
		inputString += inChar;
		// if the incoming character is a newline, set a flag so the main loop can
		// do something about it:
		if (inChar == '\n') {
			stringComplete = true;
		}
	}
}
