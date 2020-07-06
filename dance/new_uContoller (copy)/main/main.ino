#include "linear_stepper.h"
#include "robot.h"
#include "state_machine.h"
#include "motion_control.h"
#include <AccelStepper.h>
#include <DebounceInput.h>

#define STEPPER_X_STEP_PIN 14
#define STEPPER_X_DIR_PIN 27
#define STEPPER_X_EN_PIN 16

#define STEPPER_Y_STEP_PIN 5
#define STEPPER_Y_DIR_PIN 18
#define STEPPER_Y_EN_PIN 2

#define STEPPER_T_STEP_PIN 22
#define STEPPER_T_DIR_PIN 23
#define STEPPER_T_EN_PIN 17

#define STEPPER_DF_STEP_PIN 19
#define STEPPER_DF_DIR_PIN 21
#define STEPPER_DF_EN_PIN 4

#define CALIB_ENDSWITCH_X 36
#define CALIB_ENDSWITCH_Y 39
#define CALIB_LED 13
#define CALIB_LED_PHOTORES 15
#define CALIB_ENDSWITCH_DF 33

#define EXTRACT_MOT_POT 34
#define EXTRACT_MOT_PWM 25
#define EXTRACT_MOT_DIR 26

#define PI 3.14159265

Linear_Stepper StepperX(STEPPER_X_STEP_PIN, STEPPER_X_DIR_PIN, STEPPER_X_EN_PIN, 4);
Linear_Stepper StepperY(STEPPER_Y_STEP_PIN, STEPPER_Y_DIR_PIN, STEPPER_Y_EN_PIN, 4);
Angular_Stepper StepperT(STEPPER_T_STEP_PIN, STEPPER_T_DIR_PIN, STEPPER_T_EN_PIN, 4, 1.);
Angular_Stepper StepperDF(STEPPER_DF_STEP_PIN, STEPPER_DF_DIR_PIN, STEPPER_DF_EN_PIN, 4, 72./11.);

DebouncedInput xSwitchPinDB;
DebouncedInput ySwitchPinDB;
DebouncedInput dfSwitchPinDB;

// The string used for each communication
String inputString = "";
bool stringComplete = false;  // whether the string is complete

robot beedancer;

// Initialisation of the Serial Semaphore
volatile SemaphoreHandle_t syncSerialSemaphore;

// Initialisation of tzhe PQ12 linearmotor
Linear_Actuator PQ12(EXTRACT_MOT_POT, EXTRACT_MOT_PWM, EXTRACT_MOT_DIR);

Motion_Control Controller(&beedancer);

State_Machine Beebrain(&Controller);

void serialTask( void * parameter )
{
	const TickType_t xDelay = 1;
	for( ;; ) {
		while(Serial.available()) {
			// get the new byte:

			char inChar = (char)Serial.read();
			// add it to the inputString:
			inputString += inChar;
			// if the incoming character is a newline, set a flag so the main loop can
			// do something about it:
			if (inChar == '\n') {
				xSemaphoreGive(syncSerialSemaphore);
			}
		}
		vTaskDelay(1);
	}
	vTaskDelete( NULL );
}

void setup() {
	// put your setup code here, to run once:
	Serial.begin(500000);
	PQ12.init();

	StepperX.set_max_speed(0.04);
	StepperX.set_acceleration(1);
	StepperY.set_max_speed(0.04);
	StepperY.set_acceleration(1);
	StepperT.set_max_speed(2*200*PI);
	StepperT.set_acceleration(2*200*PI);
	StepperDF.set_max_speed(2*10*PI);
	StepperDF.set_acceleration(2*10*PI);


	// Setting up the debounced inputs
	pinMode(CALIB_ENDSWITCH_X, INPUT);
	pinMode(CALIB_ENDSWITCH_Y, INPUT);
	pinMode(CALIB_ENDSWITCH_DF, INPUT_PULLDOWN);
	xSwitchPinDB.attach(CALIB_ENDSWITCH_X);
	ySwitchPinDB.attach(CALIB_ENDSWITCH_Y);
	dfSwitchPinDB.attach(CALIB_ENDSWITCH_DF);

	pinMode(CALIB_LED, OUTPUT);
	pinMode(CALIB_LED_PHOTORES, INPUT);

	// All the input outputs of the robot are in a data structure
	beedancer.StepperT = &StepperT;
	beedancer.StepperY = &StepperY;
	beedancer.StepperX = &StepperX;
	beedancer.StepperDF = &StepperDF;
	beedancer.PQ12 = &PQ12;
	beedancer.xSwitch = &xSwitchPinDB;
	beedancer.ySwitch = &ySwitchPinDB;
	beedancer.dfSwitch = &dfSwitchPinDB;
	beedancer.photo_res = CALIB_LED_PHOTORES;
	beedancer.led_pin = CALIB_LED;

	syncSerialSemaphore = xSemaphoreCreateBinary();

	// reserve 200 bytes for the inputString:
	inputString.reserve(200);

	    // Creating the task who send a timeout flag to the stepper controller
	xTaskCreate(
	serialTask, // Task function.
	"Serial", // String with name of task.
	10000, // Stack size in words.
	NULL, // Parameter passed as input of the task
	1, // Priority of the task.
	NULL); // Task handle.
}

void loop() {
	if(xSemaphoreTake(syncSerialSemaphore, 0) == pdTRUE){
		Beebrain.handle_message(&inputString);
	}
	Beebrain.step();
	Controller.step();
}
