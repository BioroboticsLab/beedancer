#include "linear_stepper.h"
#include "robot.h"
#include "state_machine.h"
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

// Potentiometer is connected to GPIO 34 input of PQ12 pot
int PQ12_potPin = 34;

// PQ12 pin command output
const int PQ12_speedPin = 32;
const int PQ12_directionPin = 23;

const int xSwitchPin = 12;
const int ySwitchPin = 13;
const int dfSwitchPin = 19;

DebouncedInput xSwitchPinDB;
DebouncedInput ySwitchPinDB;
DebouncedInput dfSwitchPinDB;

// The string used for each communication
String inputString = "";
bool stringComplete = false;  // whether the string is complete

robot beedancer;

// Initialisation of tzhe PQ12 linearmotor
Linear_Actuator PQ12(PQ12_potPin, PQ12_speedPin, PQ12_directionPin);

Motion_Control Controller(&beedancer);

State_Machine Beebrain(&Controller);

// Initialisation of the TimeOut timer (Tic need a communication every second
// or they stop with timeout exeption
hw_timer_t * syncTimerStateMachine = NULL;
volatile SemaphoreHandle_t syncTimerStateMachineSemaphore;

// Initialisation of the step timer
hw_timer_t * syncTimerMotion = NULL;
volatile SemaphoreHandle_t syncTimerMotionSemaphore;

// Initialisation of the Serial Semaphore
volatile SemaphoreHandle_t syncSerialSemaphore;

const TickType_t xDelay = 1;


void IRAM_ATTR StepperError() {
	Serial.println("ISR stepper error trigger");
	Controller.check_error();
	Controller.check_error();
}

//Definition of The Interrupt Service Routine for Timeout
void IRAM_ATTR onsyncTimerStateMachine(){
  xSemaphoreGiveFromISR(syncTimerStateMachineSemaphore, NULL);
}

//Definition of The Interrupt Service Routine for step
void IRAM_ATTR onsyncTimerMotion(){
  xSemaphoreGiveFromISR(syncTimerMotionSemaphore, NULL);
}

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
	Serial.begin(115200);
	vTaskDelay(20);
	Wire.begin();
	vTaskDelay(20);

	StepperX.init();
	StepperY.init();
	StepperT.init();
	StepperDF.init();
	PQ12.init();


	// Setting up the debounced inputs
	pinMode(ySwitchPin, INPUT_PULLUP);
	pinMode(xSwitchPin, INPUT_PULLUP);
	pinMode(dfSwitchPin, INPUT_PULLUP);
	xSwitchPinDB.attach(xSwitchPin);
	ySwitchPinDB.attach(ySwitchPin);
	dfSwitchPinDB.attach(dfSwitchPin);

	// All the input outputs of the robot are in a data structure
	beedancer.StepperT = &StepperT;
	beedancer.StepperY = &StepperY;
	beedancer.StepperX = &StepperX;
	beedancer.StepperDF = &StepperDF;
	beedancer.PQ12 = &PQ12;
	beedancer.xSwitch = &xSwitchPinDB;
	beedancer.ySwitch = &ySwitchPinDB;
	beedancer.dfSwitch = &dfSwitchPinDB;
	beedancer.motorCerrorPin = MOTERR_PIN;

	pinMode(PULLUP_PIN, OUTPUT);
	digitalWrite(PULLUP_PIN, HIGH);

	pinMode(MOTERR_PIN, INPUT);
	attachInterrupt(MOTERR_PIN, StepperError, RISING);

	syncTimerStateMachineSemaphore = xSemaphoreCreateBinary();
	syncTimerMotionSemaphore = xSemaphoreCreateBinary();
	syncSerialSemaphore = xSemaphoreCreateBinary();

	// reserve 200 bytes for the inputString:
	inputString.reserve(200);

	millisPrevSM = millis();
	millisPrevMC = millis();

	syncTimerStateMachine = timerBegin(0, 80, true);
	timerAttachInterrupt(syncTimerStateMachine, &onsyncTimerStateMachine, true);
	timerAlarmWrite(syncTimerStateMachine, 10000, true);
	timerAlarmEnable(syncTimerStateMachine);

	syncTimerMotion = timerBegin(1, 80, true);
	timerAttachInterrupt(syncTimerMotion, &onsyncTimerMotion, true);
	timerAlarmWrite(syncTimerMotion, 1000, true);
	timerAlarmEnable(syncTimerMotion);

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
	
	if(xSemaphoreTake(syncTimerStateMachineSemaphore, xDelay) == pdTRUE){
		Beebrain.step();
	}
	
	if(xSemaphoreTake(syncTimerMotionSemaphore, xDelay) == pdTRUE){
		Controller.step();
	}

}
