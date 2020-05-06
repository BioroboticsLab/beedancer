#include "stepper.h"
#include "motion_control.h"
#include "calibration.h"
#include "linear_actuator.h"
#include <Tic.h>
#include <DebounceInput.h>
#include "robot.h"
#include "state_machine.h"

Stepper StepperDF(9, 600, false);
Stepper StepperT(10, 400, true);
Stepper StepperY(11, 400, true);
Stepper StepperX(12, 400, true);

robot beedancer; 

const int timeStep = 10; //Time step of step tracking in useconds
const int xSwitchPin = 12;
const int ySwitchPin = 13;
const int dfSwitchPin = 19;

// Flag to control if it's connected to PC
bool is_connected = false;

// The string used for each communication
String inputString = "";

// Potentiometer is connected to GPIO 34 input of PQ12 pot
int PQ12_potPin = 34;

// PQ12 pin command output
const int PQ12_speedPin = 32;
const int PQ12_directionPin = 23;

// Measurment of the robot mm
const float deltaXPod = -44.2;
const float deltaYpod = -34.3;
const float deltaXDF = 75.;
const float deltaYDF = 61.5;

DebouncedInput xSwitchPinDB;
DebouncedInput ySwitchPinDB;
DebouncedInput dfSwitchPinDB;

// Initialisation of the motors controller
Motion_control Controller(&beedancer);

// Initialisation of tzhe PQ12 linearmotor
Linear_Actuator PQ12(PQ12_potPin, PQ12_speedPin, PQ12_directionPin);

State_Machine braindancer(&Controller);
// Initialisation of the TimeOut timer (Tic need a communication every second
// or they stop with timeout exeption
hw_timer_t * syncTimerTimeOut = NULL;
volatile SemaphoreHandle_t syncTimerTimeOutSemaphore;

// Initialisation of the step timer
hw_timer_t * syncTimerStep = NULL;
volatile SemaphoreHandle_t syncTimerStepSemaphore;

// Initialisation of the Serial Semaphore
volatile SemaphoreHandle_t syncSerialSemaphore;

// Initialisation of the mutex for state machine usage
volatile SemaphoreHandle_t stateMachineMutex;

//Definition of The Interrupt Service Routine for Timeout
void IRAM_ATTR onsyncTimerTimeOut(){
  xSemaphoreGiveFromISR(syncTimerTimeOutSemaphore, NULL);
}

//Definition of The Interrupt Service Routine for step
void IRAM_ATTR onsyncTimerNextPos(){
  xSemaphoreGiveFromISR(syncTimerStepSemaphore, NULL);
}

void continuousTask( void * parameter )
{
  /* See if we can obtain the semaphore.  If the semaphore is not
  available wait 10 ticks to see if it becomes free. */
  const TickType_t xDelay = 1;
  for( ;; ) {
    
    if(xSemaphoreTake(stateMachineMutex, xDelay) == pdTRUE){
      braindancer.step();
      xSemaphoreGive(stateMachineMutex);
    }

    if(xSemaphoreTake(syncSerialSemaphore, xDelay) == pdTRUE){
      if(xSemaphoreTake(stateMachineMutex, xDelay) == pdTRUE){
        braindancer.handle_message(&inputString);
        xSemaphoreGive(stateMachineMutex);
      }
    }

    else {}
      
    vTaskDelay(1);  
  }
  vTaskDelete( NULL );
}

void serialTask( void * parameter )
{
  const TickType_t xDelay = 100;
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

int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(20);
  Wire.begin();

  Wire.setClock(100000);

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

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

  // Create semaphore to inform us when the syncTimerTimeOut has fired
  syncTimerTimeOutSemaphore = xSemaphoreCreateBinary();
  syncSerialSemaphore = xSemaphoreCreateBinary();
  stateMachineMutex = xSemaphoreCreateMutex();

  // Use 1st syncTimerTimeOut of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  syncTimerTimeOut = timerBegin(0, 80, true);

  // Attach onsyncTimerTimeOut function to our syncTimerTimeOut.
  timerAttachInterrupt(syncTimerTimeOut, &onsyncTimerTimeOut, true);

  // Set alarm to call onsyncTimerTimeOut function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(syncTimerTimeOut, 500000, true);

  // Start an alarm
  timerAlarmEnable(syncTimerTimeOut);

  Controller.init();

  // Creating the task who send a timeout flag to the stepper controller
  xTaskCreate(
    timeOutTask, // Task function.
    "TimeOut", // String with name of task.
    10000, // Stack size in words.
    NULL, // Parameter passed as input of the task
    1, // Priority of the task.
    NULL); // Task handle.

    // Creating the task who send a timeout flag to the stepper controller
  xTaskCreate(
    continuousTask, // Task function.
    "Step", // String with name of task.
    10000, // Stack size in words.
    NULL, // Parameter passed as input of the task
    1, // Priority of the task.
    NULL); // Task handle.

    // Creating the task who send a timeout flag to the stepper controller
  xTaskCreate(
    serialTask, // Task function.
    "Serial", // String with name of task.
    10000, // Stack size in words.
    NULL, // Parameter passed as input of the task
    1, // Priority of the task.
    NULL); // Task handle.
    
  //PQ12.retract(200);
  //delay(2000);
  //Serial.println(calibrationXY(&beedancer));
  //delay(3000);
  //PQ12.extract(200);
}

const TickType_t xDelay = 1;

void loop() {
  vTaskDelay(10);
//  PQ12.retract(200);
//  delay(1000);
//  Serial.println(StepperY.getMicro_step());
//  Serial.println("yolo");
//  StepperT.setTargetVelocity(2000000);
//  StepperY.setTargetVelocity(0);
//  delayWhileResettingCommandTimeout(1000);
//
//  StepperT.setTargetVelocity(0);
//  StepperY.setTargetVelocity(2000000);
//  delayWhileResettingCommandTimeout(1000);
//
//  StepperT.setTargetVelocity(-2000000);
//  StepperY.setTargetVelocity(0);
//  delayWhileResettingCommandTimeout(1000);
//
//  StepperT.setTargetVelocity(0);
//  StepperY.setTargetVelocity(-2000000);
//  delayWhileResettingCommandTimeout(1000);
}
