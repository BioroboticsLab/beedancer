// This example shows how to send I2C commands to two Tic Stepper
// Motor Controllers on the same I2C bus.
//
// Each Tic's control mode must be set to "Serial/I2C/USB".  The
// serial device number of one Tic must be set to its default
// value of 14, and the serial device number of another Tic must
// be set to 15.
//
// The GND, SCL, and SDA pins of the Arduino must each be
// connected to the corresponding pins on each Tic.  You might
// consider connecting the ERR lines of both Tics so that if
// either one experiences an error, both of them will shut down
// until you reset the Arduino.
//
// See the comments and instructions in I2CSpeedControl.ino for
// more information.

#include <Tic.h>
#include <SimpleTimer.h>

//Definition of stepper controller
TicI2C StepperX(12);
TicI2C StepperY(11);

const int xSwitch = 12;     // the number of the pushbutton pin
const int ySwitch =  13;      // the number of the LED pin

const int step_um_ratio = 10 // One step = 10um

int xState = 0;
int yState = 0;

void stepperInit();

SimpleTimer timer;

void setup()
{
  stepperInit();
  Serial.begin(115200);
  pinMode(xSwitch, INPUT_PULLUP);
  pinMode(ySwitch, INPUT_PULLUP);
}

void resetCommandTimeout()
{
  StepperX.resetCommandTimeout();
  StepperY.resetCommandTimeout();
}


void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

void stepperInit(){
  Wire.begin();
  delay(20);
  StepperX.exitSafeStart();
  StepperY.exitSafeStart();
  StepperX.getStepMode();
  StepperY.getStepMode();
}

void XYSimpleCalibration()
{
  
}

int step2um(int steps)
{
  return steps * step_um_ratio
}

int um2step(int um)
{
  return um / step_um_ratio
}

void loop()
{
  
//  xState = digitalRead(xSwitch);
//  yState = digitalRead(ySwitch);
//  if (xState == LOW) {
//    // turn LED on:
//    Serial.println("xPress");
//  }
//  if (yState == LOW) {
//    // turn LED off:
//    Serial.println("yPress");
//  }
  
//  StepperX.setTargetVelocity(200000000);
//  StepperY.setTargetVelocity(0);
//  delayWhileResettingCommandTimeout(1000);
//
//  StepperX.setTargetVelocity(0);
//  StepperY.setTargetVelocity(100000000);
//  delayWhileResettingCommandTimeout(1000);
//
//  StepperX.setTargetVelocity(-100000000);
//  StepperY.setTargetVelocity(0);
//  delayWhileResettingCommandTimeout(1000);
//
//  StepperX.setTargetVelocity(0);
//  StepperY.setTargetVelocity(-200000000);
//  delayWhileResettingCommandTimeout(1000);
}
