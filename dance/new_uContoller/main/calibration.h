#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <Arduino.h>
#include <Tic.h>
#include <DebounceInput.h>
#include "robot.h"
#include "linear_stepper.h"
#include <math.h>

float calibrationXY(robot* beedancer);

float calibrationDF(robot* beedancer);

float findOrigine(Linear_Stepper* Stepper, DebouncedInput* switchPin, bool setWhenFound = false, bool isCircular = false);

#endif