#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <Arduino.h>
#include <Tic.h>
#include <DebounceInput.h>
#include "robot.h"
#include <math.h>

float calibrationXY(robot* beedancer);

float calibrationDF(robot* beedancer);

float findOrigine(Stepper* Stepper, DebouncedInput* switchPin, bool setWhenFound = false, bool isCircular = false);

#endif