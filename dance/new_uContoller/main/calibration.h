#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <Arduino.h>
#include <Tic.h>
#include <DebounceInput.h>
#include "robot.h"
#include "linear_stepper.h"
#include <math.h>

float calibrationXY(robot* beedancer);

float calibrationT(robot* beedancer);

float calibrationDF(robot* beedancer);

float findOrigineDF(robot* beedancer);

float findOriginePhoto(Angular_Stepper* Stepper, int photo_diode, float from, float to, int nb_points, int exclude_border = 0);

float findOrigine(Linear_Stepper* Stepper, DebouncedInput* switchPin, bool setWhenFound = false);

#endif