#ifndef COMPASS_H
#define COMPASS_H

#include "Arduino.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#define IS_CALIBRATING false
#define FILTER_SIZE 6

bool init_compass();
void getPitchAndRoll(float* pitch, float* roll);
float getCompassHeading();
void processCompassData();

#endif