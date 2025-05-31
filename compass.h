#ifndef COMPASS_H
#define COMPASS_H

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define VALUES_PER_SAMPLE 3
#define FILTER_SIZE 6 * VALUES_PER_SAMPLE

bool init_compass();
void getPitchAndRoll(float* pitch, float* roll);
float getCompassHeading();
void processCompassData(bool isCalibrating);

#endif