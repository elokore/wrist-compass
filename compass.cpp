#include "compass.h"

#define SENSOR_MAG 0
#define SENSOR_ACCEL FILTER_SIZE

Adafruit_ICM20948 icm;

// Pre-Calibrated compass values
float minMagX = -44.70;
float maxMagX = 39.30;
float minMagY = -35.10;
float maxMagY = 63.00;
float minMagZ = -65.40;
float maxMagZ = 28.05;

// Averaging Filters
float sensor_readings[FILTER_SIZE * 2];
int mag_filter_index = 0;
int accel_filter_index = FILTER_SIZE;

// Magnetometer and accelerometer values
float accelX;
float accelY;
float accelZ;
float magX;
float magY;
float magZ;

// ** Internal Functions ** //

void log_reading(int sensor, int* filterIndex, float x, float y, float z) {
  sensor_readings[*filterIndex    ] = x;
  sensor_readings[*filterIndex + 1] = y;
  sensor_readings[*filterIndex + 2] = z;

  int offset = (sensor == SENSOR_ACCEL) ? FILTER_SIZE : 0;
  *filterIndex = offset + (*filterIndex + 3) % FILTER_SIZE;
}

void calculateCalibratedMagValues(sensors_vec_t rawMagVector, float* x, float* y, float* z) {
    float offset_x = (minMagX + maxMagX) / 2;
    float offset_y = (minMagY + maxMagY) / 2;
    float offset_z = (minMagZ + maxMagZ) / 2;
  
    float range_x = maxMagX - minMagX;
    float range_y = maxMagY - minMagY;
    float range_z = maxMagZ - minMagZ;
    float avg_range = (range_x + range_y + range_z) / 3;
  
    float scale_x = avg_range / range_x;
    float scale_y = avg_range / range_y;
    float scale_z = avg_range / range_z;
  
    *x = (rawMagVector.x - offset_x) * scale_x;
    *y = (rawMagVector.y - offset_y) * scale_y;
    *z = (rawMagVector.z - offset_z) * scale_z;
}

// Provides a reading of the magnetomer that has been filtered to make it less noisy
/*
  Provides a filtered reading of the magnetometer or the accelerometer.
  To read the magnetometer or accelerometer provide the value SENSOR_MAG or SENSOR_ACCEL
  to the `sensor` parameter respectively.
*/
void getFilteredSensorReading(int sensor, float* x, float* y, float* z) {
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;
  
    for (int i = sensor; i < (sensor + FILTER_SIZE); i += 3) {
      sumX += sensor_readings[i];
      sumY += sensor_readings[i + 1];
      sumZ += sensor_readings[i + 2];
    }

    *x = sumX / FILTER_SIZE / VALUES_PER_SAMPLE;
    *y = sumY / FILTER_SIZE / VALUES_PER_SAMPLE;
    *z = sumZ / FILTER_SIZE / VALUES_PER_SAMPLE;
}

// Compensate for the pitch and roll of the sensor so that 
// the compass does not need to be level to get an accurate reading
void compensateForTilt(float pitch, float roll, float magX, float magY, float magZ, float* cMagX, float* cMagY) {
    float sinPitch = sin(pitch);
    float sinRoll = sin(roll);
    float cosPitch = cos(pitch);
    float cosRoll = cos(roll);
  
    *cMagX = magX * cosPitch + magZ * sinPitch;
    *cMagY = magX * sinRoll * sinPitch + magY * cosRoll - magZ * sinRoll * cosPitch;
  }

// ** Header Functions ** //

// Returns true if successful, false if the compass cannot be found on the I2C bus.
bool init_compass() {
    // Init filter arrays
    memset(sensor_readings, 0, FILTER_SIZE);
    return icm.begin_I2C();
}

void resetCalibration() {
  minMagX = 500.0;
  maxMagX = -500.0;
  minMagY = 500.0;
  maxMagY = -500.0;
  minMagZ = 500.0;
  maxMagZ = -500.0;
}

// Should be called every frame that the compass has a new reading available
void processCompassData(bool isCalibrating) {
    sensors_event_t mag;
    bool mag_success = icm.getMagnetometerSensor()->getEvent(&mag);
    if (!mag_success) return;
  
    if (isCalibrating) {
      maxMagX = max(maxMagX, mag.magnetic.x);
      minMagX = min(minMagX, mag.magnetic.x);
      maxMagY = max(maxMagY, mag.magnetic.y);
      minMagY = min(minMagY, mag.magnetic.y);
      maxMagZ = max(maxMagZ, mag.magnetic.z);
      minMagZ = min(minMagZ, mag.magnetic.z);
    } else {
      sensors_event_t accel;
      float cX = 0.0;
      float cY = 0.0;
      float cZ = 0.0;

      bool accel_success = icm.getAccelerometerSensor()->getEvent(&accel);
      if (!accel_success) return;

      calculateCalibratedMagValues(mag.magnetic, &cX, &cY, &cZ);
      getFilteredSensorReading(SENSOR_MAG, &magX, &magY, &magZ);
      getFilteredSensorReading(SENSOR_ACCEL, &accelX, &accelY, &accelZ);
      log_reading(SENSOR_MAG, &mag_filter_index, cX, cY, cZ);
      log_reading(SENSOR_ACCEL, &accel_filter_index, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    }
}

// Gets the Pitch and Roll of the compass in radians
void getPitchAndRoll(float* pitch, float* roll) {
    *pitch = atan2(accelX, sqrt((accelY * accelY) + (accelZ * accelZ)));
    *roll = atan2(accelY, accelZ);
}

// Returns the compass heading (Yaw) in radians
float getCompassHeading() {
    float pitch = 0.0;
    float roll = 0.0;
    float comp_mag_x = magX;
    float comp_mag_y = magY;

    getPitchAndRoll(&pitch, &roll);
    compensateForTilt(abs(pitch), abs(roll), magX, magY, magZ, &comp_mag_x, &comp_mag_y);
  
    float heading = atan2(-comp_mag_y, comp_mag_x);
    if (heading < 0) { heading += (2 * M_PI); } // Convert to 0-360 range
  
    return heading;
}