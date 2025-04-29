#include "compass.h"

Adafruit_ICM20948 icm;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

// Pre-Calibrated compass values
float minMagX = -74.27;
float maxMagX = 22.58;
float minMagY = -45.33;
float maxMagY = 57.25;
float minMagZ = -64.22;
float maxMagZ = 39.35;

// Averaging Filters
float mag_readings_x[FILTER_SIZE];
float mag_readings_y[FILTER_SIZE];
float mag_readings_z[FILTER_SIZE];
float accel_readings_x[FILTER_SIZE];
float accel_readings_y[FILTER_SIZE];
float accel_readings_z[FILTER_SIZE];
int mag_filter_index = 0;
int accel_filter_index = 0;
bool is_filter_full = false;

// Magnetometer and accelerometer values
float accelX;
float accelY;
float accelZ;
float magX;
float magY;
float magZ;

// ** Internal Functions ** //

void log_mag_reading(float x, float y, float z) {
    mag_readings_x[mag_filter_index] = x;
    mag_readings_y[mag_filter_index] = y;
    mag_readings_z[mag_filter_index] = z;
  
    mag_filter_index = (mag_filter_index + 1);
  
    if (mag_filter_index == FILTER_SIZE && !is_filter_full) {
      is_filter_full = true;
    }
  
    mag_filter_index = mag_filter_index % FILTER_SIZE;
}
  
void log_accel_reading(float x, float y, float z) {
    accel_readings_x[accel_filter_index] = x;
    accel_readings_y[accel_filter_index] = y;
    accel_readings_z[accel_filter_index] = z;
  
    accel_filter_index = (accel_filter_index + 1) % FILTER_SIZE;
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
void getFilteredMagReading(float* x, float* y, float* z) {
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;
  
    for (int i = 0; i < FILTER_SIZE; i++) {
      sumX += mag_readings_x[i];
      sumY += mag_readings_y[i];
      sumZ += mag_readings_z[i];
    }
  
    *x = sumX / FILTER_SIZE;
    *y = sumY / FILTER_SIZE;
    *z = sumZ / FILTER_SIZE;
}

// Provides a reading of the accelerometer that has been filtered to make it less noisy
void getFilteredAccelReading(float* x, float* y, float* z) {
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;
  
    for (int i = 0; i < FILTER_SIZE; i++) {
      sumX += accel_readings_x[i];
      sumY += accel_readings_y[i];
      sumZ += accel_readings_z[i];
    }
  
    *x = sumX / FILTER_SIZE;
    *y = sumY / FILTER_SIZE;
    *z = sumZ / FILTER_SIZE;
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
    memset(mag_readings_x, 0, FILTER_SIZE);
    memset(mag_readings_y, 0, FILTER_SIZE);
    memset(mag_readings_z, 0, FILTER_SIZE);
    memset(accel_readings_x, 0, FILTER_SIZE);
    memset(accel_readings_y, 0, FILTER_SIZE);
    memset(accel_readings_z, 0, FILTER_SIZE);

    // if (IS_CALIBRATING) {
    //     maxMagX = -99999.0;
    //     minMagX = 99999.0;
    //     maxMagY = -99999.0;
    //     minMagY = 99999.0;
    //     maxMagZ = -99999.0;
    //     minMagZ = 99999.0;
    // }

    return icm.begin_I2C();
}

// Should be called every frame that the compass has a new reading available
void processCompassData() {
    bool readSuccess = icm.getEvent(&accel, &gyro, &temp, &mag);
    if (!readSuccess) return;

    float cX = 0.0;
    float cY = 0.0;
    float cZ = 0.0;
  
    // if (IS_CALIBRATING) {
    //   cX = mag.magnetic.x;
    //   cY = mag.magnetic.y;
    //   cZ = mag.magnetic.z;
    // } else {
      calculateCalibratedMagValues(mag.magnetic, &cX, &cY, &cZ);
    //}
  
    log_mag_reading(cX, cY, cZ);
    log_accel_reading(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
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
    float comp_mag_x = 0.0;
    float comp_mag_y = 0.0;
  
    getFilteredMagReading(&magX, &magY, &magZ);
    getFilteredAccelReading(&accelX, &accelY, &accelZ);
    getPitchAndRoll(&pitch, &roll);
    compensateForTilt(pitch, roll, magX, magY, magZ, &comp_mag_x, &comp_mag_y);
  
    float heading = atan2(comp_mag_y, comp_mag_x);
    if (heading < 0) { heading += (2 * M_PI); } // Convert to 0-360 range
  
    return heading;
}