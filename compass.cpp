#include "compass.h"

Adafruit_ICM20948 icm;

// Pre-Calibrated compass values
float minMagX = -44.70;
float maxMagX = 39.30;
float minMagY = -35.10;
float maxMagY = 63.00;
float minMagZ = -65.40;
float maxMagZ = 28.05;

// Averaging Filters
float mag_readings[FILTER_SIZE];
float accel_readings[FILTER_SIZE];
int mag_filter_index = 0;
int accel_filter_index = 0;

// Magnetometer and accelerometer values
float accelX;
float accelY;
float accelZ;
float magX;
float magY;
float magZ;

// ** Internal Functions ** //

void log_reading(float readingsLog[], int* filterIndex, float x, float y, float z) {
  readingsLog[*filterIndex    ] = x;
  readingsLog[*filterIndex + 1] = y;
  readingsLog[*filterIndex + 2] = z;

  *filterIndex = (*filterIndex + 3) % FILTER_SIZE;
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
  
    for (int i = 0; i < FILTER_SIZE; i += 3) {
      sumX += mag_readings[i];
      sumY += mag_readings[i + 1];
      sumZ += mag_readings[i + 2];
    }
  
    *x = sumX / FILTER_SIZE / VALUES_PER_SAMPLE;
    *y = sumY / FILTER_SIZE / VALUES_PER_SAMPLE;
    *z = sumZ / FILTER_SIZE / VALUES_PER_SAMPLE;
}

// Provides a reading of the accelerometer that has been filtered to make it less noisy
void getFilteredAccelReading(float* x, float* y, float* z) {
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;
  
    for (int i = 0; i < FILTER_SIZE; i += 3) {
      sumX += accel_readings[i];
      sumY += accel_readings[i + 1];
      sumZ += accel_readings[i + 2];
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
    memset(mag_readings, 0, FILTER_SIZE);
    memset(accel_readings, 0, FILTER_SIZE);

    if (IS_CALIBRATING) {
        maxMagX = -99999.0;
        minMagX = 99999.0;
        maxMagY = -99999.0;
        minMagY = 99999.0;
        maxMagZ = -99999.0;
        minMagZ = 99999.0;
    }

    return icm.begin_I2C();
}

// Should be called every frame that the compass has a new reading available
void processCompassData() {
    sensors_event_t mag;
    sensors_event_t accel;

    bool mag_success = icm.getMagnetometerSensor()->getEvent(&mag);
    bool accel_success = icm.getAccelerometerSensor()->getEvent(&accel);
    
    if (!mag_success) return;
    if (!accel_success) return;

    float cX = 0.0;
    float cY = 0.0;
    float cZ = 0.0;
  
    if (IS_CALIBRATING) {
      cX = mag.magnetic.x;
      cY = mag.magnetic.y;
      cZ = mag.magnetic.z;

      maxMagX = max(maxMagX, cX);
      minMagX = min(minMagX, cX);
      maxMagY = max(maxMagY, cY);
      minMagY = min(minMagY, cY);
      maxMagZ = max(maxMagZ, cZ);
      minMagZ = min(minMagZ, cZ);

      Serial.print("MinX: ");
      Serial.print(minMagX);
      Serial.print(" MaxX: ");
      Serial.print(maxMagX);

      Serial.print("MinY: ");
      Serial.print(minMagY);
      Serial.print(" MaxY: ");
      Serial.print(maxMagY);

      Serial.print("MinZ: ");
      Serial.print(minMagZ);
      Serial.print(" MaxZ: ");
      Serial.println(maxMagZ);
    } else {
      calculateCalibratedMagValues(mag.magnetic, &cX, &cY, &cZ);
    }
  
    log_reading(mag_readings, &mag_filter_index, cX, cY, cZ);
    log_reading(accel_readings, &accel_filter_index, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
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
  
    float heading = atan2(-comp_mag_y, comp_mag_x);
    if (heading < 0) { heading += (2 * M_PI); } // Convert to 0-360 range
  
    return heading;
}