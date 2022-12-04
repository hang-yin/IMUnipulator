#pragma once
#include "nrf_twi_mngr.h"

// Measurement data type
typedef struct {
  float x_axis;
  float y_axis;
  float z_axis;
} icm20948_measurement_t;

icm20948_measurement_t icm20948_read_accelerometer(void);
icm20948_measurement_t convert_accelerometer_to_tilt_angles(icm20948_measurement_t acc_measurement);