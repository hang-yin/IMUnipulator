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

static const uint8_t ICM20948_ADDRESS = 0x69;

// Register definitions for ICM20948
typedef enum {
  ICM20948_WHO_AM_I = 0x00,
  ACCEL_XOUT_H = 0x2D,
  ACCEL_XOUT_L = 0x2E,
  ACCEL_YOUT_H = 0x2F,
  ACCEL_YOUT_L = 0x30,
  ACCEL_ZOUT_H = 0x31,
  ACCEL_ZOUT_L = 0x32,
  GYRO_XOUT_H = 0x33,
  GYRO_XOUT_L = 0x34,
  GYRO_YOUT_H = 0x35,
  GYRO_YOUT_L = 0x36,
  GYRO_ZOUT_H = 0x37,
  GYRO_ZOUT_L = 0x38,
  TEMP_OUT_H = 0x39,
  TEMP_OUT_L = 0x3A,
  PWR_MGMT_1 = 0x06,
} icm20948_reg_t;