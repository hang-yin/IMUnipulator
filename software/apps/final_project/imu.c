#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "imu.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "microbit_v2.h"
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"
#include "i2c.h"

static int16_t combine_bytes(uint8_t lsb, uint8_t msb) {
  return (uint16_t)lsb | ((uint16_t)msb << 8);
}

// Read accelerometer data from ICM20948
icm20948_measurement_t icm20948_read_accelerometer(const nrf_twi_mngr_t* i2c) {
  const nrf_twi_mngr_t* i2c_manager = i2c;
  const float scaling_factor = 0.00006103515625; // 1/2^14

  uint16_t lsb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_XOUT_L, i2c_manager);
  uint16_t msb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_XOUT_H, i2c_manager);
  float x = scaling_factor*(float)(combine_bytes(lsb,msb));

  lsb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_YOUT_L, i2c_manager);
  msb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_YOUT_H, i2c_manager);
  float y = scaling_factor*(float)(combine_bytes(lsb,msb));

  lsb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_ZOUT_L, i2c_manager);
  msb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_ZOUT_H, i2c_manager);
  float z = scaling_factor*(float)(combine_bytes(lsb,msb));

  icm20948_measurement_t measurement = {
    .x_axis = x,
    .y_axis = y,
    .z_axis = z
  };
  return measurement;
}

// define a function for converting accelerometer data to tilt angles
icm20948_measurement_t convert_accelerometer_to_tilt_angles(icm20948_measurement_t acc_measurement) {
  double pow_x = acc_measurement.x_axis * acc_measurement.x_axis;
  double pow_y = acc_measurement.y_axis * acc_measurement.y_axis;
  double pow_z = acc_measurement.z_axis * acc_measurement.z_axis;

  float temp = sqrt(pow_x + pow_y) / acc_measurement.z_axis;
  float phi = atan(temp);
  temp = acc_measurement.y_axis / sqrt(pow_x + pow_z);
  float theta = atan(temp);
  temp = acc_measurement.x_axis / sqrt(pow_y + pow_z);
  float psi = atan(temp);
  // convert phi to degrees
  float phi_degrees = phi/3.14159 * 180;
  // convert theta to degrees
  float theta_degrees = theta/3.14159 * 180;
  // convert psi to degrees
  float psi_degrees = psi/3.14159 * 180;
  return (icm20948_measurement_t){phi_degrees, theta_degrees, psi_degrees};
}