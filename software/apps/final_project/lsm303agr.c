// LSM303AGR driver for Microbit_v2
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "lsm303agr.h"
#include "nrf_delay.h"
#include "app_timer.h"

// Pointer to an initialized I2C instance to use for transactions
static const nrf_twi_mngr_t* i2c_manager = NULL;
// Helper function to perform a 1-byte I2C read of a given register
//
// i2c_addr - address of the device to read from
// reg_addr - address of the register within the device to read
//
// returns 8-bit read value
static uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
      NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
      NRF_TWI_MNGR_READ(i2c_addr, &rx_buf, 1, 0),
  };
  nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  return rx_buf;
}

// Helper function to perform a 1-byte I2C write of a given register
//
// i2c_addr - address of the device to write to
// reg_addr - address of the register within the device to write
static void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  //TODO: implement me
  uint8_t tx_buf[2] = {reg_addr, data};
  //uint16_t my_data = (reg_addr << 8) | data;
  nrf_twi_mngr_transfer_t const write_transfer[] = {
      NRF_TWI_MNGR_WRITE(i2c_addr, tx_buf, 2, 0),
  };
  //Note: there should only be a single two-byte transfer to be performed
  nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);
}

// Initialize and configure the LSM303AGR accelerometer/magnetometer
//
// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c) {
  i2c_manager = i2c;
  /*
  // ---Initialize Accelerometer---

  // Reboot acclerometer
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_CTRL_REG5, 0x80);
  nrf_delay_ms(100); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_CTRL_REG4, 0x80);

  // Configure accelerometer at 100Hz, normal mode (10-bit)
  // Enable x, y and z axes
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_CTRL_REG1, 0x57);
  */

  // Read WHO AM I register
  // Always returns the same value if working
  uint8_t result = i2c_reg_read(ICM20948_ADDRESS, ICM20948_WHO_AM_I);
  //TODO: check the result of the Accelerometer WHO AM I register
  printf("WHO AM I: %x\n", result);

  // Power management reset
  //uint8_t power_mgmt_1 = i2c_reg_read(ICM20948_ADDRESS, PWR_MGMT_1);
  //printf("Power management 1: %x\n", power_mgmt_1);
  
  i2c_reg_write(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  nrf_delay_ms(100);
  

  // ---Initialize Magnetometer---

  /*
  // Reboot magnetometer
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_CFG_REG_A, 0x40);
  nrf_delay_ms(100); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_CFG_REG_C, 0x10);

  // Configure magnetometer at 100Hz, continuous mode
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_CFG_REG_A, 0x0C);
  
  // Read WHO AM I register
  result = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_WHO_AM_I_REG);
  //TODO: check the result of the Magnetometer WHO AM I register
  printf("Magnetometer WHO AM I: %x\n", result);
  */
  // ---Initialize Temperature---

  // Enable temperature sensor
  // i2c_reg_write(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_TEMP_CFG_REG, 0xC0);
}

// Read accelerometer data from ICM20948
lsm303agr_measurement_t icm20948_read_accelerometer(void) {
  const float scaling_factor = 0.00006103515625; // 1/2^14

  uint16_t lsb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_XOUT_L);
  uint16_t msb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_XOUT_H);
  //printf("LSB: %x\n", lsb);
  //printf("MSB: %x\n", msb);
  float x = scaling_factor*(float)(combine_bytes(lsb,msb));

  lsb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_YOUT_L);
  msb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_YOUT_H);

  float y = scaling_factor*(float)(combine_bytes(lsb,msb));

  lsb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_ZOUT_L);
  msb = (uint16_t)i2c_reg_read(ICM20948_ADDRESS, ACCEL_ZOUT_H);

  float z = scaling_factor*(float)(combine_bytes(lsb,msb));

  lsm303agr_measurement_t measurement = {
    .x_axis = x,
    .y_axis = y,
    .z_axis = z
  };
  return measurement;
}

int16_t combine_bytes(uint8_t lsb, uint8_t msb) {
  return (uint16_t)lsb | ((uint16_t)msb << 8);
}

// Read the internal temperature sensor
//
// Return measurement as floating point value in degrees C
float lsm303agr_read_temperature(void) {
  //TODO: implement me
  uint8_t temp_l = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_TEMP_L);
  uint8_t temp_h = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_TEMP_H);
  int16_t temp = (temp_h << 8) | temp_l;
  return (float)temp / 256.0 + 25.0;
  //return 0.0;
}

// timer callback for printing temperature
void temp_timer_callback(void * p_context) {
  //float temp = lsm303agr_read_temperature();
  //printf("Temperature: %f\n", temp);
  lsm303agr_measurement_t acc_measurement = icm20948_read_accelerometer();
  printf("Accelerometer: %f g, %f g, %f g\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
  //printf("Magnetometer: %f Gs, %f Gs, %f Gs\n", mag_measurement.x_axis, mag_measurement.y_axis, mag_measurement.z_axis);
  //float phi = convert_accelerometer_to_tilt_angles(acc_measurement);
  //printf("Phi: %f\n", phi);
  //printf("Accelerometer: %x, %x, %x\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
}

lsm303agr_measurement_t lsm303agr_read_accelerometer(void) {
  //TODO: implement me
  uint8_t x_l = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_X_L);
  //printf("x_l: %x\n", x_l);
  uint8_t x_h = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_X_H);
  //printf("x_h: %x\n", x_h);
  uint8_t y_l = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Y_L);
  uint8_t y_h = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Y_H);
  uint8_t z_l = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Z_L);
  uint8_t z_h = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Z_H);
  uint16_t x = ((uint16_t)x_h << 8) | x_l;
  uint16_t y = ((uint16_t)y_h << 8) | y_l;
  uint16_t z = ((uint16_t)z_h << 8) | z_l;
  int16_t x_16 = (int16_t)x;
  int16_t y_16 = (int16_t)y;
  int16_t z_16 = (int16_t)z;
  //printf("x: %x\n", x);
  //printf("y: %x\n", y);
  //printf("z: %x\n", z);
  int16_t x_shifted = x_16 >> 6;
  int16_t y_shifted = y_16 >> 6;
  int16_t z_shifted = z_16 >> 6;
  //printf("x_shifted: %d\n", x_shifted);
  //printf("y_shifted: %d\n", y_shifted);
  //printf("z_shifted: %d\n", z_shifted);
  int16_t x_scaled = x_shifted * 3.9;//0.244;
  int16_t y_scaled = y_shifted * 3.9;//0.244;
  int16_t z_scaled = z_shifted * 3.9;//0.244;
  //printf("x_scaled in terms of mg/LSB: %d\n", x_scaled);
  float x_in_g = x_scaled / 1000.0;
  float y_in_g = y_scaled / 1000.0;
  float z_in_g = z_scaled / 1000.0;
  lsm303agr_measurement_t result = {x_in_g, y_in_g, z_in_g};
  return result;
}

lsm303agr_measurement_t lsm303agr_read_magnetometer(void) {
  //TODO: implement me
  uint8_t x_l = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_X_L_REG);
  uint8_t x_h = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_X_H_REG);
  uint8_t y_l = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Y_L_REG);
  uint8_t y_h = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Y_H_REG);
  uint8_t z_l = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Z_L_REG);
  uint8_t z_h = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Z_H_REG);
  uint16_t x = (x_h << 8) | x_l;
  uint16_t y = (y_h << 8) | y_l;
  uint16_t z = (z_h << 8) | z_l;
  int16_t x_16 = (int16_t)x;
  int16_t y_16 = (int16_t)y;
  int16_t z_16 = (int16_t)z;
  int16_t x_scaled = x_16 * 1.5;
  int16_t y_scaled = y_16 * 1.5;
  int16_t z_scaled = z_16 * 1.5;
  float x_in_gauss = x_scaled / 1000.0;
  float y_in_gauss = y_scaled / 1000.0;
  float z_in_gauss = z_scaled / 1000.0;
  lsm303agr_measurement_t result = {x_in_gauss, y_in_gauss, z_in_gauss};

  return result;
}

// define a function for converting accelerometer data to tilt angles
float convert_accelerometer_to_tilt_angles(lsm303agr_measurement_t acc_measurement) {
  double pow_x = acc_measurement.x_axis * acc_measurement.x_axis;
  double pow_y = acc_measurement.y_axis * acc_measurement.y_axis;
  float temp = sqrt(pow_x + pow_y) / acc_measurement.z_axis;
  float phi = atan(temp);
  // convert phi to degrees
  float phi_degrees = phi/3.14159 * 180;
  return phi_degrees;
}