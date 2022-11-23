// LSM303AGR accelerometer and magnetometer

#pragma once

#include "nrf_twi_mngr.h"

// Chip addresses for accelerometer and magnetometer
static const uint8_t LSM303AGR_ACC_ADDRESS = 0x19;
static const uint8_t LSM303AGR_MAG_ADDRESS = 0x1E;
static const uint8_t ICM20948_ADDRESS = 0x69;
static const uint8_t PCA9685_ADDRESS = 0x40;

// Measurement data type
typedef struct {
  float x_axis;
  float y_axis;
  float z_axis;
} lsm303agr_measurement_t;

// Register definitions for PCA9685
typedef enum {
    LED0_ON_L = 0x06,
    LED0_ON_H = 0x07,
    LED0_OFF_L = 0x08,
    LED0_OFF_H = 0x09,
    LED1_ON_L = 0x0A,
    LED1_ON_H = 0x0B,
    LED1_OFF_L = 0x0C,
    LED1_OFF_H = 0x0D,
    LED2_ON_L = 0x0E,
    LED2_ON_H = 0x0F,
    LED2_OFF_L = 0x10,
    LED2_OFF_H = 0x11,
    PCA_PRESCALE = 0xFE,
    PCA_MODE1 = 0x00,
    PCA_MODE2 = 0x01,
    PCA_ALL_LED_ON_L = 0xFA,
    PCA_ALL_LED_ON_H = 0xFB,
    PCA_ALL_LED_OFF_L = 0xFC,
    PCA_ALL_LED_OFF_H = 0xFD,
} pca9685_reg_t;

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

// Register definitions for accelerometer
typedef enum {
  LSM303AGR_ACC_STATUS_REG_AUX = 0X07,
  LSM303AGR_ACC_TEMP_L = 0x0C,
  LSM303AGR_ACC_TEMP_H = 0x0D,
  LSM303AGR_ACC_INT_COUNTER_REG = 0X0E,
  LSM303AGR_ACC_WHO_AM_I_REG = 0X0F,
  LSM303AGR_ACC_TEMP_CFG_REG = 0X1F,
  LSM303AGR_ACC_CTRL_REG1 = 0X20,
  LSM303AGR_ACC_CTRL_REG2 = 0X21,
  LSM303AGR_ACC_CTRL_REG3 = 0X22,
  LSM303AGR_ACC_CTRL_REG4 = 0X23,
  LSM303AGR_ACC_CTRL_REG5 = 0X24,
  LSM303AGR_ACC_CTRL_REG6 = 0X25,
  LSM303AGR_ACC_REFERENCE = 0X26,
  LSM303AGR_ACC_STATUS_REG = 0X27,
  LSM303AGR_ACC_OUT_X_L = 0X28,
  LSM303AGR_ACC_OUT_X_H = 0X29,
  LSM303AGR_ACC_OUT_Y_L = 0X2A,
  LSM303AGR_ACC_OUT_Y_H = 0X2B,
  LSM303AGR_ACC_OUT_Z_L = 0X2C,
  LSM303AGR_ACC_OUT_Z_H = 0X2D,
  LSM303AGR_ACC_FIFO_CTRL_REG = 0X2E,
  LSM303AGR_ACC_FIFO_SRC_REG = 0X2F,
  LSM303AGR_ACC_INT1_CFG = 0X30,
  LSM303AGR_ACC_INT1_SRC = 0X31,
  LSM303AGR_ACC_INT1_THS = 0X32,
  LSM303AGR_ACC_INT1_DURATION = 0X33,
  LSM303AGR_ACC_INT2_CFG = 0X34,
  LSM303AGR_ACC_INT2_SRC = 0X35,
  LSM303AGR_ACC_INT2_THS = 0X36,
  LSM303AGR_ACC_INT2_DURATION = 0X37,
  LSM303AGR_ACC_CLICK_CFG = 0X38,
  LSM303AGR_ACC_CLICK_SRC = 0X39,
  LSM303AGR_ACC_CLICK_THS = 0X3A,
  LSM303AGR_ACC_TIME_LIMIT = 0X3B,
  LSM303AGR_ACC_TIME_LATENCY = 0X3C,
  LSM303AGR_ACC_TIME_WINDOW = 0X3D,
} lsm303agr_acc_reg_t;

// Register definitions for magnetometer
typedef enum {
  LSM303AGR_MAG_OFFSET_X_REG_L = 0X45,
  LSM303AGR_MAG_OFFSET_X_REG_H = 0X46,
  LSM303AGR_MAG_OFFSET_Y_REG_L = 0X47,
  LSM303AGR_MAG_OFFSET_Y_REG_H = 0X48,
  LSM303AGR_MAG_OFFSET_Z_REG_L = 0X49,
  LSM303AGR_MAG_OFFSET_Z_REG_H = 0X4A,
  LSM303AGR_MAG_WHO_AM_I_REG = 0X4F,
  LSM303AGR_MAG_CFG_REG_A = 0X60,
  LSM303AGR_MAG_CFG_REG_B = 0X61,
  LSM303AGR_MAG_CFG_REG_C = 0X62,
  LSM303AGR_MAG_INT_CTRL_REG = 0X63,
  LSM303AGR_MAG_INT_SOURCE_REG = 0X64,
  LSM303AGR_MAG_INT_THS_L_REG = 0X65,
  LSM303AGR_MAG_INT_THS_H_REG = 0X66,
  LSM303AGR_MAG_STATUS_REG = 0X67,
  LSM303AGR_MAG_OUT_X_L_REG = 0X68,
  LSM303AGR_MAG_OUT_X_H_REG = 0X69,
  LSM303AGR_MAG_OUT_Y_L_REG = 0X6A,
  LSM303AGR_MAG_OUT_Y_H_REG = 0X6B,
  LSM303AGR_MAG_OUT_Z_L_REG = 0X6C,
  LSM303AGR_MAG_OUT_Z_H_REG = 0X6D,
} lsm303agr_mag_reg_t;

// Function prototypes

// Initialize and configure the LSM303AGR accelerometer/magnetometer
//
// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c);

// Read the internal temperature sensor
//
// Return measurement as floating point value in degrees C
float lsm303agr_read_temperature(void);

void temp_timer_callback(void * p_context);

// Read all three axes on the accelerometer
//
// Return measurements as floating point values in g's
lsm303agr_measurement_t lsm303agr_read_accelerometer(void);

// Read all three axes on the magnetometer
//
// Return measurements as floating point values in uT
lsm303agr_measurement_t lsm303agr_read_magnetometer(void);

lsm303agr_measurement_t convert_accelerometer_to_tilt_angles(lsm303agr_measurement_t acc_measurement);

// OUT NEW FUNCTION HEADERS
lsm303agr_measurement_t icm20948_read_accelerometer(void);
int16_t combine_bytes(uint8_t lsb, uint8_t msb);

void set_pca9685_pwm_freq(uint16_t freq);
void set_pca9685_pwm(uint8_t channel, uint16_t on, uint16_t off);
void set_duty_cycle(uint8_t channel, float duty_cycle);
float linear_interpolate(float x,float x1,float y1,float x2, float y2);
void set_mg996r_angle(uint8_t channel, float angle);
void set_ds3218_angle(uint8_t channel, float angle);