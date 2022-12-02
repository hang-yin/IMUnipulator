// LSM303AGR driver for Microbit_v2
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "lsm303agr.h"
#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "app_timer.h"
#include "microbit_v2.h"
#include "nrfx_timer.h"

#define CAPACITIVE1 EDGE_P16
#define CAPACITIVE2 EDGE_P15
#define MAGNET EDGE_P14

// Pointer to an initialized I2C instance to use for transactions
static bool touch_active = false;
static const nrf_twi_mngr_t* i2c_manager = NULL;
int8_t state = 0;
int8_t direction = 1;
float base = 90.0;
float arm = 90.0;
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
  // Read WHO AM I register
  // Always returns the same value if working
  uint8_t result = i2c_reg_read(ICM20948_ADDRESS, ICM20948_WHO_AM_I);
  //TODO: check the result of the Accelerometer WHO AM I register
  printf("ICM20948 WHO AM I: %x\n", result);

  // Power management reset
  //uint8_t power_mgmt_1 = i2c_reg_read(ICM20948_ADDRESS, PWR_MGMT_1);
  //printf("Power management 1: %x\n", power_mgmt_1);
  
  i2c_reg_write(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  nrf_delay_ms(100);

  // Setup PCA9685
  

  // Set PWM frequency to 50Hz
  set_pca9685_pwm_freq(50);

  // gpio init
  nrf_gpio_pin_dir_set(CAPACITIVE1, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(MAGNET, NRF_GPIO_PIN_DIR_OUTPUT);

  // Set up capacitive touch

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

  // Accelerometer code
  lsm303agr_measurement_t acc_measurement = icm20948_read_accelerometer();
  //printf("Accelerometer: %f g, %f g, %f g\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
  //printf("Magnetometer: %f Gs, %f Gs, %f Gs\n", mag_measurement.x_axis, mag_measurement.y_axis, mag_measurement.z_axis);
  lsm303agr_measurement_t result = convert_accelerometer_to_tilt_angles(acc_measurement);
  //printf("Tilt: %f degrees, %f degrees, %f degrees\n", result.x_axis, result.y_axis, result.z_axis);
  // printf("Capacitive: %ld\n", nrf_gpio_pin_read(CAPACITIVE));
  uint16_t capacitive1 = nrf_gpio_pin_read(CAPACITIVE1);
  uint16_t capacitive2 = nrf_gpio_pin_read(CAPACITIVE2);
  if (capacitive2 == 0) {
    nrf_gpio_pin_write(MAGNET, 1);
  } else {
    nrf_gpio_pin_write(MAGNET, 0);
  }
  if (capacitive1 == 1) {
    arm = 90.0;
    base = 90.0;
    set_mg996r_angle(0,arm);
    set_mg996r_angle(1,base);
    return;
  }
  float x_tilt = result.z_axis;
  float y_tilt = result.y_axis;

  float x_tolerance = 20.0;
  float y_tolerance = 15.0;

  // x needs to be incremental
  if (x_tilt > x_tolerance) {
    arm = arm + 0.7;
    if (arm > 180.0) {
      arm = 180.0;
    }
    set_mg996r_angle(0,arm);
    return;
  } else if (x_tilt < -x_tolerance) {
    arm = arm - 0.7;
    if (arm < 50.0) {
      arm = 50.0;
    }
    set_mg996r_angle(0,arm);
    return;
  }
  
  // y needs to be incremental
  if (y_tilt > y_tolerance) {
    base = base - 1.0;
  } else if (y_tilt < -y_tolerance) {
    base = base + 1.0;
  }
  // set limits for the base
  if (base > 180.0) {
    base = 180.0;
  } else if (base < 0.0) {
    base = 0.0;
  }
  //float angle_y = y_tilt + 90.0;
  set_mg996r_angle(1,base);

  // printf("Tilt: %f, %f, %f\n", tilt_array[0], tilt_array[1], tilt_array[2]);
  //printf("Phi: %f\n", phi);
  //printf("Accelerometer: %x, %x, %x\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);

  /*
  float angle = (float)(state*45);
  printf("Angle: %f\n", angle);

  if (angle >= 180) {
    direction = -1;
  } else if (angle <= 0) {
    direction = 1;
  }

  set_mg996r_angle(0,angle);
  set_ds3218_angle(1,angle);

  state += direction;
  */
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
lsm303agr_measurement_t convert_accelerometer_to_tilt_angles(lsm303agr_measurement_t acc_measurement) {
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
  return (lsm303agr_measurement_t){phi_degrees, theta_degrees, psi_degrees};
  //return phi_degrees;
}

void set_pca9685_pwm_freq(uint16_t freq) {
  uint16_t prescale = (25000000 / (4096 * freq)) - 1;
  // uint8_t old_mode = i2c_reg_read(PCA9685_ADDRESS, PCA9685_MODE1);
  // uint8_t new_mode = (old_mode & 0x7F) | 0x10;

  uint8_t mode1 = i2c_reg_read(PCA9685_ADDRESS, PCA_MODE1);
  // print old_mode
  printf("mode1: %x\n", mode1);
  uint8_t mode2 = i2c_reg_read(PCA9685_ADDRESS, PCA_MODE2);
  // print old_mode2
  printf("mode2: %x\n", mode2);

  i2c_reg_write(PCA9685_ADDRESS, PCA_MODE1, 0x10);
  i2c_reg_write(PCA9685_ADDRESS, PCA_PRESCALE, prescale);
  i2c_reg_write(PCA9685_ADDRESS, PCA_MODE1, 0x80);
  i2c_reg_write(PCA9685_ADDRESS, PCA_MODE2, 0x04);
}

void set_pca9685_pwm(uint8_t channel, uint16_t on, uint16_t off) {
  on = on & 0x0FFF;
  off = off & 0x0FFF;
  i2c_reg_write(PCA9685_ADDRESS, LED0_ON_L + 4 * channel, off & 0xff);
  i2c_reg_write(PCA9685_ADDRESS, LED0_ON_H + 4 * channel, off >> 8);
  i2c_reg_write(PCA9685_ADDRESS, LED0_OFF_L + 4 * channel, on & 0xff);
  i2c_reg_write(PCA9685_ADDRESS, LED0_OFF_H + 4 * channel, on >> 8);
}

void set_duty_cycle(uint8_t channel, float duty_cycle) {
  uint16_t on = (uint16_t)(4095.0 * duty_cycle);
  // printf("on: %d\n", on);
  uint16_t off = 4095 - on;
  // printf("off: %d\n", off);
  set_pca9685_pwm(channel, on, off);
}

float linear_interpolate(float x,float x1,float y1,float x2, float y2) {
  return y1 + (x - x1)*(y2 - y1) / (x2 - x1);
}

void set_mg996r_angle(uint8_t channel, float angle) {
  //MG996R
  //0 deg = 0.0145
  //30 deg = 0.0185
  //45 deg = 0.0225
  //60 deg = 0.028
  //90 deg = 0.035
  //120 deg = 0.04
  //135 deg = 0.044
  //150 deg = 0.051
  //180 deg = 0.055
  
  float duty_cycle;
  int i;
  float angles[9] = {0,30,45,60,90,120,135,150,180};
  float duty_cycles[9] = {0.0145, 0.0185, 0.0225, 0.028, 0.035, 0.04, 0.044, 0.051, 0.055};

  if (angle <= 0) {
    duty_cycle = duty_cycles[0];
  } else if (angle >= 185) {
    duty_cycle = duty_cycles[9];
  } else {
    for (i = 1; i <= 9; i++) {
      if (angle <= angles[i]) {
        duty_cycle = linear_interpolate(angle, angles[i-1], duty_cycles[i-1], angles[i], duty_cycles[i]);
        break;
      }
    }
  }

  set_duty_cycle(channel, duty_cycle);
}

void set_ds3218_angle(uint8_t channel, float angle) {
  //DS3218
  //0 deg = 0.015
  //30 deg = 0.021
  //45 deg = 0.025
  //60 deg = 0.0325
  //90 deg = 0.04
  //120 deg = 0.0475
  //135 deg = 0.0525
  //150 deg = 0.059
  //180 deg = 0.0635
  
  float duty_cycle;
  int i;
  float angles[9] = {0,30,45,60,90,120,135,150,180};
  float duty_cycles[9] = {0.015, 0.021, 0.025, 0.0325, 0.04, 0.0475, 0.0525, 0.059, 0.0635};

  if (angle <= 0) {
    duty_cycle = duty_cycles[0];
  } else if (angle >= 181) {
    duty_cycle = duty_cycles[9];
  } else {
    for (i = 1; i <= 9; i++) {
      if (angle <= angles[i]) {
        duty_cycle = linear_interpolate(angle, angles[i-1], duty_cycles[i-1], angles[i], duty_cycles[i]);
        break;
      }
    }
  }

  set_duty_cycle(channel, duty_cycle);
}