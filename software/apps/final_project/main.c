// I2C sensors app
//
// Read from I2C accelerometer/magnetometer on the Microbit

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "microbit_v2.h"
#include "lsm303agr.h"
#include "app_timer.h"
#include "nrfx_gpiote.h"

static const nrf_twi_mngr_t* i2c_manager = NULL;
int8_t state = 0;
int8_t direction = 1;
float base = 90.0;
float arm = 90.0;

// Global variables
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);
APP_TIMER_DEF(lab5_timer);

// timer callback for printing temperature
void temp_timer_callback(void * p_context) {

  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      if (i==sensitivity_state){
        led_states[i][j] = true;
      }
      else {
        led_states[i][j] = false;
      }
    }
  }

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

  float x_tolerance = 10.0 + 10.0*sensitivity_state;
  float y_tolerance = 5.0 + 10.0*sensitivity_state;

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
}

int main(void) {
  printf("Board started!\n");

  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  //i2c_config.scl = I2C_SCL;
  //i2c_config.sda = I2C_SDA;
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  // Initialize the LSM303AGR accelerometer/magnetometer sensor
  i2c_manager = &twi_mngr_instance;
  // Read WHO AM I register
  // Always returns the same value if working
  uint8_t result = i2c_reg_read(ICM20948_ADDRESS, ICM20948_WHO_AM_I);
  //TODO: check the result of the Accelerometer WHO AM I register
  printf("ICM20948 WHO AM I: %x\n", result);
  i2c_reg_write(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  nrf_delay_ms(100);
  // Set PWM frequency to 50Hz
  set_pca9685_pwm_freq(50);
  // gpio init
  nrf_gpio_pin_dir_set(CAPACITIVE1, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(MAGNET, NRF_GPIO_PIN_DIR_OUTPUT);


  //TODO: implement me!
  nrfx_gpiote_init();
  app_timer_init();
  capacitive_touch_init();
  led_matrix_init();
  app_timer_create(&lab5_timer, APP_TIMER_MODE_REPEATED, temp_timer_callback);
  app_timer_start(lab5_timer, APP_TIMER_TICKS(10), NULL);

  // Loop forever
  while (1) {
    // Don't put any code in here. Instead put periodic code in a callback using a timer.
    nrf_delay_ms(1000);
  }
}

