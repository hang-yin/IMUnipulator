#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "microbit_v2.h"
#include "app_timer.h"
#include "nrfx_gpiote.h"
#include "servo.h"
#include "led.h"
#include "imu.h"
#include "capacitive.h"

// Define local variables
#define CAPACITIVE1 EDGE_P16
static const nrf_twi_mngr_t* i2c_manager = NULL;
static float base = 90.0;
static float arm = 90.0;
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);
APP_TIMER_DEF(main_timer);

// timer callback for printing temperature
void timer_callback(void * p_context) {
  int16_t sensitivity_state = get_sensitivity_state();
  // printf("Sensitivity state: %d\n", sensitivity_state);
  set_led_row(sensitivity_state);

  icm20948_measurement_t acc_measurement = icm20948_read_accelerometer(i2c_manager);
  icm20948_measurement_t result = convert_accelerometer_to_tilt_angles(acc_measurement);

  uint16_t capacitive1 = nrf_gpio_pin_read(CAPACITIVE1);
  if (capacitive1 == 1) {
    arm = 90.0;
    base = 90.0;
    set_mg996r_angle(0,arm);
    set_mg996r_angle(1,base);
    return;
  }

  float x_tilt = result.z_axis;
  float y_tilt = result.y_axis;
  float x_tolerance = 15.0;
  float y_tolerance = 15.0;
  float x_speed = 0.1 + 0.1 * sensitivity_state; // set 0.3 to middle
  float y_speed = 0.4 + 0.1 * sensitivity_state; // set 0.6 to middle

  if (x_tilt > x_tolerance) {
    arm = arm + x_speed;
    if (arm > 180.0) {
      arm = 180.0;
    }
    set_mg996r_angle(0,arm);
    return;
  } else if (x_tilt < -x_tolerance) {
    arm = arm - x_speed;
    if (arm < 50.0) {
      arm = 50.0;
    }
    set_mg996r_angle(0,arm);
    return;
  }
  
  if (y_tilt > y_tolerance) {
    base = base + y_speed;
    if (base > 180.0) {
      base = 180.0;
    }
  } else if (y_tilt < -y_tolerance) {
    base = base - y_speed;
    if (base < 0.0) {
      base = 0.0;
    }
  }
  set_mg996r_angle(1,base);
}

int main(void) {
  printf("Board started!\n");

  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  // Initialize the accelerometer
  i2c_manager = &twi_mngr_instance;
  // Read WHO AM I register
  // Always returns the same value if working
  uint8_t result = i2c_reg_read(ICM20948_ADDRESS, ICM20948_WHO_AM_I, i2c_manager);
  // Check the result of the Accelerometer WHO AM I register
  printf("ICM20948 WHO AM I: %x\n", result);
  i2c_reg_write(ICM20948_ADDRESS, PWR_MGMT_1, 0x01, i2c_manager);
  nrf_delay_ms(100);
  printf("here1\n");
  // Set PWM frequency to 50Hz
  set_pca9685_pwm_freq(50, i2c_manager);
  printf("here2\n");
  // gpio init
  nrfx_gpiote_init();
  printf("here3\n");
  nrf_gpio_pin_dir_set(CAPACITIVE1, NRF_GPIO_PIN_DIR_INPUT);
  printf("here4\n");
  capacitive_touch_init();
  printf("here5\n");
  led_matrix_init();
  printf("here6\n");
  // app_timer_init();
  printf("here7\n");
  app_timer_create(&main_timer, APP_TIMER_MODE_REPEATED, timer_callback);
  app_timer_start(main_timer, APP_TIMER_TICKS(10), NULL);

  // Loop forever
  while (1) {
    // Code in periodic timer
    nrf_delay_ms(1000);
  }
}
