#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "servo.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "microbit_v2.h"
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"
#include "i2c.h"

static const nrf_twi_mngr_t* i2c_manager = NULL;

void set_pca9685_pwm_freq(uint16_t freq, const nrf_twi_mngr_t* i2c) {
  i2c_manager = i2c;
  uint16_t prescale = (25000000 / (4096 * freq)) - 1;
  // uint8_t old_mode = i2c_reg_read(PCA9685_ADDRESS, PCA9685_MODE1);
  // uint8_t new_mode = (old_mode & 0x7F) | 0x10;

  uint8_t mode1 = i2c_reg_read(PCA9685_ADDRESS, PCA_MODE1, i2c_manager);
  // print old_mode
  printf("mode1: %x\n", mode1);
  uint8_t mode2 = i2c_reg_read(PCA9685_ADDRESS, PCA_MODE2, i2c_manager);
  // print old_mode2
  printf("mode2: %x\n", mode2);

  i2c_reg_write(PCA9685_ADDRESS, PCA_MODE1, 0x10, i2c_manager);
  i2c_reg_write(PCA9685_ADDRESS, PCA_PRESCALE, prescale, i2c_manager);
  i2c_reg_write(PCA9685_ADDRESS, PCA_MODE1, 0x80, i2c_manager);
  i2c_reg_write(PCA9685_ADDRESS, PCA_MODE2, 0x04, i2c_manager);
}

static void set_pca9685_pwm(uint8_t channel, uint16_t on, uint16_t off) {
  on = on & 0x0FFF;
  off = off & 0x0FFF;
  i2c_reg_write(PCA9685_ADDRESS, LED0_ON_L + 4 * channel, off & 0xff, i2c_manager);
  i2c_reg_write(PCA9685_ADDRESS, LED0_ON_H + 4 * channel, off >> 8, i2c_manager);
  i2c_reg_write(PCA9685_ADDRESS, LED0_OFF_L + 4 * channel, on & 0xff, i2c_manager);
  i2c_reg_write(PCA9685_ADDRESS, LED0_OFF_H + 4 * channel, on >> 8, i2c_manager);
}

static void set_duty_cycle(uint8_t channel, float duty_cycle) {
  uint16_t on = (uint16_t)(4095.0 * duty_cycle);
  // printf("on: %d\n", on);
  uint16_t off = 4095 - on;
  // printf("off: %d\n", off);
  set_pca9685_pwm(channel, on, off);
}

static float linear_interpolate(float x,float x1,float y1,float x2, float y2) {
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