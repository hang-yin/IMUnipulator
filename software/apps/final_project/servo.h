#pragma once
#include "nrf_twi_mngr.h"

static const uint8_t PCA9685_ADDRESS = 0x40;

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

void set_pca9685_pwm_freq(uint16_t freq, const nrf_twi_mngr_t* i2c);
void set_mg996r_angle(uint8_t channel, float angle);
void set_ds3218_angle(uint8_t channel, float angle);