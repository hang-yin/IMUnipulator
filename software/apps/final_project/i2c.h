#pragma once
#include "nrf_twi_mngr.h"

uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr);
void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);