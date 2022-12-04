#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "app_timer.h"
#include "microbit_v2.h"
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"

// Helper function to perform a 1-byte I2C read of a given register
//
// i2c_addr - address of the device to read from
// reg_addr - address of the register within the device to read
//
// returns 8-bit read value
uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
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
void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  uint8_t tx_buf[2] = {reg_addr, data};
  //uint16_t my_data = (reg_addr << 8) | data;
  nrf_twi_mngr_transfer_t const write_transfer[] = {
      NRF_TWI_MNGR_WRITE(i2c_addr, tx_buf, 2, 0),
  };
  //Note: there should only be a single two-byte transfer to be performed
  nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);
}