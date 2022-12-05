#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "led.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "microbit_v2.h"
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"

static uint32_t row_leds[] = {LED_ROW1, LED_ROW2, LED_ROW3, LED_ROW4, LED_ROW5};
static uint32_t col_leds[] = {LED_COL1, LED_COL2, LED_COL3, LED_COL4, LED_COL5};

void set_led_row(int row, bool state) {
  if (state) {
    nrf_gpio_pin_set(row_leds[row]);
  }
  else {
    nrf_gpio_pin_clear(row_leds[row]);
  }
}

void led_matrix_init(void) {
 for (int i = 0; i < 5; i++) {
   nrf_gpio_pin_dir_set(row_leds[i], NRF_GPIO_PIN_DIR_OUTPUT);
   nrf_gpio_pin_clear(row_leds[i]);
 }
 for (int i = 0; i < 5; i++) {
   nrf_gpio_pin_dir_set(col_leds[i], NRF_GPIO_PIN_DIR_OUTPUT);
   nrf_gpio_pin_clear(col_leds[i]);
 }
}