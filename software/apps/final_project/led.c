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
#include "nrfx_gpiote.h"

bool led_states[5][5] = {false};
uint32_t row_displayed = 0;

uint32_t row_leds[] = {LED_ROW1, LED_ROW2, LED_ROW3, LED_ROW4, LED_ROW5};
uint32_t col_leds[] = {LED_COL1, LED_COL2, LED_COL3, LED_COL4, LED_COL5};

static void led_matrix_timer_handler(void* _unused) {
  // turn off all rows
  for (int i = 0; i < 5; i++) {
      nrf_gpio_pin_clear(row_leds[i]);
  }
  // turn off all columns
  for (int i = 0; i < 5; i++) {
      nrf_gpio_pin_clear(col_leds[i]);
  }
  // turn on the next row
  nrf_gpio_pin_set(row_leds[row_displayed]);
  // turn on the columns that should be on
  for (int i = 0; i < 5; i++) {
      if (!led_states[row_displayed][i]) {
          nrf_gpio_pin_set(col_leds[i]);
      }
  }
  row_displayed = (row_displayed + 1) % 5;
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
 APP_TIMER_DEF(my_timer_1);
 APP_TIMER_DEF(my_timer_2);
 app_timer_create(&my_timer_1, APP_TIMER_MODE_REPEATED, led_matrix_timer_handler);
 app_timer_create(&my_timer_2, APP_TIMER_MODE_REPEATED, led_matrix_timer_handler);
 app_timer_start(my_timer_1, 32768/1000, NULL); // 32768
 app_timer_start(my_timer_2, 32768, NULL);
}