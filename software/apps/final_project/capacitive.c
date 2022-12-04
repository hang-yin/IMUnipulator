#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "capacitive.h"
#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "app_timer.h"
#include "microbit_v2.h"
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"

#define CAPACITIVE1 EDGE_P16
#define CAPACITIVE2 EDGE_P15

// Capacitive Touch setup
APP_TIMER_DEF(capacitive_touch_timer);
static nrfx_timer_t TIMER4 = NRFX_TIMER_INSTANCE(0);
static bool touch_active = false;
static bool touch_active_prev = false;
static int16_t sensitivity_state = 0;

bool get_touch_active(void) {
    return touch_active;
}

static void disable_both(void) {
  // Disable both channels
  nrfx_timer_disable(&TIMER4);
  nrfx_gpiote_in_event_disable(TOUCH_LOGO);
}

// Callback function for GPIO interrupts
static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  touch_active_prev = touch_active;
  touch_active = false;
  disable_both();
}

static void timer_handler(nrf_timer_event_t event, void* context) {
  // Implement in a later step
  //printf("Timer interrupt, touched!\n");
  touch_active_prev = touch_active;
  touch_active = true;
  if (!touch_active_prev && touch_active) {
    sensitivity_state = (sensitivity_state + 1) % 5;
  }
  disable_both();
}

static void start_capacitive_test(void* context) {
  // set pin as input and clear it
  nrf_gpio_cfg(TOUCH_LOGO, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(TOUCH_LOGO);
  nrfx_timer_clear(&TIMER4);
  nrfx_timer_resume(&TIMER4);
  nrfx_timer_compare(&TIMER4, NRF_TIMER_CC_CHANNEL1, 800, true);
  nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // high-accuracy mode
  nrfx_gpiote_in_init(TOUCH_LOGO, &in_config, gpio_handler);
  nrfx_gpiote_in_event_enable(TOUCH_LOGO, true); // enable interrupts
}

void capacitive_touch_init(void) {
  // configure high-speed timer
  // timer should be 1 MHz and 32-bit
  nrfx_timer_config_t timer_config = {
    .frequency = NRF_TIMER_FREQ_1MHz,
    .mode = NRF_TIMER_MODE_TIMER,
    .bit_width = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = 4,
    .p_context = NULL
  };
  nrfx_timer_init(&TIMER4, &timer_config, timer_handler);
  // enable, but pause the timer
  nrfx_timer_enable(&TIMER4);
  nrfx_timer_pause(&TIMER4);
  // start the touch test
  app_timer_init();
  app_timer_create(&capacitive_touch_timer, APP_TIMER_MODE_REPEATED, start_capacitive_test);
  app_timer_start(capacitive_touch_timer, APP_TIMER_TICKS(100), NULL);
}