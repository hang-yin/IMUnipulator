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

#define MAGNET EDGE_P14