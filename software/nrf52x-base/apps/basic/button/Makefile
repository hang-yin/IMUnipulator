PROJECT_NAME = $(shell basename "$(realpath ./)")

# Configurations
NRF_IC = nrf52832
SDK_VERSION = 15
SOFTDEVICE_MODEL = s132

# Source and header files
APP_HEADER_PATHS += .
APP_SOURCE_PATHS += .
APP_SOURCES = $(notdir $(wildcard ./*.c))

# Include board Makefile (if any)
#include $(NRF_BASE_DIR)/boards/<BOARD_NAME_HERE>

# Include main Makefile
NRF_BASE_DIR ?= ../../../
include $(NRF_BASE_DIR)/make/AppMakefile.mk
