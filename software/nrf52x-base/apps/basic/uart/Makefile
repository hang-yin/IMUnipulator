PROJECT_NAME = $(shell basename "$(realpath ./)")

# Configurations
NRF_IC = nrf52832
SDK_VERSION = 16
SOFTDEVICE_MODEL = s132

# Source and header files
APP_HEADER_PATHS += .
APP_SOURCE_PATHS += .
APP_SOURCES = $(notdir $(wildcard ./*.c))
APP_SOURCES += retarget.c

NRF_BASE_DIR ?= ../../../

# Include board Makefile (if any)
include $(NRF_BASE_DIR)/boards/nrf52840dk/Board.mk

# Include main Makefile
include $(NRF_BASE_DIR)/make/AppMakefile.mk
