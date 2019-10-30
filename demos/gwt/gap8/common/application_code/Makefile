APP              = test
APP_SRCS        += helloworld.c
APP_INC	        +=
APP_CFLAGS      += $(INCLUDES)

PMSIS_OS = freertos
BOARD_NAME ?= gapuino

override FREERTOS_PATH = $(GAP_SDK_HOME)/freeRTOS
override PMSIS_BSP_DIR = $(FREERTOS_PATH)/vendors/gwt/TARGET_GWT/pmsis/pmsis_bsp
override GWT_PMSIS     = $(FREERTOS_PATH)/vendors/gwt/TARGET_GWT/pmsis/pmsis_driver
override GWT_PMSIS_API = $(FREERTOS_PATH)/vendors/gwt/TARGET_GWT/pmsis/pmsis_api

include $(RULES_DIR)/pmsis_rules.mk
