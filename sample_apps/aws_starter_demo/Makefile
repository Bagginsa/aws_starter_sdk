# Copyright (C) 2008-2015 Marvell International Ltd.
# All Rights Reserved.
#

exec-y += aws_starter_demo
aws_starter_demo-objs-y := src/main.c

aws_starter_demo-cflags-y := -I$(d)/src -DAPPCONFIG_DEBUG_ENABLE=1

ifeq ($(SEN_OCC),y)
aws_starter_demo-cflags-y += -DSEN_OCC=1
endif

ifeq ($(SEN_LIGHT),y)
aws_starter_demo-cflags-y += -DSEN_LIGHT=1
endif

ifeq ($(SEN_PRESSURE),y)
aws_starter_demo-cflags-y += -DSEN_PRESSURE=1
endif

ifeq ($(SEN_TH),y)
aws_starter_demo-cflags-y += -DSEN_TH=1
endif

ifeq ($(SEN_GAS),y)
aws_starter_demo-cflags-y += -DSEN_GAS=1
endif

ifeq ($(SEN_CO2),y)
aws_starter_demo-cflags-y += -DSEN_CO2=1
endif

ifeq ($(SEN_TEMPR),y)
aws_starter_demo-cflags-y += -DSEN_TEMPR=1
endif

# Applications could also define custom linker files if required using following:
#aws_starter_demo-ld-y := /path/to/ldscript

# Applications could also define custom board files if required using following:
#aws_starter_demo-board-y := /path/to/boardfile
