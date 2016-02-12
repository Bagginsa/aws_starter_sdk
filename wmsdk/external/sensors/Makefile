# Copyright (C) 2008-2015, Marvell International Ltd.
# All Rights Reserved.

global-cflags-y += -I $(d)/incl -I $(d)/incl/grove -I $(d)/incl/generic

# To enable Sensor Debug Prints
ifeq ($(SENSOR_DEBUG_ENABLE),y)
global-cflags-y += -DSENSOR_DEBUG_ENABLE=1
endif

-include $(d)/Makefile.sensors
