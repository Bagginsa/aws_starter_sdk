# Copyright (C) 2008-2015 Marvell International Ltd.
# All Rights Reserved.
#

exec-y += aws_starter_demo
aws_starter_demo-objs-y := src/main.c
aws_starter_demo-objs-y += src/sensor_drv.c
aws_starter_demo-objs-y += src/sensor_tempr_drv.c
aws_starter_demo-objs-y += src/sensor_light_drv.c
aws_starter_demo-objs-y += src/sensor_presure_drv.c
aws_starter_demo-objs-y += src/sensor_occ_drv.c
aws_starter_demo-objs-y += src/sensor_co2_drv.c
aws_starter_demo-objs-y += src/sensor_gas_drv.c
aws_starter_demo-objs-y += src/sensor_th_drv.c
aws_starter_demo-objs-y += src/dht_drv.c

aws_starter_demo-cflags-y := -I$(d)/src -DAPPCONFIG_DEBUG_ENABLE=1

# Applications could also define custom linker files if required using following:
#aws_starter_demo-ld-y := /path/to/ldscript

# Applications could also define custom board files if required using following:
#aws_starter_demo-board-y := /path/to/boardfile
