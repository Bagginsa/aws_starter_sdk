/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * GPIO based Occupancy Sensor Low Level Driver
 *
 * Summary:
 *
 * This driver offers h/w specific abstraction to register, initialize and
 * scan and report specific sensor event to the Sensor Interface Layer
 */

#include <wm_os.h>
#include <wmstdio.h>
#include <wmtime.h>
#include <wmsdk.h>
#include <board.h>
#include <mdev_gpio.h>
#include <mdev_pinmux.h>
#include <lowlevel_drivers.h>

#include "sensor_drv.h"
#include "sensor_occ_drv.h"

#define OCC_SEN_IO		GPIO_22
#define OCC_SEN_IO_GPIO		GPIO22_GPIO22

/*
 *********************************************************
 **** Occupancy Sensor H/W Specific code
 **********************************************************
 */

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int occupancy_sensor_init(struct sensor_info *curevent)
{
	wmprintf("%s\r\n", __FUNCTION__);

	mdev_t *pinmux_dev, *gpio_dev;

	/* Initialize  pinmux driver */
	pinmux_drv_init();

	/* Open pinmux driver */
	pinmux_dev = pinmux_drv_open("MDEV_PINMUX");

	/* Initialize GPIO driver */
	gpio_drv_init();

	/* Open GPIO driver */
	gpio_dev = gpio_drv_open("MDEV_GPIO");

	/* Configure GPIO pin function for GPIO connected to LED */
	pinmux_drv_setfunc(pinmux_dev, OCC_SEN_IO, OCC_SEN_IO_GPIO);

	/* Confiugre GPIO pin direction as Input */
	gpio_drv_setdir(gpio_dev, OCC_SEN_IO, GPIO_INPUT);

	/* Close drivers */
	pinmux_drv_close(pinmux_dev);
	gpio_drv_close(gpio_dev);

	return 0;
}

/* Sensor input from IO should be read here and to be passed
	in curevent->event_curr_value variable to the upper layer

	This function will be called periodically by the upper layer
	hence you can poll your input here, and there is no need of
	callback IO interrupt, this is very usefull to sense variable
	data from analog sensors connected to ADC lines. Event this can
	be used to digital IO scanning and polling
*/
int occupancy_sensor_input_scan(struct sensor_info *curevent)
{
	int val;
	mdev_t *gpio_dev;

	/* Open GPIO driver */
	gpio_dev = gpio_drv_open("MDEV_GPIO");

	/* Read sensor GPIO level */
	gpio_drv_read(gpio_dev, OCC_SEN_IO, &val);

	dbg("%s senval=%d\r\n", __FUNCTION__, val);

	/* for testing purpose only,
		disable this line getData is functional*/
	sprintf(curevent->event_curr_value, "%d", val);
	gpio_drv_close(gpio_dev);
	return 0;
}

struct sensor_info event_occupancy_sensor = {
	.property = "occupancy",
	.init = occupancy_sensor_init,
	.read = occupancy_sensor_input_scan,
};

int occupancy_sensor_event_register(void)
{
	return sensor_event_register(&event_occupancy_sensor);
}

