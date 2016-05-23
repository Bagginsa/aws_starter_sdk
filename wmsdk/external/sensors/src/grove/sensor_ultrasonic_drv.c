/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * GPIO based Ultrasonic Sensor Low Level Driver
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
#include "sensor_ultrasonic_drv.h"

#define ULTRASONIC_SEN_IO	GPIO_22
#define ULTRASONIC_SEN_IO_GPIO	GPIO22_GPIO22

#define dly_in_usecs(x)	os_thread_sleep(x);
/*
 *********************************************************
 **** Ultrasonic Sensor H/W Specific code
 **********************************************************
 */

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int ultrasonic_sensor_init(struct sensor_info *curevent)
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
	pinmux_drv_setfunc(pinmux_dev, ULTRASONIC_SEN_IO, ULTRASONIC_SEN_IO_GPIO);

	/* Confiugre GPIO pin direction as Input */
	gpio_drv_setdir(gpio_dev, ULTRASONIC_SEN_IO, GPIO_INPUT);

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
int ultrasonic_sensor_input_scan(struct sensor_info *curevent)
{
	int val;
	mdev_t *gpio_dev;
	int duration = 0;

	/* Open GPIO driver */
	gpio_dev = gpio_drv_open("MDEV_GPIO");

	/* Confiugre GPIO pin direction as Input */
	gpio_drv_setdir(gpio_dev, ULTRASONIC_SEN_IO, GPIO_OUTPUT);

	/* Ultrosonic distance reading
		Send one altrasonic pulse and wait for
		its return responce
		Then calculate the distance between transmistted
		and received input
	*/

	/* Send a pulse */
	gpio_drv_write(gpio_dev, ULTRASONIC_SEN_IO, 0);
	dly_in_usecs(2);
	gpio_drv_write(gpio_dev, ULTRASONIC_SEN_IO, 1);
	dly_in_usecs(5);
	gpio_drv_write(gpio_dev, ULTRASONIC_SEN_IO, 0);

	/* Confiugre GPIO pin direction as Input */
	gpio_drv_setdir(gpio_dev, ULTRASONIC_SEN_IO, GPIO_INPUT);
	do {
		duration++;
		/* Read sensor GPIO level */
		gpio_drv_read(gpio_dev, ULTRASONIC_SEN_IO, &val);
		dly_in_usecs(1);

	} while (!val);

	gpio_drv_close(gpio_dev);


	/* for testing purpose only, need calibration to be done */
	dbg("%s senval=%d\r\n", __FUNCTION__, duration);
	sprintf(curevent->event_curr_value, "%d", duration);
	return 0;
}

struct sensor_info event_ultrasonic_sensor = {
	.property = "ultrasonic",
	.init = ultrasonic_sensor_init,
	.read = ultrasonic_sensor_input_scan,
};

int ultrasonic_sensor_event_register(void)
{
	return sensor_event_register(&event_ultrasonic_sensor);
}

