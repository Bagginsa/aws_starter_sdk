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

#define ULTRASONIC_SEN_IO	GPIO_0
#define ULTRASONIC_SEN_IO_GPIO	GPIO0_GPIO0

/* define this to test sensor w/o cloud */
#undef ULTRASONIC_SENSOR_TEST

#ifdef ULTRASONIC_SENSOR_TEST
void check_ultrasonic_sensor(void);
#endif /* ULTRASONIC_SENSOR_TEST */

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

#ifdef ULTRASONIC_SENSOR_TEST
	check_ultrasonic_sensor();
#endif /* ULTRASONIC_SENSOR_TEST */

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
	unsigned int duration = 0;
	unsigned int fduration = 0;

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
	os_thread_sleep(1);
	gpio_drv_write(gpio_dev, ULTRASONIC_SEN_IO, 1);
	os_thread_sleep(1);
	gpio_drv_write(gpio_dev, ULTRASONIC_SEN_IO, 0);

	/* Confiugre GPIO pin direction as Input */
	gpio_drv_setdir(gpio_dev, ULTRASONIC_SEN_IO, GPIO_INPUT);

	/* Check the line is low */
	while(1) {
		gpio_drv_read(gpio_dev, ULTRASONIC_SEN_IO, &val);
		if (!val)
			break;
	};

	/* Check the line is going high */
	while(1) {
		gpio_drv_read(gpio_dev, ULTRASONIC_SEN_IO, &val);
		if (val)
			break;
	};
	duration = os_get_timestamp(); /* start pulse width measurement */

	/* Check the line is going low */
	while(1) {
		gpio_drv_read(gpio_dev, ULTRASONIC_SEN_IO, &val);
		if (!val)
			break;
	};
	fduration = os_get_timestamp(); /* stop pulse width measurement */

	if (fduration > duration) {
		duration = fduration - duration; /* distance in usec */
		/* Calibrate distance measured in centimeters */
		duration /= 29;
		duration /= 2;

		wmprintf("%s senval=%d cm\r\n", __FUNCTION__, duration);
		sprintf(curevent->event_curr_value, "%d", duration);
	}

	gpio_drv_close(gpio_dev);

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

#ifdef ULTRASONIC_SENSOR_TEST
void check_ultrasonic_sensor(void)
{
	while(1) {
		ultrasonic_sensor_input_scan(&event_ultrasonic_sensor);
		os_thread_sleep(1000);
	}
}
#endif /* ULTRASONIC_SENSOR_TEST */

