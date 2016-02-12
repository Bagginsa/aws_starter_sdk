/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * Custom Sensor Driver for AWS Application
 *
 * Summary:
 *
 * This driver offers h/w specific abstraction to register and report
 * specific sensor event to the AWS cloud
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
#include "sensor_th_drv.h"
#include <dht_drv.h>

#define TH_SEN_IO		GPIO_22
#define TH_SEN_IO_GPIO		GPIO22_GPIO22

struct io_gpio_cfg IO_1WIRE = { /* Used for 1WIRE protocol */
	.pinno = GPIO_0,
	.val = GPIO_IO_HIGH,
	.altfun = GPIO0_GPIO0,
	.pinmode = PINMODE_DEFAULT,
	.dir = GPIO_OUTPUT,
	.irqtype = GPIO_INT_DISABLE,
	.id = GPT0_ID,
	.ch = GPT_CH_0,
};

static uint8_t tempr[4];
static uint8_t humidity[4];
static char T_ready[16];
static char H_ready[16];

/*
 *********************************************************
 **** Th Sensor H/W Specific code
 **********************************************************
 */

int dht_readings(struct dht_psmvar *dht_vars)
{
	wmprintf("Temperature =%d Humidity = %d\r\n",
			dht_vars->temperature, dht_vars->humidity * 2);
	tempr[3] = tempr[2];
	tempr[2] = tempr[1];
	tempr[1] = tempr[0];
	tempr[0] = dht_vars->temperature;
	if ((tempr[0] == tempr[1])
		&& (tempr[1] == tempr[2])
		&& (tempr[2] != tempr[3])) {
			/* Temperature reading is ready to post */
			sprintf(T_ready, "%d", tempr[3]);
			/*wmprintf("Reporting Temperature = %s\r\n", T_ready);*/
	}
	humidity[3] = humidity[2];
	humidity[2] = humidity[1];
	humidity[1] = humidity[0];
	humidity[0] = dht_vars->humidity * 2;
	if ((humidity[0] == humidity[1])
		&& (humidity[1] == humidity[2])
		&& (humidity[2] != humidity[3])) {
			/* Temperature reading is ready to post */
			sprintf(H_ready, "%d", humidity[3]);
			wmprintf("Reporting Humidity = %s\r\n", H_ready);
	}
	return 0;
}

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int th_sensor_init(struct sensor_info *curevent)
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

	/* digital TH related init */
	dht_drv_init(&IO_1WIRE);
	dht_learn(dht_readings);

	/* Close drivers */
	pinmux_drv_close(pinmux_dev);
	gpio_drv_close(gpio_dev);

	return 0;
}

/* Sensor input from IO should be read here and to be passed
	in curevent->event_curr_value variable to the upper layer

	Respective AWS event will be reported to the cloud by
	uper sensor_driver layer

	This function will be called periodically by the upper layer
	hence you can poll your input here, and there is no need of
	callback IO interrupt, this is very usefull to sense variable
	data from analog sensors connected to ADC lines. Event this can
	be used to digital IO scanning and polling
*/
int tht_sensor_input_scan(struct sensor_info *curevent)
{
	if (strlen(T_ready)) {
		sprintf(curevent->event_curr_value, "%s", T_ready);
		T_ready[0] = 0;
		/*wmprintf("Reported Temperature = %s\r\n",
				curevent->event_curr_value);*/
	}
	return 0;
}

int thh_sensor_input_scan(struct sensor_info *curevent)
{
	if (strlen(H_ready)) {
		sprintf(curevent->event_curr_value, "%s", H_ready);
		H_ready[0] = 0;
		/*wmprintf("Reported Humidity = %s\r\n",
				curevent->event_curr_value);*/
	}
	return 0;
}

struct sensor_info event_thh_sensor = {
	.property = "SEN1130P-Humidity",
	.init = th_sensor_init,
	.read = thh_sensor_input_scan,
};

struct sensor_info event_tht_sensor = {
	.property = "SEN1130P-Temprature",
	.init = th_sensor_init,
	.read = tht_sensor_input_scan,
};

int th_sensor_event_register(void)
{
	sensor_event_register(&event_tht_sensor);
	return sensor_event_register(&event_thh_sensor);
}

