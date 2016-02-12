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
#include "sensor_co2_drv.h"

#include <wm_os.h>
#include <mdev_uart.h>

/*------------------Macro Definitions ------------------*/
#define CO2_UART	UART2_ID 
#define CO2_UART_BAUD	9600 
static mdev_t *uart2_dev;

/*
 *********************************************************
 **** Co2 Sensor H/W Specific code
 **********************************************************
 */

static const uint8_t cmd_get_sensor[] =
{
	0xff, 0x01, 0x86, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x79
};

static int temperature;
static int CO2PPM;
static os_mutex_t co2uart_mutex;
static int co2_sinit_f = 0;
 
bool CO2_dataRecieve(void)
{
	uint8_t data[9];
	int i, len, sz;

	/* transmit command data */
	os_mutex_get(&co2uart_mutex, OS_WAIT_FOREVER);
	uart_drv_write(uart2_dev, cmd_get_sensor, 9); //sizeof(cmd_get_sensor));
 	os_mutex_put(&co2uart_mutex);

	os_thread_sleep(10); /* delay of 10 milisecs */

	/* begin reveiceing data */
	for(i = 0; i < 9; i++) {
		len = 0;
		while(len++ < 100) { /* Ready each byte until it is received, timeout 100msec*/
			sz = uart_drv_read(uart2_dev, &data[i], 1);
			if (sz == 1) {
				/* If a bute is read then store and move to next */
				break;
			} else
				os_thread_sleep(1); /* delay of 1 milisecs */
		}
	}
 
	if((i != 9) || (1 + (0xFF ^ (uint8_t)(data[1] + data[2] + data[3]
	+ data[4] + data[5] + data[6] + data[7]))) != data[8]) {
		wmprintf("Received Data Checksum Error\r\n");
		return false;
	}

	CO2PPM = (int)data[2] * 256 + (int)data[3];
	temperature = (int)data[4] - 40;

	wmprintf("Tempr: %d, CORPPM: %d\r\n", temperature, CO2PPM);
	return true;
}

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int co2_sensor_init(struct sensor_info *curevent)
{
	int ret;

	if (co2_sinit_f)
		return 0;
	co2_sinit_f = 1;

	/* using IO_48 and IO_49 for Sensor Communication
		Please NOTE:
		This need relevent change in board file
	 */
	uart_drv_init(UART2_ID, UART_8BIT);

	ret = os_mutex_create(&co2uart_mutex, "uart-co2",
			OS_MUTEX_INHERIT);

	if (ret != WM_SUCCESS)
		return -WM_FAIL;

	uart2_dev = uart_drv_open(UART2_ID, CO2_UART_BAUD);

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
int mhz16co2_sensor_input_scan(struct sensor_info *curevent)
{
	CO2_dataRecieve();

	/* Report newly generated value to the Sensor layer */
	sprintf(curevent->event_curr_value, "%d", CO2PPM);

	dbg("%s = %d\r\n", curevent->property, CO2PPM);
	return 0;
}

int mhz16temperature_sensor_input_scan(struct sensor_info *curevent)
{
	/* Report newly generated value to the Sensor layer */
	sprintf(curevent->event_curr_value, "%d", temperature);
	dbg("%s = %d\r\n", curevent->property, temperature);
	return 0;
}

struct sensor_info event_mhz16co2_sensor = {
	.property = "MHZ16-CO2",
	.init = co2_sensor_init,
	.read = mhz16co2_sensor_input_scan,
};

struct sensor_info event_mhz16temperature_sensor = {
	.property = "MHZ16-T",
	.init = co2_sensor_init,
	.read = mhz16temperature_sensor_input_scan,
};

int co2_sensor_event_register(void)
{
	sensor_event_register(&event_mhz16co2_sensor);
	return sensor_event_register(&event_mhz16temperature_sensor);
}

