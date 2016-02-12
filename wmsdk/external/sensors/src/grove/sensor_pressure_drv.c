/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * I2C based Grove - Barometer Sensor (BMP180) Low Level Driver
 *
 * Summary:
 *
 * This driver offers h/w specific abstraction to register, initialize,
 * scan and report specific sensor event to the Sensor Interface Layer
 *
 * Sensor used for this driver is
 * http://www.seeedstudio.com/wiki/Grove_-_Barometer_Sensor_(BMP180)
 *
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
#include "sensor_pressure_drv.h"

#include <wm_os.h>
#include <mdev_i2c.h>

/*------------------Macro Definitions ------------------*/
#define OSS	0
#define BUF_LEN 16
#define I2C_SLV_ADDR BMP085_ADDRESS 
static mdev_t *i2c0;
static uint8_t read_data[BUF_LEN];
static uint8_t write_data[BUF_LEN];
static struct Barometer bm180;

/*
 *********************************************************
 **** Pressure Sensor H/W Specific code
 **********************************************************
 */

unsigned long i2c_bmp180ReadLong(void)
{
	write_data[0] = 0xF6;
	i2c_drv_write(i2c0, write_data, 1);
	i2c_drv_read(i2c0, read_data, 3);

	return (unsigned long)(((unsigned long) read_data[0] << 16) |
				((unsigned long) read_data[1] << 8) |
				((unsigned long) read_data[2] >> (8-OSS)));
}

short i2c_bmp180ReadInt(uint8_t address)
{
	write_data[0] = address;
	i2c_drv_write(i2c0, write_data, 1);
	i2c_drv_read(i2c0, read_data, 2);

	return (short)((read_data[0] << 8) | read_data[1]);
}

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int pressure_sensor_init(struct sensor_info *curevent)
{
	int len;

	/* Initialize I2C Driver */
	/* using IO_04:SDA and IO_05 SCL (configured in board file */
	i2c_drv_init(I2C0_PORT);

	/* I2C0 is configured as master */
	i2c0 = i2c_drv_open(I2C0_PORT, I2C_SLAVEADR(I2C_SLV_ADDR));

	/* Read Calibration Data from BM180*/
	bm180.ac1 = i2c_bmp180ReadInt(0xAA);
	bm180.ac2 = i2c_bmp180ReadInt(0xAC);
	bm180.ac3 = i2c_bmp180ReadInt(0xAE);
	bm180.ac4 = i2c_bmp180ReadInt(0xB0);
	bm180.ac5 = i2c_bmp180ReadInt(0xB2);
	bm180.ac6 = i2c_bmp180ReadInt(0xB4);
	bm180.b1 = i2c_bmp180ReadInt(0xB6);
	bm180.b2 = i2c_bmp180ReadInt(0xB8);
	bm180.mb = i2c_bmp180ReadInt(0xBA);
	bm180.mc = i2c_bmp180ReadInt(0xBC);
	bm180.md = i2c_bmp180ReadInt(0xBE);

	/* Write command to read Id register */
	write_data[0] = 0xd0; /* ID register address */
	len = i2c_drv_write(i2c0, write_data, 1);
	len = i2c_drv_read(i2c0, read_data, 1);

	wmprintf("%s ID=0x0%2x (%d)\r\n", __FUNCTION__, read_data[0], len);
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
int bm180pressure_sensor_input_scan(struct sensor_info *curevent)
{
	/* Write I2C Command to Measure a Temperature */
	write_data[0] = 0xF4; /* Ctrl register address */
	write_data[1] = 0x34 + (OSS<<6); /* Measure Pressure */
	i2c_drv_write(i2c0, write_data, 2);
	os_thread_sleep(10);
	/* Write I2C Command to read UT Temperature Value */
	bm180.up = i2c_bmp180ReadLong();

	/* Calculate Pressure Value */
	long x1, x2, x3, b3, b6, p;
	unsigned long b4, b7;
	b6 = bm180.PressureCompensate - 4000;
	x1 = (bm180.b2 * (b6 * b6)>>12)>>11;
	x2 = (bm180.ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((long)bm180.ac1)*4 + x3)<<OSS) + 2)>>2;

	/* Calculate B4 */
	x1 = (bm180.ac3 * b6)>>13;
	x2 = (bm180.b1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (bm180.ac4 * (unsigned long)(x3 + 32768))>>15;

	b7 = ((unsigned long)(bm180.up - b3) * (50000>>OSS));
	if (b7 < 0x80000000)
		p = (b7<<1)/b4;
	else
		p = (b7/b4)<<1;

	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	p += (x1 + x2 + 3791)>>4;

	long temp = p;
	temp /= 2;

	dbg("%s Pressure=%d.%d\r\n", __FUNCTION__,
			wm_int_part_of(temp),
			wm_frac_part_of(temp, 2));

	/* Report newly generated value to the Sensor layer */
	sprintf(curevent->event_curr_value, "%d",
			wm_int_part_of(temp));
	return 0;
}

int bm180temperature_sensor_input_scan(struct sensor_info *curevent)
{
	/* Write I2C Command to Measure a Temperature */
	write_data[0] = 0xF4; /* Ctrl register address */
	write_data[1] = 0x2e; /* Measure Temperature */
	i2c_drv_write(i2c0, write_data, 2);
	os_thread_sleep(10);

	/* Write I2C Command to read UT Temperature Value */
	bm180.ut = i2c_bmp180ReadInt(0xF6);

	/* Calculate Temperature */
	long x1 = (((long)bm180.ut - (long)bm180.ac6)*(long)bm180.ac5) >> 15;
	long x2 = ((long)bm180.mc << 11)/(x1 + bm180.md);
	bm180.PressureCompensate = x1 + x2;

	float temp = ((bm180.PressureCompensate + 8)>>4);
	temp = temp /10;

	dbg("%s Temperature=%d.%d\r\n", __FUNCTION__,
			wm_int_part_of(temp),
			wm_frac_part_of(temp, 2));

	/* Report newly generated value to the Sensor layer */
	sprintf(curevent->event_curr_value, "%d.%d",
			wm_int_part_of(temp),
			wm_frac_part_of(temp, 2));
	return 0;
}

struct sensor_info event_bm180pressure_sensor = {
	.property = "BM180-Pressure",
	.init = pressure_sensor_init,
	.read = bm180pressure_sensor_input_scan,
};

struct sensor_info event_bm180temperature_sensor = {
	.property = "BM180-Tempereature",
	.init = pressure_sensor_init,
	.read = bm180temperature_sensor_input_scan,
};

int pressure_sensor_event_register(void)
{
	sensor_event_register(&event_bm180pressure_sensor);
	return sensor_event_register(&event_bm180temperature_sensor);
}

