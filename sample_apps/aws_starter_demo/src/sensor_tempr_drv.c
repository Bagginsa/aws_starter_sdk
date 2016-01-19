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
#include <mdev_adc.h>
#include <mdev_pinmux.h>
#include <lowlevel_drivers.h>

#include "sensor_drv.h"
#include "sensor_tempr_drv.h"

/*------------------Macro Definitions ------------------*/
#define SAMPLES	500
#define ADC_GAIN	ADC_GAIN_2
#define BIT_RESOLUTION_FACTOR 32768	/* For 16 bit resolution (2^15-1) */
#define VMAX_IN_mV	3000	/* Max input voltage in milliVolts */
/* Default is IO mode, DMA mode can be enabled as per the requirement */
/*#define ADC_DMA*/

/* Thread handle */
static os_thread_t temperature_thread;
/* Buffer to be used as stack */
static os_thread_stack_define(temperature_stack, 4 * 1024);

/*-----------------------Global declarations----------------------*/
uint16_t buffer[SAMPLES];
mdev_t *adc_dev = NULL;
int i, samples = SAMPLES;
float result;
ADC_CFG_Type config;
int dataready_flag = 0;
int dataready;

/*
 *********************************************************
 **** Temperature Sensor H/W Specific code
 **********************************************************
 */

/* Function to read ADC */
int getData(void)
{
	wmprintf("%s\r\n", __FUNCTION__);
#if 0
	if (adc_dev == NULL)
		return -1;

	adc_dev = adc_drv_open(ADC0_ID, ADC_CH0);
	
#ifdef ADC_DMA
	adc_drv_get_samples(adc_dev, buffer, samples);
		result = ((float)buffer[i] / BIT_RESOLUTION_FACTOR) *
		VMAX_IN_mV * ((float)1/(float)(config.adcGainSel != 0 ?
		config.adcGainSel : 0.5));
/*		wmprintf("Iteration %d: count %d - %d.%d mV\r\n",
					i, buffer[i],
					wm_int_part_of(result),
					wm_frac_part_of(result, 2));
*/
#else
		buffer[0] = adc_drv_result(adc_dev);
		result = ((float)buffer[0] / BIT_RESOLUTION_FACTOR) *
		VMAX_IN_mV * ((float)1/(float)(config.adcGainSel != 0 ?
		config.adcGainSel : 0.5));
/*		wmprintf("Iteration %d: count %d - %d.%d mV\r\n",
					i, buffer[0],
					wm_int_part_of(result),
					wm_frac_part_of(result, 2));
*/
#endif
#endif
	return result;
}

/* This thread reads sensor data periodically and
	reports the change to the AWS cloud */

static void temperature_sense_task(os_thread_arg_t data)
{
	#define	ITERATIONS	10
	int olddata1, olddata2, olddata3, avgdata, reporteddata;
	int i, tdata[ITERATIONS];
	int count;

	wmprintf("%s\r\n", __FUNCTION__);
	/* clear a buffer */
	for (i = 0; i < ITERATIONS; i++)
		tdata[i] = 0;

	count = 0;
	olddata1 = 0;
	olddata2 = 0;
	olddata3 = 0;
	reporteddata= 0;
	while(1) {
		/* Read ADC value ITERATIONS times*/ 
		tdata[count++]= getData();

		if (count < ITERATIONS) {
			count = 0;

			/* calcluate avg value */
			avgdata = 0;
			for (i = 0; i < ITERATIONS; i++)
				avgdata += tdata[i];

			avgdata /= ITERATIONS;

			/* push it in FIFO */
			olddata3 = olddata2;
			olddata2 = olddata1;
			olddata1 = avgdata;

			/* Report data to the cloud, if stable and not the same
				as reported earlier */
			if ((olddata1 == olddata2) &&
				(olddata2 == olddata3) &&
				(olddata1 != reporteddata)) {

				dataready = olddata1;
				dataready_flag = 1;
				reporteddata = olddata1;
			}
		}

		/* Sensor will be polled after each 10 miliseconds */
		os_thread_sleep(100);
	}
}

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int temperature_sensor_init(struct sensor_info *curevent)
{
	int ret;

	wmprintf("%s\r\n", __FUNCTION__);

	/* create a temperature thread in which you can read sensor data
		out of context of AWS framework */
	ret = os_thread_create(
		/* thread handle */
		&temperature_thread,
		/* thread name */
		"Temperature_Tr",
		/* entry function */
		temperature_sense_task,
		/* argument */
		0,
		/* stack */
		&temperature_stack,
		/* priority */
		OS_PRIO_4);
		
	if (ret != WM_SUCCESS) {
		wmprintf("Failed to start cloud_thread: %d\r\n", ret);
		return ret;
	}

	if (adc_drv_init(ADC0_ID) != WM_SUCCESS) {
		wmprintf("Error: Cannot init ADC\n\r");
		return -1;
	}

#if defined(CONFIG_CPU_MW300)
	int i;

	adc_dev = adc_drv_open(ADC0_ID, ADC_CH0);

	i = adc_drv_selfcalib(adc_dev, vref_internal);
	if (i == WM_SUCCESS)
		wmprintf("Calibration successful!\r\n");
	else
		wmprintf("Calibration failed!\r\n");

	adc_drv_close(adc_dev);
#endif

	/* get default ADC gain value */
	adc_get_config(&config);
	wmprintf("Default ADC gain value = %d\r\n", config.adcGainSel);

	/* Modify ADC gain to 2 */
	adc_modify_default_config(adcGainSel, ADC_GAIN);

	adc_get_config(&config);
	wmprintf("Modified ADC gain value to %d\r\n", config.adcGainSel);

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
int temperature_sensor_input_scan(struct sensor_info *curevent)
{
	if (dataready_flag) {
		 dataready_flag = 1;
		/* Report changed temperature value to the AWS cloud */
		curevent->event_curr_value = dataready;
		wmprintf("Reporting Temperature value %d\r\n", dataready);
	}
	return 0;
}

struct sensor_info event_temperature_sensor = {
	.property = "temperature",
	.init = temperature_sensor_init,
	.read = temperature_sensor_input_scan,
};

int temperature_sensor_event_register(void)
{
	return sensor_event_register(&event_temperature_sensor);
}

