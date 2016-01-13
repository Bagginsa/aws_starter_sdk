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


/*-----------------------Global declarations----------------------*/
uint16_t buffer[SAMPLES];
mdev_t *adc_dev;
int i, samples = SAMPLES;
float result;
ADC_CFG_Type config;

/*
 *********************************************************
 **** Temperature Sensor H/W Specific code
 **********************************************************
 */

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int temperature_sensor_init(struct sensor_info *curevent)
{
	wmprintf("%s\r\n", __FUNCTION__);

#if 0
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
//	adc_get_config(&config);
	wmprintf("Default ADC gain value = %d\r\n", config.adcGainSel);

	/* Modify ADC gain to 2 */
	adc_modify_default_config(adcGainSel, ADC_GAIN);

	adc_get_config(&config);
	wmprintf("Modified ADC gain value to %d\r\n", config.adcGainSel);

	return 0;
#endif
}

/* Function to read ADC */
int getData(void)
{
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
	return result;
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

	/* wmprintf("%s\r\n", __FUNCTION__); */

	/* Read ADC value as current sensor value */
	//curevent->event_curr_value = getData();

	/* for testing purpose only,
		disable this line getData is functional*/
	curevent->event_curr_value++;
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

