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
 *
 * Temperature Sensor Used here is :
 * http://www.seeedstudio.com/wiki/Grove_-_Temperature_Sensor_V1.2
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
#include <math.h>

const int B=4275;	/* B value of the thermistor */
const int R0 = 100000;	/* R0 = 100k */
/* Grove - Temperature Sensor connect to GPIO42
	Please note, GPIO42 to be connected to A1 input of BASE Shild */
const int pinTempSensor = ADC_CH0;

/*------------------Macro Definitions ------------------*/
#define ADC_DMA
#define	ITERATIONS	16
#define SAMPLES	800
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
uint16_t buffer[SAMPLES+10];
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

/* Function to read Integer portion of temperature value */
int getData(void)
{
        int avgdata=0;

	if (adc_dev == NULL)
		return -1;

	adc_dev = adc_drv_open(ADC0_ID, pinTempSensor);

#ifdef ADC_DMA
	adc_drv_get_samples(adc_dev, buffer, samples);
        for (i=0;i<samples;++i) {
		avgdata+=buffer[i];
	}
        avgdata/=samples;
#else
        for (i=0;i<ITERATIONS;++i) {
		avgdata += adc_drv_result(adc_dev);
		os_thread_sleep(5);
        }
        avgdata/=ITERATIONS;
#endif

	adc_drv_close(adc_dev);

	/* Convert ADC Value to the milivolts, Ref: io_demo/adc sample app */
	result = ((float)avgdata / BIT_RESOLUTION_FACTOR)
		* VMAX_IN_mV
		* ((float)1/(float)(config.adcGainSel != 0 ?
				config.adcGainSel : 0.5));

	/* Convert Milivolts to the temperature, Ref: Code on Sensor URL */
	float R = 1023.0/((float)avgdata)-1.0;
	R = 100000.0*R;
	/* convert to temperature via datasheet */
	float temperature=1.0/(/*log*/(R/100000.0)/B+1/298.15)-273.15;

	wmprintf("ADC val=%d, milivolts %d.%d, temperature %d.%d\r\n",
			avgdata,
			wm_int_part_of(result),
			wm_frac_part_of(result, 2),
			wm_int_part_of(temperature),
			wm_frac_part_of(temperature, 2));

	return wm_int_part_of(temperature);
}

/* This thread reads sensor data periodically and
	reports the change to the AWS cloud */

static void temperature_sense_task(os_thread_arg_t data)
{
	int old_adc_data, new_adc_data;
	int reporteddata= 0;

	while(1) {
		/* Read ADC value ITERATIONS times*/ 
		old_adc_data = new_adc_data;
		new_adc_data = getData();

		/* Report to the cloud if,
			two succesive ADC readings are unequal */
		if (old_adc_data != new_adc_data) {
			/* Report temperature data to the cloud */
			dataready = new_adc_data;
		        dataready_flag = 1;
			reporteddata = new_adc_data;

			wmprintf("Reported Tempr Data: %d\r\n",
						reporteddata);
		}
		os_thread_sleep(250);
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
#else
#error "Unsupported MCU..."
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
	if (dataready_flag==1) {
		dataready_flag = 0; /* Clear flag to indicate processed */
		/* Report changed temperature value to the AWS cloud */
		curevent->event_curr_value = dataready;
		/*wmprintf("Reporting Temperature value %d\r\n", dataready);*/
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

