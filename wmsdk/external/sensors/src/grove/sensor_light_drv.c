/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * ADC based Ambient Light Sensor Low Level Driver
 *
 * Summary:
 *
 * This driver offers h/w specific abstraction to register, initialize and
 * scan and report specific sensor event to the Sensor Interface Layer
 *
 * Light Sensor Used here is :
 * http://www.seeedstudio.com/depot/Grove-Light-Sensor-p-746.html
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
#include "sensor_light_drv.h"
#include <math.h>

static const int B=4275;	/* B value of the thermistor */
static const int R0 = 100000;	/* R0 = 100k */
/* Grove - Light Sensor connect to GPIO43
	Please note, GPIO42 to be connected to A2 input of BASE Shild */
static const int pinLightSensor = ADC_CH1;

/*------------------Macro Definitions ------------------*/
#define ADC_DMA
#define	ITERATIONS	16
#define SAMPLES	400
#define ADC_GAIN	ADC_GAIN_2
#define BIT_RESOLUTION_FACTOR 32768	/* For 16 bit resolution (2^15-1) */
#define VMAX_IN_mV	3000	/* Max input voltage in milliVolts */
/* Default is IO mode, DMA mode can be enabled as per the requirement */
/*#define ADC_DMA*/

/*-----------------------Global declarations----------------------*/
static uint16_t buffer[SAMPLES+10];
static mdev_t *adc_dev = NULL;
static int i, samples = SAMPLES;
static float result;
static ADC_CFG_Type config;

/*
 *********************************************************
 **** Light Sensor H/W Specific code
 **********************************************************
 */

/* Function to read Integer portion of light value */
int getLightData(void)
{
        int avgdata=0;

	if (adc_dev == NULL)
		return -1;

	adc_dev = adc_drv_open(ADC0_ID, pinLightSensor);

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

	/* Convert Milivolts to the light, Ref: Code on Sensor URL */
	float Rsensor=(float)(1023-result)*10/result;

	wmprintf("ADC val=%d, milivolts %d.%d, light %d.%d\r\n",
			avgdata,
			wm_int_part_of(result),
			wm_frac_part_of(result, 2),
			wm_int_part_of(Rsensor),
			wm_frac_part_of(Rsensor, 2));

	return wm_int_part_of(Rsensor);
}

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int light_sensor_init(struct sensor_info *curevent)
{
	wmprintf("%s\r\n", __FUNCTION__);

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
	dbg("Default ADC gain value = %d\r\n", config.adcGainSel);

	/* Modify ADC gain to 2 */
	adc_modify_default_config(adcGainSel, ADC_GAIN);

	adc_get_config(&config);
	dbg("Modified ADC gain value to %d\r\n", config.adcGainSel);

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
int light_sensor_input_scan(struct sensor_info *curevent)
{
	/* Report changed light value to the Sensor Interface Layer */
	sprintf(curevent->event_curr_value, "%d", getLightData());
	dbg("Reporting Light value %d\r\n", getLightData());
	return 0;
}

struct sensor_info event_light_sensor = {
	.property = "light",
	.init = light_sensor_init,
	.read = light_sensor_input_scan,
};

int light_sensor_event_register(void)
{
	return sensor_event_register(&event_light_sensor);
}

