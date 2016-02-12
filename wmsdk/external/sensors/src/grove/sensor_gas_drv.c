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
#include "sensor_gas_drv.h"
#include <math.h>

/* Grove - Gas Sensor MQ9 connect to GPIO47
	Please note, GPIO47 to be connected to A3 input of BASE Shild */
#define GASSEN_ADCCH	ADC_CH5

/*------------------Macro Definitions ------------------*/
#define	ITERATIONS	16
#define SAMPLES	400
#define ADC_GAIN	ADC_GAIN_2
#define BIT_RESOLUTION_FACTOR 32768	/* For 16 bit resolution (2^15-1) */
#define VMAX_IN_mV	3000	/* Max input voltage in milliVolts */
/* Default was IO mode, DMA mode is enabled for better accuracy averaging more samples */
#define ADC_DMA

/*-----------------------Global declarations----------------------*/
static uint16_t buffer[SAMPLES+10];
static mdev_t *adc_dev = NULL;
static int i, samples = SAMPLES;
static float result;
static ADC_CFG_Type config;

/*
 *********************************************************
 **** Gas Sensor H/W Specific code
 **********************************************************
 */

/* Function to read Integer portion of gas value */
float getGasSensorData(void)
{
        int avgdata=0;

	if (adc_dev == NULL)
		return -1;

	adc_dev = adc_drv_open(ADC0_ID, GASSEN_ADCCH);

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

	dbg("ADC val=%d, milivolts %d.%d\r\n",
			avgdata,
			wm_int_part_of(result),
			wm_frac_part_of(result, 2));

	return result;
}

/* Basic Sensor IO initialization to be done here

	This function will be called only once during sensor registration
 */
int gas_sensor_init(struct sensor_info *curevent)
{
	wmprintf("%s\r\n", __FUNCTION__);
	if (adc_drv_init(ADC0_ID) != WM_SUCCESS) {
		wmprintf("Error: Cannot init ADC\n\r");
		return -1;
	}

#if defined(CONFIG_CPU_MW300)
	int i;

	adc_dev = adc_drv_open(ADC0_ID, GASSEN_ADCCH);

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

	Respective AWS event will be reported to the cloud by
	uper sensor_driver layer

	This function will be called periodically by the upper layer
	hence you can poll your input here, and there is no need of
	callback IO interrupt, this is very usefull to sense variable
	data from analog sensors connected to ADC lines. Event this can
	be used to digital IO scanning and polling
*/
int gas_sensor_input_scan(struct sensor_info *curevent)
{
	float sdata = getGasSensorData();

	/* Report changed gas value to the AWS cloud */
	sprintf(curevent->event_curr_value, "%d.%d", 
			wm_int_part_of(sdata),
			wm_frac_part_of(sdata, 2));
	dbg("Reporting Temperature value %d\r\n",
			wm_int_part_of(sdata),
			wm_frac_part_of(sdata, 2));
	return 0;
}

struct sensor_info event_gas_sensor = {
	.property = "Gas-MQ9",
	.init = gas_sensor_init,
	.read = gas_sensor_input_scan,
};

int gas_sensor_event_register(void)
{
	return sensor_event_register(&event_gas_sensor);
}

