/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * Custom Sensor Driver header file
 */

#ifndef __SENSOR_GAS_DRV_H__
#define __SENSOR_GAS_DRV_H__

#define BMP085_ADDRESS 0x77

struct gas_sen_data {
	/* public: */
	void (*init)(void);
	/* private: */
};

int gas_sensor_event_register(void);

#endif /* __SENSOR_GAS_DRV_H__ */
