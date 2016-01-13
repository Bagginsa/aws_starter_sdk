/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * Sensor Event Driver for AWS Application
 *
 * Summary:
 *
 * This driver offers common infrastructure to register and report
 * Mulrtiple sensor event to the AWS cloud
 */

#include <wm_os.h>
#include <wmstdio.h>
#include <wmtime.h>
#include <wmsdk.h>
#include <board.h>

#include "sensor_drv.h"

/* Link list global pointer */
struct sensor_info *sensor_ll = NULL;
/* Flag to track driver initialization */
int sensor_drvinit_flag = 0;
/* Thread handle */
static os_thread_t sensor_th;
/* Buffer to be used as stack */
static os_thread_stack_define(sensor_thread_stack, 2 * 1024);


/* Construct JSON keyvalue pair as per status of sensors */
int sensor_msg_construct(char *src, char *dest, int len)
{
	struct sensor_info *curevent = sensor_ll;
	int ret = WM_SUCCESS;
	char buf[500];

	while(curevent) {

		/*wmprintf("%s preval=%d, newval=%d\r\n",
			curevent->property,
			curevent->event_prev_value,
			curevent->event_curr_value);
		*/
		if (curevent->event_prev_value != curevent->event_curr_value) {
			curevent->event_prev_value = curevent->event_curr_value;	
			snprintf(dest, len, ",\"%s\":%d",
					curevent->property,
					curevent->event_curr_value);

			/*wmprintf("%p Constructed Sensor msg: %s\r\n", curevent, dest);*/
			strcat(src, dest);
		}
		/* point to next sensor event data */
		curevent = curevent->next;
	}

	return ret;
}

void sensor_inputs_scan(void)
{
	struct sensor_info *curevent;

	curevent = sensor_ll;
	/* Scan All sensor events here in this loop periodically */
	while(curevent) {
		/*wmprintf("Reading svent %p\r\n", curevent);*/
		curevent->read(curevent);

		curevent = curevent->next;
		if (curevent == NULL)
			return;
	};
}

int sensor_event_register(struct sensor_info *sevnt)
{
	struct sensor_info *curevent = sensor_ll;

	if (!sensor_drvinit_flag) {
		wmprintf("Register called before Sensor driver init\r\n");
		return -1;
	}

	if (!sevnt) {
		wmprintf("NULL sensor info strct passed\r\n");
		return -1;
	}

	/* Add new event registerd at the end of link list */
	while (1) {
		if (!curevent) {
			/* Very first registration */
			curevent = sevnt;
			sensor_ll = curevent;
			break;
		} else if (curevent) {
			if (curevent == sevnt) {
				wmprintf("Sensor event %s(%p) is aready registered\r\n",
					sevnt->property, sevnt);
				return -1;
			}

			if (!curevent->next) {
				/* Second onward registration */
				curevent->next = sevnt;
				break;
			}
		} else {
			/* Continue next registration */
			curevent = curevent->next;
			continue;
		}
	}

	sevnt->event_prev_value = -1;
	/* make sure registered event is last in link list */
	sevnt->next = NULL;
	/* Sensor initialization call */ 
	sevnt->init(sevnt);
	wmprintf("Sensor event %s(%p) is registered sucessfully\r\n",
			sevnt->property, sevnt);

	return 0;
}

int sensor_drv_init(void)
{
	int ret;

	if (!sensor_drvinit_flag) {
		sensor_drvinit_flag = 1; /* flag to indiate init done */
		sensor_ll = NULL; /* flush registered actions */
	};
	wmprintf("%s done\r\n", __FUNCTION__);
	return WM_SUCCESS;
}
