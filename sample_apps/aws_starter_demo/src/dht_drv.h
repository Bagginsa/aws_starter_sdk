/*
 * @file     dht_drv.h
 * @brief    This file provides dht module driver header file.
 * @version  
 * @date     13 Aug 2014
 * @author   
 *
 * @note
 * Copyright (C) 2014 VADACTRO (www.vadactro.org.in). All rights reserved.
 *
 * @par
 * VADACTRO has developed this driver for their internal product development
 * VADACTRO has no responsibility or liability for the use of the software.
 * VADACTRO does not guarantee the correctness of this software. VADACTRO
 * reserves the right to make changes in the software without notification.
 *
 ********************************************************************************
 * CHANGE HISTORY
 *
 *  dd/mmm/yy     Code Ver   Author             Description
 *  ---------     --------   ------             -----------
 *  13/Aug/14     V0.0.01    			File created
 *
 ********************************************************************************
 */

#ifndef __DHT_DRV_H__
#define __DHT_DRV_H__

#include <stdint.h>
#include <generic_io.h>

enum dhtpacket_flags {
	DHTPACKET_READ_START = 0,
	DHTPACKET_CAPTURING,
	DHTPACKET_CAPTURED,
	DHTPACKET_ERROR,
	DHTPACKET_READY,
};

struct dht_psmvar {
        uint8_t humidity;       /**< Device Operating state */
        uint8_t humidity_f;      /**< Device Operating state */
        uint8_t temperature;        /**< Mode of operation */
        uint8_t temperature_f;        /**< Mode of operation */
};

struct dht_packet {
	enum dhtpacket_flags  packetflag;	/**< Flag to check status of dht packet */
	int (*dhtlearncb)(struct dht_psmvar *dht_vars);
};

/* Public function for dht init except 'dht_hw_init' for temporary usage */
int dht_drv_init(struct io_gpio_cfg *io);
int  dht_learn(int (*learncb)(struct dht_psmvar* dht_vars));
int  dht_transmit(uint32_t *irtransdata, uint32_t irrecdata_cnt );
void dht_timer(void);

#endif				/* __DHT_DRV_H__ */
