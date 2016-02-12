/*
 * @file     dht_drv.c
 * @brief    This file provides dht module driver source code file.
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

#include <wm_os.h>
#include <wmstdio.h>
#include <mdev.h>
#include <mdev_gpio.h>
#include <mdev_gpt.h>
#include <mdev_pinmux.h>
#include <generic_io.h>
#include <dht_drv.h>

#define DHT11_HDR_MARK		70
#define DHT11_HDR_SPACE		70
#define DHT11_BIT_MARK		50
#define DHT11_ONE_SPACE		70
#define DHT11_ZERO_SPACE	26
#define DHT11_BITS		40
#define REC_BUF_SIZE		70
#define DHT_DATA_BUF_SIZE	8
#define USECPERTICK 1
#define MARK_EXCESS 100
#define _GAP 5000 // Minimum map between transmissions
#define GAP_TICKS (_GAP/USECPERTICK)
#define TOLERANCE 20  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.)
#define UTOL (1.0 + TOLERANCE/100.)
#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))

static struct io_gpio_cfg *dhtio = NULL; /* holds used GPIO info */
static os_timer_t dht_os_timer = NULL;
struct dht_psmvar dht_data;
static mdev_t MDEV_dht;
static char *MDEV_NAME_dht = "dht";
static struct dht_packet dht;
static uint32_t dht_cap_tmr_data[REC_BUF_SIZE];
static uint8_t bitcnt, dht_data_buf[DHT_DATA_BUF_SIZE];
static uint8_t dht_capture_ptr = 0;
static int temp_tmr = 50;
static int dht22_start_delay = 200 ;
static mdev_t *gpt_dev_dht;

inline static int MATCH(int measured, int desired){return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);}
inline static int MATCH_MARK(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us + MARK_EXCESS));}
inline static int MATCH_SPACE(int measured_ticks, int desired_us) {return MATCH(measured_ticks, (desired_us - MARK_EXCESS));}

static void RotateBitRou(unsigned char j)
{
	uint8_t  irTemp,i=0 ;
        if(bitcnt > DHT11_BITS) return;
	i = bitcnt;
        irTemp  = i;
        irTemp  =  irTemp >> 3;
        irTemp  &=  0x07;
        dht_data_buf[irTemp] <<= 1;
        dht_data_buf[irTemp] |=j;
	bitcnt++;
}

static void  dht_read_BitRou(uint32_t tmcnt)
{
	/* Currently header check is skipped 
	if (MATCH(tmcnt, DHT11_HDR_MARK + DHT11_HDR_SPACE )) {
		bitcnt = 112;
	}
	else 
	*/
	if (MATCH(tmcnt,  DHT11_BIT_MARK +  DHT11_ONE_SPACE )) {
		 RotateBitRou( 0x01 ) ;
	}
	else if (MATCH(tmcnt, DHT11_BIT_MARK + DHT11_ZERO_SPACE)) {
		RotateBitRou( 0x00 );
	}
}

/** GPIO falling  edge triggered interrupt for DHT capture.
 * io->pinno line is used
 * If HIGH  on DHT Line  then return.
 */
static void  dht_learn_cb(int pin, void *data)
{
        uint32_t timer_val;
        timer_val=GPT_GetCounterVal(dhtio->id);
        gpt_drv_stop(gpt_dev_dht);
        gpt_drv_set(gpt_dev_dht, 0x01ffffff);
        gpt_drv_start(gpt_dev_dht);
        if (timer_val > 50 ) {
                if (!GPIO_ReadPinLevel(dhtio->pinno)) {
			dht_cap_tmr_data[dht_capture_ptr] = timer_val/50;
			if(dht_capture_ptr < REC_BUF_SIZE){
				dht_capture_ptr++;
			}
		}
	}
}

static void init_dht(struct io_gpio_cfg *io)
{
	mdev_t *gpio_dev,*pinmux_dev;

	gpt_drv_init(io->id);
	/* Opened one time */
	gpt_dev_dht = gpt_drv_open(io->id);
	pinmux_drv_init();
	pinmux_dev = pinmux_drv_open("MDEV_PINMUX");
	if (pinmux_dev == NULL) {
		wmprintf("Pinmux driver init is required before open\r\n");
		return;
	}
	gpio_drv_init();
	gpio_dev = gpio_drv_open("MDEV_GPIO");
	if (gpio_dev == NULL) {
		wmprintf("GPIO driver init is required before open\r\n");
		return;
	}
	pinmux_drv_setfunc(pinmux_dev, io->pinno, io->altfun);
	gpio_drv_close(gpio_dev);
        pinmux_drv_close(pinmux_dev);
}

/*This function is called from timer callbak.....__scan every fourty Second
 *
 */
void dht_timer(void)
{
	mdev_t  *gpio_dev;
	int i;
	/* Power on delay of 4 second is required to stablise DHT */
	if(dht22_start_delay) dht22_start_delay--;
	if(!dht22_start_delay) {
		if (++temp_tmr > 50) temp_tmr = 0;
		if (temp_tmr == 0) {
			gpio_dev = gpio_drv_open("MDEV_GPIO");
			gpio_drv_setdir(gpio_dev, dhtio->pinno, GPIO_OUTPUT);
			gpio_drv_write(gpio_dev, dhtio->pinno,0);
		}
		if (temp_tmr == 1) {
			bitcnt = 41;
			dht_capture_ptr = 0;
			/*gpt_dev_dht = gpt_drv_open(io->id); */
			gpt_drv_stop(gpt_dev_dht);
			gpt_drv_set(gpt_dev_dht, 0x01ffffff);
			/*gpt_drv_close(gpt_dev_dht);*/

			gpio_dev = gpio_drv_open("MDEV_GPIO");
			gpio_drv_setdir(gpio_dev, dhtio->pinno, GPIO_INPUT);
			gpio_drv_set_cb(gpio_dev, dhtio->pinno, GPIO_INT_FALLING_EDGE,
						NULL, dht_learn_cb);
			gpio_drv_close(gpio_dev);
			for ( i= 0; i< DHT_DATA_BUF_SIZE ;i++) dht_data_buf[i] = 0;
		}
		if (temp_tmr == 2) {
		/* Open GPIO driver */
		gpio_dev = gpio_drv_open("MDEV_GPIO");
			gpio_drv_set_cb(gpio_dev, dhtio->pinno, GPIO_INT_DISABLE, NULL, NULL);
			gpio_drv_close(gpio_dev);
			bitcnt = 0;
			for (i= 1; i < dht_capture_ptr; i++){
				dht_read_BitRou(dht_cap_tmr_data[i]);
			}
			/*checksum*/
			i = dht_data_buf[0] + dht_data_buf[1] + dht_data_buf[2] + dht_data_buf[3];
			if( i != dht_data_buf[4] ){
				wmprintf("Check sum NOk, InValid data received \r\n");
			} else {
				if (dht.dhtlearncb) {
					dht.dhtlearncb(&dht_data);
				}
				dht_data.temperature = dht_data_buf[2];
				dht_data.humidity = dht_data_buf[0];
			}
		}
	}
}

void dht_os_timer_cb (os_timer_arg_t targ)
{
	dht_timer();
}

/*@}end of DHT_Private_Functions */
/** @{defgroup _Public_Functions
 *
 */
int dht_learn(int(*learncb)(struct dht_psmvar *dht_vars))
{
	if (NULL == learncb ) {
		 wmprintf("call back function is not assigned \r\n");
		return 0;
	}
	dht.dhtlearncb = learncb;
	return 0;
}

int dht_drv_init(struct io_gpio_cfg *io)
{
	wmprintf("%s\r\n", __FUNCTION__);
	MDEV_dht.name = MDEV_NAME_dht;

	mdev_register(&MDEV_dht);
	MDEV_dht.private_data = (uint32_t) &dht;
	dhtio = io;
	init_dht(io);
	//dht_cap_data = dht_cap_tmr_data; //Temparary Data
	dht.dhtlearncb = NULL;

	if (dht_os_timer)
		return WM_SUCCESS;

	if (os_timer_create(&dht_os_timer,
			    "dht_os_timer",
			    os_msec_to_ticks(40),
			    &dht_os_timer_cb,
			    NULL,
			    OS_TIMER_PERIODIC,
			    OS_TIMER_AUTO_ACTIVATE) != WM_SUCCESS) {
		wmprintf("Failed to create dht_os_timer\r\n");
		return -WM_FAIL;
	}
	return WM_SUCCESS;
}

/*@} end of DHT_Public_Functions */

/*@} end of group DHT */

/*@} end of group DIGGER_Midware_Driver */
