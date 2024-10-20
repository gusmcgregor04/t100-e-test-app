/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "usb.h"
#include "bootloader.h"
#include "led.h"
#include "gnss.h"
#include "button.h"
#include "lis3dh.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

int main(void)
{
	bool led_state = true;

	if(led_init() == false)
	{
		LOG_ERR("Failed to initialise LED");
		return 1;
	}

	if(usb_init() == false)
	{
		LOG_ERR("Failed to initialise USB");
		return 1;
	}

	if(gnss_init() == false)
	{
		LOG_ERR("Failed to initialise GNSS");
		return 1;
	}

	if(button_init() == false)
	{
		LOG_ERR("Failed to initialise BUTTON");
		return 1;
	}

	// if(acc_init() == false)
	// {
	// 	LOG_ERR("Failed to initialise ACC");
	// 	return 1;
	// }

	printk ("Running t1000-1 test app\n");
	while (1) {
		
		led_state = !led_state;
		set_led(led_state);
		//acc_fetch_display();
		//i2c_scan();
		print_latest_gnss();
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

