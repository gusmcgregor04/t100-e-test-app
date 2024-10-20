#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include "gnss.h"


#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(gnss, CONFIG_GNSS_LOG_LEVEL);

#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))


#define GNSS_PINS DT_NODELABEL(gnss_control)
static const struct gpio_dt_spec gnss_enable = GPIO_DT_SPEC_GET(GNSS_PINS, enable_gpios);
static const struct gpio_dt_spec gnss_reset = GPIO_DT_SPEC_GET(GNSS_PINS, reset_gpios);
static const struct gpio_dt_spec gnss_vrtc = GPIO_DT_SPEC_GET(GNSS_PINS, vrtc_gpios);
static const struct gpio_dt_spec gnss_sleep = GPIO_DT_SPEC_GET(GNSS_PINS, sleep_gpios);
static const struct gpio_dt_spec gnss_rtc = GPIO_DT_SPEC_GET(GNSS_PINS, rtc_gpios);
static const struct gpio_dt_spec gnss_resetb = GPIO_DT_SPEC_GET(GNSS_PINS, resetb_gpios);



bool gnss_init() {

    if (!gpio_is_ready_dt(&gnss_enable) & 
	!gpio_is_ready_dt(&gnss_reset) & 
	!gpio_is_ready_dt(&gnss_vrtc) & 
	!gpio_is_ready_dt(&gnss_sleep) & 
	!gpio_is_ready_dt(&gnss_rtc)) {

		LOG_ERR("Failed gnss pin setup");
		return false;
	}

	int ret = gpio_pin_configure_dt(&gnss_enable, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&gnss_reset, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&gnss_vrtc, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&gnss_sleep, GPIO_OUTPUT_ACTIVE);
	ret = gpio_pin_configure_dt(&gnss_rtc, GPIO_OUTPUT_ACTIVE);

	if (ret < 0) {
		LOG_ERR("Failed to set gnss  pin as a output");
		return false;
	}
	
	ret = gpio_pin_set_dt(&gnss_vrtc, true);
	k_msleep(10);
	ret = gpio_pin_set_dt(&gnss_sleep, true);
	k_msleep(10);
	ret = gpio_pin_set_dt(&gnss_reset, false);
	k_msleep(10);
	ret = gpio_pin_set_dt(&gnss_rtc, false);
	k_msleep(10);
	ret = gpio_pin_set_dt(&gnss_enable, true);
	if (ret < 0) {
        LOG_ERR("Failed to change gnss pins");
	}

    return true;
}



static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	printk("%s: location (%lld, %lld) with %d tracked satellites\n", 
				dev->name, data->nav_data.latitude, data->nav_data.longitude, data->info.satellites_cnt);

	printk("%s: date: %02d:%02d:%02d:%03d %02d-%02d-%04d\n", 
				dev->name, data->utc.hour, data->utc.minute, data->utc.millisecond/1000, data->utc.millisecond%1000, 
				data->utc.month_day, data->utc.month, data->utc.century_year + 2000);

	// if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
	// 	printk("Got a fix!\n");
	// }
}

GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);


static void gnss_satellites_cb(const struct device *dev, const struct gnss_satellite *satellites,
			       uint16_t size)
{
	printk("%u satellites reported\n", size);
}

GNSS_SATELLITES_CALLBACK_DEFINE(GNSS_MODEM, gnss_satellites_cb);

