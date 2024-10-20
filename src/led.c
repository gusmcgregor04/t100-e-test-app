
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "led.h"

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(led);


#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

bool led_init (void){

    	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("Failed to set LED pin");
		return false;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to set LED pin as a output");
		return false;
	}
    return true;
}

void set_led(bool state) {
    int ret = gpio_pin_set_dt(&led, state);
	if (ret < 0) {
        LOG_ERR("Failed to change LED state");
	}
}