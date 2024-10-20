
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "button.h"
#include "bootloader.h"

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(button);


#define BUTTON_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON_NODE, gpios, {0});

static struct gpio_callback button_cb_data;


void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	reset_to_bootloader();
}


bool button_init (void){

	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Button device is not ready");
		return false;
	}

	int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure button pin");
		return false;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure button interrupt");
		return false;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

    return true;
}



