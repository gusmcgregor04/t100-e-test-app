
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
static uint64_t press_start_time;
static const uint64_t LONG_PRESS_DURATION_MS = 2000;
static bool is_button_pressed = false;

void short_press_action(void) {
    LOG_INF("Short press detected.");
    // Add your short press handling code here
}

void long_press_action(void) {
    LOG_INF("Long press detected.");
    reset_to_bootloader();
}

void button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint64_t current_time = k_uptime_get();

    if (!is_button_pressed) {
        // Button was just pressed
        press_start_time = current_time;
        is_button_pressed = true;
    } else {
        // Button was just released
        uint64_t press_duration = current_time - press_start_time;
        
        if (press_duration >= LONG_PRESS_DURATION_MS) {
            long_press_action();
        } else {
            short_press_action();
        }

        is_button_pressed = false;
    }
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

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		LOG_ERR("Failed to configure button interrupt");
		return false;
	}

    // Initialize the callback for both press and release events
    gpio_init_callback(&button_cb_data, button_callback, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);


    return true;
}



