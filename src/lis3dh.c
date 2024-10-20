#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include "lis3dh.h"

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(acc);

#define ACC_NODE DT_NODELABEL(lis3dh)
const struct device *const sensor = DEVICE_DT_GET_ANY(lis3dh);
static const struct gpio_dt_spec acc_enable = GPIO_DT_SPEC_GET(ACC_NODE, supply_gpios);


#define I2C_NODE DT_ALIAS(my_i2c)
#define I2C_DEV	DT_PROP(I2C_NODE, label)






bool acc_init(void) {

    if (!gpio_is_ready_dt(&acc_enable)) {

		LOG_ERR("Failed acc enable pin setup");
		return false;
	}

	int ret = gpio_pin_configure_dt(&acc_enable, GPIO_OUTPUT_ACTIVE);

    if (ret < 0) {
		LOG_ERR("Failed to set acc enable pin as a output");
		return false;
	}

    ret = gpio_pin_set_dt(&acc_enable, true);
    if (ret < 0) {
        LOG_ERR("Failed to enable acc");
	}
	k_msleep(50);

    if (sensor == NULL) {
        LOG_ERR("No device found");
		return false;
	}

	if (!device_is_ready(sensor)) {
        LOG_ERR("Acc is not ready");
		return false;
	}
    return true;
}

void acc_fetch_display(void) {
	static unsigned int count;
	struct sensor_value accel[3];
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		printk("ERROR: Update failed: %d\n", rc);
	} else {
		printk("#%u @ %u ms: %sx %f , y %f , z %f",
		       count, k_uptime_get_32(), overrun,
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}
	printk("\n");
}

void i2c_scan(void) {
	const struct device *i2c_dev;

	k_sleep(K_SECONDS(1));
	printk("*** I2C scanner started.                    ***\nBoard name      : %s\n", CONFIG_BOARD);

	i2c_dev = device_get_binding(I2C_DEV);
		if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	printk("I2C Port        : %s \n",I2C_DEV);
	printk("Clock Frequency : %d \n",DT_PROP(I2C_NODE, clock_frequency));

	printk("\n    | 0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09 0x0a 0x0b 0x0c 0x0d 0x0e 0x0f |\n");
	printk(  "----|---------------------------------------------------------------------------------");
	
	uint8_t error = 0u;
	uint8_t dst;
	uint8_t i2c_dev_cnt = 0;
	struct i2c_msg msgs[1];
	msgs[0].buf = &dst;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/* Use the full range of I2C address for display purpose */	
	for (uint16_t x = 0; x <= 0x7f; x++) {
		/* New line every 0x10 address */
		if (x % 0x10 == 0) {
			printk("|\n0x%02x| ",x);	
		}
		/* Range the test with the start and stop value configured in the kconfig */
		if (x >= 0 && x <= 255)	{	
			/* Send the address to read from */
			error = i2c_transfer(i2c_dev, &msgs[0], 1, x);
				/* I2C device found on current address */
				if (error == 0) {
					printk("0x%02x ",x);
					i2c_dev_cnt++;
				}
				else {
					printk(" --  ");
				}
		} else {
			/* Scan value out of range, not scanned */
			printk("     ");
		}
	}
	printk("|\n");
	printk("\nI2C device(s) found on the bus: %d\nScanning done.\n\n", i2c_dev_cnt);
	printk("Find the registered I2C address on: https://i2cdevices.org/addresses\n\n");
}