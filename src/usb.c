#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
#include "usb.h"

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(usb);

static const struct device *const consoleDev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static void usb_status_cb(enum usb_dc_status_code cb_status,
				       const uint8_t *param)
{
    switch(cb_status)
    {
        case USB_DC_RESUME:
        case USB_DC_CONNECTED:
        {
            LOG_DBG("USB connected");
            break;
        }
        case USB_DC_DISCONNECTED:
        case USB_DC_SUSPEND:
        {
            LOG_DBG("USB disconnected");
            break;
        }
        default:
        {
            break;
        }
    }
}

bool usb_init(void)
{
    int rc = usb_enable(usb_status_cb);
	if (rc)
	{
		LOG_ERR("Failed to enable USB");
		return false;
	}

	bool waitForUsb = true;
    uint64_t startTime = k_uptime_get();
	while (waitForUsb)
	{
		k_sleep(K_MSEC(100));
        
		// wait until UART DTR goes up (pc is connected) before continuing so we don't miss any debug messages
		uint32_t dtr = 0;
		if(uart_line_ctrl_get(consoleDev, UART_LINE_CTRL_DTR, &dtr) != 0)
        {
            dtr = 0;
        }

		// or if 5 seconds have passed, continue anyway
		if(k_uptime_get() - startTime > 5000 || dtr > 0)
		{
			waitForUsb = false;
		}
	}
    return true;
}