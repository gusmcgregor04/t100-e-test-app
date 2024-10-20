#include <zephyr/kernel.h>

void reset_to_bootloader() {
	printk ("returning bootloader\n");
	NRF_POWER->GPREGRET = 0x57; // 0xA8 OTA, 0x4e Serial
	NVIC_SystemReset();
}