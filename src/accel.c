#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "accel.h"
#include "qma6100p_regs.h"

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(acc);

#define I2C_NODE DT_NODELABEL(qma6100p)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
static const struct gpio_dt_spec qma6100p_enable = GPIO_DT_SPEC_GET(I2C_NODE, supply_gpios);


static int _range = -1;
static float xOffset = 0.0;
static float yOffset = 0.0;
static float zOffset = 0.0;


const float convRange2G = 0.000244F;
const float convRange4G = 0.000488F;
const float convRange8G = 0.000976F;
const float convRange16G = 0.001953F;
const float convRange32G = 0.003906F;



bool accel_init(void) {

    if (!gpio_is_ready_dt(&qma6100p_enable)) {

		LOG_ERR("Failed acc enable pin setup");
		return false;
	}

	int ret = gpio_pin_configure_dt(&qma6100p_enable, GPIO_OUTPUT_ACTIVE);

    if (ret < 0) {
		LOG_ERR("Failed to set acc enable pin as a output");
		return false;
	}

    ret = gpio_pin_set_dt(&qma6100p_enable, true);
    if (ret < 0) {
        LOG_ERR("Failed to enable acc");
	}
	k_msleep(50);

	if (!device_is_ready(dev_i2c.bus)) {
	printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
	return false;
	}

    return true;
}


uint8_t accel_get_unique_ID() {
	uint8_t data;
	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_CHIP_ID,&data,1);
	if(ret != 0){
		return 0xFF;
	}
  return data;
}

bool accel_software_reset() {
	uint8_t data = 0xb6;
	int ret = i2c_burst_write_dt(&dev_i2c,SFE_QMA6100P_SR, &data, 1);
	if(ret != 0){
		printk("Failed to write to accel");
		return false;
	}
	sfe_qma6100p_sr_bitfeild_t sr;
	for (int i = 0; i < 10; i++) {
		ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_SR,&sr.all,1);
		if(ret != 0){
			return false;
		}
		if (sr.all == 0xb6)
      	break;
    	k_msleep(1);
  	}

	ret = i2c_burst_write_dt(&dev_i2c,SFE_QMA6100P_SR, 0x00, 1);
	if(ret != 0){
		printk("Failed to write to accel");
		return false;
	}
  return true;
}

bool accel_enable(bool enable) {
  	uint8_t tempVal;

	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_PM,&tempVal,1);
	if(ret != 0){
		return false;
	}

  	sfe_qma6100p_pm_bitfield_t pm;
  	pm.all = tempVal;
  	pm.bits.mode_bit = enable; // sets QMA6100P to active mode
  	tempVal = pm.all;

	ret = i2c_burst_write_dt(&dev_i2c,SFE_QMA6100P_PM, &tempVal, sizeof(tempVal));
	if(ret != 0){
		printk("Failed to write to accel");
		return false;
	}
  	return true;
}

uint8_t accel_get_operating_mode() {
	uint8_t data;

	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_PM, &data, 1);
	if(ret != 0){
		return false;
	}

  sfe_qma6100p_pm_bitfield_t pm;
  pm.all = data; 

  return (pm.bits.mode_bit);
}

bool accel_set_range(uint8_t range) {
	uint8_t data;
	if (range > SFE_QMA6100P_RANGE32G) {
		return false;
	}
    
	// Read - Modify - Write
	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_FSR, &data, 1);
	if(ret != 0){
		return false;
	}

  	sfe_qma6100p_fsr_bitfield_t fsr;
  	fsr.all = data;
  	fsr.bits.range = range; // This is a long winded but definitive way of setting
                          // the range (g select)
	data = fsr.all;

	ret = i2c_burst_write_dt(&dev_i2c,SFE_QMA6100P_FSR, &data, sizeof(data));
	if(ret != 0){
		printk("Failed to write range to accel");
		return false;
	}
	
	_range = range; // Update our local copy
	return true;
}


uint8_t accel_get_range() {
	uint8_t data;
	uint8_t range;

	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_FSR, &data, 1);
	if(ret != 0){
		return false;
	}
  	sfe_qma6100p_fsr_bitfield_t fsr;
  	fsr.all = data;
  	range = fsr.bits.range;

  	return range;
}

bool accel_enable_data_engine(bool enable) {
  	uint8_t data;

	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_PM, &data, 1);
	if(ret != 0){
		return false;
	}

  	sfe_qma6100p_int_map1_bitfield_t int_map1;
  	int_map1.all = data;
  	int_map1.bits.int1_data = enable; // data ready interrupt to INT1
  	data = int_map1.all;


	ret = i2c_burst_write_dt(&dev_i2c,SFE_QMA6100P_INT_MAP1, &data, sizeof(data));
	if(ret != 0){
		printk("Failed to write INT setup to accel");
		return false;
	}

	// enable data ready interrupt
	ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_INT_EN1, &data, 1);
	if(ret != 0){
		return false;
	}

  	sfe_qma6100p_int_en1_bitfield_t int_en1;
  	int_en1.all = data;
  	int_en1.bits.int_data_en = enable; // set data ready interrupt
  	data = int_en1.all;

	ret = i2c_burst_write_dt(&dev_i2c,SFE_QMA6100P_INT_EN1, &data, sizeof(data));
	if(ret != 0){
		printk("Failed to write INT enable to accel");
		return false;
	}

  	return true;
}

bool accel_set_fifo_mode(uint8_t fifo_mode) {
  	uint8_t data;

  	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_FIFO_CFG0, &data, 1);
	if(ret != 0){
		return false;
	}

  	sfe_qma6100p_fifo_cfg0_bitfield_t fifo_cfg0;
  	fifo_cfg0.all = data;
  	fifo_cfg0.bits.fifo_mode = fifo_mode; // data ready interrupt to INT1
  	fifo_cfg0.bits.fifo_en_xyz = 0b111;
  	data = fifo_cfg0.all;

	ret = i2c_burst_write_dt(&dev_i2c,SFE_QMA6100P_FIFO_CFG0, &data, sizeof(data));
	if(ret != 0){
		printk("Failed to write fifo mode to accel");
		return false;
	}

  	return true;
}


bool accel_get_raw_accel_register_data(rawOutputData *rawAccelData) {
  	uint8_t regData[6] = {0};
  	int16_t data = 0;

	int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_DX_L, regData, 6);
	if(ret != 0){
		return false;
	}

  	// check newData_X
  	if (regData[0] & 0x1) {
    	data = (int16_t)(((uint16_t)(regData[1] << 8)) | (regData[0]));
    	rawAccelData->xData = data >> 2;
  	}
  	// check newData_Y
  	if (regData[2] & 0x1) {
    	data = (int16_t)(((uint16_t)(regData[3] << 8)) | (regData[2]));
    	rawAccelData->yData = data >> 2;
  	}
  	// check newData_Z
  	if (regData[4] & 0x1) {
    	data = (int16_t)(((uint16_t)(regData[5] << 8)) | (regData[4]));
    rawAccelData->zData = data >> 2;
  }

  return true;
}


bool accel_calibrate_offsets() {
  	outputData data;
  	int numSamples = 100;
  	float xSum = 0.0, ySum = 0.0, zSum = 0.0;

  	// Take multiple samples to average out noise
  	for (int i = 0; i < numSamples; i++) {
    if (!accel_get_data(&data)){
		return false;
	}
    xSum += data.xData;
    ySum += data.yData;
    zSum += data.zData - 1;
    k_msleep(10);
  }

  	// Calculate average
  	xOffset = xSum / numSamples;
  	yOffset = ySum / numSamples;
  	zOffset = zSum / numSamples; // Assuming z-axis aligned with gravity

  	return true;
}


bool accel_get_data(outputData *userData) {
	rawOutputData rawAccelData;
  	if (!accel_get_raw_accel_register_data(&rawAccelData)) {
		return false;
	}

  	if (!accel_conv_data(userData, &rawAccelData)) {
		return false;
	}

  	return true;
}


bool accel_conv_data(outputData *userAccel, rawOutputData *rawAccelData) {
  	if (_range < 0) // If the G-range is unknown, read it
  	{
    	uint8_t regVal;

		int ret = i2c_burst_read_dt(&dev_i2c, SFE_QMA6100P_FSR, &regVal, 1);
		if(ret != 0){
			return false;
		}

    	sfe_qma6100p_fsr_bitfield_t fsr;
    	fsr.all = regVal;

    	_range = fsr.bits.range; // Record the range
  	}

  	switch (_range) {
  	case SFE_QMA6100P_RANGE2G:
   		userAccel->xData = (float)rawAccelData->xData * convRange2G;
    	userAccel->yData = (float)rawAccelData->yData * convRange2G;
    	userAccel->zData = (float)rawAccelData->zData * convRange2G;
    	break;
  	case SFE_QMA6100P_RANGE4G:
    	userAccel->xData = (float)rawAccelData->xData * convRange4G;
    	userAccel->yData = (float)rawAccelData->yData * convRange4G;
    	userAccel->zData = (float)rawAccelData->zData * convRange4G;
    	break;
  	case SFE_QMA6100P_RANGE8G:
    	userAccel->xData = (float)rawAccelData->xData * convRange8G;
    	userAccel->yData = (float)rawAccelData->yData * convRange8G;
    	userAccel->zData = (float)rawAccelData->zData * convRange8G;
    	break;
  	case SFE_QMA6100P_RANGE16G:
    	userAccel->xData = (float)rawAccelData->xData * convRange16G;
    	userAccel->yData = (float)rawAccelData->yData * convRange16G;
    	userAccel->zData = (float)rawAccelData->zData * convRange16G;
    	break;
  	case SFE_QMA6100P_RANGE32G:
    	userAccel->xData = (float)rawAccelData->xData * convRange32G;
    	userAccel->yData = (float)rawAccelData->yData * convRange32G;
    	userAccel->zData = (float)rawAccelData->zData * convRange32G;
    	break;
  	default:
    	return false;
  	}

  	return true;
}

void accel_set_offset(float x, float y, float z) {
  xOffset = x;
  yOffset = y;
  zOffset = z;
}

void accel_offset_values(float *x, float *y, float *z) {
  *x = *x - xOffset;
  *y = *y - yOffset;
  *z = *z - zOffset;
}


void accel_start_test(void) {

	if (!accel_software_reset()) {
		printk("failed reset accel");
	}

	k_msleep(5);


	if(!accel_set_range(SFE_QMA6100P_RANGE32G)){   
    	printk("failed set range of accel");
  	}

	if(!accel_enable(true)){
    	printk("failed to enable the accel");
 	} 

}

void accel_print_data(void) {
	rawOutputData data;

	if (!accel_get_raw_accel_register_data(&data)) {
		printk("failed to get data for accel");
	}
	printk("Accel: X: %i,Y: %i,Z: %i\n", (int) data.xData, (int) data.yData, (int) data.zData);

}

