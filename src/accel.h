#pragma once
#include <stdbool.h>

//#define QMA_6100P_INT_PIN (32 + 2) // P1.02 from meshtastic


#define QMA6100P_CHIP_ID 0x90

#define SFE_QMA6100P_RANGE2G 0b0001
#define SFE_QMA6100P_RANGE4G 0b0010
#define SFE_QMA6100P_RANGE8G 0b0100
#define SFE_QMA6100P_RANGE16G 0b1000
#define SFE_QMA6100P_RANGE32G 0b1111

#define SFE_QMA6100P_FIFO_MODE_BYPASS 0b00
#define SFE_QMA6100P_FIFO_MODE_FIFO 0b01
#define SFE_QMA6100P_FIFO_MODE_STREAM 0b10
#define SFE_QMA6100P_FIFO_MODE_AUX 0b11

#define SENSORS_GRAVITY_EARTH (9.80665F)

typedef struct {
  float xData;
  float yData;
  float zData;
} outputData;


typedef struct {
  int16_t xData;
  int16_t yData;
  int16_t zData;
} rawOutputData;

bool accel_init(void);
uint8_t accel_get_unique_ID();
bool accel_software_reset();
bool accel_enable(bool enable);
uint8_t accel_get_operating_mode();
bool accel_set_range(uint8_t range);
uint8_t accel_get_range();
bool accel_enable_data_engine(bool enable);
bool accel_set_fifo_mode(uint8_t fifo_mode);
bool accel_get_raw_accel_register_data(rawOutputData *rawAccelData);
bool accel_calibrate_offsets();
bool accel_get_data(outputData *userData);
bool accel_conv_data(outputData *userAccel, rawOutputData *rawAccelData);
void accel_offset_values(float *x, float *y, float *z);
void accel_set_offset(float x, float y, float z);

void accel_start_test(void);
void accel_print_data(void);
