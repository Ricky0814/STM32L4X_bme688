#ifndef __BSEC_IOT_EXAMPLE_H__
#define __BSEC_IOT_EXAMPLE_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "main.h"
#include <stdio.h>
#include "bme68x.h"
#include "common.h"
#include <string.h>
#include "i2c.h"
#include "bsec_integration.h"
#include "Selectivity_Config.h"

#include "2023_11_22_08_59_test3_HP-354_RDC-1-0Continuous.h"

int8_t bus_write(uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr);

int8_t bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr);

void bst_delay_us(uint32_t t_us, void *intf_ptr);

int64_t get_timestamp_us(void);

int bsec_iot_example(void);

#define USE_BOSCH_SENSOR_API

#define USE_I2C_INTERFACE
//#define USE_SPI_INTERFACE

#define USE_BME68X

#if defined(USE_BME68X)
#define BME68X_OUTPUT_GAS_ESTIMATE   //气体检测
//#define BME68X_OUTPUT_GAS_IAQ      //空气质量评估

#endif


#ifdef __cplusplus
}
#endif

#endif /* __BSEC_IOT_EXAMPLE_H__ */








