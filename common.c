/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "common.h"



/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    HAL_StatusTypeDef status;
    (void)intf_ptr;
    
    status = HAL_I2C_Mem_Read(&hi2c2, device_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT,(uint8_t*)reg_data, len,HAL_MAX_DELAY);

    if (status == HAL_OK) {
        return BME68X_INTF_RET_SUCCESS;
    } 
    else{
        return 0;
    } 
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    HAL_StatusTypeDef status;
    (void)intf_ptr;
    
    status = HAL_I2C_Mem_Write(&hi2c2, device_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        return BME68X_INTF_RET_SUCCESS;
    } 
    else{
        return 0;
    }   
}

/*!
 * Delay function map to COINES platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
      // 获取 CPU 的时钟频率
    uint32_t clk_freq = HAL_RCC_GetHCLKFreq();

    // DWT 寄存器的计数器周期为 1 / (CPU 时钟频率)
    // 根据 CPU 的时钟频率计算出每个计数器周期需要的时间（单位：微秒）
    float period_us = 1.0f / (float)(clk_freq / 1000000);

    // 计算需要的计数器周期数
    uint32_t cycles = (uint32_t)((float)period / period_us);

    // 启用 DWT 寄存器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 使用 DWT 寄存器进行延时
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
        // 空循环，等待指定的延时时间
    }
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

static int count = 0; // 静态变量用于保持计数器的值

int8_t bme68x_interface_init(struct bme68x_dev *bme )
{
	int8_t rslt = BME68X_OK;

	if (bme != NULL)
	{
		/* Bus configuration : I2C */
        HAL_Delay(100);
		printf("I2C Interface - Count: %d\n", ++count);
        HAL_Delay(100);
		dev_addr =  BME68X_I2C_ADDR_HIGH;
		bme->read = bme68x_i2c_read;
		bme->write = bme68x_i2c_write;
		bme->intf = BME68X_I2C_INTF;

		bme->delay_us = bme68x_delay_us;
		bme->intf_ptr = &dev_addr;

        
		bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
	}
	else
	{
		rslt = BME68X_E_NULL_PTR;
	}
     
    return rslt;
}


