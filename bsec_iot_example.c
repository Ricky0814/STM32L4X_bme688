/*!
 * @file bsec_iot_example.c
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME68x sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include "bsec_iot_example.h"



/*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 * param[in]        intf_ptr        interface pointer
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    HAL_StatusTypeDef status;
    (void)intf_ptr;
    
    status = HAL_I2C_Mem_Write(&hi2c2, device_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data_ptr, data_len, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        return BME68X_INTF_RET_SUCCESS;
    } 
    else{
        return 0;
    }
}

/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 * param[in]        intf_ptr        interface pointer
 * 
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    HAL_StatusTypeDef status;
    (void)intf_ptr;
    
    status = HAL_I2C_Mem_Read(&hi2c2, device_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT,(uint8_t*)reg_data_ptr, data_len,HAL_MAX_DELAY);

    if (status == HAL_OK) {
        return BME68X_INTF_RET_SUCCESS;
    } 
    else{
        return 0;
    }
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_us     Time in microseconds
 * @param[in]       intf_ptr Pointer to the interface descriptor
 * 
 * @return          none
 */
void bst_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 10; i++)
		{
			;
		}
	}
   
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us(void)
{
    int64_t system_current_time = 0;
	uint32_t tick;
	tick = HAL_GetTick();
	system_current_time = 1000*(int64_t)tick;
	return system_current_time;
}

#if defined(BME68X_OUTPUT_GAS_ESTIMATE)

void output_ready(int64_t timestamp, float gas_estimate_1, float gas_estimate_2, float gas_estimate_3, float gas_estimate_4,
     float raw_pressure, float raw_temp, float raw_humidity, float raw_gas, uint8_t raw_gas_index, bsec_library_return_t bsec_status)
{
    if (raw_gas_index == 9) {
        printf("%lld, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d\r\n",
               timestamp / 1000000, gas_estimate_1, gas_estimate_2, gas_estimate_3, gas_estimate_4, raw_pressure, raw_temp, raw_humidity, raw_gas, raw_gas_index, bsec_status);
    }
}
#elif defined(BME68X_OUTPUT_GAS_IAQ)

void output_iaq_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float static_iaq, float co2_equivalent, float breath_voc_equivalent,
     float raw_pressure, float raw_temp, float raw_humidity, float raw_gas, uint8_t raw_gas_index, bsec_library_return_t bsec_status)
{

printf("%lld, %f,%d, %f, %f, %f, %f, %f, %f, %f, %d, %d\r\n",
       timestamp / 1000000, iaq, iaq_accuracy, static_iaq, co2_equivalent, breath_voc_equivalent, raw_pressure, raw_temp, raw_humidity, raw_gas, raw_gas_index, bsec_status);
}
#endif
/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
//#ifdef USE_EEPROM
//    

//    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
//    {
//        /* Existing state in EEPROM */
//        Serial.println("Reading state from EEPROM");
//        Serial.print("State file: ");
//        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
//        {
//            bsecState[i] = EEPROM.read(i + 1);
//            Serial.print(String(bsecState[i], HEX) + ", ");
//        }
//        Serial.println();

//        if (!bsec.setState(bsecState))
//            return false;
//    } else
//    {
//        /* Erase the EEPROM with zeroes */
//        Serial.println("Erasing EEPROM");

//        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
//            EEPROM.write(i, 0);

//        EEPROM.commit();
//    }
//#endif
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
//#ifdef USE_EEPROM
//    if (!bsec.getState(bsecState))
//        return false;

//    Serial.println("Writing state to EEPROM");
//    Serial.print("State file: ");

//    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
//    {
//        EEPROM.write(i + 1, bsecState[i]);
//        Serial.print(String(bsecState[i], HEX) + ", ");
//    }
//    Serial.println();

//    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
//    EEPROM.commit();
//#endif

}
 
/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    memcpy(config_buffer, bsec_config_selectivity, sizeof(bsec_config_selectivity));
    return sizeof(bsec_config_selectivity);    

}

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
int bsec_iot_example(void)
{
    
    return_values_init ret;
    struct bme68x_dev bme_dev;
	memset(&bme_dev,0,sizeof(bme_dev)); 
    
    bme68x_interface_init(&bme_dev);
    

    #if defined(BME68X_OUTPUT_GAS_ESTIMATE)
	  ret = bsec_iot_init(BSEC_SAMPLE_RATE_SCAN, 0.0f, bus_write, bus_read, bst_delay_us, state_load, config_load, bme_dev);
    #elif defined(BME68X_OUTPUT_GAS_IAQ)
        ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, bst_delay_us, state_load, config_load, bme_dev);
    #endif  
    
    if (ret.bme68x_status)
    {
        /* Could not intialize BME68x */
         printf("Could not intialize BSEC library, bme688_status=%d\r\n", ret.bme68x_status);
       
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        printf("Could not intialize BSEC library, bsec_status=%d\r\n", ret.bsec_status);
      
    }    
    
    #if defined(BME68X_OUTPUT_GAS_ESTIMATE)
    printf("timestamp, gas_estimate_1, gas_estimate_2, gas_estimate_3, gas_estimate_4, raw_pressure, raw_temp, raw_humidity, raw_gas, raw_gas_index, bsec_status\r\n");
    bsec_iot_loop(bst_delay_us, get_timestamp_us, output_ready, state_save, 10);
    #elif defined(BME68X_OUTPUT_GAS_IAQ)
    printf("timestamp, IAQ, IAQ_accuracy, static_IAQ, co2_equivalent, breath_voc_equivalent, raw_pressure, raw_temp, raw_humidity, raw_gas, raw_gas_index, bsec_status\r\n");
     bsec_iot_loop(bst_delay_us, get_timestamp_us, output_iaq_ready, state_save, 1000);
    #endif
    
    return 0;
}

/*! @}*/

