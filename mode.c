/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "mode.h"

/***********************************************************************/
/*                         Test code                                   */
/***********************************************************************/

int self_test(void)
{
    struct bme68x_dev bme;
    int8_t rslt;
    
	rslt = bme68x_interface_init(&bme);
	bme68x_check_rslt("bme68x_interface_init", rslt);

	bme68x_soft_reset(&bme);
	bme.delay_us(10, bme.intf_ptr);
    
    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    rslt = bme68x_selftest_check(&bme);
    bme68x_check_rslt("bme68x_selftest_check", rslt);

    if (rslt == BME68X_OK)
    {
        printf("Self-test passed\n");
    }

    if (rslt == BME68X_E_SELF_TEST)
    {
        printf("Self-test failed\n");
    }



    return rslt;
}

/***********************************************************************/
/*                         Macros                                      */
/***********************************************************************/

/* Macro for count of samples to be displayed */
#define forced_mode_SAMPLE_COUNT  UINT16_C(300)

/***********************************************************************/
/*                         Test code                                   */
/***********************************************************************/
int forced_mode(void)
{
    struct bme68x_dev bme;
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data;
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 1;
    
	rslt = bme68x_interface_init(&bme);
	bme68x_check_rslt("bme68x_interface_init", rslt);

	bme68x_soft_reset(&bme);
	bme.delay_us(10, bme.intf_ptr);

    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    printf("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status\n");

    while (sample_count <= forced_mode_SAMPLE_COUNT)
    {
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        bme68x_check_rslt("bme68x_set_op_mode", rslt);

        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        time_ms = HAL_GetTick();

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", rslt);

        if (n_fields)
        {
#ifdef BME68X_USE_FPU
            printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x\n",
                   sample_count,
                   (long unsigned int)time_ms,
                   data.temperature,
                   data.pressure,
                   data.humidity,
                   data.gas_resistance,
                   data.status);
#else
            printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x\n",
                   sample_count,
                   (long unsigned int)time_ms,
                   (data.temperature / 100),
                   (long unsigned int)data.pressure,
                   (long unsigned int)(data.humidity / 1000),
                   (long unsigned int)data.gas_resistance,
                   data.status);
#endif
            sample_count++;
        }
    }



    return rslt;
}


/***********************************************************************/
/*                         Macros                                      */
/***********************************************************************/

/*
 * Macro definition for valid new data (0x80) AND
 * heater stability (0x10) AND gas resistance (0x20) values
 */
#define parallel_mode_BME68X_VALID_DATA  UINT8_C(0xB0)

/* Macro for count of samples to be displayed */
#define parallel_mode_SAMPLE_COUNT       UINT8_C(50)

/***********************************************************************/
/*                         Test code                                   */
/***********************************************************************/

int parallel_mode(void)
{
    struct bme68x_dev bme;
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data[3];
    uint32_t del_period;
    uint8_t n_fields;
    uint32_t time_ms = 0;
    uint16_t sample_count = 1;

    /* Heater temperature in degree Celsius */
    uint16_t temp_prof[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };

    /* Multiplier to the shared heater duration */
    uint16_t mul_prof[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };

    rslt = bme68x_interface_init(&bme);
	bme68x_check_rslt("bme68x_interface_init", rslt);

	bme68x_soft_reset(&bme);
	bme.delay_us(10, bme.intf_ptr);
    
    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_get_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = mul_prof;

    /* Shared heating duration in milliseconds */
    heatr_conf.shared_heatr_dur = (uint16_t)(140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) / 1000));

    heatr_conf.profile_len = 10;
    rslt = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);

    printf(
        "Print parallel mode data if mask for new data(0x80), gas measurement(0x20) and heater stability(0x10) are set\n\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    printf(
        "Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status, Gas index, Meas index\n");
    while (sample_count <= parallel_mode_SAMPLE_COUNT)
    {
        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) + (heatr_conf.shared_heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        time_ms = HAL_GetTick();

        rslt = bme68x_get_data(BME68X_PARALLEL_MODE, data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", rslt);

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        for (uint8_t i = 0; i < n_fields; i++)
        {
            if (data[i].status == parallel_mode_BME68X_VALID_DATA)
            {
#ifdef BME68X_USE_FPU
                printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
                       sample_count,
                       (long unsigned int)time_ms,
                       data[i].temperature,
                       data[i].pressure,
                       data[i].humidity,
                       data[i].gas_resistance,
                       data[i].status,
                       data[i].gas_index,
                       data[i].meas_index);
#else
                printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x, %d, %d\n",
                       sample_count,
                       (long unsigned int)time_ms,
                       (data[i].temperature / 100),
                       (long unsigned int)data[i].pressure,
                       (long unsigned int)(data[i].humidity / 1000),
                       (long unsigned int)data[i].gas_resistance,
                       data[i].status,
                       data[i].gas_index,
                       data[i].meas_index);
#endif
                sample_count++;
            }
        }
    }



    return 0;
}


/***********************************************************************/
/*                         Macros                                      */
/***********************************************************************/

/* Macro for count of samples to be displayed */
#define sequential_mode_SAMPLE_COUNT  UINT8_C(300)

/***********************************************************************/
/*                         Test code                                   */
/***********************************************************************/

int sequential_mode(void)
{
    struct bme68x_dev bme;
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data[3];
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 1;

    /* Heater temperature in degree Celsius */
    uint16_t temp_prof[10] = { 200, 240, 280, 320, 360, 360, 320, 280, 240, 200 };

    /* Heating duration in milliseconds */
    uint16_t dur_prof[10] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };
    
    rslt = bme68x_interface_init(&bme);
	bme68x_check_rslt("bme68x_interface_init", rslt);
    
	bme68x_soft_reset(&bme);
	bme.delay_us(10, bme.intf_ptr);
   
    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);
    
    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_get_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);
    
    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE; /* This parameter defines the sleep duration after each profile */
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);
  
    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = dur_prof;
    heatr_conf.profile_len = 10;
    rslt = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_set_op_mode(BME68X_SEQUENTIAL_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    printf(
        "Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status, Profile index, Measurement index\n");
    while (sample_count <= sequential_mode_SAMPLE_COUNT)
    {
        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_SEQUENTIAL_MODE, &conf, &bme) + (heatr_conf.heatr_dur_prof[0] * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        time_ms = HAL_GetTick();

        rslt = bme68x_get_data(BME68X_SEQUENTIAL_MODE, data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", rslt);

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        for (uint8_t i = 0; i < n_fields; i++)
        {
#ifdef BME68X_USE_FPU
            printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
                   sample_count,
                   (long unsigned int)time_ms + (i * (del_period / 2000)),
                   data[i].temperature,
                   data[i].pressure,
                   data[i].humidity,
                   data[i].gas_resistance,
                   data[i].status,
                   data[i].gas_index,
                   data[i].meas_index);
#else
            printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x, %d, %d\n",
                   sample_count,
                   (long unsigned int)time_ms + (i * (del_period / 2000)),
                   (data[i].temperature / 100),
                   (long unsigned int)data[i].pressure,
                   (long unsigned int)(data[i].humidity / 1000),
                   (long unsigned int)data[i].gas_resistance,
                   data[i].status,
                   data[i].gas_index,
                   data[i].meas_index);
#endif
            sample_count++;
        }
    }



    return 0;
}
