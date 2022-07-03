/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdio.h>
#include <pico/stdlib.h>

#include "../bme688/bme68x.h"
#include "common.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME688 Pimoroni breakout addr */
#define BME68X_PIMORONI_ID 0x76

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to PICO platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (len < 1)
    {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(I2C_PORT, dev_addr, &reg_addr, 1, true);
    num_bytes_read = i2c_read_blocking(I2C_PORT, dev_addr, reg_data, len, false);

    if(num_bytes_read == -2) {
        return -1;
    } else {
        return 0;
    }
}

/*!
 * I2C write function map to PICO platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;

    int num_bytes_read = 0;
    uint8_t msg[len + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (len < 1)
    {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg_addr;
    for (int i = 0; i < len; i++)
    {
        msg[i + 1] = reg_data[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(I2C_PORT, dev_addr, msg, (len + 1), false);

    return num_bytes_read;
}

/*!
 * Delay function map to PICO platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    sleep_us(period);
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

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;

    if (bme != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
            printf("I2C Interface\n");
            dev_addr = BME68X_I2C_ADDR_LOW;
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;
        }

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
