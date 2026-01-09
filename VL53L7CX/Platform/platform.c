/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "platform.h"

#include <string.h>         // memcpy
#include <unistd.h>         // usleep
#include <stdint.h>         // uint8_t, uint16_t, etc.
#include <linux/i2c-dev.h>  // I2C constants
#include <linux/i2c.h>      // struct i2c_msg, i2c_rdwr_ioctl_data
#include <sys/ioctl.h>      // ioctl
#include <fcntl.h>          // open (used in app)
#include <stdlib.h>         // malloc, free
#include <stdio.h>          // printf, perror
#include <errno.h>          // errno

/* Limit per I2C transfer (payload bytes, not counting 2-byte register) */
#define I2C_MAX_XFER  256U

/* -------------------------------------------------------------------------- */
/*  Internal I2C helpers                                                      */
/* -------------------------------------------------------------------------- */

/* Write multiple bytes: 16-bit register address + payload via I2C_RDWR,
 * broken into smaller chunks if needed.
 */
static uint8_t _i2c_write_multi(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    if (p_platform == NULL || p_platform->i2c_fd < 0) {
        printf("[WRITE] Invalid platform or i2c_fd\n");
        return 255;
    }

    uint32_t remaining = size;
    uint32_t offset    = 0;
    uint16_t reg       = RegisterAdress;

    while (remaining > 0) {
        uint32_t chunk = (remaining > I2C_MAX_XFER) ? I2C_MAX_XFER : remaining;
        uint32_t total = chunk + 2U;

        uint8_t *buf = (uint8_t *)malloc(total);
        if (buf == NULL) {
            printf("[WRITE] malloc failed for %lu bytes\n", (unsigned long)total);
            return 255;
        }

        buf[0] = (uint8_t)(reg >> 8);
        buf[1] = (uint8_t)(reg & 0xFF);
        memcpy(buf + 2, p_values + offset, chunk);

        struct i2c_msg msg;
        struct i2c_rdwr_ioctl_data packets;

        msg.addr  = p_platform->address;    /* 7-bit address, e.g., 0x29 */
        msg.flags = 0;                      /* Write */
        msg.len   = (uint16_t)total;
        msg.buf   = buf;

        packets.msgs  = &msg;
        packets.nmsgs = 1;

        int ret = ioctl(p_platform->i2c_fd, I2C_RDWR, &packets);
        if (ret < 0) {
            perror("[WRITE] I2C_RDWR failed");
            printf("[WRITE] reg=0x%04X, chunk=%lu, total=%lu, errno=%d\n",
                   reg,
                   (unsigned long)chunk,
                   (unsigned long)total,
                   errno);
            free(buf);
            return 255;
        }

        free(buf);

        remaining -= chunk;
        offset    += chunk;
        reg       = (uint16_t)(reg + chunk);
    }

    return 0;
}

/* Read multiple bytes: write 16-bit register, then read N bytes via I2C_RDWR,
 * broken into smaller chunks if needed.
 */
static uint8_t _i2c_read_multi(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    if (p_platform == NULL || p_platform->i2c_fd < 0) {
        printf("[READ] Invalid platform or i2c_fd\n");
        return 255;
    }

    uint32_t remaining = size;
    uint32_t offset    = 0;
    uint16_t reg       = RegisterAdress;

    while (remaining > 0) {
        uint32_t chunk = (remaining > I2C_MAX_XFER) ? I2C_MAX_XFER : remaining;

        uint8_t reg_buf[2];
        reg_buf[0] = (uint8_t)(reg >> 8);
        reg_buf[1] = (uint8_t)(reg & 0xFF);

        struct i2c_msg msgs[2];
        struct i2c_rdwr_ioctl_data packets;

        /* First message: write register address */
        msgs[0].addr  = p_platform->address;
        msgs[0].flags = 0;          /* Write */
        msgs[0].len   = 2;
        msgs[0].buf   = reg_buf;

        /* Second message: read data */
        msgs[1].addr  = p_platform->address;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len   = (uint16_t)chunk;
        msgs[1].buf   = p_values + offset;

        packets.msgs  = msgs;
        packets.nmsgs = 2;

        int ret = ioctl(p_platform->i2c_fd, I2C_RDWR, &packets);
        if (ret < 0) {
            perror("[READ] I2C_RDWR failed");
            printf("[READ] reg=0x%04X, chunk=%lu, errno=%d\n",
                   reg,
                   (unsigned long)chunk,
                   errno);
            return 255;
        }

        remaining -= chunk;
        offset    += chunk;
        reg       = (uint16_t)(reg + chunk);
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/*  API functions used by the VL53L7CX ULD                                    */
/* -------------------------------------------------------------------------- */

uint8_t VL53L7CX_RdByte(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_value)
{
    return _i2c_read_multi(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L7CX_WrByte(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t value)
{
    return _i2c_write_multi(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L7CX_WrMulti(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    return _i2c_write_multi(p_platform, RegisterAdress, p_values, size);
}

uint8_t VL53L7CX_RdMulti(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    return _i2c_read_multi(p_platform, RegisterAdress, p_values, size);
}

uint8_t VL53L7CX_Reset_Sensor(
        VL53L7CX_Platform *p_platform)
{
    (void)p_platform;  /* unused */

    VL53L7CX_WaitMs(p_platform, 100);
    VL53L7CX_WaitMs(p_platform, 100);

    return 0;
}

void VL53L7CX_SwapBuffer(
        uint8_t  *buffer,
        uint16_t  size)
{
    uint32_t i, tmp;

    for (i = 0; i < size; i = i + 4)
    {
        tmp =  ( (uint32_t)buffer[i]     << 24 )
             | ( (uint32_t)buffer[i + 1] << 16 )
             | ( (uint32_t)buffer[i + 2] << 8  )
             | ( (uint32_t)buffer[i + 3] );

        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t VL53L7CX_WaitMs(
        VL53L7CX_Platform *p_platform,
        uint32_t TimeMs)
{
    (void)p_platform;  /* unused */

    usleep(TimeMs * 1000U);

    return 0;
}
