/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *******************************************************************************/

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

/**
 * @brief Structure VL53L7CX_Platform needs to be filled by the customer,
 * depending on his platform. At least, it contains the VL53L7CX I2C address.
 * Some additional fields can be added, as descriptors, or platform
 * dependencies. Anything added into this structure is visible into the platform
 * layer.
 */

typedef struct
{
    /* File descriptor for /dev/i2c-1 */
    int      i2c_fd;

    /* 7-bit I2C address of the sensor (0x29 by default) */
    uint16_t address;

} VL53L7CX_Platform;


/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#define VL53L7CX_NB_TARGET_PER_ZONE   1U


/*
 * @brief The macro below can be used to avoid data conversion into the driver.
 * By default there is a conversion between firmware and user data. Using this
 * macro allows to use the firmware format instead of user format. The firmware
 * format allows an increased precision.
 */

// #define  VL53L7CX_USE_RAW_FORMAT


/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

// #define VL53L7CX_DISABLE_AMBIENT_PER_SPAD
// #define VL53L7CX_DISABLE_NB_SPADS_ENABLED
// #define VL53L7CX_DISABLE_NB_TARGET_DETECTED
// #define VL53L7CX_DISABLE_SIGNAL_PER_SPAD
// #define VL53L7CX_DISABLE_RANGE_SIGMA_MM
// #define VL53L7CX_DISABLE_DISTANCE_MM
// #define VL53L7CX_DISABLE_REFLECTANCE_PERCENT
// #define VL53L7CX_DISABLE_TARGET_STATUS
// #define VL53L7CX_DISABLE_MOTION_INDICATOR


/* -------------------------------------------------------------------------- */
/*  Function prototypes required by ST's API                                   */
/* -------------------------------------------------------------------------- */

uint8_t VL53L7CX_RdByte(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_value);

uint8_t VL53L7CX_WrByte(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t value);

uint8_t VL53L7CX_RdMulti(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size);

uint8_t VL53L7CX_WrMulti(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size);

uint8_t VL53L7CX_Reset_Sensor(
        VL53L7CX_Platform *p_platform);

void VL53L7CX_SwapBuffer(
        uint8_t *buffer,
        uint16_t size);

uint8_t VL53L7CX_WaitMs(
        VL53L7CX_Platform *p_platform,
        uint32_t TimeMs);

#endif  /* _PLATFORM_H_ */
