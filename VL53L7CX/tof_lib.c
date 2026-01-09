#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "vl53l7cx_api.h"
#include "platform.h"

// Single global device config for the library
static VL53L7CX_Configuration Dev;
static int tof_initialized = 0;

// Initialize I2C + VL53L7CX and start ranging at 2 Hz
int tof_init(void) {
    if (tof_initialized) {
        // Already initialized
        return 0;
    }

    uint8_t status, isAlive;

    memset(&Dev, 0, sizeof(Dev));

    printf("[tof_init] Opening I2C bus /dev/i2c-1...\n");
    Dev.platform.i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (Dev.platform.i2c_fd < 0) {
        perror("[tof_init] Failed to open /dev/i2c-1");
        return -1;
    }

    Dev.platform.address = 0x29;

    if (ioctl(Dev.platform.i2c_fd, I2C_SLAVE, Dev.platform.address) < 0) {
        perror("[tof_init] Failed to set I2C slave address");
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return -1;
    }

    printf("[tof_init] Checking if VL53L7CX is alive...\n");
    status = vl53l7cx_is_alive(&Dev, &isAlive);
    if (!isAlive || status) {
        printf("[tof_init] VL53L7CX not detected at address 0x%02X\n",
               Dev.platform.address);
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return status ? status : 255;
    }

    status = vl53l7cx_init(&Dev);
    if (status) {
        printf("[tof_init] VL53L7CX ULD init failed (status=%u)\n", status);
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return status;
    }

    printf("[tof_init] VL53L7CX ULD ready! (version %s)\n", VL53L7CX_API_REVISION);

    status = vl53l7cx_set_ranging_frequency_hz(&Dev, 2);
    if (status) {
        printf("[tof_init] Failed to set 2 Hz ranging (status=%u)\n", status);
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return status;
    }

    status = vl53l7cx_start_ranging(&Dev);
    if (status) {
        printf("[tof_init] Start ranging failed (status=%u)\n", status);
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return status;
    }

    tof_initialized = 1;
    printf("[tof_init] ToF sensor initialized.\n");
    return 0;
}

// Take ONE reading and return average distance in mm, or -1.0 on error/no target
double tof_get_avg_distance_mm(void) {
    if (!tof_initialized) {
        int rc = tof_init();
        if (rc != 0) {
            return -1.0;
        }
    }

    uint8_t status, isReady, i;
    VL53L7CX_ResultsData Results;
    uint32_t sum_mm = 0;
    uint8_t valid = 0;
    int tries = 0;

    // Wait for a new frame to be ready
    do {
        status = vl53l7cx_check_data_ready(&Dev, &isReady);
        if (status) {
            printf("[tof_get_avg_distance_mm] check_data_ready error (status=%u)\n", status);
            return -1.0;
        }
        if (!isReady) {
            VL53L7CX_WaitMs(&(Dev.platform), 5);
        }
        tries++;
    } while (!isReady && tries < 100);

    if (!isReady) {
        printf("[tof_get_avg_distance_mm] Timeout waiting for data ready\n");
        return -1.0;
    }

    status = vl53l7cx_get_ranging_data(&Dev, &Results);
    if (status) {
        printf("[tof_get_avg_distance_mm] get_ranging_data error (status=%u)\n", status);
        return -1.0;
    }

    // 4x4 = 16 zones, same averaging idea as your example
    for (i = 0; i < 16; i++) {
        uint8_t idx = VL53L7CX_NB_TARGET_PER_ZONE * i;
        uint8_t zone_status = Results.target_status[idx];
        int16_t dist_mm     = Results.distance_mm[idx];

        if (zone_status != 255 && dist_mm > 0) {
            sum_mm += dist_mm;
            valid++;
        }
    }

    if (valid == 0) {
        return -1.0;
    }

    double avg = (double)sum_mm / valid;
    return avg;
}

void tof_shutdown(void) {
    if (!tof_initialized) {
        return;
    }

    vl53l7cx_stop_ranging(&Dev);
    if (Dev.platform.i2c_fd >= 0) {
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
    }
    tof_initialized = 0;
    printf("[tof_shutdown] ToF sensor stopped.\n");
}
