// tof_ascii_8x8.c
// Standalone C program to print 8x8 VL53L7CX distance data as ASCII grid.

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "vl53l7cx_api.h"
#include "platform.h"

#define NB_ZONES 64  // 8x8 = 64 zones

static VL53L7CX_Configuration Dev;

// Simple init: open I2C, init sensor, 8x8 resolution, start ranging
static int tof_init_ascii(void) {
    uint8_t status, isAlive;

    memset(&Dev, 0, sizeof(Dev));

    printf("[tof_init_ascii] Opening I2C bus /dev/i2c-1...\n");
    Dev.platform.i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (Dev.platform.i2c_fd < 0) {
        perror("[tof_init_ascii] Failed to open /dev/i2c-1");
        return -1;
    }

    Dev.platform.address = 0x29;

    if (ioctl(Dev.platform.i2c_fd, I2C_SLAVE, Dev.platform.address) < 0) {
        perror("[tof_init_ascii] Failed to set I2C slave address");
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return -1;
    }

    printf("[tof_init_ascii] Checking if VL53L7CX is alive...\n");
    status = vl53l7cx_is_alive(&Dev, &isAlive);
    if (status || !isAlive) {
        printf("[tof_init_ascii] VL53L7CX not detected at 0x%02X (status=%u, alive=%u)\n",
               Dev.platform.address, status, isAlive);
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return status ? status : 255;
    }

    status = vl53l7cx_init(&Dev);
    if (status) {
        printf("[tof_init_ascii] vl53l7cx_init failed (status=%u)\n", status);
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
        return status;
    }

    printf("[tof_init_ascii] VL53L7CX initialized. API rev: %s\n", VL53L7CX_API_REVISION);

    // Set 8x8 resolution
    status = vl53l7cx_set_resolution(&Dev, VL53L7CX_RESOLUTION_8X8);
    if (status) {
        printf("[tof_init_ascii] set_resolution 8x8 failed (status=%u)\n", status);
        return status;
    }

    // Ranging frequency (Hz)
    status = vl53l7cx_set_ranging_frequency_hz(&Dev, 5); // 5 Hz
    if (status) {
        printf("[tof_init_ascii] set_ranging_frequency_hz failed (status=%u)\n", status);
        return status;
    }

    status = vl53l7cx_start_ranging(&Dev);
    if (status) {
        printf("[tof_init_ascii] start_ranging failed (status=%u)\n", status);
        return status;
    }

    printf("[tof_init_ascii] ToF sensor started (8x8, 5 Hz).\n");
    return 0;
}

static void tof_shutdown_ascii(void) {
    vl53l7cx_stop_ranging(&Dev);
    if (Dev.platform.i2c_fd >= 0) {
        close(Dev.platform.i2c_fd);
        Dev.platform.i2c_fd = -1;
    }
    printf("[tof_shutdown_ascii] ToF sensor stopped.\n");
}

// Read one 8x8 frame into distances_mm (NB_ZONES elements)
static int tof_read_grid(uint16_t *distances_mm) {
    uint8_t status, isReady;
    VL53L7CX_ResultsData Results;
    int tries = 0;

    // Wait for data ready
    do {
        status = vl53l7cx_check_data_ready(&Dev, &isReady);
        if (status) {
            printf("[tof_read_grid] check_data_ready error (status=%u)\n", status);
            return -1;
        }
        if (!isReady) {
            VL53L7CX_WaitMs(&(Dev.platform), 5);
        }
        tries++;
    } while (!isReady && tries < 100);

    if (!isReady) {
        printf("[tof_read_grid] Timeout waiting for data ready\n");
        return -1;
    }

    status = vl53l7cx_get_ranging_data(&Dev, &Results);
    if (status) {
        printf("[tof_read_grid] get_ranging_data error (status=%u)\n", status);
        return -1;
    }

    // Copy distances; use first target per zone
    for (uint8_t zone = 0; zone < NB_ZONES; zone++) {
        uint8_t idx = VL53L7CX_NB_TARGET_PER_ZONE * zone;
        uint8_t zone_status = Results.target_status[idx];
        int16_t dist_mm = Results.distance_mm[idx];

        if (zone_status != 255 && dist_mm > 0) {
            distances_mm[zone] = (uint16_t)dist_mm;
        } else {
            distances_mm[zone] = 0;  // 0 = no valid target
        }
    }

    return 0;
}

int main(void) {
    if (tof_init_ascii() != 0) {
        printf("Failed to initialize ToF sensor.\n");
        return 1;
    }

    uint16_t grid[NB_ZONES];

    while (1) {
        if (tof_read_grid(grid) != 0) {
            printf("Error reading grid, exiting.\n");
            break;
        }

        // Clear screen and move cursor home
        printf("\033[2J\033[H");
        printf("VL53L7CX 8x8 Distance Map (mm)\n");
        printf("(0 means no valid target)\n\n");

        // Print 8x8 grid
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 8; col++) {
                int idx = row * 8 + col;
                uint16_t d = grid[idx];

                if (d == 0) {
                    printf("  --- ");
                } else {
                    printf("%5u ", d);
                }
            }
            printf("\n");
        }

        fflush(stdout);
        usleep(200000);  // 200 ms (~5 Hz)
    }

    tof_shutdown_ascii();
    return 0;
}