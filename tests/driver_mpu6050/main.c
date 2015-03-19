/*
 * Copyright (C) 2015 airfy GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief   Test application for the MPU-6050 6-Axis driver
 *
 * @author  Ludwig Ortmann <ludwig@airfy.com>
 *
 * @}
 */

#ifndef TEST_I2C
#error "TEST_I2C not defined"
#endif
#ifndef TEST_HW_ADDR
#error "TEST_HW_ADDR not defined"
#endif

#include <stdio.h>

#include "mpu6050.h"
#include "vtimer.h"
#include "board.h"

#define SLEEP   (1000 * 1000u)

int main(void)
{
    mpu6050_t dev;
    mpu6050_results_t measurement;
    int32_t temperature;
    int result;

    puts("MPU-6050 test application\n");

    vtimer_init();

    printf("+------------Initializing------------+\n");
    result = mpu6050_init(&dev, TEST_I2C, TEST_HW_ADDR);

    if (result == -1) {
        puts("[Error] The given i2c is not enabled");
        return 1;
    }

    mpu6050_set_sample_rate(&dev, 200);
    if (dev.conf.sample_rate != 200) {
        puts("[Error] The sample rate was not set correctly");
        return 1;
    }

    printf("Initialization successful\n\n");
    printf("+------------Configuration------------+\n");
    printf("Sample rate: %"PRIu16" Hz\n", dev.conf.sample_rate);
    printf("Gyro full-scale range: 2000 DPS\n");
    printf("Accel full-scale range: 2 G\n");

    printf("\n+--------Starting Measurements--------+\n");
    while (1) {
        /* Get accel data in milli g */
        mpu6050_read_accel(&dev, &measurement);
        printf("Accel data [milli g] - X: %d   Y: %d   Z: %d\n",
                measurement.x_axis, measurement.y_axis, measurement.z_axis);
        /* Get gyro data in dps */
        mpu6050_read_gyro(&dev, &measurement);
        printf("Gyro data [dps] - X: %d   Y: %d   Z: %d\n",
                measurement.x_axis, measurement.y_axis, measurement.z_axis);
        /* Get temperature in milli degrees celsius */
        mpu6050_read_temperature(&dev, &temperature);
        printf("Temperature [milli deg] : %ld\n", temperature);
        printf("\n+-------------------------------------+\n");

        vtimer_usleep(SLEEP);
    }

    return 0;
}
