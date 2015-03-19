/*
 * Copyright (C) 2015 Freie Universität Berlin
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
 * @author  Fabian Nack <nack@inf.fu-berlin.de>
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
#ifndef TEST_GPIO
#error "TEST_GPIO not defined"
#endif

#include <stdio.h>

#include "mpu6050.h"
#include "vtimer.h"
#include "board.h"
#include "msg.h"
#include "thread.h"

#define SLEEP   (1000 * 1000u)

char mpu_handler_stack[KERNEL_CONF_STACKSIZE_MAIN];
mpu6050_t dev;

void* mpu_handler(void *arg)
{
    msg_t msg_q[1];
    msg_init_queue(msg_q, 1);

    printf("Registering MPU handler thread...     %s\n",
           mpu6050_register_thread(&dev) == 0 ? "[OK]" : "[Failed]");

    msg_t m;
    while (msg_receive(&m)) {
        printf("MPU handler got a message: ");
        switch (m.type) {
            case MPU6050_MSG_INT:
                puts("something moved.");
                break;
            default:
                puts("stray message.");
                break;
        }
    }
    puts("MPU handler: this should not have happened!");

    return NULL;
}

int main(void)
{
    mpu6050_results_t measurement;
    int32_t temperature;
    int result;

    puts("MPU-6050 test application\n");

    //vtimer_init();

    printf("+------------Initializing------------+\n");
    result = mpu6050_init(&dev, TEST_I2C, TEST_HW_ADDR, TEST_GPIO);

    if (result == -1) {
        puts("[Error] The given i2c is not enabled");
        return 1;
    }

   thread_create(
           mpu_handler_stack, sizeof(mpu_handler_stack), PRIORITY_MAIN - 1,
           CREATE_WOUT_YIELD | CREATE_STACKTEST,
           mpu_handler, NULL, "mpu_handler");

#if 0
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
#endif

    return 0;
}
