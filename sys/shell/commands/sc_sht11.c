/**
 * Shell commands for temperature and humidity sensor
 *
 * Copyright 2013 INRIA
 * Copyright 2014 Ludwig Ortmann
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * @ingroup shell_commands
 * @{
 * @file
 * @brief   Shell commands for the sht11 sensor
 * @author  Oliver Hahm <oliver.hahm@inria.fr>
 * @author  Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sht11.h"

extern float sht11_temperature_offset;

static void _printfloat(float val)
{
    int i = (int) val;
    int f = (int) ((val - ((int)val)) * 100.0);
    if (i < 0) {
        f *= -1;
    }
    printf("%i.%02i", i, f);
}

void _get_humidity_handler(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    uint8_t success;
    sht11_val_t sht11_val;
    success = sht11_read_sensor(&sht11_val, HUMIDITY | TEMPERATURE);

    if (!success) {
        printf("Error reading SHT11\n");
    }
    else {
        printf("Relative humidity: ");
        _printfloat(sht11_val.relhum);
        printf("%% / Temperature compensated humidity: ");
        _printfloat(sht11_val.relhum_temp);
        puts("%");
    }
}

void _get_temperature_handler(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    uint8_t success;
    sht11_val_t sht11_val;
    success = sht11_read_sensor(&sht11_val, TEMPERATURE);

    if (!success) {
        printf("Error reading SHT11\n");
    }
    else {
        printf("Temperature: ");
        _printfloat(sht11_val.temperature);
        puts("C");
    }
}

void _get_weather_handler(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    uint8_t success;
    sht11_val_t sht11_val;
    success = sht11_read_sensor(&sht11_val, HUMIDITY | TEMPERATURE);

    if (!success) {
        printf("Error reading SHT11\n");
    }
    else {
        printf("Relative humidity: ");
        _printfloat(sht11_val.relhum);
        printf("%% / Temperature compensated humidity: ");
        _printfloat(sht11_val.relhum_temp);
        printf("%%, Temperature: ");
        _printfloat(sht11_val.temperature);
        puts("C");
    }
}

void _set_offset_handler(int argc, char **argv)
{
    if (argc != 2) {
        printf("Temperature offset is: ");
        _printfloat(sht11_temperature_offset);
        puts("");
    }
    else {
        sht11_temperature_offset = atoi(argv[1]);
        printf("Temperature offset set to ");
        _printfloat(sht11_temperature_offset);
        puts("C");
    }
}
