/*
 * Copyright 2015 Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     core_log
 * @{
 *
 * @file
 * @brief       logging implementation that acts like printf
 *
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdarg.h>

#include "log.h"

void log_info(const char *format, ...)
{
    printf("I: ");

    va_list argp;
    va_start(argp, format);
    vprintf(format, argp);
    va_end(argp);
}

void log_warning(const char *format, ...)
{
    printf("W: ");

    va_list argp;
    va_start(argp, format);
    vprintf(format, argp);
    va_end(argp);
}

void log_error(const char *format, ...)
{
    printf("E: ");

    va_list argp;
    va_start(argp, format);
    vprintf(format, argp);
    va_end(argp);
}
