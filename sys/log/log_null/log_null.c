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
 * @brief       logging implementation that does nothing
 *
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */


#include "log.h"

void log_info(const char *format, ...)
{
    (void) format;
}

void log_warning(const char *format, ...)
{
    (void) format;
}

void log_error(const char *format, ...)
{
    (void) format;
}
