/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "thread.h"
#include "auto_init.h"

static char main_stack[KERNEL_CONF_STACKSIZE_MAIN];

void* main_thread(void *arg)
{
    (void) arg;

#ifdef MODULE_AUTO_INIT
    auto_init();
#endif

    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

    return 0;
}

void application_init(void)
{
    if (thread_create(main_stack, sizeof(main_stack), PRIORITY_MAIN, CREATE_WOUT_YIELD | CREATE_STACKTEST, main_thread, NULL, "main") < 0) {
        printf("application_init(): error creating thread.\n");
    }
}
