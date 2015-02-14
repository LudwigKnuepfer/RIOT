/*
 * Copyright (C) 2013 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     core_internal
 * @{
 *
 * @file        kernel_init.c
 * @brief       Platform-independent kernel initilization
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include "tcb.h"
#include "kernel.h"
#include "kernel_internal.h"
#include "sched.h"
#include "flags.h"
#include "cpu.h"
#include "lpm.h"
#include "thread.h"
#include "hwtimer.h"
#include "irq.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef MODULE_AUTO_INIT
#include <auto_init.h>
#endif

volatile int lpm_prevent_sleep = 0;

extern void application_init(void);

static void *idle_thread(void *arg)
{
    (void) arg;

    while (1) {
        if (lpm_prevent_sleep) {
            lpm_set(LPM_IDLE);
        }
        else {
            lpm_set(LPM_IDLE);
            /* lpm_set(LPM_SLEEP); */
            /* lpm_set(LPM_POWERDOWN); */
        }
    }

    return NULL;
}

const char *idle_name = "idle";

static char idle_stack[KERNEL_CONF_STACKSIZE_IDLE];

void kernel_init(void)
{
    (void) disableIRQ();
    printf("kernel_init(): This is RIOT! (Version: %s)\n", RIOT_VERSION);

    hwtimer_init();

    if (thread_create(idle_stack, sizeof(idle_stack), PRIORITY_IDLE, CREATE_WOUT_YIELD | CREATE_STACKTEST, idle_thread, NULL, idle_name) < 0) {
        printf("kernel_init(): error creating idle task.\n");
    }

    application_init();

    printf("kernel_init(): jumping into first task...\n");

    cpu_switch_context_exit();
}
