/*
 * Copyright (C) 2015 Cenk Gündoğan
 * Copyright (C) 2015 Ludwig Knüpfer
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
 * @brief       Example application for mqtt-sn
 *
 * @author      Cenk Gündoğan <cnkgndgn@gmail.com>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "net/mqttsn.h"
#include "shell.h"
#include "msg.h"

#include "mqtt_shell.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

static const shell_command_t shell_commands[] = {
#if 0
    {"sub", "subscribe to topic", mqtt_shell_subscribe},
    {"pub", "publish message to topic", mqtt_shell_publish},
    {"con", "connect to a server", mqtt_shell_connect},
    {"dis", "disconnect from server", mqtt_shell_disconnect},
#endif
    {NULL, NULL, NULL}
};

mqttsn_state_t mqtt_state;

int main(void)
{
    puts("RIOT MQTT-SN example application");

    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    /* start shell */
    puts("Starting the shell");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* not reached */
    return 0;
}
