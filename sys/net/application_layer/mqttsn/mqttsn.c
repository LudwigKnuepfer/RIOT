/*
 * Copyright 2015 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 * Copyright 2015 Cenk Gündoğan <cnkgndgn@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for
 * more details.
 */

/**
 * @{
 *
 * @file
 *
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 * @author      Cenk Gündoğan <cnkgndgn@gmail.com>
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "net/conn.h"
#include "net/mqttsn.h"
#include "net/af.h"
#include "net/conn/udp.h"
#include "net/addr.h"
#include "byteorder.h"

static conn_udp_t conn;

typedef enum {
    MQTTSN_TYPE_ADVERTISE       = 0x00,
    MQTTSN_TYPE_SEARCHGW        = 0x01,
    MQTTSN_TYPE_GWINFO          = 0x02,
    MQTTSN_TYPE_CONNECT         = 0x04,
    MQTTSN_TYPE_CONNACK         = 0x05,
    MQTTSN_TYPE_WILLTOPICREQ    = 0x06,
    MQTTSN_TYPE_WILLTOPIC       = 0x07,
    MQTTSN_TYPE_WILLMSGREQ      = 0x08,
    MQTTSN_TYPE_WILLMSG         = 0x09,
    MQTTSN_TYPE_REGISTER        = 0x0A,
    MQTTSN_TYPE_REGACK          = 0x0B,
    MQTTSN_TYPE_PUBLISH         = 0x0C,
    MQTTSN_TYPE_PUBACK          = 0x0D,
    MQTTSN_TYPE_PUBCOMP         = 0x0E,
    MQTTSN_TYPE_PUBREC          = 0x0F,
    MQTTSN_TYPE_PUBREL          = 0x10,
    MQTTSN_TYPE_SUBSCRIBE       = 0x12,
    MQTTSN_TYPE_SUBACK          = 0x13,
    MQTTSN_TYPE_UNSUBSCRIBE     = 0x14,
    MQTTSN_TYPE_UNSUBACK        = 0x15,
    MQTTSN_TYPE_PINGREQ         = 0x16,
    MQTTSN_TYPE_PINGRESP        = 0x17,
    MQTTSN_TYPE_DISCONNECT      = 0x18,
    MQTTSN_TYPE_WILLTOPICUPD    = 0x1A,
    MQTTSN_TYPE_WILLTOPICRESP   = 0x1B,
    MQTTSN_TYPE_WILLMSGUPD      = 0x1C,
    MQTTSN_TYPE_WILLMSGRESP     = 0x1D,
    MQTTSN_TYPE_ENCMSG          = 0xFE
} mqttsn_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t length;
    uint8_t msg_type;
    uint8_t flags;
    uint8_t prot_id;
    uint16_t duration;
} mqttsn_msg_connect;

typedef struct __attribute__((packed)) {
    uint8_t length;
    uint8_t msg_type;
    uint8_t ret_code;
} mqttsn_msg_connack;

int mqttsn_connect(mqttsn_state_t *mqtt, ipv6_addr_t address, char *client_id,
                   size_t client_id_len, uint16_t port, char *will_topic, char *will_msg,
                   uint16_t *will_id, int clean_session) {
    ipv6_addr_t src;
    uint16_t sport, rcv_port;
    ipv6_addr_t rcv_addr;

    size_t blen = sizeof(mqttsn_msg_connect) + client_id_len;
    char buf[blen];
    mqttsn_msg_connect *msg_conn = (mqttsn_msg_connect *) buf;

    msg_conn->msg_type = MQTTSN_TYPE_CONNECT;
    msg_conn->prot_id = 1;
    msg_conn->flags = 0;
    msg_conn->length = sizeof(buf);
    msg_conn->duration = HTONS(30U);
    memcpy((msg_conn + 1), client_id, client_id_len);

    conn_udp_create(&conn, &address, sizeof(address), AF_INET6, port);
    conn_udp_getlocaladdr(&conn, &src, &sport);
    conn_udp_sendto(buf, blen, src, sizeof(src), address, sizeof(address), AF_INET6, 0, port);
    int32_t rcv_size = conn_udp_recvfrom(&conn, (void *)buf, blen, &rcv_addr,
                                         sizeof(rcv_addr), &rcv_port);
}
/**
 * @}
 */
