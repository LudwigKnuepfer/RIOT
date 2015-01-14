/*
 * Copyright (C) 2015 Cenk Gündoğan
 * Copyright (C) 2015 Ludwig Ortmann
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    mqttsn MQTT-SN - MQTT For Sensor Networks
 * @ingroup     net
 * @brief       MQTT-SN is an adaption of MQTT for sensor networks
 *
 * MQTT-SN caters to the needs of wireless sensor networks by reducing
 * message sizes and implementation state size reduction.
 *
 * ## Gateway
 * In order to connect to a regular MQTT broker, MQTT-SN needs a
 * gateway which converts between MQTT-SN and MQTT message formats and
 * saves some of the expensive state information.
 * The Mosqitto project includes an implementation of such a gateway:
 * http://www.eclipse.org/proposals/technology.mosquitto/
 *
 * ## Topic IDs
 * With MQTT-SN, topic ids are used instead of topic strings for
 * publish/subscribe messages. These topic ids can either be
 * dynamically retrieved from the gateway, or they can be prearranged.
 * In the latter case, the gateway would need to know the topic strings
 * which are going to be used. The corresponding topic ids the MQTT-SN
 * clients will use would need to be known at compile time, for example
 * through a macro definition the application uses as a topic_id for
 * publish/subscribe messages.
 *
 * ## MQTT-SN Specification
 * This implementation is based on version 1.2 of the MQTT-SN
 * specification:
 * http://mqtt.org/new/wp-content/uploads/2009/06/MQTT-SN_spec_v1.2.pdf
 *
 * ## API Usage
 * The workflow with this API is as follows:
 * - create and initialize the mqtt handle
 * @code
 * mqttsn_state_t mqtt;
 * if (mqttsn_init(&mqtt, ...) == -1) handle_error();
 * @endcode
 *
 * - connect
 * @code
 * if (mqttsn_connect(&mqtt, ...) == -1) handle_error();
 * @endcode
 *
 * - initialize a topic (optional)
 * @code
 * uint16_t topic_id;
 * if (mqttsn_register_topic(&mqtt, "example/topic") == -1) handle_error();
 * @endcode
 *
 * - subscribe to a topic... (optional)
 * @code
 * if (mqttsn_subscribe(&mqtt, topic_id) == -1) handle_error();
 * @endcode
 *
 * - .. and/or publish to a topic (optional)
 * @code
 * if (mqttsn_publish(&mqtt, topic_id, ...) == -1) handle_error();
 * @endcode
 *
 * - disconnect (optional)
 * @code
 * if (mqttsn_disconnect(mqtt) == -1) handle_error();
 * @endcode
 *
 * @{
 *
 * @file
 * @brief       MQTT-SN API declaration
 *
 * @author      Cenk Gündoğan <cnkgndgn@gmail.com>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 */

#ifndef __MQTTSN_H
#define __MQTTSN_H

#include <inttypes.h>

#include "msg.h"
// #include "ipv6.h" // where is it?
#include "socket_base/socket.h"

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

typedef enum {
    MQTTSN_QOS_AT_LEAST_ONCE,
    MQTTSN_QOS_AT_MOST_ONCE,
    MQTTSN_QOS_EXACTLY_ONCE
} mqttsn_qos_t;

typedef struct {
    int state;              /**< connected, ..? */
    uint16_t port;          /**< UDP port */
    ipv6_addr_t address;    /**< address */
    int socket;             /**< socket, -1 if not connected */
    /**< topic_foo (support for several in parallel?) */
} mqttsn_state_t;

/**
 * @brief process a message from the network layer
 *
 * @param[in]   mqtt        mqtt connection object
 * @param[in]   msg         the message
 */
int mqttsn_process_msg(
        msg_t msg
        );

/**
 * @brief initialize an mqtt-sn state object
 *
 * @param[out]  mqtt        mqtt connection object
 * @param[in]   port        remote port
 * @param[in]   address     remote address
 */
int mqttsn_init(
        mqttsn_state_t *mqtt,
        uint16_t port,
        ipv6_addr_t address
        );

/**
 * @brief establish the connection
 *
 * @param[in]   mqtt        mqtt connection object
 * @param[in]   will_msg    either a will message or NULL if no will is to
 *                          be set
 * @param[in]   will_topic  the topic to which the will_msg will be sent
 * @param[out]  will_id     will be set to the id of will_topic
 */
int mqttsn_connect(
        mqttsn_state_t *mqtt,
        char *will_topic,
        char *will_msg,
        uint16_t *will_id,
        int clean_session,
        );

int mqttsn_disconnect(
        mqttsn_state_t *mqtt
        );

int mqttsn_register_topic(
        char *topic_name,
        uint16_t *topic_id,
        );

int mqttsn_publish(mqttsn_state_t mqtt,
        uint16_t topic_id,
        mqttsn_qos_t level
        );

int mqttsn_subscribe(mqttsn_state_t mqtt,
        uint16_t topic_id,
        );

int mqttsn_unsubscribe(
        uint16_t topic_id,
        );

#endif __MQTTSN_H
