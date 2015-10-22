/*
 * Copyright (C) 2015 Cenk Gündoğan
 * Copyright (C) 2015 Ludwig Knüpfer
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
 * - connect (optional)
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
 * - disconnect (optional)
 * @code
 * if (mqttsn_disconnect(mqtt) == -1) handle_error();
 * @endcode
 *
 * ### Consumer
 * - subscribe to a topic
 * @code
 * if (mqttsn_subscribe(&mqtt, topic_id) == -1) handle_error();
 * @endcode
 *
 * - listen for publishes
 * @code
 * while (mqttsn_recv(&mqtt, &mqttsn_msg) != -1) {
 *     switch (mqttsn_msg.topic_id) {
 *         case (MQTTSN_TOPIC_LIGHTS_OUT):
 *             handle_lights_out();
 *             break;
 *         case (MQTTSN_TOPIC_LIGHTS_ON):
 *             handle_lights_on();
 *             break;
 *         default:
 *             break;
 *     }
 * }
 * @endcode
 *
 * ### Producer
 * - publish to a topic
 * @code
 * while (msg_receive(&msg)) {
 *     mqttsn_msg_t mqttsn_msg;
 *     switch (msg.type) {
 *         case (SENSOR_STATUS_HI):
 *             mqttsn_msg.topic_id = MQTTSN_TOPIC_SENSOR_HI;
 *             if (mqttsn_publish(&mqtt, &mqttsn_msg) == -1) {
 *                 handle_error();
 *             }
 *             break;
 *         case (SENSOR_STATUS_LOW):
 *             mqttsn_msg.topic_id = MQTTSN_TOPIC_SENSOR_LOW;
 *             if (mqttsn_publish(&mqtt, &mqttsn_msg) == -1) {
 *                 handle_error();
 *             }
 *             break;
 *         default:
 *             break;
 *     }
 * }
 * @endcode
 *
 * @{
 *
 * @file
 * @brief       MQTT-SN API declaration
 *
 * @author      Cenk Gündoğan <cnkgndgn@gmail.com>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 */

#ifndef __MQTTSN_H
#define __MQTTSN_H

#include <inttypes.h>

#include "msg.h"
#include "net/ipv6.h"
#include "net/conn/udp.h"

typedef enum {
    MQTTSN_QOS_AT_LEAST_ONCE,
    MQTTSN_QOS_AT_MOST_ONCE,
    MQTTSN_QOS_EXACTLY_ONCE
} mqttsn_qos_t;

typedef struct {
    int state;              /**< connected, ..? */
    conn_udp_t conn;        /**< connection state */
    char *client_id;        /**< id of the client */
    uint16_t port;          /**< UDP port */
    ipv6_addr_t address;    /**< address */
} mqttsn_state_t;

/**
 * @brief process a message from the network layer
 *
 * @param[in]   mqtt        mqtt connection object
 * @param[in]   msg         the message
 */
int mqttsn_process_msg(msg_t msg);

/**
 * @brief establish the connection
 *
 * @param[in]   mqtt        mqtt connection object
 * @param[in]   address     server address
 * @param[in]   port        server port
 * @param[in]   client_id   client identification string
 * @param[in]   client_id_len
 *                          length of the client identification string
 */
int mqttsn_connect(
        mqttsn_state_t *mqtt,
        ipv6_addr_t address,
        uint16_t port,
        char *client_id,
        size_t client_id_len
#if 0
 * @param[in]   will_msg    either a will message or NULL if no will is to
 *                          be set
 * @param[in]   will_topic  the topic to which the will_msg will be sent
 * @param[out]  will_id     will be set to the id of will_topic
        char *will_topic,
        char *will_msg,
        uint16_t *will_id,
        int clean_session,
#endif
        );

int mqttsn_disconnect(
        mqttsn_state_t *mqtt
        );

int mqttsn_register_topic(
        char *topic_name,
        uint16_t *topic_id
        );

int mqttsn_publish(mqttsn_state_t mqtt,
        uint16_t topic_id,
        mqttsn_qos_t level
        );

int mqttsn_subscribe(mqttsn_state_t mqtt,
        uint16_t topic_id
        );

int mqttsn_unsubscribe(
        uint16_t topic_id
        );

#endif /* __MQTTSN_H */
