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
