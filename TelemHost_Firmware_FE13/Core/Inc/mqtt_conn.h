/*
 * mqtt_conn.h
 *
 * MQTT connection management using wolfMQTT + LwIP sockets.
 *
 * Before building, add these source files to your STM32CubeIDE project:
 *   Middlewares/Third_Party/wolfSSL_wolfMQTT_wolfMQTT/wolfmqtt/src/mqtt_client.c
 *   Middlewares/Third_Party/wolfSSL_wolfMQTT_wolfMQTT/wolfmqtt/src/mqtt_packet.c
 *   Middlewares/Third_Party/wolfSSL_wolfMQTT_wolfMQTT/wolfmqtt/src/mqtt_socket.c
 */

#ifndef INC_MQTT_CONN_H_
#define INC_MQTT_CONN_H_

#include "wolfmqtt/mqtt_client.h"

/* ---------- Configuration ---------- */

/* Broker address (IP string for LwIP DNS or raw IP) */
#define MQTT_BROKER_HOST    "192.168.1.100"

/* Broker port (1883 for plain MQTT) */
#define MQTT_BROKER_PORT    1883

/* Client identification */
#define MQTT_CLIENT_ID      "TelemHost_FE13"

/* Keep-alive interval in seconds (0 = disabled) */
#define MQTT_KEEP_ALIVE_S   60

/* Command timeout in milliseconds */
#define MQTT_CMD_TIMEOUT_MS 5000

/* TX/RX buffer sizes */
#define MQTT_TX_BUF_SIZE    512
#define MQTT_RX_BUF_SIZE    512

/* ---------- API ---------- */

/* Socket context passed through MqttNet.context */
typedef struct _MqttNetCtx {
    int fd;  /* LwIP socket file descriptor */
} MqttNetCtx;

MqttClient  mqtt_client;
MqttNet     mqtt_net;
MqttNetCtx  mqtt_net_ctx;
MqttConnect mqtt_connect;

/**
 * @brief  Initialize wolfMQTT client, open TCP socket, and perform
 *         MQTT CONNECT handshake with the broker.
 *         Must be called AFTER MX_LWIP_Init() and once the network is up.
 * @retval  MQTT_CODE_SUCCESS (0) on success, negative on error
 */
int mqtt_conn_init(void);

/**
 * @brief  Returns 1 if the MQTT client is currently connected, 0 otherwise.
 */
int mqtt_conn_is_connected(void);

/**
 * @brief  Send MQTT DISCONNECT, close TCP socket, and free resources.
 */
void mqtt_conn_disconnect(void);

#endif /* INC_MQTT_CONN_H_ */
