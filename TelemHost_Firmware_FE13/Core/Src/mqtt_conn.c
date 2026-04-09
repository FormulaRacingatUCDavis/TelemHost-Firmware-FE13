/*
 * mqtt_conn.c
 *
 * MQTT connection management using wolfMQTT library with LwIP BSD sockets.
 *
 * Provides the four MqttNet callbacks (connect, read, write, disconnect)
 * that wolfMQTT needs, using lwip/sockets.h (BSD socket API).
 *
 * wolfMQTT config is auto-loaded from wolfSSL.I-CUBE-wolfMQTT_conf.h
 * which sets: FREERTOS, WOLFSSL_LWIP, HAVE_SOCKET, WOLFMQTT_V5, no TLS.
 */

#include "mqtt_conn.h"
#include "wolfmqtt/mqtt_client.h"

/* LwIP BSD sockets */
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <string.h>

/* ================================================================== */
/*  Private state                                                      */
/* ================================================================== */

static byte tx_buf[MQTT_TX_BUF_SIZE];
static byte rx_buf[MQTT_RX_BUF_SIZE];

static volatile int mqtt_connected = 0;

/* ================================================================== */
/*  LwIP Socket Network Callbacks for wolfMQTT                        */
/* ================================================================== */

/**
 * @brief  Open a TCP socket and connect to the broker.
 */
static int mqtt_net_connect_cb(void *context,
    const char *host, word16 port, int timeout_ms)
{
    MqttNetCtx *ctx = (MqttNetCtx *)context;
    struct sockaddr_in addr;
    int rc;

    (void)timeout_ms;

    /* Create TCP socket */
    ctx->fd = socket(AF_INET, SOCK_STREAM, 0);
    if (ctx->fd < 0) {
        return MQTT_CODE_ERROR_NETWORK;
    }

    /* Set up server address */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);

    /* Convert IP string to binary */
    addr.sin_addr.s_addr = ipaddr_addr(host);
    if (addr.sin_addr.s_addr == IPADDR_NONE) {
        close(ctx->fd);
        ctx->fd = -1;
        return MQTT_CODE_ERROR_NETWORK;
    }

    /* Connect */
    rc = connect(ctx->fd, (struct sockaddr *)&addr, sizeof(addr));
    if (rc != 0) {
        close(ctx->fd);
        ctx->fd = -1;
        return MQTT_CODE_ERROR_NETWORK;
    }

    return MQTT_CODE_SUCCESS;
}

/**
 * @brief  Read data from the TCP socket.
 *         Must return the number of bytes read, or a negative error code.
 */
static int mqtt_net_read_cb(void *context,
    byte *buf, int buf_len, int timeout_ms)
{
    MqttNetCtx *ctx = (MqttNetCtx *)context;
    int rc;

    /* Set receive timeout */
    if (timeout_ms > 0) {
        struct timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(ctx->fd, SOL_SOCKET, SO_RCVTIMEO,
                   (const void *)&tv, sizeof(tv));
    }

    rc = (int)recv(ctx->fd, (char *)buf, (size_t)buf_len, 0);
    if (rc == 0) {
        /* Connection closed */
        return MQTT_CODE_ERROR_NETWORK;
    }
    if (rc < 0) {
        /* Check for timeout vs real error */
        int err = errno;
        if (err == EWOULDBLOCK || err == EAGAIN) {
            return MQTT_CODE_ERROR_TIMEOUT;
        }
        return MQTT_CODE_ERROR_NETWORK;
    }

    return rc;  /* bytes read */
}

/**
 * @brief  Write data to the TCP socket.
 *         Must return the number of bytes written, or a negative error code.
 */
static int mqtt_net_write_cb(void *context,
    const byte *buf, int buf_len, int timeout_ms)
{
    MqttNetCtx *ctx = (MqttNetCtx *)context;
    int rc;

    (void)timeout_ms;

    rc = (int)send(ctx->fd, (const char *)buf, (size_t)buf_len, 0);
    if (rc < 0) {
        return MQTT_CODE_ERROR_NETWORK;
    }

    return rc;  /* bytes written */
}

/**
 * @brief  Close the TCP socket.
 */
static int mqtt_net_disconnect_cb(void *context)
{
    MqttNetCtx *ctx = (MqttNetCtx *)context;

    if (ctx->fd >= 0) {
        close(ctx->fd);
        ctx->fd = -1;
    }

    return MQTT_CODE_SUCCESS;
}

/* ================================================================== */
/*  wolfMQTT message callback                                          */
/* ================================================================== */

/**
 * @brief  Called by wolfMQTT when an incoming PUBLISH arrives.
 *         Stub for now — fill in when data rx is needed.
 */
static int mqtt_message_cb(MqttClient *client, MqttMessage *msg,
    byte msg_new, byte msg_done)
{
    (void)client;
    (void)msg;
    (void)msg_new;
    (void)msg_done;
    return MQTT_CODE_SUCCESS;
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

int mqtt_conn_init(void)
{
    int rc;

    /* --- 1. Set up network callbacks --- */
    memset(&mqtt_net_ctx, 0, sizeof(mqtt_net_ctx));
    mqtt_net_ctx.fd = -1;

    memset(&mqtt_net, 0, sizeof(mqtt_net));
    mqtt_net.connect    = mqtt_net_connect_cb;
    mqtt_net.read       = mqtt_net_read_cb;
    mqtt_net.write      = mqtt_net_write_cb;
    mqtt_net.disconnect = mqtt_net_disconnect_cb;
    mqtt_net.context    = &mqtt_net_ctx;

    /* --- 2. Initialize wolfMQTT client --- */
    rc = MqttClient_Init(&mqtt_client, &mqtt_net,
                          mqtt_message_cb,
                          tx_buf, MQTT_TX_BUF_SIZE,
                          rx_buf, MQTT_RX_BUF_SIZE,
                          MQTT_CMD_TIMEOUT_MS);
    if (rc != MQTT_CODE_SUCCESS) {
        return rc;
    }

    /* --- 3. TCP connect (no TLS) --- */
    rc = MqttClient_NetConnect(&mqtt_client,
                                MQTT_BROKER_HOST,
                                MQTT_BROKER_PORT,
                                MQTT_CMD_TIMEOUT_MS,
                                0,     /* use_tls = 0 */
                                NULL); /* no TLS callback */
    if (rc != MQTT_CODE_SUCCESS) {
        return rc;
    }

    /* --- 4. MQTT CONNECT handshake --- */
    memset(&mqtt_connect, 0, sizeof(mqtt_connect));
    mqtt_connect.keep_alive_sec = MQTT_KEEP_ALIVE_S;
    mqtt_connect.clean_session  = 1;
    mqtt_connect.client_id      = MQTT_CLIENT_ID;
    /* mqtt_connect.username = "user";  -- optional */
    /* mqtt_connect.password = "pass";  -- optional */

    rc = MqttClient_Connect(&mqtt_client, &mqtt_connect);
    if (rc != MQTT_CODE_SUCCESS) {
        MqttClient_NetDisconnect(&mqtt_client);
        return rc;
    }

    mqtt_connected = 1;
    return MQTT_CODE_SUCCESS;
}

int mqtt_conn_is_connected(void)
{
    return mqtt_connected;
}

void mqtt_conn_disconnect(void)
{
    if (mqtt_connected) {
        MqttClient_Disconnect(&mqtt_client);
        MqttClient_NetDisconnect(&mqtt_client);
        MqttClient_DeInit(&mqtt_client);
        mqtt_connected = 0;
    }
}
