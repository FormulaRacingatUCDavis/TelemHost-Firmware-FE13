/*
 * udp.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Vicle
 */

#ifndef INC_UDP_H_
#define INC_UDP_H_

#include "lwip.h"

#include "lwip/opt.h"

#include "lwip/api.h"
#include "lwip/sys.h"
#include <wolfmqtt/mqtt_client.h>

#define UDP_SERVER_PORT 7

typedef struct {
	char json_string[128];
	char topic_name[16];
} MQTTMessageFormat_t;



void MQTT_update(struct netconn *conn, struct netbuf *buf, char* smsg, err_t *err, struct pbuf *txBuf, osMessageQueueId_t MQTT_queueHandle, MqttPublish *publish, MqttClient *client);



#endif /* INC_UDP_H_ */
