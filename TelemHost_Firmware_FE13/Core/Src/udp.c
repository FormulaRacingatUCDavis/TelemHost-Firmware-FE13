/*
 * udp.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Vicle
 */

// https://controllerstech.com/udp-server-using-netconn-with-rtos-in-stm32/

#include "udp.h"
#include "string.h"
#include "cmsis_os.h"
#include <wolfmqtt/mqtt_client.h>
#include "mqtt_conn.h"

//MQTT init and connect

//#include "main.h"

// Size of the transmit string buffer we expect callers to provide (smsg).

#define UDP_TX_SMSG_LEN 200


word16 packet_id = 0;
/*-----------------------------------------------------------------------------------*/
/**** Send RESPONSE every time the queue has some data ******/
void MQTT_update(struct netconn *conn, struct netbuf *buf, char *smsg, err_t *err, struct pbuf *txBuf, osMessageQueueId_t *MQTT_queueHandle, MqttPublish *publish, MqttClient *client)
{
//smsg is a pointer to the specific character??????
	MQTTMessageFormat_t recievedData;
	smsg[0] = '\0';
	int index = 0;

	osStatus_t status = osMessageQueueGet(*MQTT_queueHandle, &recievedData, NULL, 0U);

	if (status == osOK)
	{

		index = sprintf(smsg, "ID: %d, Value:\n", recievedData.id);
		for (int j = 0; j < recievedData.length; j++) {
			index += sprintf(smsg + index, "%d", recievedData.data[j]);
		}
			index += sprintf(smsg + index, "\n");

		}

	publish->buffer_len = index;

	publish->packet_id = ++packet_id;

	//PUBLISH DATA
	int rc = MqttClient_Publish(client, publish);

	if (rc != MQTT_CODE_SUCCESS) {
	//error
	}
}

