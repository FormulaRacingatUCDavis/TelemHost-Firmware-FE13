/*
 * udp.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Vicle
 */

// https://controllerstech.com/udp-server-using-netconn-with-rtos-in-stm32/

#include "udp.h"
#include "string.h"
#include "cmsis_os2.h"
#include <wolfmqtt/mqtt_client.h>
#include "mqtt_conn.h"
#include "xsens/xsens_mti.h"
#include <string.h>

//MQTT init and connect

//#include "main.h"

// Size of the transmit string buffer we expect callers to provide (smsg).

#define UDP_TX_SMSG_LEN 200


word16 packet_id = 0;
/*-----------------------------------------------------------------------------------*/
/**** Send RESPONSE every time the queue has some data ******/
void MQTT_update(struct netconn *conn, struct netbuf *buf, char *smsg, err_t *err, struct pbuf *txBuf, osMessageQueueId_t MQTT_queueHandle, MqttPublish *publish, MqttClient *client)
{
	MQTTMessageFormat_t receivedData;
	smsg[0] = '\0'; // should get overwritten immediately

	osStatus_t status = osMessageQueueGet(*MQTT_queueHandle, &receivedData, NULL, 0U);
	osStatus_t status = osMessageQueueGet(MQTT_queueHandle, &receivedData, NULL, 0);

	if (status == osOK)
	{
		publish->buffer_len = strnlen(receivedData.json_string, 128);

		publish->packet_id = ++packet_id;

		strncpy(smsg, receivedData.json_string, publish->buffer_len);
		smsg[publish->buffer_len] = '\0'; // strncpy DOES NOT automatically null-terminate

		publish->topic_name = receivedData.topic_name;

		//PUBLISH DATA
		int rc = MqttClient_Publish(client, publish);

		if (rc != MQTT_CODE_SUCCESS) {
			//error
		}

	}
}






