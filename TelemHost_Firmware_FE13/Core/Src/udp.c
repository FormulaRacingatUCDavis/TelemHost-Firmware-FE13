/*
 * udp.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Vicle
 */

// https://controllerstech.com/udp-server-using-netconn-with-rtos-in-stm32/

#include "lwip.h"

#include "lwip/opt.h"

#include "lwip/api.h"
#include "lwip/sys.h"

#include "udp.h"
#include "string.h"

#define UDP_SERVER_PORT 7

static struct netconn *conn;
static struct netbuf *buf;
//static ip_addr_t *addr;
//static unsigned short port;
char msg[100];
char smsg[200];

/*-----------------------------------------------------------------------------------*/
/**** Send RESPONSE every time the client sends some data ******/
static void udp_thread(void *arg)
{
	err_t err;
	//err_t recv_err;
	struct pbuf *txBuf;

	/* Create a new connection identifier */
	conn = netconn_new(NETCONN_UDP);

	if (conn!= NULL)
	{
		/* Bind connection to the port 7 */
		err = netconn_bind(conn, IP_ADDR_ANY, UDP_SERVER_PORT);

		if (err == ERR_OK)
		{
			/* The while loop will run everytime this Task is executed */
			while (1)
			{
				if (1) // TODO if data is available in the queue
				{
					// allocate new packet buffer
					buf = netbuf_new();

					if (buf != NULL) {
						// prepare message to send
						int len = sprintf(smsg, "TODO add message \n");

						/* allocate pbuf from RAM*/
						txBuf = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_RAM);

						/* copy the data into the buffer  */
						pbuf_take(txBuf, smsg, len);

						// refer the netbuf->pbuf to our pbuf
						buf->p = txBuf;

						// broadcast the message
						netconn_sendto(conn, buf, IP_ADDR_BROADCAST, UDP_SERVER_PORT);

						buf->addr.addr = 0;  // clear the address
						pbuf_free(txBuf);   // clear the pbuf
						netbuf_delete(buf);  // delete the netbuf
					}
				}
			}
		}
		else
		{
			netconn_delete(conn);
		}
	}
}
