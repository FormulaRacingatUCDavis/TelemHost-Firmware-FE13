/*
 * udp.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Vicle
 */

// https://controllerstech.com/udp-server-using-netconn-with-rtos-in-stm32/

#include "udp.h"
#include "string.h"

//static ip_addr_t *addr;
//static unsigned short port;
//char msg[100];

/*-----------------------------------------------------------------------------------*/
/**** Send RESPONSE every time the queue has some data ******/
void udp_update(struct netconn *conn, struct netbuf *buf, char* smsg, err_t *err, struct pbuf *txBuf)
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
