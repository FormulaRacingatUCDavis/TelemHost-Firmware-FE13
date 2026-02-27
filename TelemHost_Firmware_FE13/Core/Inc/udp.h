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

#define UDP_SERVER_PORT 7

void udp_update(struct netconn *conn, struct netbuf *buf, char* smsg, err_t *err, struct pbuf *txBuf);

typedef struct {
	uint8_t id;
	uint8_t data[8];
	uint8_t length;
} UDPMessageFormat_t;

#endif /* INC_UDP_H_ */
