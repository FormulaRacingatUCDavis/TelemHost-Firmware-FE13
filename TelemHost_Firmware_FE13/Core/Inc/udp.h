/*
 * udp.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Vicle
 */

#ifndef INC_UDP_H_
#define INC_UDP_H_

static void udp_thread(void *arg);

typedef struct {
	uint8_t id;
	uint8_t data[8];
	uint8_t length;
} UDPMessageFormat_t;

#endif /* INC_UDP_H_ */
