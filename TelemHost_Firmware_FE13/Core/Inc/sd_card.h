#ifndef __CAN_SD_H__
#define __CAN_SD_H__

#include "fatfs.h"
#include "stm32f7xx_hal.h"

typedef enum {
	SD_CARD_MOUNT_RESULT_SUCCESS = 0,
	SD_CARD_MOUNT_RESULT_FAILED = 1,
} sd_card_mount_result_t;

/* Initialization */
sd_card_mount_result_t sd_card_mount(void);

/* Writing functions */
void sd_card_write_data(uint32_t id, uint8_t data[]);
void sd_card_write_can_rx(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]);
void sd_card_write_can_tx(CAN_TxHeaderTypeDef txHeader, uint8_t txData[]);

/* Filesystem */
void sd_card_update_sync(void);
void sd_card_update_async(void);
void sd_card_flush(void);

#endif
