/*
 * sd_card.c
 *
 *  Created on: May 20, 2024
 *      Author: Abhineet, IanKM
 */

#include "sd_card.h"
#include "stdio.h"
#include "semphr.h"
#include "string.h"

#define BUFLEN 8192
#define ENTRY_SIZE (4 + 8 + 4)

#define MAX_ASYNC_WRITES 3

extern FATFS SDFatFS;
extern FIL SDFile;

static char buffer[BUFLEN];
static uint32_t buffer_size = 0;

static uint32_t writes_since_flush = 0;

/*
 * SD Card data is usually stored in .biscuit files,
 * which can be trivially converted to plaintext for further
 * analysis.
 */

typedef struct {
	uint32_t magic; // Should equal 0xB155CC17
	uint16_t version; // Should equal 1 for now
	uint16_t padding; // Not needed
} biscuit_header_t;

static void sd_card_flush_internal(void);
static void sd_card_write_from_buffer(void);
static void sd_card_write_data_bytes(uint8_t* bytes, uint32_t count);

sd_card_mount_result_t sd_card_mount(void) {
	FRESULT res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	if (res == FR_OK) {
		char filename[20];

		/* name the file, increment until filename hasn't been taken */
	  	 uint8_t num = 0;
	  	 while (1) {
	  		  FIL F1;

	  		  sprintf(filename, "run_%u.txt", num);

	  		  FRESULT f_open_status = f_open(&F1, filename, FA_READ);

	  		  /* if found filename thats not taken, use it */
	  		  if (f_open_status == FR_NO_FILE) {
	  			  f_close(&F1);
	  			  break;
	  		  }

	  		  ++num;
	  		  f_close(&F1);
	  	 }

		 res = f_open(&SDFile, filename,  FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_WRITE);
		 if (res == FR_OK) {
			 // Write the header
			 biscuit_header_t header = {
					 .magic = 0xB155CC17,
					 .version = 1,
					 .padding = 0
			 };
			 sd_card_write_data_bytes((uint8_t*)(&header), sizeof(header));
			 sd_card_write_from_buffer(); // Write out immediately
			 sd_card_flush_internal();
		 }
	}

	return (res == FR_OK) ? SD_CARD_MOUNT_RESULT_SUCCESS : SD_CARD_MOUNT_RESULT_FAILED;
}

void sd_card_write_data(uint32_t id, uint8_t data[]) {
	uint32_t tick;

	// make sure we don't reach the end of the buffer
	if ((buffer_size + ENTRY_SIZE) >= BUFLEN) {
		sd_card_write_from_buffer();
	}

	// Write the ids
	buffer[buffer_size] = id & 0xFF;
	buffer[buffer_size + 1] = (id >> 8) & 0xFF;
	buffer[buffer_size + 2] = (id >> 16) & 0xFF;
	buffer[buffer_size + 3] = (id >> 24) & 0xFF;

	// Now write the data
	for (int32_t i = 0; i < 8; ++i)
		buffer[buffer_size + 4 + i] = data[i];

	// Now write the tick
	tick = HAL_GetTick();
	buffer[buffer_size + 12] = tick & 0xFF;
	buffer[buffer_size + 13] = (tick >> 8) & 0xFF;
	buffer[buffer_size + 14] = (tick >> 16) & 0xFF;
	buffer[buffer_size + 15] = (tick >> 24) & 0xFF;

	buffer_size += ENTRY_SIZE;
}

void sd_card_write_can_rx(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]) {
	sd_card_write_data(rxHeader.StdId, rxData);
}

void sd_card_write_can_tx(CAN_TxHeaderTypeDef txHeader, uint8_t txData[]) {
	sd_card_write_data(txHeader.StdId, txData);
}

void sd_card_update_sync(void) {
	sd_card_write_from_buffer();
	sd_card_flush_internal();
}

void sd_card_update_async(void) {
	sd_card_write_from_buffer();

	/* If we've gone too long without syncing, force a flush */
	if (writes_since_flush == MAX_ASYNC_WRITES) {
		sd_card_flush_internal();
	}
}

/* Public variant, locks mutex */
void sd_card_flush(void) {
	sd_card_flush_internal();
}

/* Only to be used if mutex is active. */
static void sd_card_flush_internal(void) {
	f_sync(&SDFile);
	writes_since_flush = 0;
}

static void sd_card_write_data_bytes(uint8_t* bytes, uint32_t count) {
	// make sure we don't reach the end of the buffer
	if ((buffer_size + count) >= BUFLEN) {
		sd_card_write_from_buffer();
	}

	memcpy(buffer + buffer_size, bytes, count);
	buffer_size += count;
}

static void sd_card_write_from_buffer(void) {
	static UINT bytes_written = 0;

	if (buffer_size == 0) return;

	f_write(&SDFile, buffer, buffer_size, &bytes_written);

	++writes_since_flush;

#ifdef DEBUG
	if (bytes_written != buffer_size) {
		// TODO: mark error
	}
#endif

	buffer_size = 0;

}
