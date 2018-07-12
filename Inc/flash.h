#ifndef FLASH_HEADER
#define FLASH_HEADER
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct{
	uint32_t magic; // komntrola flash zda li jsuo tam spravne data
	uint8_t validity [4];
	uint16_t min_val_acc[2];
	uint16_t max_val_acc[2];
	uint16_t min_val_breake[2];
	uint16_t max_val_breake[2];
}Calib_t;

bool FlashErasePage( uint8_t* pageAddress);
bool FlashWrite(uint8_t* flashAddress, const void* data, size_t length);
void load_flash(Calib_t* calibrate);
void write_flash(Calib_t* calibrate);

#endif

