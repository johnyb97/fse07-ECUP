#include "flash.h"
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_flash.h>
#include <string.h>


#define FLASH_ADRESS 0x0803F800
uint32_t flashadress = FLASH_ADRESS; //adresa flash vypocitat z manualu


bool FlashErasePage( uint8_t* pageAddress) {
    FLASH_EraseInitTypeDef erase;
    uint32_t error;

    __disable_irq();
    HAL_FLASH_Unlock();
    FLASH->SR = FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = (uint32_t) pageAddress;
    erase.NbPages = 1;
    HAL_FLASHEx_Erase(&erase, &error);
    __enable_irq();

    HAL_FLASH_Lock();
    return true;
}

bool FlashWrite(uint8_t* flashAddress, const void* data, size_t length) {
    const size_t wordsToFlush = (length + 3) / 4;

    // FIXME: check page overflow
    //if ((uintptr_t)s_pointer + wordsToFlush * 4 >= s_readAddress + FLIGHT_DIARY_AREA_SIZE)
    //	return;

    __disable_irq();
    HAL_FLASH_Unlock();
    FLASH->SR = FLASH_SR_EOP | FLASH_FLAG_PGERR;

    const uint32_t* words = (uint32_t*) data;

    for (size_t i = 0; i < wordsToFlush; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)flashAddress, *words++);
        flashAddress += 4;
    }

    __enable_irq();
    HAL_FLASH_Lock();
    return true;
}

void load_flash(Calib_t* calibrate){
		memcpy(calibrate,(uint8_t*) flashadress, sizeof(Calib_t)); //flash nacitani magic
}

void write_flash(Calib_t* calibrate){
	FlashErasePage((uint8_t*)flashadress);
	FlashWrite((uint8_t*)flashadress, calibrate, sizeof(Calib_t));//zapsat svoji strukturu magic
}
