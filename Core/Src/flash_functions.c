/*
 * flash_functions.c
 *
 *  Created on: Aug 17, 2023
 *      Author: SBrewerton
 */

#include "flash_functions.h"
#include "stm32l4xx_hal.h"


void erase_flash_sector(uint8_t sector_number) {
	uint32_t PAGEError = 0;
	uint32_t pageStartIdx = 0;
	uint32_t pageNumPages = 0;

	//sector is assumed to be 0x10000 in length
	//one page is 2kb = 0x800
	//one sector is 32 pages
	//flash offset is 0x08008000 for application, so sectors start there
	//each block is 256 pages of 2kb
	//sector 3 is used for fault memory and app calibration

	switch(sector_number) {
	case 0:
		pageStartIdx = SECTOR0_PAGE_START;
		pageNumPages = SECTOR0_PAGE_SIZE;
		break;
	case 1:
		pageStartIdx = SECTOR1_PAGE_START;
		pageNumPages = SECTOR1_PAGE_SIZE;
		break;
	case 2:
		pageStartIdx = SECTOR2_PAGE_START;
		pageNumPages = SECTOR2_PAGE_SIZE;
		break;
	case 3:
		pageStartIdx = SECTOR3_PAGE_START;
		pageNumPages = SECTOR3_PAGE_SIZE;
		break;
	}

	FLASH_EraseInitTypeDef EraseInitStruct;
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = pageStartIdx;  //starting page to erase
	EraseInitStruct.Banks = FLASH_BANK_1;  // can be FLASH_BANK_1 or FLASH_BANK_2
	EraseInitStruct.NbPages = pageNumPages;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);    //return HAL_OK if success
	HAL_FLASH_Lock();

}

void program_flash_double_word(uint32_t Address, uint64_t DataToProg64) {
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, DataToProg64);
	HAL_FLASH_Lock();
}

