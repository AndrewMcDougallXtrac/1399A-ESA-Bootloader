/*
 * memory_checks.c
 *
 *  Created on: Aug 18, 2023
 *      Author: SBrewerton
 */

#include "memory_checks.h"
#include "memory_layout.h"

uint32_t crc_start_address = ADDR_FLASH_BOOTLOADER_START;
uint32_t crc_end_address   = ADDR_FLASH_BOOTLOADER_END;
uint32_t crc_result   = 0;
HAL_CRC_StateTypeDef crc_state   = HAL_CRC_STATE_RESET;
CRC_HandleTypeDef *current_hcrc;


void Set_CRC_Instance(CRC_HandleTypeDef *hcrc_used) {
	current_hcrc = hcrc_used;
}

void Set_CRC_Start_Address(uint32_t address) {
	crc_start_address = address;
}

void Set_CRC_End_Address(uint32_t address) {
	crc_end_address = address;
}

void Start_CRC_Calculation (void) {
	// use the HAL to perform a CRC over the provided address ranges
	uint32_t crc_length   = crc_end_address - crc_start_address;
	crc_length = crc_length / 4; // range is defined in bytes, but length is define in 32bit words
	crc_result = HAL_CRC_Calculate(current_hcrc, (uint32_t*)crc_start_address, crc_length);
}

HAL_CRC_StateTypeDef Report_CRC_Progress (void) {
	// use the HAL to perform a CRC over the provided address ranges
	crc_state = HAL_CRC_GetState(current_hcrc);
	return(crc_state);
}

uint32_t Report_CRC_Result_Value (void) {
	// use the HAL to perform a CRC over the provided address ranges
	return(crc_result);
}
