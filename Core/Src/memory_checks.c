/*
 * memory_checks.c
 *
 *  Created on: Aug 18, 2023
 *      Author: SBrewerton
 */

#include "memory_checks.h"
#include "memory_layout.h"

CRC_HandleTypeDef *current_hcrc  __attribute__((section (".crcHandleStorageSection"))); //placed at 0x20000000 to allow SRAM testing without corruption
uint32_t crc_start_address = ADDR_FLASH_BOOTLOADER_START;
uint32_t crc_end_address   = ADDR_FLASH_BOOTLOADER_END;
uint32_t crc_result   = 0;
HAL_CRC_StateTypeDef crc_state   = HAL_CRC_STATE_RESET;


uint8_t Do_Sram_Pattern_Test(CRC_HandleTypeDef *hcrc_used) {
	uint8_t test_result = SRAM_TEST_RESULT_UNKNOWN;
	uint32_t crc_length   = (ADDR_RAM_TEST_END - ADDR_RAM_TEST_START);
	uint32_t mem_addr;
	uint32_t *memPtr;
	memPtr = (uint32_t*)ADDR_RAM_TEST_START;
	current_hcrc = hcrc_used;

	crc_length = crc_length / 4; // range is defined in bytes, but length is define in 32bit words
	crc_length = crc_length /4; //algorithm tests 4 words at a time

	/* Reset CRC Calculation Unit (hcrc->Instance->INIT is
	*  written in hcrc->Instance->DR) */
	__HAL_CRC_DR_RESET(current_hcrc);

	current_hcrc->InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
	/* Change CRC peripheral state */
	current_hcrc->State = HAL_CRC_STATE_BUSY;


	//init crc peripheral input register
	current_hcrc->Instance->INIT = CRC_TEST_SEED;

	volatile uint32_t tempStorage[4]; //volatile needed to ensure write are not optimised away
	for(mem_addr = 0; mem_addr < crc_length; mem_addr++) {
		//loop through SRAM memory and fill it with test pattern
		//take a snapshot so as not to destroy data
		tempStorage[0] = *(memPtr+0);
		tempStorage[1] = *(memPtr+1);
		tempStorage[2] = *(memPtr+2);
		tempStorage[3] = *(memPtr+3);
		//write test pattern
		*(memPtr+0) = CRC_TEST_SEED;
		*(memPtr+1) = ~CRC_TEST_SEED;
		*(memPtr+2) = CRC_TEST_SEED;
		*(memPtr+3) = ~CRC_TEST_SEED;
		//CRC memory to ensure it was written
		current_hcrc->Instance->DR = *(memPtr+0);
		current_hcrc->Instance->DR = *(memPtr+1);
		current_hcrc->Instance->DR = *(memPtr+2);
		current_hcrc->Instance->DR = *(memPtr+3);
		//write inverted test pattern
		*(memPtr+0) = ~CRC_TEST_SEED;
		*(memPtr+1) = CRC_TEST_SEED;
		*(memPtr+2) = ~CRC_TEST_SEED;
		*(memPtr+3) = CRC_TEST_SEED;
		//CRC memory to ensure it was written
		current_hcrc->Instance->DR = *(memPtr+0);
		current_hcrc->Instance->DR = *(memPtr+1);
		current_hcrc->Instance->DR = *(memPtr+2);
		current_hcrc->Instance->DR = *(memPtr+3);
		//restore the snapshot data
		*(memPtr+0) = tempStorage[0];
		*(memPtr+1) = tempStorage[1];
		*(memPtr+2) = tempStorage[2];
		*(memPtr+3) = tempStorage[3];
		memPtr += 4;
	}
	/* Change CRC peripheral state */
	current_hcrc->State = HAL_CRC_STATE_READY;
	//read back the SRAM memory and ensure it weas all written
	crc_result = current_hcrc->Instance->DR;
	if(crc_result == SRAM_CRC_EXPECTED_RESULT) {
		test_result = SRAM_TEST_RESULT_PATTERN_PASS;
	} else {
		test_result = SRAM_TEST_RESULT_PATTERN_FAIL;
	}
	return test_result;
}

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
	//init crc peripheral input register
	current_hcrc->Instance->INIT = CRC_TEST_SEED;
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
