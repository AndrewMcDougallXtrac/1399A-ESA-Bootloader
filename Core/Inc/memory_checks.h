/*
 * memory_checks.h
 *
 *  Created on: Aug 18, 2023
 *      Author: SBrewerton
 */

#ifndef INC_MEMORY_CHECKS_H_
#define INC_MEMORY_CHECKS_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define SRAM_TEST_RESULT_UNKNOWN			0x55u
#define SRAM_TEST_RESULT_PATTERN_FAIL		0xFFu
#define SRAM_TEST_RESULT_PATTERN_PASS		0xAAu
#define SRAM_TEST_RESULT_PASS				0xAAu

extern void Set_CRC_Instance(CRC_HandleTypeDef *hcrc_used);
extern void Set_CRC_Start_Address(uint32_t address);
extern void Set_CRC_End_Address(uint32_t address);
extern void Start_CRC_Calculation (void);
extern HAL_CRC_StateTypeDef Report_CRC_Progress (void);
extern uint32_t Report_CRC_Result_Value (void);
uint8_t Do_Sram_Pattern_Test(CRC_HandleTypeDef *hcrc_used);


#endif /* INC_MEMORY_CHECKS_H_ */
