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


extern void Set_CRC_Instance(CRC_HandleTypeDef *hcrc_used);
extern void Set_CRC_Start_Address(uint32_t address);
extern void Set_CRC_End_Address(uint32_t address);
extern void Start_CRC_Calculation (void);
extern HAL_CRC_StateTypeDef Report_CRC_Progress (void);
extern uint32_t Report_CRC_Result_Value (void);

#endif /* INC_MEMORY_CHECKS_H_ */
