/*
 * flash_functions.h
 *
 *  Created on: Aug 17, 2023
 *      Author: SBrewerton
 */

#ifndef INC_FLASH_FUNCTIONS_H_
#define INC_FLASH_FUNCTIONS_H_

#include <stdint.h>

#define SECTOR0_PAGE_START	16u  // 0x08008000, is 0x8000  from flash base address, so 16 x 2Kb pages up
#define SECTOR1_PAGE_START	32u  // 0x08010000, is 0x10000 from flash base address, so 32 x 2Kb pages up
#define SECTOR2_PAGE_START	64u  // 0x08020000, is 0x20000 from flash base address, so 64 x 2Kb pages up
#define SECTOR3_PAGE_START	96u  // 0x08030000, is 0x30000 from flash base address, so 96 x 2Kb pages up
#define SECTOR0_PAGE_SIZE	16u  // 16 x 2Kb pages up
#define SECTOR1_PAGE_SIZE	32u  // 32 x 2Kb pages up
#define SECTOR2_PAGE_SIZE	32u  // 32 x 2Kb pages up
#define SECTOR3_PAGE_SIZE	32u  // 32 x 2Kb pages up

extern void program_flash_double_word(uint32_t Address, uint64_t DataToProg64);
extern void erase_flash_sector(uint8_t sector_number);

#endif /* INC_FLASH_FUNCTIONS_H_ */
