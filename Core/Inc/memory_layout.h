/*
 * memory_layout.h
 *
 *  Created on: Aug 16, 2023
 *      Author: SBrewerton
 */

#ifndef INC_MEMORY_LAYOUT_H_
#define INC_MEMORY_LAYOUT_H_

#include <stdint.h>

//Memory range defines for diag access and checksum calcs
#define ADDR_RAM_START				0x20000000u  //start
#define ADDR_RAM_END				0x2000FFFFu  //64k = 64 *1024 bytes = 0x1'0000
#define UNTESTABLE_SRAM_STACK_SIZE	64u  //bytes
#define UNTESTABLE_SRAM_VAR_SIZE	0x28u  //bytes
#define ADDR_RAM_TEST_START			(ADDR_RAM_START + UNTESTABLE_SRAM_VAR_SIZE)
#define ADDR_RAM_TEST_END			(ADDR_RAM_END - UNTESTABLE_SRAM_STACK_SIZE)
#define CRC_TEST_SEED				0xFFFFFFFFu
#define SRAM_CRC_EXPECTED_RESULT	0xf0815032 // before CRC poly changed 0x551b2038u

#define LEN_RAM					0x10000u    //64K
#define ADDR_RAM2_START			0x10000000u
#define ADDR_RAM2_END			0x10003FFFu
#define LEN_RAM2				0x4000u     //16K
#define ADDR_FLASH_START		0x08000000u
#define ADDR_FLASH_END			0x0803FFFFu
#define LEN_FLASH				0x40000u    //256K

#define ADDR_FLASH_BOOTLOADER_START			0x08000000u
#define ADDR_FLASH_BOOTLOADER_END			0x08003FFFu
#define ADDR_FLASH_APPLICATION_START		0x08008000u
#define ADDR_FLASH_APPLICATION_END			0x0802FFFFu
#define ADDR_FLASH_APPLICATION_CODE_START	0x08008000u
#define ADDR_FLASH_APPLICATION_CODE_END		0x0802FFF0

#define STACK_AREA_WATERMARK_ADDR 		0x2000FFC0u
#define APP_PRESENT_MARKER_ADDR			0x0802FFF8U

//program memory image defines
#define CHAR_MARKER_LEN	32u
#define BUILD_DATE_MARKER_ADDR			0x0802ff70
#define MODEL_NUMBER_MARKER_ADDR		0x0802ff90
#define	HARDWARE_VERSION_MARKER_ADDR	0x0802ffb0
#define SOFTWARE_VERSION_MARKER_ADDR	0x0802ffd0
#define CODE_PRESENT_MARKER_ADDR		0x0802fff8
#define FLASH_CHECKSUM_MARKER_ADDR		0x0802fffC

#define BUILD_DATE_MARKER_DATA			"25/08/2023                      "
#define MODEL_NUMBER_MARKER_DATA		"Xtrac 1399A                     "
#define	HARDWARE_VERSION_MARKER_DATA	"Ver B                           "
#define SOFTWARE_VERSION_MARKER_DATA	"Xtrac 1399A BSL ver 4.1         "
#define CODE_PRESENT_MARKER_DATA		0xDEADBEEFu
#define FLASH_CHECKSUM_MARKER_DATA		0xFFFFFFFFu


extern const uint8_t  buildDateMarker[CHAR_MARKER_LEN];
extern const uint8_t  modelNumberMarker[CHAR_MARKER_LEN];
extern const uint8_t  hardwareVersionMarker[CHAR_MARKER_LEN];
extern const uint8_t  softwareVersionMarker[CHAR_MARKER_LEN];
extern const uint32_t CodePresentMarker;
extern const uint32_t flashChecksumMarker;


extern const uint8_t  buildDateMarker[CHAR_MARKER_LEN];
extern const uint8_t  modelNumberMarker[CHAR_MARKER_LEN];
extern const uint8_t  hardwareVersionMarker[CHAR_MARKER_LEN];
extern const uint8_t  softwareVersionMarker[CHAR_MARKER_LEN];
extern const uint32_t CodePresentMarker;
extern const uint32_t flashChecksumMarker;

#endif /* INC_MEMORY_LAYOUT_H_ */
