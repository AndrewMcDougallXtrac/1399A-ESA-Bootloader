/*
 * memory_layout.c
 *
 *  Created on: Aug 16, 2023
 *      Author: SBrewerton
 */
#include "memory_layout.h"

const uint8_t  __attribute__((section (".buildDateSection")))   		buildDateMarker[CHAR_MARKER_LEN] = 			BUILD_DATE_MARKER_DATA;
const uint8_t  __attribute__((section (".modelNumberSection")))   		modelNumberMarker[CHAR_MARKER_LEN] = 		MODEL_NUMBER_MARKER_DATA;
const uint8_t  __attribute__((section (".hardwareVersionSection")))   	hardwareVersionMarker[CHAR_MARKER_LEN] = 	HARDWARE_VERSION_MARKER_DATA;
const uint8_t  __attribute__((section (".softwareVersionSection")))   	softwareVersionMarker[CHAR_MARKER_LEN] = 	SOFTWARE_VERSION_MARKER_DATA;
const uint32_t __attribute__((section (".codePresentSection")))   		codePresentMarker					   =	CODE_PRESENT_MARKER_DATA;
const uint32_t __attribute__((section (".flashChecksumSection"))) 		flashChecksumMarker 				   =	FLASH_CHECKSUM_MARKER_DATA;
