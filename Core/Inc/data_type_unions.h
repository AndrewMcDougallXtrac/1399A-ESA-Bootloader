/*
 * data_type_unions.h
 *
 *  Created on: Aug 18, 2023
 *      Author: SBrewerton
 */

#ifndef INC_DATA_TYPE_UNIONS_H_
#define INC_DATA_TYPE_UNIONS_H_

#include <stdint.h>


typedef struct {
	uint8_t byte_ll;
	uint8_t byte_lh;
	uint8_t byte_hl;
	uint8_t byte_hh;
} t_array_4u8;

typedef union convert_4u8_to_1u32_t{
	t_array_4u8 bytes;
	uint32_t	word32;
} convert_4u8_to_1u32_t;

#endif /* INC_DATA_TYPE_UNIONS_H_ */
