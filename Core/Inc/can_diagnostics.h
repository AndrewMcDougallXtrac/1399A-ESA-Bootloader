/*
 * can_diagnostics.h
 *
 *  Created on: Aug 16, 2023
 *      Author: SBrewerton
 */

#ifndef INC_CAN_DIAGNOSTICS_H_
#define INC_CAN_DIAGNOSTICS_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"


//diagnostic defines
#define CAN_MSG_ID_DIAG_REQUEST   0x055u
#define CAN_MSG_ID_DIAG_REPLY     0x056u
#define CAN_MSG_NODE_ID		    0x9Au
#define CAN_MSG_TESTER_ID	    0xFFu
//diagnostic commands
#define DIAG_CMD_IDLE					0x00u
#define DIAG_CMD_READ_MEM8				0x11u
#define DIAG_CMD_READ_MEM16				0x12u
#define DIAG_CMD_READ_MEM32				0x13u
#define DIAG_CMD_READ_MEM8_CONTINUED	0x81u
#define DIAG_CMD_READ_MEM16_CONTINUED	0x82u
#define DIAG_CMD_READ_MEM32_CONTINUED	0x83u

#define DIAG_CMD_WRITE_IMM8				0x21u
#define DIAG_CMD_WRITE_IMM16			0x22u
#define DIAG_CMD_WRITE_MEM_PTR			0x30u
#define DIAG_CMD_WRITE_IND8				0x31u
#define DIAG_CMD_WRITE_IND16			0x32u
#define DIAG_CMD_WRITE_IND32			0x33u
#define DIAG_CMD_RESET_REQ				0xAAu
#define DIAG_CMD_BOOTLOAD_REQ			0xBBu
#define DIAG_CMD_WRITE_CRC_START_ADDR	0xC0u
#define DIAG_CMD_WRITE_CRC_END_ADDR		0xC1u
#define DIAG_CMD_WRITE_CRC_CALC_REQ		0xC2u
#define DIAG_CMD_READ_CRC_RESULT		0xC3u

#define DIAG_CMD_FLASH_ERASE_SECT		0x40u
#define DIAG_CMD_FLASH_PROG_LOW			0x41u
#define DIAG_CMD_FLASH_PROG_HIGH		0x42u


// Diag functions
extern void doJumpToApplication(void);
extern void CAN_RxCheckMsgPendingCallback (CAN_HandleTypeDef *hcan);
extern uint8_t CAN_Diag_Not_Idle(void);
extern void CAN_Diag_Decode_Request(uint8_t* dataBuff);
extern void CAN_Diag_Generate_Reply(CAN_HandleTypeDef *hcan);
extern void CAN_Init_Diag_Messages(CAN_HandleTypeDef *hcan);


#endif /* INC_CAN_DIAGNOSTICS_H_ */
