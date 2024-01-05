/*
 * can_diagnostics.c
 *
 *  Created on: Aug 16, 2023
 *      Author: SBrewerton
 */

#include "can_diagnostics.h"
#include "main.h"
#include "led_driver.h"
#include "data_type_unions.h"
#include "memory_checks.h"
#include "memory_layout.h"
#include "flash_functions.h"

//CAN message variables
uint8_t CanRxCount = 0; //Received CAN message counter
static uint32_t CanTxMailbox;
static uint32_t* pCanTxMailbox;
static CAN_TxHeaderTypeDef CanHeader1;	//0x056
static CAN_TxHeaderTypeDef* pCanHeader;

uint8_t RecData[8];
uint8_t* pRecData;
CAN_RxHeaderTypeDef RecHeader;
CAN_RxHeaderTypeDef* pRecHeader;
static CAN_FilterTypeDef CanFilterConfig; //create a CAN filter structure of type CAN_FilterTypeDef
static CAN_FilterTypeDef* PtrCanFilterConfig; //Create a pointer to the CAN filter data structure

convert_4u8_to_1u32_t flashAddr;
convert_4u8_to_1u32_t flashDataLo;
convert_4u8_to_1u32_t flashDataHi;
uint64_t dataToProg64;

// Diag variables
uint16_t	DiagReplyIdx = 0;
uint16_t	DiagReplyTxCnt = 0;
uint8_t		DiagCommand = DIAG_CMD_IDLE;
uint8_t		DiagTesterNodeId = 0;
uint32_t 	*pMemory; //pointer to indirect read memory

uint8_t		DiagReplyData[8] = {0,0,0,0,0,0,0,0};
uint8_t		DiagRequestData[8] = {0,0,0,0,0,0,0,0};

void CAN_Init_Diag_Messages(CAN_HandleTypeDef *hcan) {
	  CanHeader1.StdId = CAN_MSG_ID_DIAG_REPLY;
	  CanHeader1.ExtId = 0x01u;
	  CanHeader1.DLC = 8;
	  CanHeader1.IDE = CAN_ID_STD; //data field type 'Can_identifier_Type'
	  CanHeader1.RTR = CAN_RTR_DATA; //data of type 'CAN_remote_transmission_request'

	  pCanHeader = &CanHeader1;

	  //Prepare for CAN reception
	  //Configure CAN FIFO 0, filter 0 & 1.
	  //CAN_FilterTypeDef CanFilterConfig
	  CanFilterConfig.FilterIdHigh = 0; //set message identifier to look for, was 0x321<<5
	  CanFilterConfig.FilterIdLow = 0;  //was 0x321<<5

	  //mask identifier - default type of filter is a list NOT mask
	  CanFilterConfig.FilterMaskIdHigh = CAN_MSG_ID_DIAG_REQUEST<<5; //set message identifier to look for, was 0x321<<5;
	  CanFilterConfig.FilterMaskIdLow = CAN_MSG_ID_DIAG_REQUEST<<5;  //was 0x321<<5;

	  CanFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //variable type: CAN_filter_FIFO (FIFO 0)
	  CanFilterConfig.FilterBank = 0;//specifies which filter bank will be initialised
	  CanFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; //variable type: CAN_filter_modeTjis controls bit FBMx=1
	  CanFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; //variable type: CAN_filter_scale (2off 16bit filters) This controls bit FSCx=0
	  CanFilterConfig.FilterActivation = CAN_FILTER_ENABLE; //variable of type: CAN_filter_activation
	  PtrCanFilterConfig = &CanFilterConfig; //set pointer to point to variable

	  HAL_CAN_ConfigFilter(hcan, PtrCanFilterConfig); //Configure the CAN reception filters (HAL CAN configuration function)


	  HAL_GPIO_WritePin(CAN_RS_GPIO_Port, CAN_RS_Pin, GPIO_PIN_RESET); //Enable CAN transceiver

	  //CanTxMailbox = CAN_TX_MAILBOX0;
	  CanTxMailbox = 0;
	  pCanTxMailbox = &CanTxMailbox;

	  //Enable CAN RX Fifo 0 interrupts
	  // uses polling mode instead
	  // HAL_CAN_ActivateNotification (&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	  HAL_CAN_Start(hcan);

}


void CAN_RxCheckMsgPendingCallback (CAN_HandleTypeDef *hcan)
{
	//CAN handler to see if new data received
	//Function called by polling ~1ms
	uint8_t CanRx0FillLevel;

	if (hcan->Instance == CAN1)
	{
		CanRx0FillLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);

		if (CanRx0FillLevel != 0) //test to ensure there is something in the CAN receive FIFO 0 buffer
		{

			pRecHeader = &RecHeader;
			pRecData = RecData;  //don't need to add the ampersand!!!
			if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, pRecHeader, pRecData) == HAL_OK)
			{
				CanRxCount++;
				if (pRecHeader->IDE == CAN_ID_STD)
				{
					// Standard CAN identifier received
					// Test current received message
					if (pRecHeader->StdId == CAN_MSG_ID_DIAG_REQUEST)
					{
						// Diagnostic mailbox message
						Request_Green_Led_State(LED_GREEN_STATE_ON);
						CAN_Diag_Decode_Request(&RecData[0]);
					}
				}
			}
		}
		CAN_Diag_Generate_Reply(hcan); //send a reply if needed or continue reply
	}
}



uint8_t CAN_Diag_Not_Idle(void) {
	uint8_t retVal = 0;
	if(DiagCommand != DIAG_CMD_IDLE) {
		retVal = 1;
	} else {
		retVal = 0;
	}
	return(retVal);
}

void CAN_Diag_Decode_Request(uint8_t* dataBuff) {
	DiagRequestData[0] = dataBuff[0];
	DiagRequestData[1] = dataBuff[1];
	DiagRequestData[2] = dataBuff[2];
	DiagRequestData[3] = dataBuff[3];
	DiagRequestData[4] = dataBuff[4];
	DiagRequestData[5] = dataBuff[5];
	DiagRequestData[6] = dataBuff[6];
	DiagRequestData[7] = dataBuff[7];
	DiagTesterNodeId = DiagRequestData[0];
	DiagCommand = DiagRequestData[1];

}

void CAN_Diag_Generate_Reply(CAN_HandleTypeDef *hcan) {

	uint8_t sector;
	uint8_t txDataBuff[8];
	convert_4u8_to_1u32_t tempData;

	switch(DiagCommand) {
	case DIAG_CMD_WRITE_CRC_START_ADDR:
		// decode the requested address
		tempData.bytes.byte_hh = DiagRequestData[2];
		tempData.bytes.byte_hl = DiagRequestData[3];
		tempData.bytes.byte_lh = DiagRequestData[4];
		tempData.bytes.byte_ll = DiagRequestData[5];
		Set_CRC_Start_Address(tempData.word32);
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = tempData.bytes.byte_hh;
		txDataBuff[3] = tempData.bytes.byte_hl;
		txDataBuff[4] = tempData.bytes.byte_lh;
		txDataBuff[5] = tempData.bytes.byte_ll;
		txDataBuff[6] = 0x00;
		txDataBuff[7] = 0x00;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;
	case DIAG_CMD_WRITE_CRC_END_ADDR:
		// decode the requested address
		tempData.bytes.byte_hh = DiagRequestData[2];
		tempData.bytes.byte_hl = DiagRequestData[3];
		tempData.bytes.byte_lh = DiagRequestData[4];
		tempData.bytes.byte_ll = DiagRequestData[5];
		Set_CRC_End_Address(tempData.word32);
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = tempData.bytes.byte_hh;
		txDataBuff[3] = tempData.bytes.byte_hl;
		txDataBuff[4] = tempData.bytes.byte_lh;
		txDataBuff[5] = tempData.bytes.byte_ll;
		txDataBuff[6] = 0x00;
		txDataBuff[7] = 0x00;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;
	case DIAG_CMD_WRITE_CRC_CALC_REQ:
		// decode the requested address
		Start_CRC_Calculation();
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = 0xDE;
		txDataBuff[3] = 0xAD;
		txDataBuff[4] = 0xBE;
		txDataBuff[5] = 0xEF;
		txDataBuff[6] = 0x00;
		txDataBuff[7] = 0x00;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;
	case DIAG_CMD_READ_CRC_RESULT:
		// decode the requested address
		tempData.word32 = Report_CRC_Result_Value();
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = tempData.bytes.byte_hh;
		txDataBuff[3] = tempData.bytes.byte_hl;
		txDataBuff[4] = tempData.bytes.byte_lh;
		txDataBuff[5] = tempData.bytes.byte_ll;
		txDataBuff[6] = (uint8_t)Report_CRC_Progress();
		txDataBuff[7] = 0x00;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;
	case DIAG_CMD_READ_MEM32:
		// decode the requested address
		tempData.bytes.byte_hh = DiagRequestData[2];
		tempData.bytes.byte_hl = DiagRequestData[3];
		tempData.bytes.byte_lh = DiagRequestData[4];
		tempData.bytes.byte_ll = DiagRequestData[5];
		// decode address and initialise pointer
		pMemory = (uint32_t *)tempData.word32;
		tempData.bytes.byte_hh = 0x00;
		tempData.bytes.byte_hl = 0x00;
		tempData.bytes.byte_lh = DiagRequestData[6];
		tempData.bytes.byte_ll = DiagRequestData[7];
		DiagReplyTxCnt = 1 + tempData.word32; //send whatever number of reply messages
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		// read memory location and store the data
		tempData.word32 = *pMemory;
		txDataBuff[2] = tempData.bytes.byte_hh;
		txDataBuff[3] = tempData.bytes.byte_hl;
		txDataBuff[4] = tempData.bytes.byte_lh;
		txDataBuff[5] = tempData.bytes.byte_ll;
		DiagReplyIdx = 1;
		tempData.word32 = DiagReplyIdx - 1; //send whatever number of reply messages
		txDataBuff[6] = tempData.bytes.byte_lh;;
		txDataBuff[7] = tempData.bytes.byte_ll;
		pMemory++;			//add 4 byte offset for next read
		DiagReplyIdx++;
		if(DiagReplyTxCnt > 1) { //this is a multi-part reply message spread over several transmissions
			DiagCommand = DIAG_CMD_READ_MEM32_CONTINUED;
		} else {
			DiagCommand = DIAG_CMD_IDLE;
		}
		break;
	case DIAG_CMD_READ_MEM32_CONTINUED:
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~(uint8_t)DIAG_CMD_READ_MEM32;     //inverted command byte
		// read memory location and store the data
		tempData.word32 = *pMemory;
		txDataBuff[2] = tempData.bytes.byte_hh;
		txDataBuff[3] = tempData.bytes.byte_hl;
		txDataBuff[4] = tempData.bytes.byte_lh;
		txDataBuff[5] = tempData.bytes.byte_ll;
		tempData.word32 = DiagReplyIdx -  1; //send whatever number of reply messages
		txDataBuff[6] = tempData.bytes.byte_lh;;
		txDataBuff[7] = tempData.bytes.byte_ll;
		pMemory++;			//add 4 byte offset for next read
		DiagReplyIdx++;
		if(DiagReplyTxCnt > 1) { //this is a multi-part reply message spread over several transmissions
			DiagCommand = DIAG_CMD_READ_MEM32_CONTINUED;
		} else {
			DiagCommand = DIAG_CMD_IDLE;
		}
		break;
	case DIAG_CMD_FLASH_ERASE_SECT:
		// decode the requested address
		sector = DiagRequestData[7];
		erase_flash_sector(sector);
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = 0x00;
		txDataBuff[3] = 0x00;
		txDataBuff[4] = 0x00;
		txDataBuff[5] = 0x00;
		txDataBuff[6] = 0x00;
		txDataBuff[7] = sector;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;
	case DIAG_CMD_FLASH_PROG_LOW:
		// decode the requested address
		flashDataLo.bytes.byte_hh = DiagRequestData[2];
		flashDataLo.bytes.byte_hl = DiagRequestData[3];
		flashDataLo.bytes.byte_lh = DiagRequestData[4];
		flashDataLo.bytes.byte_ll = DiagRequestData[5];
		flashAddr.bytes.byte_lh = DiagRequestData[6];
		flashAddr.bytes.byte_ll = DiagRequestData[7];
		// build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = flashDataLo.bytes.byte_hh;
		txDataBuff[3] = flashDataLo.bytes.byte_hl;
		txDataBuff[4] = flashDataLo.bytes.byte_lh;
		txDataBuff[5] = flashDataLo.bytes.byte_ll;
		txDataBuff[6] = flashAddr.bytes.byte_lh;
		txDataBuff[7] = flashAddr.bytes.byte_ll;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;
	case DIAG_CMD_FLASH_PROG_HIGH:
		// decode the requested address
		flashDataHi.bytes.byte_hh = DiagRequestData[2];
		flashDataHi.bytes.byte_hl = DiagRequestData[3];
		flashDataHi.bytes.byte_lh = DiagRequestData[4];
		flashDataHi.bytes.byte_ll = DiagRequestData[5];
		flashAddr.bytes.byte_hh = DiagRequestData[6];
		flashAddr.bytes.byte_hl = DiagRequestData[7];
		// build the reply message
		dataToProg64 = flashDataLo.word32 | (((uint64_t)flashDataHi.word32) << 32);
		program_flash_double_word(flashAddr.word32,dataToProg64);
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = flashDataHi.bytes.byte_hh;
		txDataBuff[3] = flashDataHi.bytes.byte_hl;
		txDataBuff[4] = flashDataHi.bytes.byte_lh;
		txDataBuff[5] = flashDataHi.bytes.byte_ll;
		txDataBuff[6] = flashAddr.bytes.byte_hh;
		txDataBuff[7] = flashAddr.bytes.byte_hl;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;
	case DIAG_CMD_RESET_REQ:
		// decode the requested address
		tempData.bytes.byte_hh = DiagRequestData[2];
		tempData.bytes.byte_hl = DiagRequestData[3];
		tempData.bytes.byte_lh = DiagRequestData[4];
		tempData.bytes.byte_ll = DiagRequestData[5];
		if(tempData.word32 == CODE_PRESENT_MARKER_DATA) {
			//jump to application
			//does not return if app is there
			doJumpToApplication();
		}
		// no need to build the reply message
		txDataBuff[0] = CAN_MSG_NODE_ID;  //this ECU node address
		txDataBuff[1] = ~DiagCommand;     //inverted command byte
		txDataBuff[2] = tempData.bytes.byte_hh;
		txDataBuff[3] = tempData.bytes.byte_hl;
		txDataBuff[4] = tempData.bytes.byte_lh;
		txDataBuff[5] = tempData.bytes.byte_ll;
		txDataBuff[6] = 0x00;
		txDataBuff[7] = 0x00;
		DiagReplyTxCnt = 1;  //send one reply message
		DiagCommand = DIAG_CMD_IDLE;
		break;

	}

	if(DiagReplyTxCnt > 0) {
		DiagReplyTxCnt--;
		// send out reply
		if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0)
		{
			HAL_CAN_AddTxMessage(hcan, pCanHeader, txDataBuff, pCanTxMailbox);  //initiate transmission of message 0x254
		}
	} else {
		//we are done with this command
		DiagReplyTxCnt = 0;
		DiagCommand = DIAG_CMD_IDLE;
	}
}

void doJumpToApplication(void) {
#define APPLICATION_ADDRESS (uint32_t)0x08008000
	typedef void (*pFunction)(void);

	//check if there is a valid application
	//check code present flag
	pMemory = (uint32_t*)APP_PRESENT_MARKER_ADDR;
	if(*pMemory == CODE_PRESENT_MARKER_DATA) {
		pFunction Jump_To_Application;
		uint32_t JumpAddress;
		/* Jump to user application */
		JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
		Jump_To_Application();
	}
}
