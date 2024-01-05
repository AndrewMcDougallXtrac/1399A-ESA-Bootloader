/*
 * LedDriver.c
 *
 *  Created on: Aug 18, 2023
 *      Author: SBrewerton
 */

#include <led_driver.h>
#include "main.h"

#define SET_LED_GREEN_OFF			(HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET))
#define SET_LED_GREEN_ON			(HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET))
#define SET_LED_RED_OFF				(HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET))
#define SET_LED_RED_ON				(HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET))

uint16_t LedRedDelayCount = 0;
uint16_t LedGreenDelayCount = 0;
uint8_t  LedRedState = 0;
uint8_t  LedGreenState = 0;
uint8_t  LedRedPattern = 0;
uint8_t  LedGreenPattern = 0;


void Set_Green_Led_On_Off(uint8_t state) {
	switch (state) {
	case (LED_GREEN_STATE_OFF):
		SET_LED_GREEN_OFF;
		break;
	case (LED_GREEN_STATE_ON):
		SET_LED_GREEN_ON;
		break;
	default:
		SET_LED_GREEN_OFF;
		break;
	}
}

void Set_Red_Led_On_Off(uint8_t state) {
	switch (state) {
	case (LED_RED_STATE_OFF):
		SET_LED_RED_OFF;
		break;
	case (LED_RED_STATE_ON):
		SET_LED_RED_ON;
		break;
	default:
		SET_LED_RED_OFF;
		break;
	}
}

void Request_Green_Led_State(uint8_t state) {
	LedGreenState = state;
}

void Request_Red_Led_State(uint8_t state) {
	LedRedState = state;
}

void Update_Green_Led_State(void) {
	if(LedGreenDelayCount > 0) {
		LedGreenDelayCount--;
	} else {
	  switch (LedGreenState) {
		case(LED_GREEN_STATE_OFF): // Green
			Set_Green_Led_On_Off(LED_GREEN_STATE_OFF);
			LedGreenDelayCount = LED_GREEN_OFF_TIME;
			LedGreenState = LED_GREEN_STATE_OFF;
			break;
		case(LED_GREEN_STATE_ON): // Green
			Set_Green_Led_On_Off(LED_GREEN_STATE_ON);
			LedGreenDelayCount = LED_GREEN_ON_TIME;
			LedGreenState = LED_GREEN_STATE_OFF;
			break;
		default: //OFF
			Set_Green_Led_On_Off(LED_GREEN_STATE_OFF);
			LedGreenDelayCount = LED_GREEN_OFF_TIME;
			LedGreenState = LED_GREEN_STATE_OFF;
			break;
	  }
	}
}

void Update_Red_Led_State(void) {
	if(LedRedDelayCount > 0) {
		LedRedDelayCount--;
	} else {
	  switch (LedRedState) {
		case(LED_RED_STATE_OFF): // Red
			Set_Red_Led_On_Off(LED_RED_STATE_OFF);
			LedRedDelayCount = LED_RED_OFF_TIME;
			LedRedState = LED_RED_STATE_ON;
			break;
		case(LED_RED_STATE_ON): // Red
			Set_Red_Led_On_Off(LED_RED_STATE_ON);
			LedRedDelayCount = LED_RED_ON_TIME;
			LedRedState = LED_RED_STATE_OFF;
			break;
		default: //OFF
			Set_Red_Led_On_Off(LED_RED_STATE_OFF);
			LedRedDelayCount = LED_RED_OFF_TIME;
			LedRedState = LED_RED_STATE_OFF;
			break;
	  }
	}
}
