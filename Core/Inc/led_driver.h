/*
 * LedDriver.h
 *
 *  Created on: Aug 18, 2023
 *      Author: SBrewerton
 */

#ifndef INC_LED_DRIVER_H_
#define INC_LED_DRIVER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"


#define LED_GREEN_STATE_OFF			0u
#define LED_GREEN_STATE_ON			1u
#define LED_RED_STATE_OFF			0u
#define LED_RED_STATE_ON			1u

#define LED_GREEN_OFF_TIME			100u //in ms
#define LED_GREEN_ON_TIME			100u //in ms
#define LED_RED_OFF_TIME			800u //in ms
#define LED_RED_ON_TIME				200u //in ms

extern uint16_t LedRedDelayCount;
extern uint16_t LedGreenDelayCount;
extern uint8_t  LedRedState;
extern uint8_t  LedGreenState;
extern uint8_t  LedRedPattern;
extern uint8_t  LedGreenPattern;

#define SET_LED_GREEN_OFF			(HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET))
#define SET_LED_GREEN_ON			(HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET))
#define SET_LED_RED_OFF				(HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET))
#define SET_LED_RED_ON				(HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET))


extern void Request_Green_Led_State(uint8_t state);
extern void Request_Red_Led_State(uint8_t state);

extern void Update_Green_Led_State(void);
extern void Update_Red_Led_State(void);

#endif /* INC_LED_DRIVER_H_ */
