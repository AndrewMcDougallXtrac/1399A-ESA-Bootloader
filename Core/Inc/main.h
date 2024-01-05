/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define TIMER_1MS_CNT	100u
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define H2_Pin GPIO_PIN_3
#define H2_GPIO_Port GPIOB
#define H1_Pin GPIO_PIN_15
#define H1_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define CPOUT_Pin GPIO_PIN_12
#define CPOUT_GPIO_Port GPIOA
#define LEDR_Pin GPIO_PIN_7
#define LEDR_GPIO_Port GPIOB
#define LEDG_Pin GPIO_PIN_6
#define LEDG_GPIO_Port GPIOB
#define DW_Pin GPIO_PIN_10
#define DW_GPIO_Port GPIOC
#define CAN_RS_Pin GPIO_PIN_5
#define CAN_RS_GPIO_Port GPIOB
#define UP_Pin GPIO_PIN_11
#define UP_GPIO_Port GPIOC
#define WH_Pin GPIO_PIN_10
#define WH_GPIO_Port GPIOA
#define VH_Pin GPIO_PIN_9
#define VH_GPIO_Port GPIOA
#define UH_Pin GPIO_PIN_8
#define UH_GPIO_Port GPIOA
#define NSS_EEPROM_Pin GPIO_PIN_9
#define NSS_EEPROM_GPIO_Port GPIOC
#define PROG_Pin GPIO_PIN_7
#define PROG_GPIO_Port GPIOC
#define CURR_REF_Pin GPIO_PIN_0
#define CURR_REF_GPIO_Port GPIOC
#define CURR_IN_Pin GPIO_PIN_1
#define CURR_IN_GPIO_Port GPIOC
#define PCB_TEMP_Pin GPIO_PIN_2
#define PCB_TEMP_GPIO_Port GPIOC
#define MOT_TEMP_Pin GPIO_PIN_3
#define MOT_TEMP_GPIO_Port GPIOC
#define POT_OUT_Pin GPIO_PIN_5
#define POT_OUT_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define H3_Pin GPIO_PIN_10
#define H3_GPIO_Port GPIOB
#define NSS_GPS_Pin GPIO_PIN_12
#define NSS_GPS_GPIO_Port GPIOB
#define VOLTS_IN_Pin GPIO_PIN_1
#define VOLTS_IN_GPIO_Port GPIOA
#define CURR_REFA4_Pin GPIO_PIN_4
#define CURR_REFA4_GPIO_Port GPIOA
#define UL_Pin GPIO_PIN_7
#define UL_GPIO_Port GPIOA
#define VL_Pin GPIO_PIN_0
#define VL_GPIO_Port GPIOB
#define WL_Pin GPIO_PIN_1
#define WL_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_7
#define SW1_GPIO_Port GPIOE
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
