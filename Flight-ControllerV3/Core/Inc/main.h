/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOE
#define USB_DETECT_Pin GPIO_PIN_6
#define USB_DETECT_GPIO_Port GPIOE
#define VBAT_MON_Pin GPIO_PIN_1
#define VBAT_MON_GPIO_Port GPIOC
#define TP_Pin GPIO_PIN_2
#define TP_GPIO_Port GPIOC
#define CURR_MON_Pin GPIO_PIN_3
#define CURR_MON_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_0
#define M1_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_1
#define M2_GPIO_Port GPIOA
#define M3_Pin GPIO_PIN_2
#define M3_GPIO_Port GPIOA
#define M4_Pin GPIO_PIN_3
#define M4_GPIO_Port GPIOA
#define IMU1_CS_Pin GPIO_PIN_4
#define IMU1_CS_GPIO_Port GPIOA
#define IMU1_SCK_Pin GPIO_PIN_5
#define IMU1_SCK_GPIO_Port GPIOA
#define IMU1_MISO_Pin GPIO_PIN_6
#define IMU1_MISO_GPIO_Port GPIOA
#define IMU1_MOSI_Pin GPIO_PIN_7
#define IMU1_MOSI_GPIO_Port GPIOA
#define IMU1_INT_Pin GPIO_PIN_4
#define IMU1_INT_GPIO_Port GPIOC
#define GIMB_PAN_Pin GPIO_PIN_0
#define GIMB_PAN_GPIO_Port GPIOB
#define GIMB_TILT_Pin GPIO_PIN_1
#define GIMB_TILT_GPIO_Port GPIOB
#define BB_MOSI_Pin GPIO_PIN_2
#define BB_MOSI_GPIO_Port GPIOB
#define IMU2_CS_Pin GPIO_PIN_11
#define IMU2_CS_GPIO_Port GPIOE
#define IMU2_SCK_Pin GPIO_PIN_12
#define IMU2_SCK_GPIO_Port GPIOE
#define IMU2_MISO_Pin GPIO_PIN_13
#define IMU2_MISO_GPIO_Port GPIOE
#define IMU2_MOSI_Pin GPIO_PIN_14
#define IMU2_MOSI_GPIO_Port GPIOE
#define IMU2_INT_Pin GPIO_PIN_15
#define IMU2_INT_GPIO_Port GPIOE
#define OSD_CS_Pin GPIO_PIN_12
#define OSD_CS_GPIO_Port GPIOB
#define OSD_SCK_Pin GPIO_PIN_13
#define OSD_SCK_GPIO_Port GPIOB
#define OSD_MISO_Pin GPIO_PIN_14
#define OSD_MISO_GPIO_Port GPIOB
#define OSD_MOSI_Pin GPIO_PIN_15
#define OSD_MOSI_GPIO_Port GPIOB
#define TX3_Pin GPIO_PIN_8
#define TX3_GPIO_Port GPIOD
#define RX3_Pin GPIO_PIN_9
#define RX3_GPIO_Port GPIOD
#define TX1_Pin GPIO_PIN_9
#define TX1_GPIO_Port GPIOA
#define RX1_Pin GPIO_PIN_10
#define RX1_GPIO_Port GPIOA
#define BB_CS_Pin GPIO_PIN_15
#define BB_CS_GPIO_Port GPIOA
#define BB_SCK_Pin GPIO_PIN_10
#define BB_SCK_GPIO_Port GPIOC
#define BB_MISO_Pin GPIO_PIN_11
#define BB_MISO_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_0
#define GPS_RX_GPIO_Port GPIOD
#define GPS_TX_Pin GPIO_PIN_1
#define GPS_TX_GPIO_Port GPIOD
#define TX2_Pin GPIO_PIN_5
#define TX2_GPIO_Port GPIOD
#define RX2_Pin GPIO_PIN_6
#define RX2_GPIO_Port GPIOD
#define MAG_BARO_SCL_Pin GPIO_PIN_6
#define MAG_BARO_SCL_GPIO_Port GPIOB
#define MAG_BARO_SDA_Pin GPIO_PIN_7
#define MAG_BARO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
