/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define rightKick_Pin GPIO_PIN_0
#define rightKick_GPIO_Port GPIOF
#define motorVcc_Pin GPIO_PIN_1
#define motorVcc_GPIO_Port GPIOF
#define step1_Pin GPIO_PIN_0
#define step1_GPIO_Port GPIOA
#define dir1_Pin GPIO_PIN_1
#define dir1_GPIO_Port GPIOA
#define step2_Pin GPIO_PIN_2
#define step2_GPIO_Port GPIOA
#define dir2_Pin GPIO_PIN_3
#define dir2_GPIO_Port GPIOA
#define IR1_Pin GPIO_PIN_4
#define IR1_GPIO_Port GPIOA
#define batteryReading_Pin GPIO_PIN_5
#define batteryReading_GPIO_Port GPIOA
#define IR2_Pin GPIO_PIN_6
#define IR2_GPIO_Port GPIOA
#define LEDG_Pin GPIO_PIN_7
#define LEDG_GPIO_Port GPIOA
#define LEDR_Pin GPIO_PIN_0
#define LEDR_GPIO_Port GPIOB
#define IR3_Pin GPIO_PIN_1
#define IR3_GPIO_Port GPIOB
#define LEDB_Pin GPIO_PIN_8
#define LEDB_GPIO_Port GPIOA
#define IRTransmit_Pin GPIO_PIN_9
#define IRTransmit_GPIO_Port GPIOA
#define IRREC_Pin GPIO_PIN_10
#define IRREC_GPIO_Port GPIOA
#define motorFault_Pin GPIO_PIN_11
#define motorFault_GPIO_Port GPIOA
#define LeftKick_Pin GPIO_PIN_12
#define LeftKick_GPIO_Port GPIOA
#define SensorPwrSig_Pin GPIO_PIN_3
#define SensorPwrSig_GPIO_Port GPIOB
#define auxPwr_Pin GPIO_PIN_4
#define auxPwr_GPIO_Port GPIOB
#define IRTransmitGND_Pin GPIO_PIN_5
#define IRTransmitGND_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
