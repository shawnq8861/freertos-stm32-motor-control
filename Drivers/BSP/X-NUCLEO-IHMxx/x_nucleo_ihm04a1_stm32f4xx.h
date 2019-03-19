/** 
  ******************************************************************************
  * @file    x_nucleo_ihm04a1_stm32f4xx.h
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    September 29, 2016
  * @brief   Header for BSP driver for x-nucleo-ihm04a1 Nucleo extension board 
  *  (based on L6206)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#ifndef X_NUCLEO_IHM04A1_STM32F4XX_H
#define X_NUCLEO_IHM04A1_STM32F4XX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_nucleo.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup X_NUCLEO_IHM04A1_STM32F4XX
  * @{   
  */   
   
/* Exported Constants --------------------------------------------------------*/
   
/** @defgroup IHM04A1_Exported_Constants IHM04A1 Exported Constants
  * @{
  */   
   
/******************************************************************************/
/* USE_STM32F4XX_NUCLEO                                                       */
/******************************************************************************/

 /** @defgroup Constants_For_STM32F4XX_NUCLEO  Constants For STM32F4XX NUCLEO
* @{
*/   
/// Interrupt line used for L6206 Over Current Detection and over Temperature On Bridge A
#define EXTI_FLAG_A_IRQn           (EXTI15_10_IRQn)

/// Interrupt line used for L6206 Over Current Detection and over Temperature On Bridge B
#define EXTI_FLAG_B_IRQn           (EXTI1_IRQn)   
   
/// Timer used for PWM_1A
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A      (TIM3)

/// Timer used for PWM_2A
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A      (TIM3)

/// Timer used for PWM_1B
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B      (TIM2)

/// Timer used for PWM_2B
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B      (TIM2)   

/// Channel Timer used for PWM_1A
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1A      (TIM_CHANNEL_1)

/// Channel Timer used for PWM_2A
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2A      (TIM_CHANNEL_2)

/// Channel Timer used for PWM_1B
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1B      (TIM_CHANNEL_1)

/// Channel Timer used for PWM_2B
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2B      (TIM_CHANNEL_2)

/// HAL Active Channel Timer used for PWM_1A
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1A      (HAL_TIM_ACTIVE_CHANNEL_1)

/// HAL Active Channel Timer used for PWM_2A
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2A      (HAL_TIM_ACTIVE_CHANNEL_2)

/// HAL Active Channel Timer used for PWM_1B
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1B      (HAL_TIM_ACTIVE_CHANNEL_1)

/// HAL Active Channel Timer used for PWM_2B
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2B      (HAL_TIM_ACTIVE_CHANNEL_2)
   
/// Timer Clock Enable for PWM_1A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A_CLCK_ENABLE()  __TIM3_CLK_ENABLE()

/// Timer Clock Enable for PWM_2A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A_CLCK_ENABLE()  __TIM3_CLK_ENABLE()

/// Timer Clock Enable for PWM_1B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B_CLCK_ENABLE()  __TIM2_CLK_ENABLE()

/// Timer Clock Enable for PWM_2B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B_CLCK_ENABLE()  __TIM2_CLK_ENABLE()
   
/// Timer Clock Enable for PWM_1A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A_CLCK_DISABLE()  __TIM3_CLK_DISABLE()

/// Timer Clock Enable for PWM_2A
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A_CLCK_DISABLE()  __TIM3_CLK_DISABLE()

/// Timer Clock Enable for PWM_1B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B_CLCK_DISABLE()  __TIM2_CLK_DISABLE()

/// Timer Clock Enable for PWM_2B
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B_CLCK_DISABLE()  __TIM2_CLK_DISABLE()

/// PWM1A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1A  (GPIO_AF2_TIM3)

/// PWM2A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM2A  (GPIO_AF2_TIM3)

/// PWM1A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1B  (GPIO_AF1_TIM2)

/// PWM2A GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM2B  (GPIO_AF1_TIM2)

 /**
* @}
*/

/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

   /** @defgroup Constants_For_All_Nucleo_Platforms Constants For All Nucleo Platforms
* @{
*/   

/// GPIO Pin used for the PWM of the L6206 Brige A Input 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1A_PIN  (GPIO_PIN_4)
/// GPIO Port used for the PWM of the L6206 Brige A Input 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1A_PORT  (GPIOB)

/// GPIO Pin used for the PWM of the L6206 Brige A Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2A_PIN  (GPIO_PIN_5)
/// GPIO Port used for the PWM of the L6206 Brige A Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2A_PORT  (GPIOB)
   
/// GPIO Pin used for the PWM of the L6206 Brige BInput 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1B_PIN  (GPIO_PIN_0)
/// GPIO Port used for the PWM of the L6206 Brige B Input 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_1B_PORT  (GPIOA)

/// GPIO Pin used for the PWM of the L6206 Brige B Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2B_PIN  (GPIO_PIN_1)
/// GPIO Port used for the PWM of the L6206 Brige B Input 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_2B_PORT  (GPIOA)
   
/// GPIO Pin used for the L6206 Bridge A Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PIN  (GPIO_PIN_10)
/// GPIO port used for the L6206 Bridge A Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PORT (GPIOA)
/// Flag of bridge A interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PRIORITY             (4)

/// GPIO Pin used for the L6206 Bridge B Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PIN  (GPIO_PIN_1)
/// GPIO port used for the L6206 Bridge B Enable pin and OCD and OVT alarms
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PORT (GPIOC)  
/// Flag of bridge B interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PRIORITY             (4)
    
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* X_NUCLEO_IHM04A1_STM32F4XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
