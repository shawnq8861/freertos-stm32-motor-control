/**
  ******************************************************************************
  * @file    x_nucleo_ihm04a1_stm32f4xx.c
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    September 29, 2016
  * @brief   BSP driver for x-nucleo-ihm04a1 Nucleo extension board 
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
  
/* Includes ------------------------------------------------------------------*/
#include "x_nucleo_ihm04a1_stm32f4xx.h"

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup X_NUCLEO_IHM04A1_STM32F4XX NUCLEO IHM04A1 STM32F4XX
  * @{
  */   
    
/* Private constants ---------------------------------------------------------*/    

/** @defgroup IHM04A1_Private_Constants IHM04A1 Private Constants
  * @{
  */   
    
/// Timer Prescaler
#define TIMER_PRESCALER (64)


/// MCU wait time in ms after power bridges are enabled
#define BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY    (20)


/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/

/** @defgroup IHM04A1_Board_Private_Variables IHM04A1 Board Private Variables
  * @{
  */       

/// Timer handler for PWM of Input 1 Bridge A 
TIM_HandleTypeDef hTimPwm1A;
/// imer handler for PWM of Input 2 Bridge A 
TIM_HandleTypeDef hTimPwm2A;
/// Timer handler for PWM of Input 1 Bridge B 
TIM_HandleTypeDef hTimPwm1B;
/// Timer handler for PWM of Input 2 Bridge B 
TIM_HandleTypeDef hTimPwm2B;
/**
  * @}
  */ 

/** @defgroup IHM04A1_Board_Private_Function_Prototypes IHM04A1 Board Private Function Prototypes
  * @{
  */   

void L6206_Board_Delay(uint32_t delay);         //Delay of the requested number of milliseconds
void L6206_Board_DisableBridge(uint8_t bridgeId);     //Disable the specified bridge
void L6206_Board_EnableBridge(uint8_t bridgeId, uint8_t addDelay);      //Enable the specified bridge
uint32_t L6206_Board_GetFlagPinState(uint8_t bridgeId); //Get the status of the Enable and Flag pin
void L6206_Board_GpioInit(void);   //Initialise GPIOs used for L6206s
void L6206_Board_PwmDeInit(uint8_t bridgeInput); ///Deinitialise the PWM of the specified bridge input
void L6206_Board_PwmInit(uint8_t bridgeInput);    //Init the PWM of the specified bridge
void L6206_Board_PwmSetFreq(uint8_t bridgeInput, uint32_t newFreq, uint8_t duty); //Set Briges Inputs PWM frequency and start it
void L6206_Board_PwmStop(uint8_t bridgeInput);   //Stop the PWM of the specified bridge
/**
  * @}
  */


/** @defgroup  IHM04A1_Board_Private_Functions IHM04A1 Board Private Functions
  * @{
  */   

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
 * @retval None
 **********************************************************/
void L6206_Board_Delay(uint32_t delay)
{
  HAL_Delay(delay);
}

/******************************************************//**
 * @brief Disable the power bridges (leave the output bridges HiZ)
 * @param[in]  bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval None
 **********************************************************/
void L6206_Board_DisableBridge(uint8_t bridgeId)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;
  
  if (bridgeId == 0)
  {
    gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PIN;
    gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PORT;
  }
  else
  {
    gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PIN;
    gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PORT;    
  }
  
  /* Configure the GPIO connected to EN pin as an output */
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);
  
  __disable_irq();
  HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET);  
  __HAL_GPIO_EXTI_CLEAR_IT(gpioPin);
  __enable_irq();
    
}

/******************************************************//**
 * @brief Enable the power bridges (leave the output bridges HiZ)
 * @param[in]  bridgeId (from 0 for bridge A to 1 for bridge B)
 * @param[in]  addDelay if different from 0, a delay is added after bridge activation
 * @retval None
 **********************************************************/
void L6206_Board_EnableBridge(uint8_t bridgeId, uint8_t addDelay)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;
  IRQn_Type flagIrqn; 
  
  if (bridgeId == 0)
  {
    gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PIN;
    gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PORT;
    flagIrqn = EXTI_FLAG_A_IRQn;    
  }
  else
  {
    gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PIN;
    gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PORT;    
    flagIrqn = EXTI_FLAG_B_IRQn;    
  }
  
  HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET);
  if (addDelay != 0)
  {
    HAL_Delay(BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY);
  }
  /* Configure the GPIO connected to EN pin to take interrupt */
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);
  
  __HAL_GPIO_EXTI_CLEAR_IT(gpioPin);
  HAL_NVIC_ClearPendingIRQ(flagIrqn);
  HAL_NVIC_EnableIRQ(flagIrqn);  
}

/******************************************************//**
 * @brief  Returns the FLAG pin state.
 * @param[in]  bridgeId (from 0 for bridge A to 1 for bridge B)
 * @retval The FLAG pin value.
 **********************************************************/
uint32_t L6206_Board_GetFlagPinState(uint8_t bridgeId)
{
  uint32_t gpioPin;
  GPIO_TypeDef* gpioPort;

  if (bridgeId == 0)
  {
    gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PIN;
    gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PORT;
  }
  else
  {
    gpioPin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PIN;
    gpioPort = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PORT;    
  }
  return HAL_GPIO_ReadPin(gpioPort, gpioPin);
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the L6206s
 * @retval None
  **********************************************************/
void L6206_Board_GpioInit(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /* Configure L6206 Enable pin of Bridge A ------------------------------*/
  /* This pin is reconfigured later for OCD and OVT as GPIO_MODE_IT_FALLING with GPIO_PULLUP */
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PORT, BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PIN, GPIO_PIN_RESET);
  
  /* Set Priority of External Line Interrupt used for the OCD OVT interrupt*/ 
  HAL_NVIC_SetPriority(EXTI_FLAG_A_IRQn, BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_A_PRIORITY, 0);
    
  /* Enable the External Line Interrupt used for the OCD OVT interrupt*/
  HAL_NVIC_EnableIRQ(EXTI_FLAG_A_IRQn);    
  
/* Configure L6206 Enable pin of Bridge B ------------------------------*/
  /* This pin is reconfigured later for OCD and OVT as GPIO_MODE_IT_FALLING with GPIO_PULLUP */
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PORT, BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PIN, GPIO_PIN_RESET);
  
  /* Set Priority of External Line Interrupt used for the OCD OVT interrupt*/ 
  HAL_NVIC_SetPriority(EXTI_FLAG_B_IRQn, BSP_MOTOR_CONTROL_BOARD_EN_AND_FLAG_B_PRIORITY, 0);
    
  /* Enable the External Line Interrupt used for the OCD OVT interrupt*/
  HAL_NVIC_EnableIRQ(EXTI_FLAG_B_IRQn);    

}

/******************************************************//**
 * @brief  Reset the PWM for the specified brigde input
 * @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
 * 2 for input 1B, 3 for input 2B
 * @retval None
  **********************************************************/
void L6206_Board_PwmDeInit(uint8_t bridgeInput)
{
  TIM_HandleTypeDef *pHTim;

  switch (bridgeInput)
  {
    case 0:
    default:
      pHTim = &hTimPwm1A;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1A;

      break;
    case  1:
      pHTim = &hTimPwm2A;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2A;

      break;
    case 2:
      pHTim = &hTimPwm1B;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1B;

      break;
    case 3:
      pHTim = &hTimPwm2B;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2B;

      break;      
  }
  HAL_TIM_PWM_DeInit(pHTim);
}

/******************************************************//**
 * @brief  Set the PWM frequency the for the specified bridge input
 * @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
 * 2 for input 1B, 3 for input 2B
 * @retval None
  **********************************************************/
void L6206_Board_PwmInit(uint8_t bridgeInput)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_HandleTypeDef *pHTim;
  uint32_t  channel;

  switch (bridgeInput)
  {
    case 0:
    default:
      pHTim = &hTimPwm1A;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1A;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1A;
      break;
    case  1:
      pHTim = &hTimPwm2A;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2A;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2A;
      break;
    case 2:
      pHTim = &hTimPwm1B;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1B;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1B;
      break;
    case 3:
      pHTim = &hTimPwm2B;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B;
      pHTim->Channel = BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2B;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2B;
      break;      
  }
  pHTim->Init.Prescaler = TIMER_PRESCALER -1;
  pHTim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pHTim->Init.Period = 0;
  pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(pHTim);
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(pHTim, &sConfigOC, channel);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(pHTim, &sMasterConfig);
}

/******************************************************//**
 * @brief  Sets the frequency of PWM used for bridges inputs
 * @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
 * 2 for input 1B,  3 for input 2B
 * @param[in] newFreq in Hz
 * @param[in] duty Duty cycle
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void L6206_Board_PwmSetFreq(uint8_t bridgeInput, uint32_t newFreq, uint8_t duty)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  TIM_HandleTypeDef *pHTim;
  uint32_t period;
  uint32_t pulse;
  uint32_t channel;
  
  switch (bridgeInput)
  {
    case 0:
    default:
      pHTim = &hTimPwm1A;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1A;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1A;
      break;
    case  1:
      pHTim = &hTimPwm2A;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2A;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2A;
      break;
    case 2:
      pHTim = &hTimPwm1B;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1B;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1B;
      break;
    case 3:
      pHTim = &hTimPwm2B;
      pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2B;
      channel = BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2B;
      break;      
  }
  
   period = (uint32_t)( (uint32_t)sysFreq / (uint32_t)(TIMER_PRESCALER * newFreq)) - 1;


  __HAL_TIM_SetAutoreload(pHTim, period);
  
  if (duty == 0) 
  {
    pulse = 0 ;
  }
  else 
  {
    if (duty > 100) duty = 100;  
    pulse = (uint32_t) ( ((uint32_t)period * (uint32_t)duty) / 100) + 1;
  }    
  __HAL_TIM_SetCompare(pHTim, channel, pulse);
  HAL_TIM_PWM_Start(pHTim, channel);

}

/******************************************************//**
 * @brief  Stops the PWM uses for the specified brige input
 * @param[in] bridgeInput 0 for input 1A, 1 for input 2A,
 * 2 for input 1B, 3 for input 2B
 * @retval None
 **********************************************************/
void L6206_Board_PwmStop(uint8_t bridgeInput)
{
  switch (bridgeInput)
  {
    case 0:
       HAL_TIM_PWM_Stop(&hTimPwm1A,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1A);
      break;
    case  1:
      HAL_TIM_PWM_Stop(&hTimPwm2A,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2A);
      break;
    case  2:
      HAL_TIM_PWM_Stop(&hTimPwm1B,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1B);
      break;
    case  3:
      HAL_TIM_PWM_Stop(&hTimPwm2B,BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2B);
      break;      
    default:
      break;//ignore error
  }
}

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
