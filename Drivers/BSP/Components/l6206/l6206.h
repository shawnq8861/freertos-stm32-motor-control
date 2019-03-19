/**
  ******************************************************************************
  * @file    l6206.h 
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    September 29th, 2016
  * @brief   Header for L6206 driver (dual full bridge driver)
  * @note    (C) COPYRIGHT 2016 STMicroelectronics
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
#ifndef __L6206_H
#define __L6206_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "l6206_target_config.h"
#include "motor.h"

/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup L6206
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup L6206_Exported_Constants L6206 Exported Constants
  * @{
  */   

/// Current FW version
#define L6206_FW_VERSION (0)
///Max number of Brush DC motors
#define L6206_NB_MAX_MOTORS (4)
///Max number of Bridges
#define L6206_NB_MAX_BRIDGES (2)
///Max number of input of Bridges
#define L6206_NB_MAX_BRIDGE_INPUT (2 * L6206_NB_MAX_BRIDGES)   
 /**
  * @}
  */

/** @addtogroup L6206_Exported_Variables
  * @{
  */    
    extern motorDrv_t   l6206Drv;
/**
  * @}
  */
     
/* Exported Types  -------------------------------------------------------*/

/** @defgroup L6206_Exported_Types L6206 Exported Types
  * @{
  */   


/** @defgroup Device_Parameters Device Parameters
  * @{
  */

/// Device Parameters Structure Type
typedef struct {
	/// Dual full bridge configuration
    dualFullBridgeConfig_t config;

    /// Pwm frequency of the bridge input
    uint32_t pwmFreq[L6206_NB_MAX_BRIDGE_INPUT];

    /// Speed% (from 0 to 100) of the corresponding motor
    uint16_t speed[L6206_NB_MAX_MOTORS];

    /// FORWARD or BACKWARD direction of the motors
    motorDir_t direction[L6206_NB_MAX_MOTORS];

    /// Current State of the motors
    motorState_t motionState[L6206_NB_MAX_MOTORS];

    /// Current State of the bridges
    bool bridgeEnabled[L6206_NB_MAX_BRIDGES];
}deviceParams_t; 

/**
  * @}
  */

/** @defgroup Initialization_Structure Initialization Structure
  * @{
  */

typedef struct
{
	/// Bridge configuration structure.
	dualFullBridgeConfig_t config;
	/// PWM frequency
    uint32_t pwmFreq;
    /// Motor speed
    uint16_t speed;
    /// Motor direction
    motorDir_t direction;
    /// Current motor state
    motorState_t motionState;
    /// Bridge enabled (true) or not (false)
    bool bridgeEnabled;
} L6206_InitTypeDef;

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/


/** @defgroup L6206_Exported_Functions L6206 Exported Functions
  * @{
  */   

void L6206_TickHandler(uint8_t deviceId);                                    //Handle the device state machine at each tick timer pulse end



void L6206_AttachErrorHandler(void (*callback)(uint16_t));               //Attach a user callback to the error handler
void L6206_AttachFlagInterrupt(void (*callback)(void));                  //Attach a user callback to the flag Interrupt
void L6206_DisableBridge(uint8_t bridgeId);                              //Disable the specified bridge
void L6206_EnableBridge(uint8_t bridgeId);                               //Enable the specified bridge
uint16_t L6206_GetBridgeStatus(uint8_t deviceId);                        //Get bridge status
void L6206_Init(void* init);                                      		 //Start the L6206 library
uint16_t L6206_GetCurrentSpeed(uint8_t motorId);                         //Return the current speed in pps
motorState_t L6206_GetDeviceState(uint8_t motorId);                      //Return the device state
uint32_t L6206_GetFwVersion(void);                                        //Return the FW version
uint16_t L6206_GetMaxSpeed(uint8_t motorId);                             //Return the max speed in pps
motorDrv_t* L6206_GetMotorHandle(void);                                  //Return handle of the motor driver handle
void L6206_HardHiz(uint8_t motorId);                                     //Stop the motor and disable the power bridge
void L6206_HardStop(uint8_t motorId);                                    //Stop the motor without disabling the power bridge
uint16_t L6206_ReadId(void);                                             //Read Id to get driver instance
void L6206_Run(uint8_t motorId, motorDir_t direction);                   //Run the motor 
uint32_t L6206_GetBridgeInputPwmFreq(uint8_t bridgeId);                  // Get the PWM frequency of the bridge input
void L6206_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq);    // Set the PWM frequency of the bridge input
void L6206_SetDualFullBridgeConfig(uint8_t newConfig);                   // Set dual full bridge configuration
bool L6206_SetMaxSpeed(uint8_t motorId,uint16_t newMaxSpeed);            //Set the max speed in pps
bool L6206_SetNbDevices(uint8_t nbDevices);								 //Set the number of driver devices

/**
  * @}
  */

/** @defgroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */   
///Delay of the requested number of milliseconds
extern void L6206_Board_Delay(uint32_t delay);
///Disable the specified bridge
extern void L6206_Board_DisableBridge(uint8_t bridgeId);
///Enable the specified bridge
extern void L6206_Board_EnableBridge(uint8_t bridgeId, uint8_t addDelay);
//Get the status of the flag and enable Pin
extern uint32_t L6206_Board_GetFlagPinState(uint8_t bridgeId);
///Initialise GPIOs used for L6206s
extern void L6206_Board_GpioInit(void);
///Set Briges Inputs PWM frequency and start it
extern void L6206_Board_PwmSetFreq(uint8_t bridgeInput, uint32_t newFreq, uint8_t duty);
///Deinitialise the PWM of the specified bridge input
extern void L6206_Board_PwmDeInit(uint8_t bridgeInput);
///Init the PWM of the specified bridge input
extern void L6206_Board_PwmInit(uint8_t bridgeInput);
///Stop the PWM of the specified bridge input
extern void L6206_Board_PwmStop(uint8_t bridgeInput);



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

#endif /* #ifndef __L6206_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
