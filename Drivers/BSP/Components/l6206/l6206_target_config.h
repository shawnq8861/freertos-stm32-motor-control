/**************************************************************************//**
  * @file    L6206_target_config.h
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    September 29th, 2016
  * @brief   Predefines values for the L6206 parameters
  * and for the devices parameters
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
#ifndef __L6206_TARGET_CONFIG_H
#define __L6206_TARGET_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup L6474
  * @{
  */   

/** @addtogroup L6206_Exported_Constants
  * @{
  */   
   
/** @defgroup Predefined_L6206_Parameters_Values Predefined L6206 Parameters Values
  * @{
  */   

/// The maximum number of L6206 devices 
#define MAX_NUMBER_OF_DEVICES                 (1)
   
/// The maximum number of BLDC motors connected to the L6206
#define MAX_NUMBER_OF_BRUSH_DC_MOTORS                    (4)

/// Frequency of PWM of Input 1 Bridge A in Hz up to 100000Hz
#define L6206_CONF_PARAM_FREQ_PWM1A  (20000)
/// Frequency of PWM of Input 2 Bridge A in Hz up to 100000Hz
/// ON IHM04A1, must be identical to L6206_CONF_PARAM_FREQ_PWM1A as used timer is the same
#define L6206_CONF_PARAM_FREQ_PWM2A  (20000)
/// Frequency of PWM of Input 1 Bridge B in Hz up to 100000Hz
#define L6206_CONF_PARAM_FREQ_PWM1B  (20000)
/// Frequency of PWM of Input 2 Bridge B in Hz up to 100000Hz
/// On IHM04A1, must be identical to L6206_CONF_PARAM_FREQ_PWM2B as used timer is the same    
#define L6206_CONF_PARAM_FREQ_PWM2B  (20000)

/// Frequency of PWM of Input 2 Bridge B (in kHz)
#define L6206_CONF_PARAM_PARALLE_BRIDGES (PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B)
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

#endif /* __L6206_TARGET_CONFIG_H */
