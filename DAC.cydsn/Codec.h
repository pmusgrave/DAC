/*****************************************************************************
* File Name		: Codec.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  Codec.c
*
*******************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the Right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*******************************************************************************/

#ifndef CODEC_H
	#define CODEC_H	

	#include "cytypes.h"
		
	#define CODEC_I2C_ADDR			(0x1A)


	/**************************************************************************************************
	* Register Addresses for Codec I2C Interface
	**************************************************************************************************/
    #define CODEC_REG_DP_MODE_SELECT                (0x02)  /*DSD/PCM SELECTION*/
    #define CODEC_REG_DIGITAL_FILTER_SETTING_SD     (0x01)
    #define CODEC_REG_DIGITAL_FILTER_SETTING_SLOW   (0x02)
    #define CODEC_REG_AUTO_MAN_MODE_SETTING         (0x00)
    #define CODEC_REG_INTERFACE_SET                 (0x00)
    #define CODEC_REG_LCH_VOLUME                    (0x03)
    #define CODEC_REG_RCH_VOLUME                    (0x04)
    #define CODEC_REG_SOUND_QUALITY                 (0x08)
    #define CODEC_REG_MUTE                          (0x01)
    #define CODEC_REG_RESET                         (0x00)
    
    /**************************************************************************************************
    * Bit number locations within each Register Address
    **************************************************************************************************/
    #define CODEC_BIT_DP_MODE_SELECT                (7)
    #define CODEC_BIT_DIGITAL_FILTER_SETTING_SD     (5)
    #define CODEC_BIT_DIGITAL_FILTER_SETTING_SLOW   (0)
    #define CODEC_BIT_AUTO_MAN_MODE_SETTING         (7)
    #define CODEC_BIT_INTERFACE_SET                 (1)
    #define CODEC_BIT_LCH_VOLUME                    (0)
    #define CODEC_BIT_RCH_VOLUME                    (0)
    #define CODEC_BIT_SOUND_QUALITY                 (0)
    #define CODEC_BIT_MUTE                          (0)
    
    /**************************************************************************************************
    * Bit values for desired initial settings
    **************************************************************************************************/
    #define CODEC_DATA_DP_MODE_SELECT                (0)
    #define CODEC_DATA_DIGITAL_FILTER_SETTING_SD     (0)
    #define CODEC_DATA_DIGITAL_FILTER_SETTING_SLOW   (0)
    #define CODEC_DATA_AUTO_MAN_MODE_SETTING         (1)
    #define CODEC_DATA_INTERFACE_SET                 (010)
    #define CODEC_DATA_LCH_VOLUME                    (0xFF)
    #define CODEC_DATA_RCH_VOLUME                    (0XFF)
    #define CODEC_DATA_SOUND_QUALITY                 (00)
    #define CODEC_DATA_MUTE                          (0)

	#define CODEC_RESET_WAIT_DELAY				(10)	 /* in milli seconds */

	#define CODEC_HP_DEFAULT_VOLUME				(0)
	#define CODEC_HP_VOLUME_MAX					(255) 	/* 256 levels including MUTE, 0.5dB steps */	
	#define CODEC_HP_MUTE_VALUE					(0x2F) 	/* Writing <= 0x2F mutes the headphone output */
		
	/* Value format: bit[0] - BOSR, bit[4:1] - SR[3:0] */	
	#define CODEC_SRATE_NORMAL_48KHZ_256FS		(0x00) 	/* BOSR = 0 for 256fs and SRx = 0b0000 for 48 KHz in Normal mode */
	#define CODEC_SRATE_NORMAL_44KHZ_256FS		(0x20) 	/* BOSR = 0 for 256fs and SRx = 0b1000 for 44.1 KHz in Normal mode */
		
	#define CODEC_DEF_SAMPLING_RATE				CODEC_SRATE_NORMAL_48KHZ_256FS
	#define CODEC_DEF_ANALOG_CTRL				(CODEC_ANALOG_CTRL_INSEL | CODEC_ANALOG_CTRL_DACSEL | CODEC_ANALOG_CTRL_SIDETONE)
	#define CODEC_DEF_DIGITAL_CTRL				(CODEC_DIGITAL_CTRL_HPOR)
	#define CODEC_DEF_POWER_CTRL				(CODEC_POWER_CTRL_CLKOUTPD | CODEC_POWER_CTRL_OSCPD | CODEC_POWER_CTRL_LINEINPD)
    #define CODEC_CTRL_RESET                    (0b00000000)
		
	uint8 Codec_Init(void);
	uint8 Codec_SelectMicInputToADC(void);
	uint8 Codec_AdjustBothHeadphoneVolume(uint8 volume);
	uint8 Codec_MuteMic(_Bool isMuteOrUnmute);
    uint8 Codec_SetMicBoost(_Bool micBoost);
	uint8 Codec_SetSamplingRate(uint8 srCtrlField);
	uint8 Codec_Activate(void);
	uint8 Codec_Deactivate(void);
	uint8 Codec_ResetOverI2C(void);
	uint8 Codec_SendData(uint8 regAddr, uint16 data);
    uint8 Codec_PowerOffControl(uint32 powerOffMask);
    uint8 Codec_PowerOnControl(uint32 powerOnMask);
		
	
#endif /* #ifndef CODEC_H */

/* [] END OF FILE */
