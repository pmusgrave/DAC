/*******************************************************************************
* File Name: AudioIn.h
*
* Version 1.0
*
*  Description:  This file contains the Audio In path routine declarations and 
*                constants.
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

#define AUDIO_SAMPLES_HIGH_48KHZ               49
#define AUDIO_SAMPLES_LOW_48KHZ                47
#define AUDIO_SAMPLES_HIGH_44KHZ               45
#define AUDIO_SAMPLES_LOW_44KHZ                43
#define AUDIO_SAMPLES_HIGH_32KHZ               33
#define AUDIO_SAMPLES_LOW_32KHZ                31
#define SINGLE_STEREO_AUDIO_SAMPLE_SIZE        6

/* Function definitions */
void InitializeAudioInPath(void);
void ProcessAudioIn(void);
void Stop_I2S_Rx(void);	

/* [] END OF FILE */
