/**
  ******************************************************************************
  * @file           : USB_DEVICE  
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device 
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_dfu.h"
#include "usbd_dfu_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stm32f4xx.h"

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "nn_spi.h"

extern SPI_HandleTypeDef hspi1;

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

uint8_t tstStr[50] = "String to USB.\r\n";

/* init function */                                        
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  
  /* USER CODE END USB_DEVICE_Init_PreTreatment */
  
  /* Init Device Library,Add Supported Class and Start the library*/
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);

  USBD_RegisterClass(&hUsbDeviceFS, &USBD_DFU);

  USBD_DFU_RegisterMedia(&hUsbDeviceFS, &USBD_DFU_fops_FS);
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);

  USBD_Start(&hUsbDeviceFS);

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  
  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

// ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~
void ledTask(void *args)
{
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    // uint8_t buf[10];
    // buf[0] = 0x9E;
	for(;;)
	{
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		//HAL_SPI_Transmit(&hspi1, buf, 13, 0);
		//HAL_SPI_Transmit_DMA(&hspi1, buf, 1);
		// HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); // - For NUCLEO  BOARD
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);   // - For BFK 3.1 BOARD
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

// ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~
void usbSendToTask(void *args)
{
	//USBD_StatusTypeDef trStatus = USBD_OK;
	for(;;)
	{
		//trStatus = USBD_LL_Transmit(&hUsbDeviceFS, 0x82, tstStr, strlen((const char*)tstStr));
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		CDC_Transmit_FS(tstStr, strlen((const char*)tstStr));
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

// ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~
void usbSendToConsole(void)
{
	// xTaskCreate(usbSendToTask, "usbSendToTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	//xTaskCreate(ledTask, "ledTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
