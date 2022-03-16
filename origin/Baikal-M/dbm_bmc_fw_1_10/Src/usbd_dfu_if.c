/**
  ******************************************************************************
  * @file           : usbd_dfu_if.c
  * @brief          :
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
#include "usbd_dfu_if.h"

/* USER CODE BEGIN INCLUDE */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "dbm_common.h"
#include "MT25Q.h"

extern FLASH_DEVICE_OBJECT  fdo;
extern TaskHandle_t  flashWriteTaskHandle;
extern QueueHandle_t flashWriteQueue;
extern uint32_t      flashAddrDest;
//extern struct flashBuffer_t gFlashBuffer;

uint8_t cmpBuff[1030];

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
/** @defgroup USBD_DFU 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_DFU_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_DFU_Private_Defines
  * @{
  */ 
#define FLASH_DESC_STR_0      "@Internal Flash   /0x08000000/04*016Kg,01*064Kg,07*128Kg"
#define FLASH_DESC_STR_1      "@Internal Flash   /0x09000000/03*016Ka,01*016Kg,01*064Kg,07*128Kg,04*016Kg,01*064Kg,07*128Kg"
#define FLASH_RW_BLOCK_SIZE   256
/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */
  
/**
  * @}
  */ 

/** @defgroup USBD_DFU_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_IF_Private_Variables
  * @{
  */
/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_DFU_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_DFU_Private_FunctionPrototypes
  * @{
  */
static uint16_t MEM_If_Init_FS(void);
static uint16_t MEM_If_Erase_FS (uint32_t Add);
static uint16_t MEM_If_Write_FS (uint8_t *src, uint8_t *dest, uint32_t Len);
static uint8_t *MEM_If_Read_FS  (uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t MEM_If_DeInit_FS(void);
static uint16_t MEM_If_GetStatus_FS (uint32_t Add, uint8_t Cmd, uint8_t *buffer);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
__ALIGN_BEGIN USBD_DFU_MediaTypeDef USBD_DFU_fops_FS __ALIGN_END =
{
   (uint8_t*)FLASH_DESC_STR_0,
    MEM_If_Init_FS,
    MEM_If_DeInit_FS,
    MEM_If_Erase_FS,
    MEM_If_Write_FS,
    MEM_If_Read_FS,
    MEM_If_GetStatus_FS,   
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  MEM_If_Init_FS
  *         Memory initialization routine.
  * @param  None
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Init_FS(void)
{ 
  /* USER CODE BEGIN 0 */
    return (USBD_OK);
  /* USER CODE END 0 */ 
}

/**
  * @brief  MEM_If_DeInit_FS
  *         De-Initializes Memory.
  * @param  None
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_DeInit_FS(void)
{ 
  /* USER CODE BEGIN 1 */ 
  return (USBD_OK);
  /* USER CODE END 1 */ 
}

/**
  * @brief  MEM_If_Erase_FS
  *         Erase sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
//static uint32_t sectorNumber[300];
uint16_t MEM_If_Erase_FS(uint32_t Add)
{
  /* USER CODE BEGIN 2 */
//	static uint32_t num = 0;
//	sectorNumber[num] = Add;
//	num++;

  // It was correct code
   fdo.GenOp.SectorErase(Add);
   flashAddrDest = 0x00;

  return (USBD_OK);
  /* USER CODE END 2 */ 
}

/**
  * @brief  MEM_If_Write_FS
  *         Memory write routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
//static uint8_t tmpBuff[1030];
//static uint8_t ffToCompare[1030];

uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
	/* USER CODE BEGIN 3 */
// This code don't uses. The function is exist
// only for interface compatibility
//	BaseType_t xHigherTaskWakeUp = pdFALSE;
//
//	for (uint16_t i = 0; i < Len; i++)
//	{
//		// xQueueSendToBackFromISR(flashWriteQueue, src + i, &xHigherTaskWakeUp);
//	}
//	vTaskNotifyGiveFromISR(flashWriteTaskHandle, &xHigherTaskWakeUp);
//
//
//    if (xHigherTaskWakeUp == pdTRUE)
//    {
//    	portYIELD_FROM_ISR(xHigherTaskWakeUp);
//    }



  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  MEM_If_Read_FS
  *         Memory read routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *MEM_If_Read_FS (uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* Return a valid address to avoid HardFault */
  /* USER CODE BEGIN 4 */
  //src -= 0x08000001;

  ParameterType para;
  para.Read.udAddr = src;
  para.Read.pArray = dest;
  para.Read.udNrOfElementsToRead = Len;

  fdo.GenOp.Enter4ByteAddressMode();
  fdo.Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;

  fdo.GenOp.DataRead(Read, &para);

  fdo.GenOp.Exit4ByteAddressMode();
  fdo.Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
  //return (uint8_t*)(USBD_OK);
  return (uint8_t*)(dest);
  /* USER CODE END 4 */ 
}

/**
  * @brief  Flash_If_GetStatus_FS
  *         Get status routine.
  * @param  Add: Address to be read from.
  * @param  Cmd: Number of data to be read (in bytes).
  * @param  buffer: used for returning the time necessary for a program or an erase operation
  * @retval 0 if operation is successful
  */
uint16_t MEM_If_GetStatus_FS (uint32_t Add, uint8_t Cmd, uint8_t *buffer)
{
  /* USER CODE BEGIN 5 */ 
  switch (Cmd)
  {
  case DFU_MEDIA_PROGRAM:

    break;
    
  case DFU_MEDIA_ERASE:
  default:

    break;
  }                             
  return  (USBD_OK);
  /* USER CODE END 5 */  
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

