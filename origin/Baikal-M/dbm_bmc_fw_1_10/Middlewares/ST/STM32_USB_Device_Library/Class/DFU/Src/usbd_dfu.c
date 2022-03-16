/**
  ******************************************************************************
  * @file    usbd_dfu.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the DFU core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                DFU Class Driver Description
  *          =================================================================== 
  *           This driver manages the DFU class V1.1 following the "Device Class Specification for 
  *           Device Firmware Upgrade Version 1.1 Aug 5, 2004".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as DFU device (in DFU mode only)
  *             - Requests management (supporting ST DFU sub-protocol)
  *             - Memory operations management (Download/Upload/Erase/Detach/GetState/GetStatus)
  *             - DFU state machine implementation.
  *          
  *           @note
  *            ST DFU sub-protocol is compliant with DFU protocol and use sub-requests to manage
  *            memory addressing, commands processing, specific memories operations (ie. Erase) ...
  *            As required by the DFU specification, only endpoint 0 is used in this application.
  *            Other endpoints and functions may be added to the application (ie. DFU ...)
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *           This driver doesn't implement the following aspects of the specification 
  *           (but it is possible to manage these features with some modifications on this driver):
  *             - Manifestation Tolerant mode
  *      
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "MT25Q.h"
#include "dbm_pins.h"
#include "usbd_cdc_if.h"

#include "nn_serial.h"

SemaphoreHandle_t sduResBlk = NULL;

extern FLASH_DEVICE_OBJECT fdo;

extern SemaphoreHandle_t xSpiWrite;
extern TaskHandle_t  flashEraseTaskHandle;
extern TaskHandle_t  flashWriteTaskHandle;
extern TaskHandle_t  flashOperationTaskHandle;


int gs_tst = 0;
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_DFU 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_DFU_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_DFU_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_DFU_Private_Macros
  * @{
  */ 
#define DFU_SAMPLE_FREQ(frq)      (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define DFU_PACKET_SZE(frq)          (uint8_t)(((frq * 2 * 2)/1000) & 0xFF), \
                                       (uint8_t)((((frq * 2 * 2)/1000) >> 8) & 0xFF)
                                         
/**
  * @}
  */ 



#define USB_ENDPOINT_DESCRIPTOR_TYPE                0x05
//#define CDC_DATA_FS_MAX_PACKET_SIZE                 128  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */
#define USB_INTERFACE_DESCRIPTOR_TYPE               0x04
//#define RNDIS_NOTIFICATION_IN_SZ 0x08
//#define RNDIS_DATA_IN_SZ         0x40
//#define RNDIS_DATA_OUT_SZ        0x40

/** @defgroup USBD_DFU_Private_FunctionPrototypes
  * @{
  */

static uint8_t  USBD_CDC_DFU_Init(USBD_HandleTypeDef *pdev,
                                  uint8_t cfgidx);
static uint8_t  USBD_DFU_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_CDC_DFU_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_DFU_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_CDC_DFU_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_DFU_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_DFU_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_DFU_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_CDC_DFU_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_DFU_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_CDC_DFU_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_DFU_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_DFU_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_DFU_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_DFU_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_DFU_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_DFU_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

#if (USBD_SUPPORT_USER_STRING == 1)  
static uint8_t* USBD_DFU_GetUsrStringDesc ( USBD_HandleTypeDef *pdev, uint8_t index , uint16_t *length);
#endif

static void DFU_Detach    (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void DFU_Download  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void DFU_Upload    (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void DFU_GetStatus (USBD_HandleTypeDef *pdev);

static void DFU_ClearStatus (USBD_HandleTypeDef *pdev);

static void DFU_GetState  (USBD_HandleTypeDef *pdev);

static void DFU_Abort     (USBD_HandleTypeDef *pdev);

static void DFU_Leave  (USBD_HandleTypeDef *pdev); 

/**
  * @}
  */ 

/** @defgroup USBD_DFU_Private_Variables
  * @{
  */ 

USBD_ClassTypeDef  USBD_DFU = 
{
  USBD_CDC_DFU_Init,
  USBD_CDC_DFU_DeInit,
  USBD_CDC_DFU_Setup,
  USBD_DFU_EP0_TxReady,  
  USBD_DFU_EP0_RxReady, // Just for test; was USBD_CDC_EP0_RxReady
  USBD_CDC_DFU_DataIn,
  USBD_CDC_DFU_DataOut,
  USBD_DFU_SOF,
  USBD_DFU_IsoINIncomplete,
  USBD_DFU_IsoOutIncomplete,      
  USBD_DFU_GetCfgDesc,
  USBD_DFU_GetCfgDesc, 
  USBD_DFU_GetCfgDesc,
  USBD_DFU_GetDeviceQualifierDesc,
#if (USBD_SUPPORT_USER_STRING == 1)  
  USBD_DFU_GetUsrStringDesc
#endif
};

/* USB DFU device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_DFU_CfgDesc[USB_DFU_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_DFU_CONFIG_DESC_SIZ,     /* wTotalLength: Bytes returned */
  0x00,
  0x04,         /*bNumInterfaces: 2 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supprts Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /* ----------  ---------- ---------- ---------- ---------- ---------- ---------- ----------*/
  /* IAD descriptor for ACM */
  0x08, /* bLength */
  0x0B, /* bDescriptorType */
  0x00, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass (Wireless Controller) */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x00, /* iFunction */
  	  /*---------------------------------------------------------------------------*/
	  /* Interface 0 Descriptor */
	  0x09,   /* bLength: Interface Descriptor size */
	  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
	  0x00,   /* bInterfaceNumber: Number of Interface */
	  0x00,   /* bAlternateSetting: Alternate setting */
	  0x01,   /* bNumEndpoints: One endpoints used */
	  0x02,   /* bInterfaceClass: Communication Interface Class */
	  0x02,   /* bInterfaceSubClass: Abstract Control Model */
	  0x01,   /* bInterfaceProtocol: Common AT commands */
	  0x08,   /* iInterface: */

	      /* Functional descriptors */
		  /* Header Functional Descriptor*/
		  0x05,   /* bLength: Endpoint Descriptor size */
		  0x24,   /* bDescriptorType: CS_INTERFACE */
		  0x00,   /* bDescriptorSubtype: Header Func Desc */
		  0x10,   /* bcdCDC: spec release number */
		  0x01,

		  /* Call Management Functional Descriptor*/
		  0x05,   /* bFunctionLength */
		  0x24,   /* bDescriptorType: CS_INTERFACE */
		  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
		  0x00,   /* bmCapabilities: D0+D1 */
		  0x01,   /* bDataInterface: 1 */

		  /* ACM Functional Descriptor*/
		  0x04,   /* bFunctionLength */
		  0x24,   /* bDescriptorType: CS_INTERFACE */
		  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
		  0x02,   /* bmCapabilities */

		  /* Union Functional Descriptor*/
		  0x05,   /* bFunctionLength */
		  0x24,   /* bDescriptorType: CS_INTERFACE */
		  0x06,   /* bDescriptorSubtype: Union func desc */
		  0x00,   /* bMasterInterface: Communication class interface */
		  0x01,   /* bSlaveInterface0: Data Class Interface */

			  /* EP1 Descriptor for interface 0*/
			  0x07,                           /* bLength: Endpoint Descriptor size */
			  USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
			  CDC_CMD_EP,                     /* bEndpointAddress */
			  0x03,                           /* bmAttributes: Interrupt */
			  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
			  HIBYTE(CDC_CMD_PACKET_SIZE),
			  0x01,                           /* bInterval: */
	  /*---------------------------------------------------------------------------*/
	  /*Interface 1 descriptor for Data */
	  0x09,   /* bLength: Endpoint Descriptor size */
	  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
	  0x01,   /* bInterfaceNumber: Number of Interface */
	  0x00,   /* bAlternateSetting: Alternate setting */
	  0x02,   /* bNumEndpoints: Two endpoints used */
	  0x0A,   /* bInterfaceClass: CDC */
	  0x00,   /* bInterfaceSubClass: */
	  0x00,   /* bInterfaceProtocol: */
	  0x09,   /* iInterface: */

		  /*Endpoint OUT Descriptor*/
		  0x07,   /* bLength: Endpoint Descriptor size */
		  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		  CDC_OUT_EP,                        /* bEndpointAddress */
		  0x02,                              /* bmAttributes: Bulk */
		  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
		  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
		  0x00,                              /* bInterval: ignore for Bulk transfer */

		  /*Endpoint IN Descriptor*/
		  0x07,   /* bLength: Endpoint Descriptor size */
		  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		  CDC_IN_EP,                         /* bEndpointAddress */
		  0x02,                              /* bmAttributes: Bulk */
		  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
		  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
		  0x00,                               /* bInterval: ignore for Bulk transfer */

  /**********  Descriptor of DFU interface 0 Alternate setting 0 **************/  
  USBD_DFU_IF_DESC(0), /* This interface is mandatory for all devices */
  
#if (USBD_DFU_MAX_ITF_NUM > 1)
  /**********  Descriptor of DFU interface 0 Alternate setting 1 **************/ 
  USBD_DFU_IF_DESC(1),
#endif /* (USBD_DFU_MAX_ITF_NUM > 1) */

#if (USBD_DFU_MAX_ITF_NUM > 2)
  /**********  Descriptor of DFU interface 0 Alternate setting 2 **************/ 
  USBD_DFU_IF_DESC(2),
#endif /* (USBD_DFU_MAX_ITF_NUM > 2) */

#if (USBD_DFU_MAX_ITF_NUM > 3)
  /**********  Descriptor of DFU interface 0 Alternate setting 3 **************/ 
  USBD_DFU_IF_DESC(3),
#endif /* (USBD_DFU_MAX_ITF_NUM > 3) */

#if (USBD_DFU_MAX_ITF_NUM > 4)
  /**********  Descriptor of DFU interface 0 Alternate setting 4 **************/ 
  USBD_DFU_IF_DESC(4),
#endif /* (USBD_DFU_MAX_ITF_NUM > 4) */

#if (USBD_DFU_MAX_ITF_NUM > 5)
  /**********  Descriptor of DFU interface 0 Alternate setting 5 **************/ 
  USBD_DFU_IF_DESC(5),
#endif /* (USBD_DFU_MAX_ITF_NUM > 5) */

#if (USBD_DFU_MAX_ITF_NUM > 6)
#error "ERROR: usbd_dfu_core.c: Modify the file to support more descriptors!"
#endif /* (USBD_DFU_MAX_ITF_NUM > 6) */

  /******************** DFU Functional Descriptor********************/
  0x09,   /*blength = 9 Bytes*/
  DFU_DESCRIPTOR_TYPE,   /* DFU Functional Descriptor*/
  0x0B,   /*bmAttribute
                bitCanDnload             = 1      (bit 0)
                bitCanUpload             = 1      (bit 1)
                bitManifestationTolerant = 0      (bit 2)
                bitWillDetach            = 1      (bit 3)
                Reserved                          (bit4-6)
                bitAcceleratedST         = 0      (bit 7)*/
  0xFF,   /*DetachTimeOut= 255 ms*/
  0x00,
  /*WARNING: In DMA mode the multiple MPS packets feature is still not supported
   ==> In this case, when using DMA USBD_DFU_XFER_SIZE should be set to 64 in usbd_conf.h */
  TRANSFER_SIZE_BYTES(USBD_DFU_XFER_SIZE),       /* TransferSize = 1024 Byte*/         
  0x1A,                                /* bcdDFUVersion*/
  0x01,
  /* ----------  ---------- ---------- ---------- ---------- ---------- ---------- ----------*/
  /* IAD descriptor for Mass Storage BEGIN */
  0x08, /* bLength */
  0x0B, /* bDescriptorType */
  0x03, /* bFirstInterface */
  0x01, /* bInterfaceCount */
  0x08, /* bFunctionClass (Mass Storage) */
  0x06, /* bFunctionSubClass (SCSI)*/
  0x50, /* bFunctionProtocol (Bulk-Only)*/
  0x00, /* iFunction */
	  /*---------------------------------------------------------------------------*/
	  /* Interface 3 Descriptor */
	  0x09,   /* bLength: Interface Descriptor size */
	  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
	  0x03,   /* bInterfaceNumber: Number of Interface */
	  0x00,   /* bAlternateSetting: Alternate setting */
	  0x02,   /* bNumEndpoints: Two endpoints used */
	  0x08,   /* bInterfaceClass: Mass Storage */
	  0x06,   /* bInterfaceSubClass: SCSI */
	  0x50,   /* bInterfaceProtocol: Bulk-Only */
	  0x00,   /* iInterface: */
		  /*Endpoint OUT Descriptor*/
		  0x07,   /* bLength: Endpoint Descriptor size */
		  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		  0x83,                        /* bEndpointAddress */
		  0x02,                        /* bmAttributes: Bulk */
		  0x00,                        /* wMaxPacketSize: */
		  0x02,
		  0x00,                        /* bInterval: ignore for Bulk transfer */

		  /*Endpoint IN Descriptor*/
		  0x07,   /* bLength: Endpoint Descriptor size */
		  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		  0x03,                        /* bEndpointAddress */
		  0x02,                        /* bmAttributes: Bulk */
		  0x00,  /* wMaxPacketSize: */
		  0x02,
		  0x00,
  /* IAD descriptor for Mass Storage END */
};
  
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_DFU_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_DFU_Private_Functions
  * @{
  */ 

static uint8_t  USBD_CDC_DFU_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	//sduResBlk = xSemaphoreCreateBinary();
	uint8_t result = USBD_OK;
	result = USBD_DFU_Init(pdev, cfgidx);
	result = USBD_CDC_Init(pdev, cfgidx);
	return result;
}
/**
  * @brief  USBD_DFU_Init
  *         Initialize the DFU interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_DFU_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  USBD_DFU_HandleTypeDef   *hdfu;
  
 /* Allocate Audio structure */
  pdev->pDfuClassData = USBD_malloc(sizeof (USBD_DFU_HandleTypeDef));
  
  if(pdev->pDfuClassData == NULL)
  {
    return USBD_FAIL; 
  }
  else
  {
    hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
    
    hdfu->alt_setting = 0;
    hdfu->data_ptr = USBD_DFU_APP_DEFAULT_ADD;
    hdfu->wblock_num = 0;
    hdfu->wlength = 0;
    
    hdfu->manif_state = DFU_MANIFEST_COMPLETE;
    hdfu->dev_state = DFU_STATE_IDLE;
    
    hdfu->dev_status[0] = DFU_ERROR_NONE;
    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;   
    hdfu->dev_status[3] = 0;
    hdfu->dev_status[4] = DFU_STATE_IDLE;    
    hdfu->dev_status[5] = 0;    
    
    /* Initialize Hardware layer */
    if (((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Init() != USBD_OK)
    {
      return USBD_FAIL;
    }   
  }
  return USBD_OK;
}

static uint8_t  USBD_CDC_DFU_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	uint8_t result = USBD_OK;
	result = USBD_DFU_DeInit(pdev, cfgidx);
	result = USBD_CDC_DeInit(pdev, cfgidx);
	return result;
}
/**
  * @brief  USBD_DFU_Init
  *         De-Initialize the DFU layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_DFU_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  USBD_DFU_HandleTypeDef   *hdfu;
  hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
  
  hdfu->wblock_num = 0;
  hdfu->wlength = 0;

  hdfu->dev_state = DFU_STATE_IDLE;
  hdfu->dev_status[0] = DFU_ERROR_NONE;
  hdfu->dev_status[4] = DFU_STATE_IDLE;
 
  /* DeInit  physical Interface components */
  if(pdev->pDfuClassData != NULL)
  {
    /* De-Initialize Hardware layer */
    ((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->DeInit();
    USBD_free(pdev->pDfuClassData);
    pdev->pDfuClassData = NULL;
  } 

  return USBD_OK;
}

static uint8_t  USBD_CDC_DFU_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	uint8_t result = USBD_OK;
	if (((req->bmRequest & 0x1F) == 0x1) && (req->wIndex == 0x2))
	{
		result = USBD_DFU_Setup(pdev, req);
	}
	else
	{
		result = USBD_CDC_Setup(pdev, req);
	}
	return result;
}

/**
  * @brief  USBD_DFU_Setup
  *         Handle the DFU specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_DFU_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  uint8_t *pbuf = 0;
  uint16_t len = 0;
  uint8_t ret = USBD_OK;
  USBD_DFU_HandleTypeDef   *hdfu;
  
  hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
  

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :  
    switch (req->bRequest)
    {
    case DFU_DNLOAD:
      DFU_Download(pdev, req);
      break;
      
    case DFU_UPLOAD:
      DFU_Upload(pdev, req);   
      break;
      
    case DFU_GETSTATUS:
    {
    	hdfu->wlength    = req->wLength;
    	hdfu->wblock_num = req->wValue;
        DFU_GetStatus(pdev);
        break;
    }
      
    case DFU_CLRSTATUS:
      DFU_ClearStatus(pdev);
      break;      
      
    case DFU_GETSTATE:
      DFU_GetState(pdev);
      break;  
      
    case DFU_ABORT:
      DFU_Abort(pdev);
      break;
      
    case DFU_DETACH:
      DFU_Detach(pdev, req);
      break;
      
      
    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL; 
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == DFU_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_DFU_CfgDesc + (9 * (USBD_DFU_MAX_ITF_NUM + 1));
        len = MIN(USB_DFU_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&hdfu->alt_setting,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < USBD_DFU_MAX_ITF_NUM)
      {
        hdfu->alt_setting = (uint8_t)(req->wValue);
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;  
      }
      break;
      
    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;     
    }
  }
  return ret;
}


/**
  * @brief  USBD_DFU_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_DFU_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_DFU_CfgDesc);
  return USBD_DFU_CfgDesc;
}

static uint8_t  USBD_CDC_DFU_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t result = USBD_OK;

	//if (epnum == 0x1)
	if (epnum == (CDC_IN_EP & (~0x80))) // Reset direction flag
	{
		result = USBD_CDC_DataIn(pdev, epnum);
	}
	else
	{
		result = USBD_DFU_DataIn(pdev, epnum);
	}
	return result;
}
/**
  * @brief  USBD_DFU_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_DFU_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{

  return USBD_OK;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static uint8_t USBD_DFU_ProcessCommand(USBD_HandleTypeDef *pdev)
{
	USBD_SetupReqTypedef     req;
	USBD_DFU_HandleTypeDef   *hdfu;
	hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;

	uint8_t outCmd = hdfu->buffer.d8[0];

	if (( outCmd ==  DFU_CMD_ERASE ) && (hdfu->wlength == 5))
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreTakeFromISR(xSpiWrite, &xHigherPriorityTaskWoken);

		if (flashEraseTaskHandle != NULL)
		{
			hdfu->data_ptr  = hdfu->buffer.d8[1];
			hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
			hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
			hdfu->data_ptr += hdfu->buffer.d8[4] << 24;

			// We have two DFU devices. One starts for 0x08000000
			// the other from 0x0A000000. To align to 0x00000000
			// subtract base from address.
			if (hdfu->data_ptr >= 0x0A000000)
			{
				hdfu->data_ptr -= 0x0A000000;
			}
			else
			{
				hdfu->data_ptr -= 0x08000000;
			}

			/* Reset writeBlockCount it will be used during write operations */

			BaseType_t xHigherTaskWakeUp = pdFALSE;
			xSemaphoreTakeFromISR(xSpiWrite, &xHigherTaskWakeUp);

			USBD_DFU_HandleTypeDef   *hdfu;
			hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;

			xTaskNotifyFromISR(flashEraseTaskHandle, (uint32_t) hdfu, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
		}

		hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;
		hdfu->dev_status[1] = 30;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;

		hdfu->processedCmd = 1;

		if (xHigherPriorityTaskWoken == pdTRUE)
		{
			portYIELD_FROM_ISR(&xHigherPriorityTaskWoken);
		}
		// There we need to call Erase Function;
		return USBD_OK;
	}
	else if (( outCmd ==  DFU_CMD_MASS_ERASE ) && (hdfu->wlength == 1))
	{
		hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;
		hdfu->dev_status[1] = 10;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;

		hdfu->processedCmd = 2;
		// There we need to call Mass Erase Function;
		return USBD_OK;
	}
	else if  (( outCmd ==  DFU_CMD_SETADDRESSPOINTER ) && (hdfu->wlength == 5))
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreTakeFromISR(xSpiWrite, &xHigherPriorityTaskWoken);

		// if (flashEraseTaskHandle != NULL)
//		if (flashOperationTaskHandle != NULL)
//		{
//			xTaskNotifyFromISR(flashOperationTaskHandle, 0, 0, &xHigherPriorityTaskWoken);
//		}

//		if (xHigherPriorityTaskWoken == pdTRUE)
//		{
//			portYIELD_FROM_ISR(&xHigherPriorityTaskWoken);
//		}

		//char mess[] = "STATUS 1";
		//vSerialPutString(NULL, mess, strlen(mess));
		hdfu->data_ptr  = hdfu->buffer.d8[1];
		hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
		hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
		hdfu->data_ptr += hdfu->buffer.d8[4] << 24;

		// We have two DFU devices. One starts for 0x08000000
		// the other from 0x0A000000. To align to 0x00000000
		// subtract base from address.
		if (hdfu->data_ptr >= 0x0A000000)
		{
			hdfu->data_ptr -= 0x0A000000;
		}
		else
		{
			hdfu->data_ptr -= 0x08000000;
		}

		hdfu->wlength = 0;
		hdfu->wblock_num = 0;
		hdfu->processedCmd = 3;

		/* Update the state machine */
		hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;
		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;

//		if (xHigherPriorityTaskWoken == pdTRUE)
//		{
//			portYIELD_FROM_ISR(&xHigherPriorityTaskWoken);
//		}

		//mess[7] = "2";
		//vSerialPutString(NULL, mess, strlen(mess));
		gs_tst = 1;
		return USBD_OK;

	}
	else if  (( outCmd ==  DFU_CMD_READ_UNPROTECT ) && (hdfu->wlength == 1))
	{
		hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;

		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;

		hdfu->processedCmd = 0;
		// There we need to call Read data
		return USBD_OK;
	}
    if ((outCmd ==  DFU_CMD_GETCOMMANDS) && (hdfu->wlength == 1))
    {
    	return USBD_OK;
    }
	// It seems to me to switch to Idle state if incorrect command
	// and return error to DFU
	else
	{
		hdfu->dev_state =  DFU_STATE_DNLOAD_IDLE;

		hdfu->dev_status[1] = 0;
		hdfu->dev_status[2] = 0;
		hdfu->dev_status[3] = 0;
		hdfu->dev_status[4] = hdfu->dev_state;

        req.bmRequest = 0;
        req.wLength = 1;
        hdfu->processedCmd = 0;
        USBD_CtlError (pdev, &req);
		return USBD_OK;
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static uint8_t USBD_DFU_ProcessWriteOp(USBD_HandleTypeDef *pdev)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// xSemaphoreTakeFromISR(xSpiWrite, &xHigherPriorityTaskWoken);

	if (flashWriteTaskHandle != NULL)
	{
//		USBD_DFU_HandleTypeDef   *hdfu;
//		hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
//		xTaskNotifyFromISR(flashEraseTaskHandle, hdfu->data_ptr/65536, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
		// hdfu->data_ptr contains flash address where data should be written;
		// hdfu->buffer contains data that should be written;
		USBD_DFU_HandleTypeDef   *hdfu;
		hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
		hdfu->dataLength = hdfu->wlength;
		if (pdTRUE == xSemaphoreTakeFromISR(xSpiWrite, &xHigherPriorityTaskWoken))
		{
			// xSemaphoreGiveFromISR(xSpiWrite, &xHigherPriorityTaskWoken);
			xTaskNotifyFromISR(flashWriteTaskHandle, (uint32_t) hdfu, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
		}
	}

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR(&xHigherPriorityTaskWoken);
	}
	return USBD_OK;
}
/**
  * @brief  USBD_DFU_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_DFU_EP0_RxReady (USBD_HandleTypeDef *pdev)
{

	// volatile static uint32_t tst = 0;
	//tst++;
	//BaseType_t res = pdFALSE;
	//xSemaphoreGiveFromISR(sduResBlk, &res);
	USBD_DFU_HandleTypeDef   *hdfu;
	hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;


    if (hdfu->dev_state == DFU_STATE_DNLOAD_SYNC)
    {
        if (hdfu->wblock_num == 0)
        {
        	USBD_DFU_ProcessCommand(pdev);
        }
        else
        {
        	USBD_DFU_ProcessWriteOp(pdev);
        }

    }

    return USBD_OK;
}
/*
 * This function calls when communication completes.
 */
/**
  * @brief  USBD_DFU_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_DFU_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
    uint32_t addr;
    USBD_SetupReqTypedef     req;
    USBD_DFU_HandleTypeDef   *hdfu;
    static uint32_t stateIt = 0;
    USBD_StatusTypeDef       usbResult = USBD_OK;

    static uint32_t eraseCount = 0;

    static uint32_t writeBlockCount = 0;

//    hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
//
//    if (hdfu->dev_state == DFU_STATE_DNLOAD_BUSY)
//    {
//    	hdfu->dev_state =  DFU_STATE_DNLOAD_IDLE;
//		hdfu->processedCmd  = 0;
//		hdfu->dev_status[1] = 0;
//		hdfu->dev_status[2] = 0;
//		hdfu->dev_status[3] = 0;
//		hdfu->dev_status[4] = hdfu->dev_state;
//    }

//    if ((hdfu->dev_state == DFU_STATE_DNLOAD_BUSY) && (hdfu->processedCmd == 0))
//    {
//    	hdfu->processedCmd++;
//    }
//    else
//    {
//		hdfu->dev_state =  DFU_STATE_DNLOAD_IDLE;
//
//		hdfu->processedCmd  = 0;
//		hdfu->dev_status[1] = 0;
//		hdfu->dev_status[2] = 0;
//		hdfu->dev_status[3] = 0;
//		hdfu->dev_status[4] = hdfu->dev_state;
//    }


//    hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
//    if ((hdfu->dev_state == DFU_STATE_DNLOAD_BUSY) || (hdfu->dev_state == DFU_STATE_DNLOAD_SYNC))
//    {
//        /* Decode the Special Command*/
//        if (hdfu->wblock_num == 0)
//        {
//            if ((hdfu->buffer.d8[0] ==  DFU_CMD_GETCOMMANDS) && (hdfu->wlength == 1))
//            {
//
//            }
//            else if  (( hdfu->buffer.d8[0] ==  DFU_CMD_SETADDRESSPOINTER ) && (hdfu->wlength == 5))
//            {
//                hdfu->data_ptr  = hdfu->buffer.d8[1];
//                hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
//                hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
//                hdfu->data_ptr += hdfu->buffer.d8[4] << 24;
//
//                hdfu->wlength = 0;
//                hdfu->wblock_num = 0;
//
//                /* Update the state machine */
//                hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;
//
//                hdfu->dev_status[1] = 0;
//                hdfu->dev_status[2] = 0;
//                hdfu->dev_status[3] = 0;
//                hdfu->dev_status[4] = hdfu->dev_state;
//                return USBD_OK;
//            }
//            else if (( hdfu->buffer.d8[0] ==  DFU_CMD_ERASE ) && (hdfu->wlength == 5))
//            {
//                hdfu->data_ptr  = hdfu->buffer.d8[1];
//                hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
//                hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
//                hdfu->data_ptr += hdfu->buffer.d8[4] << 24;
//
//                hdfu->data_ptr -= 0x08000000;
//
//                /* Reset writeBlockCount it will be used during write operations */
//                writeBlockCount = 0;
//
//    	        BaseType_t xHigherTaskWakeUp = pdFALSE;
//                xSemaphoreTakeFromISR(xSpiWrite, &xHigherTaskWakeUp);
//                // 4096 - Is flash subsector size. We send subsector number into erase function;
//                // eraseBlockNum[tmpEraswCount] = (hdfu->data_ptr)/4096;
//                // tmpEraswCount++;
//                // We do sector EraseCommand 0xD8 16Mbyte flash has 256 sectors from 0 to 255
//                // The sector size is 65536(bytes) or 64Kbyte
//                if (((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Erase((hdfu->data_ptr)/65536) != USBD_OK)
//                {
//                    return USBD_FAIL;
//                }
//
//
//    	        vTaskNotifyGiveFromISR(flashEraseTaskHandle, &xHigherTaskWakeUp);
//                if (xHigherTaskWakeUp == pdTRUE)
//                {
//        	        portYIELD_FROM_ISR(xHigherTaskWakeUp);
//                }
//            }
//            else if (( hdfu->buffer.d8[0] ==  DFU_CMD_ERASE ) && (hdfu->wlength == 0))
//            {
//    	        if (eraseCount < 5)
//    	        {
//    		        eraseCount++;
//    	        }
//    	        else
//    	        {
//    		        hdfu->wlength = 0;
//    		        hdfu->wblock_num = 0;
//
//    		        /* Update the state machine */
//    		        //hdfu->dev_state =  DFU_STATE_DNLOAD_SYNC;
//    		        hdfu->dev_state =  DFU_STATE_DNLOAD_IDLE;
//
//    		        hdfu->dev_status[1] = 0;
//    		        hdfu->dev_status[2] = 0;
//    		        hdfu->dev_status[3] = 0;
//    		        hdfu->dev_status[4] = hdfu->dev_state;
//
//    		        eraseCount = 0;
//    		        return USBD_OK;
//    	        }
//                // This IF means previous command was DFU_CMD_ERASE it means wait until it finishes
//    	        // Here we need to check erase operation finish
//            }
//            else if  (( hdfu->buffer.d8[0] ==  DFU_CMD_SETADDRESSPOINTER ) && (hdfu->wlength == 0))
//            {
//                hdfu->data_ptr  = hdfu->buffer.d8[1];
//                hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
//                hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
//                hdfu->data_ptr += hdfu->buffer.d8[4] << 24;
//
//                hdfu->wlength = 0;
//                hdfu->wblock_num = 0;
//
//                /* Update the state machine */
//                hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;
//
//                hdfu->dev_status[1] = 0;
//                hdfu->dev_status[2] = 0;
//                hdfu->dev_status[3] = 0;
//                hdfu->dev_status[4] = hdfu->dev_state;
//                return USBD_OK;
//            }
//            else
//            {
//                /* Reset the global length and block number */
//                hdfu->wlength = 0;
//                hdfu->wblock_num = 0;
//                /* Call the error management function (command will be nacked) */
//                req.bmRequest = 0;
//                req.wLength = 1;
//                USBD_CtlError (pdev, &req);
//            }
//        }
//        /* Regular Download Command */
//        else if (hdfu->wblock_num > 1)
//        {
//            /* Decode the required address */
//            //addr = ((hdfu->wblock_num - 2) * USBD_DFU_XFER_SIZE) + hdfu->data_ptr;
//
//            //addr = ((hdfu->wblock_num - 2) * USBD_DFU_XFER_SIZE);
//            addr = writeBlockCount * USBD_DFU_XFER_SIZE;
//            writeBlockCount++;
//
//            // transferSize += hdfu->wlength;
//
//            /* Preform the write operation */
//            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//            if (pdTRUE == xSemaphoreTakeFromISR(xSpiWrite, &xHigherPriorityTaskWoken))
//            {
//    	        usbResult = ((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Write(hdfu->buffer.d8, (uint8_t *)addr, hdfu->wlength);
//    	        if (pdTRUE == xHigherPriorityTaskWoken)
//    	        {
//    		        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    	        }
//            }
//
//
//            // if (((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Write(hdfu->buffer.d8, (uint8_t *)addr, hdfu->wlength) != USBD_OK)
//            // {
//            // return USBD_FAIL;
//            // }
//        }
//
//        /* Reset the global length and block number */
//        hdfu->wlength = 0;
//        hdfu->wblock_num = 0;
//
//        /* Update the state machine */
//        hdfu->dev_state =  DFU_STATE_DNLOAD_SYNC;
//
//        hdfu->dev_status[1] = 0;
//        hdfu->dev_status[2] = 0;
//        hdfu->dev_status[3] = 0;
//        hdfu->dev_status[4] = hdfu->dev_state;
//        return USBD_OK;
//    }
//    else if (hdfu->dev_state == DFU_STATE_MANIFEST)/* Manifestation in progress*/
//    {
//        /* Start leaving DFU mode */
//        DFU_Leave(pdev);
//    }

// --------------------------------------------------------------------------------------------------------------------
//    if ((hdfu->dev_state == DFU_STATE_DNLOAD_BUSY) || (hdfu->dev_state == DFU_STATE_DNLOAD_SYNC))
//    {
//        /* Decode the Special Command*/
//        if (hdfu->wblock_num == 0)
//        {
//            if ((hdfu->buffer.d8[0] ==  DFU_CMD_GETCOMMANDS) && (hdfu->wlength == 1))
//            {
//
//            }
//            else if  (( hdfu->buffer.d8[0] ==  DFU_CMD_SETADDRESSPOINTER ) && (hdfu->wlength == 5))
//            {
//                hdfu->data_ptr  = hdfu->buffer.d8[1];
//                hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
//                hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
//                hdfu->data_ptr += hdfu->buffer.d8[4] << 24;
//
//                hdfu->wlength = 0;
//                hdfu->wblock_num = 0;
//
//                /* Update the state machine */
//                hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;
//
//                hdfu->dev_status[1] = 0;
//                hdfu->dev_status[2] = 0;
//                hdfu->dev_status[3] = 0;
//                hdfu->dev_status[4] = hdfu->dev_state;
//                return USBD_OK;
//            }
//            else if (( hdfu->buffer.d8[0] ==  DFU_CMD_ERASE ) && (hdfu->wlength == 5))
//            {
//                hdfu->data_ptr  = hdfu->buffer.d8[1];
//                hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
//                hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
//                hdfu->data_ptr += hdfu->buffer.d8[4] << 24;
//
//                hdfu->data_ptr -= 0x08000000;
//
//                /* Reset writeBlockCount it will be used during write operations */
//                writeBlockCount = 0;
//
//    	        BaseType_t xHigherTaskWakeUp = pdFALSE;
//                xSemaphoreTakeFromISR(xSpiWrite, &xHigherTaskWakeUp);
//                // 4096 - Is flash subsector size. We send subsector number into erase function;
//                // eraseBlockNum[tmpEraswCount] = (hdfu->data_ptr)/4096;
//                // tmpEraswCount++;
//                // We do sector EraseCommand 0xD8 16Mbyte flash has 256 sectors from 0 to 255
//                // The sector size is 65536(bytes) or 64Kbyte
//                if (((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Erase((hdfu->data_ptr)/65536) != USBD_OK)
//                {
//                    return USBD_FAIL;
//                }
//
//
//    	        vTaskNotifyGiveFromISR(flashEraseTaskHandle, &xHigherTaskWakeUp);
//                if (xHigherTaskWakeUp == pdTRUE)
//                {
//        	        portYIELD_FROM_ISR(xHigherTaskWakeUp);
//                }
//            }
//            else if (( hdfu->buffer.d8[0] ==  DFU_CMD_ERASE ) && (hdfu->wlength == 0))
//            {
//    	        if (eraseCount < 5)
//    	        {
//    		        eraseCount++;
//    	        }
//    	        else
//    	        {
//    		        hdfu->wlength = 0;
//    		        hdfu->wblock_num = 0;
//
//    		        /* Update the state machine */
//    		        //hdfu->dev_state =  DFU_STATE_DNLOAD_SYNC;
//    		        hdfu->dev_state =  DFU_STATE_DNLOAD_IDLE;
//
//    		        hdfu->dev_status[1] = 0;
//    		        hdfu->dev_status[2] = 0;
//    		        hdfu->dev_status[3] = 0;
//    		        hdfu->dev_status[4] = hdfu->dev_state;
//
//    		        eraseCount = 0;
//    		        return USBD_OK;
//    	        }
//                // This IF means previous command was DFU_CMD_ERASE it means wait until it finishes
//    	        // Here we need to check erase operation finish
//            }
//            else if  (( hdfu->buffer.d8[0] ==  DFU_CMD_SETADDRESSPOINTER ) && (hdfu->wlength == 0))
//            {
//                hdfu->data_ptr  = hdfu->buffer.d8[1];
//                hdfu->data_ptr += hdfu->buffer.d8[2] << 8;
//                hdfu->data_ptr += hdfu->buffer.d8[3] << 16;
//                hdfu->data_ptr += hdfu->buffer.d8[4] << 24;
//
//                hdfu->wlength = 0;
//                hdfu->wblock_num = 0;
//
//                /* Update the state machine */
//                hdfu->dev_state =  DFU_STATE_DNLOAD_BUSY;
//
//                hdfu->dev_status[1] = 0;
//                hdfu->dev_status[2] = 0;
//                hdfu->dev_status[3] = 0;
//                hdfu->dev_status[4] = hdfu->dev_state;
//                return USBD_OK;
//            }
//            else
//            {
//                /* Reset the global length and block number */
//                hdfu->wlength = 0;
//                hdfu->wblock_num = 0;
//                /* Call the error management function (command will be nacked) */
//                req.bmRequest = 0;
//                req.wLength = 1;
//                USBD_CtlError (pdev, &req);
//            }
//        }
//        /* Regular Download Command */
//        else if (hdfu->wblock_num > 1)
//        {
//            /* Decode the required address */
//            //addr = ((hdfu->wblock_num - 2) * USBD_DFU_XFER_SIZE) + hdfu->data_ptr;
//
//            //addr = ((hdfu->wblock_num - 2) * USBD_DFU_XFER_SIZE);
//            addr = writeBlockCount * USBD_DFU_XFER_SIZE;
//            writeBlockCount++;
//
//            // transferSize += hdfu->wlength;
//
//            /* Preform the write operation */
//            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//            if (pdTRUE == xSemaphoreTakeFromISR(xSpiWrite, &xHigherPriorityTaskWoken))
//            {
//    	        usbResult = ((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Write(hdfu->buffer.d8, (uint8_t *)addr, hdfu->wlength);
//    	        if (pdTRUE == xHigherPriorityTaskWoken)
//    	        {
//    		        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    	        }
//            }
//
//
//            // if (((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Write(hdfu->buffer.d8, (uint8_t *)addr, hdfu->wlength) != USBD_OK)
//            // {
//            // return USBD_FAIL;
//            // }
//        }
//
//        /* Reset the global length and block number */
//        hdfu->wlength = 0;
//        hdfu->wblock_num = 0;
//
//        /* Update the state machine */
//        hdfu->dev_state =  DFU_STATE_DNLOAD_SYNC;
//
//        hdfu->dev_status[1] = 0;
//        hdfu->dev_status[2] = 0;
//        hdfu->dev_status[3] = 0;
//        hdfu->dev_status[4] = hdfu->dev_state;
//        return USBD_OK;
//    }
//    else if (hdfu->dev_state == DFU_STATE_MANIFEST)/* Manifestation in progress*/
//    {
//        /* Start leaving DFU mode */
//        DFU_Leave(pdev);
//    }

    return USBD_OK;
}
/**
  * @brief  USBD_DFU_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_DFU_SOF (USBD_HandleTypeDef *pdev)
{

  return USBD_OK;
}
/**
  * @brief  USBD_DFU_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_DFU_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_DFU_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_DFU_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}

static uint8_t  USBD_CDC_DFU_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t result = USBD_OK;

	if (epnum == (CDC_OUT_EP & (~0x80)))
	{
		result = USBD_CDC_DataOut(pdev, epnum);
	}
	else
	{
		result = USBD_DFU_DataOut(pdev, epnum);
	}
	return result;
}

/**
  * @brief  USBD_DFU_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_DFU_DataOut (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{

  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_DFU_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_DFU_DeviceQualifierDesc);
  return USBD_DFU_DeviceQualifierDesc;
}

/**
  * @brief  USBD_DFU_GetUsrStringDesc
  *         Manages the transfer of memory interfaces string descriptors.
  * @param  speed : current device speed
  * @param  index: desciptor index
  * @param  length : pointer data length
  * @retval pointer to the descriptor table or NULL if the descriptor is not supported.
  */
#if (USBD_SUPPORT_USER_STRING == 1)  
static uint8_t* USBD_DFU_GetUsrStringDesc (USBD_HandleTypeDef *pdev, uint8_t index , uint16_t *length)
{
  static uint8_t USBD_StrDesc[255];

  //char str1[] = "@Boot Flash   /0x08000000/04*016Kg,01*064Kg,07*128Kg";
  char str1[] = "@Boot Flash   /0x08000000/512*064Kg";
  char str2[] = "@Secure Flash   /0x0A000000/512*064Kg";
  char str3[] = "ACM command Interface";
  char str4[] = "ACM data interface";
  //char str2[] = "@Internal Flash   /0x09000000/03*016Ka,01*016Kg,01*064Kg,07*128Kg,04*016Kg,01*064Kg,07*128Kg";
  //uint16_t strLen = 0;
  /* Check if the requested string interface is supported */
  if (index <= (USBD_IDX_INTERFACE_STR + USBD_DFU_MAX_ITF_NUM + 2))
  {
	  if (index == 6)
	  {
		  USBD_GetString ((uint8_t*) str1, USBD_StrDesc, length);
		  //USBD_GetString ((uint8_t *)((USBD_DFU_MediaTypeDef *)pdev->pUserData)->pStrDesc, USBD_StrDesc, length);
	  }
	  else if (index == 7)
	  {
		  USBD_GetString ((uint8_t*) str2, USBD_StrDesc, length);
		  //USBD_GetString ((uint8_t *)((USBD_DFU_MediaTypeDef *)pdev->pUserData)->pStrDesc, USBD_StrDesc, length);
	  }
	  else if (index == 8)
	  {
		  USBD_GetString ((uint8_t*) str3, USBD_StrDesc, length);
	  }
	  else if (index == 9)
	  {
		  USBD_GetString ((uint8_t*) str4, USBD_StrDesc, length);
	  }
	  else
	  {
		  USBD_GetString ((uint8_t*) str4, USBD_StrDesc, length);
	  }

    return USBD_StrDesc;  
  }
  /* Not supported Interface Descriptor index */
  else
  {
    return NULL;
  }
}
#endif

/**
* @brief  USBD_MSC_RegisterStorage
* @param  fops: storage callback
* @retval status
*/
uint8_t  USBD_DFU_RegisterMedia    (USBD_HandleTypeDef   *pdev, 
                                    USBD_DFU_MediaTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pDfuUserData= fops;
  }
  return 0;
}

/******************************************************************************
     DFU Class requests management
******************************************************************************/
/**
  * @brief  DFU_Detach
  *         Handles the DFU DETACH request.
  * @param  pdev: device instance
  * @param  req: pointer to the request structure.
  * @retval None.
  */
static void DFU_Detach(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
 USBD_DFU_HandleTypeDef   *hdfu;
 
 hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
 
  if (hdfu->dev_state == DFU_STATE_IDLE || hdfu->dev_state == DFU_STATE_DNLOAD_SYNC
      || hdfu->dev_state == DFU_STATE_DNLOAD_IDLE || hdfu->dev_state == DFU_STATE_MANIFEST_SYNC
        || hdfu->dev_state == DFU_STATE_UPLOAD_IDLE )
  {
    /* Update the state machine */
    hdfu->dev_state = DFU_STATE_IDLE;
    hdfu->dev_status[0] = DFU_ERROR_NONE;
    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;
    hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
    hdfu->dev_status[4] = hdfu->dev_state;
    hdfu->dev_status[5] = 0; /*iString*/
    hdfu->wblock_num = 0;
    hdfu->wlength = 0;
  } 
  
  /* Check the detach capability in the DFU functional descriptor */
  if ((USBD_DFU_CfgDesc[12 + (9 * USBD_DFU_MAX_ITF_NUM)]) & DFU_DETACH_MASK)
  {
    /* Perform an Attach-Detach operation on USB bus */
    USBD_Stop (pdev);
    USBD_Start (pdev);  
  }
  else
  {
    /* Wait for the period of time specified in Detach request */
    USBD_Delay (req->wValue);  
  }
}

/**
  * @brief  DFU_Download
  *         Handles the DFU DNLOAD request.
  * @param  pdev: device instance
  * @param  req: pointer to the request structure
  * @retval None
  */
static void DFU_Download(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
 USBD_DFU_HandleTypeDef   *hdfu;
 
 hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;

  /* Data setup request */
  if (req->wLength > 0)
  {
    if ((hdfu->dev_state == DFU_STATE_IDLE) || (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE))
    {
      /* Update the global length and block number */
      hdfu->wblock_num = req->wValue;
      hdfu->wlength = req->wLength;
      

      /* Update the state machine */
      hdfu->dev_state = DFU_STATE_DNLOAD_SYNC;
      hdfu->dev_status[4] = hdfu->dev_state;

      /* Prepare the reception of the buffer over EP0 */
      USBD_CtlPrepareRx (pdev,
                         (uint8_t*)hdfu->buffer.d8,                                  
                         hdfu->wlength);

    }
    /* Unsupported state */
    else
    {
      //bytesReceived = 0;
      /* Call the error management function (command will be nacked */
      USBD_CtlError (pdev, req);
    }
  }
  /* 0 Data DNLOAD request */
  else
  {
    /* End of DNLOAD operation*/
    if (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE || hdfu->dev_state == DFU_STATE_IDLE )
    {
      hdfu->manif_state = DFU_MANIFEST_IN_PROGRESS;
      hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;
      hdfu->dev_status[1] = 0;
      hdfu->dev_status[2] = 0;
      hdfu->dev_status[3] = 0;
      hdfu->dev_status[4] = hdfu->dev_state;
    }
    else
    {
      /* Call the error management function (command will be nacked */
      USBD_CtlError (pdev, req);
    }
  }  
}

/**
  * @brief  DFU_Upload
  *         Handles the DFU UPLOAD request.
  * @param  pdev: instance
  * @param  req: pointer to the request structure
  * @retval status
  */
static void DFU_Upload(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	static tmpCount = 0;
 USBD_DFU_HandleTypeDef   *hdfu;
 
 hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
 
  uint8_t *phaddr = NULL;
  uint32_t addr = 0;
  
  /* Data setup request */
  if (req->wLength > 0)
  {
    if ((hdfu->dev_state == DFU_STATE_IDLE) || (hdfu->dev_state == DFU_STATE_UPLOAD_IDLE))
    {
      /* Update the global length and block number */
      hdfu->wblock_num = req->wValue;
      hdfu->wlength = req->wLength;
      
      /* DFU Get Command */
      if (hdfu->wblock_num == 0)  
      {
        /* Update the state machine */
        hdfu->dev_state = (hdfu->wlength > 3)? DFU_STATE_IDLE:DFU_STATE_UPLOAD_IDLE;        
    
        hdfu->dev_status[1] = 0;
        hdfu->dev_status[2] = 0;
        hdfu->dev_status[3] = 0;
        hdfu->dev_status[4] = hdfu->dev_state;       
        
        /* Store the values of all supported commands */
        hdfu->buffer.d8[0] = DFU_CMD_GETCOMMANDS;
        hdfu->buffer.d8[1] = DFU_CMD_SETADDRESSPOINTER;
        hdfu->buffer.d8[2] = DFU_CMD_ERASE;
        
        /* Send the status data over EP0 */
        USBD_CtlSendData (pdev,
                          (uint8_t *)(&(hdfu->buffer.d8[0])),
                          3);
      }
      else if (hdfu->wblock_num > 1)
      {
        hdfu->dev_state = DFU_STATE_UPLOAD_IDLE ;
        
        hdfu->dev_status[1] = 0;
        hdfu->dev_status[2] = 0;
        hdfu->dev_status[3] = 0;
        hdfu->dev_status[4] = hdfu->dev_state;
        
        // This was default realisation
        // addr = ((hdfu->wblock_num - 2) * USBD_DFU_XFER_SIZE) + hdfu->data_ptr;  /* Change is Accelerated*/
        if (hdfu->alt_setting == 1)
        {
        	// hdfu->wblock_num - begins from 2
        	// Flash address always begins from 0;
        	// This for device from address 0x09000000;
        	addr = ((hdfu->wblock_num - 2) * USBD_DFU_XFER_SIZE);
        }
        else
        {
        	// hdfu->wblock_num - begins from 2
        	// Flash address always begins from 0;
        	// This for device from address 0x08000000;
        	addr = ((hdfu->wblock_num - 2) * USBD_DFU_XFER_SIZE);
        }

        
//        ptrs[tmpCount] = hdfu->data_ptr;
//        bloks[tmpCount]= hdfu->wblock_num;
//        tmpCount++;
        /* Return the physical address where data are stored */
    	if (hdfu->alt_setting == 0)
    	{
    		ENABLE_BOOT_FLASH();
    	}
    	else
    	{
    		ENABLE_SECURE_FLASH();
    	}
        phaddr =  ((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->Read((uint8_t *)addr, hdfu->buffer.d8, hdfu->wlength);
        DISABLE_FLASH();

// Only for debug
//        if ((hdfu->wblock_num - 2) == 382)
//        {
//			ParameterType para;
//			para.Read.udAddr = 0;
//			para.Read.pArray = rFlashBuffer;
//			para.Read.udNrOfElementsToRead = 16;
//			fdo.GenOp.DataRead(Read, &para);
//        }

//        ParameterType para;
//    	para.Read.udAddr = 0;
//    	para.Read.pArray = rFlashBuffer;
//    	para.Read.udNrOfElementsToRead = 16;
//    	fdo.GenOp.DataRead(Read, &para);

        /* Send the data over EP0 */
        USBD_CtlSendData (pdev,
        		          phaddr,
                          hdfu->wlength);
      }
      else  /* unsupported hdfu->wblock_num */
      {
        hdfu->dev_state = DFU_ERROR_STALLEDPKT;
        
        hdfu->dev_status[1] = 0;
        hdfu->dev_status[2] = 0;
        hdfu->dev_status[3] = 0;
        hdfu->dev_status[4] = hdfu->dev_state;        
        
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req); 
      }
    }
    /* Unsupported state */
    else
    {
      hdfu->wlength = 0;
      hdfu->wblock_num = 0;   
      /* Call the error management function (command will be nacked */
      USBD_CtlError (pdev, req);
    }
  }
  /* No Data setup request */
  else
  {
    hdfu->dev_state = DFU_STATE_IDLE;
     
    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;
    hdfu->dev_status[3] = 0;
    hdfu->dev_status[4] = hdfu->dev_state;
  }
}

/**
  * @brief  DFU_GetStatus
  *         Handles the DFU GETSTATUS request.
  * @param  pdev: instance
  * @retval status
  */
static void DFU_GetStatus(USBD_HandleTypeDef *pdev)
{
  USBD_DFU_HandleTypeDef   *hdfu;
  hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;

  if (gs_tst >= 1)
  {
	  gs_tst++;
  }
  // GetStatus command must come with wlength == 6. If not it is error.
  // Set device in to IDLE state
  USBD_SetupReqTypedef     req;
  if (hdfu->wlength != 6)
  {
	  hdfu->dev_state = DFU_STATE_IDLE;
	  hdfu->dev_status[1] = 0;
	  hdfu->dev_status[2] = 0;
	  hdfu->dev_status[3] = 0;
	  hdfu->dev_status[4] = hdfu->dev_state;
	  USBD_CtlError (pdev, &req);
  }

  BaseType_t spiWriteStatus = pdFALSE;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  spiWriteStatus = xSemaphoreTakeFromISR(xSpiWrite, &xHigherPriorityTaskWoken);
  if (spiWriteStatus == pdTRUE)
  {
	  xSemaphoreGiveFromISR(xSpiWrite, &xHigherPriorityTaskWoken);
  }

  //CDC_Transmit_FS((uint8_t*)"DFU_GetStatus\n\r", 15);

  switch (hdfu->dev_state)
  {
  	  case   DFU_STATE_DNLOAD_SYNC:
  	  {
  		  // Write operation still in progress
		  if (spiWriteStatus == pdFALSE)
		  {
			  hdfu->dev_state = DFU_STATE_DNLOAD_BUSY;
			  hdfu->dev_status[1] = 0;
			  hdfu->dev_status[2] = 0;
			  hdfu->dev_status[3] = 0;
			  hdfu->dev_status[4] = hdfu->dev_state;
		  }
		  else
		  {
			  hdfu->dev_state = DFU_STATE_DNLOAD_IDLE;
			  hdfu->dev_status[1] = 0;
			  hdfu->dev_status[2] = 0;
			  hdfu->dev_status[3] = 0;
			  hdfu->dev_status[4] = hdfu->dev_state;
		  }
		  break;
  	  }
  	  case   DFU_STATE_DNLOAD_BUSY:
  	  {
		  if ((spiWriteStatus == pdTRUE) && (hdfu->processedCmd == 0))
		  {
			  hdfu->dev_state = DFU_STATE_DNLOAD_IDLE;
			  hdfu->dev_status[1] = 0;
			  hdfu->dev_status[2] = 0;
			  hdfu->dev_status[3] = 0;
			  hdfu->dev_status[4] = hdfu->dev_state;
		  }
		  else
		  {
			  hdfu->dev_state = DFU_STATE_DNLOAD_BUSY;
			  hdfu->dev_status[1] = 0;
			  hdfu->dev_status[2] = 0;
			  hdfu->dev_status[3] = 0;
			  hdfu->dev_status[4] = hdfu->dev_state;
			  // hdfu->processedCmd = 0;
			 if ((hdfu->processedCmd == 3) || (hdfu->processedCmd == 0))
			 {
				  xSemaphoreGiveFromISR(xSpiWrite, &xHigherPriorityTaskWoken);
				  hdfu->processedCmd = 0;
			 }
		  }
		  break;
  	  }
  	  case   DFU_STATE_MANIFEST_SYNC :
  	  {
		  if (hdfu->manif_state == DFU_MANIFEST_IN_PROGRESS)
		  {
		      hdfu->dev_state = DFU_STATE_MANIFEST;

		      hdfu->dev_status[1] = 1;             /*bwPollTimeout = 1ms*/
		      hdfu->dev_status[2] = 0;
		      hdfu->dev_status[3] = 0;
		      hdfu->dev_status[4] = hdfu->dev_state;
		  }
		  else if ((hdfu->manif_state == DFU_MANIFEST_COMPLETE) && \
		  ((USBD_DFU_CfgDesc[(11 + (9 * USBD_DFU_MAX_ITF_NUM))]) & 0x04))
		  {
			  hdfu->dev_state = DFU_STATE_IDLE;

			  hdfu->dev_status[1] = 0;
			  hdfu->dev_status[2] = 0;
			  hdfu->dev_status[3] = 0;
			  hdfu->dev_status[4] = hdfu->dev_state;
		  }
	      break;
  	  }
  	  default :
  		  break;
  }
  
  /* Send the status data over EP0 */
  USBD_CtlSendData (pdev,
                    (uint8_t *)(&(hdfu->dev_status[0])),
                    6);
}

/**
  * @brief  DFU_ClearStatus 
  *         Handles the DFU CLRSTATUS request.
  * @param  pdev: device instance
  * @retval status
  */
static void DFU_ClearStatus(USBD_HandleTypeDef *pdev)
{
 USBD_DFU_HandleTypeDef   *hdfu;
 
 hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
 
  if (hdfu->dev_state == DFU_STATE_ERROR)
  {
    hdfu->dev_state = DFU_STATE_IDLE;
    hdfu->dev_status[0] = DFU_ERROR_NONE;/*bStatus*/
    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;
    hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
    hdfu->dev_status[4] = hdfu->dev_state;/*bState*/
    hdfu->dev_status[5] = 0;/*iString*/
  }
  else
  {   /*State Error*/
    hdfu->dev_state = DFU_STATE_ERROR;
    hdfu->dev_status[0] = DFU_ERROR_UNKNOWN;/*bStatus*/
    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;
    hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
    hdfu->dev_status[4] = hdfu->dev_state;/*bState*/
    hdfu->dev_status[5] = 0;/*iString*/
  }
}

/**
  * @brief  DFU_GetState
  *         Handles the DFU GETSTATE request.
  * @param  pdev: device instance
  * @retval None
  */
static void DFU_GetState(USBD_HandleTypeDef *pdev)
{
 USBD_DFU_HandleTypeDef   *hdfu;
 
 hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
 
  /* Return the current state of the DFU interface */
  USBD_CtlSendData (pdev, 
                    &hdfu->dev_state,
                    1);  
}

/**
  * @brief  DFU_Abort
  *         Handles the DFU ABORT request.
  * @param  pdev: device instance
  * @retval None
  */
static void DFU_Abort(USBD_HandleTypeDef *pdev)
{
 USBD_DFU_HandleTypeDef   *hdfu;
 
 hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
 
  if (hdfu->dev_state == DFU_STATE_IDLE || hdfu->dev_state == DFU_STATE_DNLOAD_SYNC
      || hdfu->dev_state == DFU_STATE_DNLOAD_IDLE || hdfu->dev_state == DFU_STATE_MANIFEST_SYNC
        || hdfu->dev_state == DFU_STATE_UPLOAD_IDLE )
  {
    hdfu->dev_state = DFU_STATE_IDLE;
    hdfu->dev_status[0] = DFU_ERROR_NONE;
    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;
    hdfu->dev_status[3] = 0; /*bwPollTimeout=0ms*/
    hdfu->dev_status[4] = hdfu->dev_state;
    hdfu->dev_status[5] = 0; /*iString*/
    hdfu->wblock_num = 0;
    hdfu->wlength = 0;
  }  
}

/**
  * @brief  DFU_Leave
  *         Handles the sub-protocol DFU leave DFU mode request (leaves DFU mode
  *         and resets device to jump to user loaded code).
  * @param  pdev: device instance
  * @retval None
  */
void DFU_Leave(USBD_HandleTypeDef *pdev)
{
 USBD_DFU_HandleTypeDef   *hdfu;
 
 hdfu = (USBD_DFU_HandleTypeDef*) pdev->pDfuClassData;
 
 hdfu->manif_state = DFU_MANIFEST_COMPLETE;

  if ((USBD_DFU_CfgDesc[(11 + (9 * USBD_DFU_MAX_ITF_NUM))]) & 0x04)
  {
    hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;

    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;
    hdfu->dev_status[3] = 0;
    hdfu->dev_status[4] = hdfu->dev_state;       
    return;
  }
  else
  {
    
    hdfu->dev_state = DFU_STATE_MANIFEST_WAIT_RESET;
    
    hdfu->dev_status[1] = 0;
    hdfu->dev_status[2] = 0;
    hdfu->dev_status[3] = 0;
    hdfu->dev_status[4] = hdfu->dev_state;     
    
    /* Disconnect the USB device */
    USBD_Stop (pdev);

    /* DeInitilialize the MAL(Media Access Layer) */
    ((USBD_DFU_MediaTypeDef *)pdev->pDfuUserData)->DeInit();
    
    /* Generate system reset to allow jumping to the user code */
    NVIC_SystemReset();
   
    /* This instruction will not be reached (system reset) */
    for(;;);
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
