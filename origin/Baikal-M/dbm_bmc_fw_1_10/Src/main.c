/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "nn_console.h"
#include "nn_spi.h"
#include "nn_i2c.h"
#include "dbm_pins.h"
#include "dbm_tasks.h"
#include "dbm_common.h"
#include "pac1720.h"

//extern SemaphoreHandle_t xSpiMutex;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//extern volatile uint8_t _shttp_page;
//extern volatile uint32_t gHttpPage;
extern I2C_HandleTypeDef hI2C2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void NN_GPIO_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern SemaphoreHandle_t sduResBlk;
extern SemaphoreHandle_t xRxSemph;
extern TaskHandle_t  systemStartThreadHandle;

//extern void ledEnable(void);
/* USER CODE END 0 */

int main(void)

{

 sduResBlk = xSemaphoreCreateBinary();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  //NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //ledEnable();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  /*
   * Switching MUX for SPI flash.
   * Must be done obligatory
   */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

  NN_GPIO_Init();
  NN_MX_DMA_Init();
  NN_MX_SPI1_Init();

  NN_FlashInit();

  NN_MX_I2C2_Init();

  //nnInitFanSense();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  HAL_GPIO_WritePin(PORT_RESET_MEZ_LVDS, PIN_RESET_MEZ_LVDS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PORT_RESET_MEZ10G, PIN_RESET_MEZ10G, GPIO_PIN_RESET);


  /*
   * PORT_RESET_SPSTX8 - must be enabled to talk with
   * ADG715 - analog key.
   */
  //HAL_GPIO_WritePin(PORT_RESET_SPSTX8, PIN_RESET_SPSTX8, GPIO_PIN_SET);


  initDevice(&hI2C2, 0x98);
  enableAlertPin(&hI2C2, 0x98);
  setCurrentHighLimitCh1(&hI2C2, 0x98, 2, 10); // Set CORE current limit to 10A


  initDevice(&hI2C2, 0x9A);
  initDevice(&hI2C2, 0x9C);
  initDevice(&hI2C2, 0x9E);
  initDevice(&hI2C2, 0x92);
  initDevice(&hI2C2, 0x96);


  if (xRxSemph == NULL)
  {
	  xRxSemph = xSemaphoreCreateBinary();
	  xSemaphoreTake(xRxSemph, 0);
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // setCoreVolatage(V_0v95);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  vInitFlashWriteThread();

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  vStartConsole(configMINIMAL_STACK_SIZE * 2, osPriorityNormal + 2);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
     * Default config
    */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = 16;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 12;
//  RCC_OscInitStruct.PLL.PLLN = 192;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 8;

  /*
   * CONFIG 2
   * This config is on maximum speed
   * This is working configuration
   */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 24;
//  RCC_OscInitStruct.PLL.PLLN = 240;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 5;

  /*
   * TEST: Speeding up clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
     * Default config
    */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  /*
   * This config is on maximum speed
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  // FLASH_LATENCY_3 is used for CONFIG1
  //if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC1   ------> ETH_MDC
     PA1   ------> ETH_REF_CLK
     PA2   ------> ETH_MDIO
     PA7   ------> ETH_CRS_DV
     PC4   ------> ETH_RXD0
     PC5   ------> ETH_RXD1
     PB13   ------> ETH_TXD1
     PG11   ------> ETH_TX_EN
     PG13   ------> ETH_TXD0
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

#if defined(NUCLEO_BOARD)
  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);
#endif

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Led for BFK 3.1 board
   * This pin is used only for debug test
   */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*
   * For BFK 3.1 switch SPI	mux
   */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void NN_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*
	 * PIN_PS_ON - first must be PULLEDUP to disable ATX source
	 * We Enable ATX source by set down PIN_PS_ON.
	 */
	INIT_PIN_AS_OUT(PORT_PS_ON, PIN_PS_ON, GPIO_PIN_SET);

	INIT_PIN_AS_OUT(PORT_EN_VDD_CORE, PIN_EN_VDD_CORE, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_RESET_SPSTX8, PIN_RESET_SPSTX8, GPIO_PIN_SET);

	INIT_PIN_AS_OUT(PORT_EN_VDD1V5, PIN_EN_VDD1V5, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_VDD1V8, PIN_EN_VDD1V8, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_VDD_RESERVE, PIN_EN_VDD_RESERVE, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_VDD3V3_CLK, PIN_EN_VDD3V3_CLK, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_VDD_PLL, PIN_EN_VDD_PLL, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // I2C0 - enable
	INIT_PIN_AS_OUT(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_LVDS_VPLL_27M, PIN_EN_LVDS_VPLL_27M, GPIO_PIN_RESET);

	/*
	 * If we see power_pins.ods document next pins are
	 * PIN_PLL_PLUS_50
	 * PIN_PLL_PLUS_100
	 * PIN_PLL_PLUS_200
	 * PIN_PLL_PLUS_400
	 * This pins must be configured during voltage regulation.
	 * To DISABLE plus_xxx voltage pins must be floating,
	 * NOT pull down
	 */
	GPIO_InitStruct.Pin = PIN_PLL_PLUS_50;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /*!< Input Floating Mode                   */
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PLL_PLUS_50, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PLL_PLUS_100;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /*!< Input Floating Mode                   */
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PLL_PLUS_100, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PLL_PLUS_200;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /*!< Input Floating Mode                   */
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PLL_PLUS_200, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PLL_PLUS_400;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /*!< Input Floating Mode                   */
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PLL_PLUS_400, &GPIO_InitStruct);

	INIT_PIN_AS_OUT(PORT_VDQ_S3_CH1, PIN_VDQ_S3_CH1, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_VDQ_S5_CH1, PIN_VDQ_S5_CH1, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_VPP_CH1, PIN_EN_VPP_CH1, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_VDQ_S3_CH2, PIN_VDQ_S3_CH2, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_VDQ_S5_CH2, PIN_VDQ_S5_CH2, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_VPP_CH2, PIN_EN_VPP_CH2, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_HDMI_PWR, PIN_EN_HDMI_PWR, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_EN_CLK0_XG, PIN_EN_CLK0_XG, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_EN_CLK1_XG, PIN_EN_CLK1_XG, GPIO_PIN_RESET);

	// It is used as LED_GREEN on DISCO
	// INIT_PIN_AS_OUT(PORT_EN_CLK0_XG, PIN_EN_CLK0_XG, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_EN_CLK27MHZ_3V3, PIN_EN_CLK27MHZ_3V3, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_EN_CLK25MHZ_3V3, PIN_EN_CLK25MHZ_3V3, GPIO_PIN_RESET);


	/*
	 * BAIKAL-M reset pins
	 */
	INIT_PIN_AS_OUT(PORT_RESET_SPSTX8, PIN_RESET_SPSTX8, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_JTAG_RESET, PIN_JTAG_RESET, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_PJTAG_RESET, PIN_PJTAG_RESET, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_CPU_RESET_BMC, PIN_CPU_RESET_BMC, GPIO_PIN_RESET);

	/*
	 * SPI flash mux pins
	 */
	INIT_PIN_AS_OUT(PORT_SPI1_MUX, PIN_SPI1_MUX, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_BOOT_PROG, PIN_BOOT_PROG, GPIO_PIN_RESET);

	// I2C1_EO
	INIT_PIN_AS_OUT(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	INIT_PIN_AS_OUT(PORT_RESET_MEZ_LVDS, PIN_RESET_MEZ_LVDS, GPIO_PIN_SET);
	INIT_PIN_AS_OUT(PORT_RESET_MEZ10G, PIN_RESET_MEZ10G, GPIO_PIN_SET);
	/* SMBus configuration
	 * I need to add a little bit later
	 * PIN_SMBUS_CLK_CURSE
	 * PIN_SMBUS_SDA_CURSE
	 *
	 */

	GPIO_InitStruct.Pin = PIN_FAB_PWM_BAIKAL;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_FAB_PWM_BAIKAL, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PWR_OK;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PWR_OK, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PGOOD_CLK;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PGOOD_CLK, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PGOOD_PLL;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PGOOD_PLL, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PGOOD_VDQ_CH1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PGOOD_VDQ_CH1, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PGOOD_VPP_CH1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PGOOD_VPP_CH1, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PGOOD_VDQ_CH2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PGOOD_VDQ_CH2, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_PGOOD_VPP_CH2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PGOOD_VPP_CH2, &GPIO_InitStruct);

	/*
	 * PIN_ALERT_CURSE
	 * PIN_FAN_SENS_BMC
	 * This pins must be configured after
	 *
	 */

	// Config for temporaty I2C0 pins - left floating
	// PA8 - I2C0 CLK
	// PC9 - i2C0 SDA
	GPIO_InitStruct.Pin = PIN_I2C0_SDA_3V3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /*!< Input Floating Mode                   */
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_I2C0_SDA_3V3, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_I2C0_SCL_3V3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /*!< Input Floating Mode                   */
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_I2C0_SCL_3V3, &GPIO_InitStruct);

	/*
	 * Configuring Buttons pins
	 */
	INIT_PIN_AS_IT_RISING(PORT_SYS_RESET_BTN, PIN_SYS_RESET_BTN);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	GPIO_InitStruct.Pin = PIN_PWR_RESET_BTN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_PWR_RESET_BTN, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/*
	 * PCI Express pins
	 */
	INIT_PIN_AS_INPUT(PORT_PE_PRSNT0, PIN_PE_PRSNT0);
	INIT_PIN_AS_INPUT(PORT_PE_PRSNT1, PIN_PE_PRSNT1);
	INIT_PIN_AS_INPUT(PORT_PE_PRSNT2, PIN_PE_PRSNT2);

	INIT_PIN_AS_OUT(PORT_PE_RST_0, PIN_PE_RST_0, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_PE_RST_1, PIN_PE_RST_1, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_PE_RST_2, PIN_PE_RST_2, GPIO_PIN_RESET);

	/*
	 * Configuring LED pins
	 */
	INIT_PIN_AS_OUT(PORT_LED_GREEN, PIN_LED_GREEN, GPIO_PIN_RESET);
	INIT_PIN_AS_OUT(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_RESET);
}

/*
 * PB0 - System reset press event
 */
void EXTI0_IRQHandler(void)
{
	BaseType_t baseTypePriority;
	//vTaskNotifyGiveFromISR(systemStartThreadHandle, &baseTypePriority);
	xTaskNotifyFromISR(systemStartThreadHandle, (uint8_t) 3, eSetValueWithOverwrite, &baseTypePriority);
	//osDelay(20);
	HAL_GPIO_EXTI_IRQHandler(PIN_SYS_RESET_BTN);
}

/*
 * PB1 - System reset press event
 */
void EXTI1_IRQHandler(void)
{
	// HAL_GPIO_WritePin(PORT_PS_ON, PIN_PS_ON, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_SET);
	HAL_GPIO_EXTI_IRQHandler(PIN_PWR_RESET_BTN);
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  usbSendToConsole();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
