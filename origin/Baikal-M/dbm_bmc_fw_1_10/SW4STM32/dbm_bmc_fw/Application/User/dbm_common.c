/*
 * dbm_common.c
 *
 */

#include "dbm_common.h"

struct flashBuffer_t gFlashBuffer;
ADC_HandleTypeDef hadc;

void nnInitFanSense(void)
{
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_ADC3_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = PIN_FAN_SENS_BMC;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PORT_FAN_SENS_BMC, &GPIO_InitStruct);

	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	ADC_InitStruct.Resolution = ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlign = ADC_DATAALIGN_RIGHT;
	ADC_InitStruct.ScanConvMode = DISABLE;
	ADC_InitStruct.EOCSelection = ADC_EOC_SINGLE_CONV;
	ADC_InitStruct.ContinuousConvMode = DISABLE;
	ADC_InitStruct.NbrOfConversion = 1;
	ADC_InitStruct.DiscontinuousConvMode = DISABLE;
//	ADC_InitStruct.NbrOfDiscConversion
	ADC_InitStruct.ExternalTrigConv = ADC_SOFTWARE_START;
	ADC_InitStruct.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//	ADC_InitStruct.DMAContinuousRequests

	hadc.Instance = ADC3;
	hadc.Init = ADC_InitStruct;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}



	HAL_ADC_Start(&hadc);
}
