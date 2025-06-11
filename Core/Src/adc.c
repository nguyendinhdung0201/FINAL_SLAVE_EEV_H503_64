/*
 * adc.c
 *
 *  Created on: Nov 6, 2024
 *      Author: PC
 */
#include "adc.h"
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
      Error_Handler();
    }
	HAL_ADCEx_Calibration_Start(hadc,ADC_SINGLE_ENDED);
}
static void ADC_ConfigureChannel(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef config = {
        .Channel = channel,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
		.SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
		.Offset = 0
    };
    HAL_ADC_ConfigChannel(hadc, &config);
}

uint16_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ConfigureChannel(hadc, channel);
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint32_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return (uint16_t)value;
}
float ADC_ConvertToVoltage(uint16_t value, float vref)
{
	float voltage = value*vref/4095.0f;
	return voltage;
}
