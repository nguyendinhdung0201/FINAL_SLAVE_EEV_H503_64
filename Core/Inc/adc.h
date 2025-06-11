/*
 * adc.h
 *
 *  Created on: Nov 6, 2024
 *      Author: PC
 */

#ifndef ADC_H_
#define ADC_H_

#include "main.h"

void ADC_Init(ADC_HandleTypeDef* hadc);
uint16_t ADC_ReadChannel(ADC_HandleTypeDef* hadc, uint32_t channel);
float ADC_ConvertToVoltage(uint16_t raw_value, float vref);

#endif /* INC_ADC_H_ */
