/*
 * Input_parameters.h
 *
 *  Created on: Nov 7, 2024
 *      Author: PC
 */

#ifndef INPUT_PARAMETERS_H_
#define INPUT_PARAMETERS_H_
#include "SimpleKalmanFilter_v2.h"
#include "adc.h"
// Khai báo typedef cho các struct

typedef struct {
    uint16_t ADC_hoi_ve;
    uint16_t ADC_dau_day;
} ADC_Temperature_Sensors;

typedef struct {
    float hoi_ve;
    float dau_day;
} Temperature_Sensors;


////////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint16_t ADC_high_pressure;
    uint16_t ADC_low_pressure;
} ADC_Pressure_Sensors;

typedef struct {
    float high_pressure_sensor;
    float low_pressure_sensor;
} Pressure_Sensors;


//////////////////////////////////////////////////////////////////////////////////
typedef struct{
	SimpleKalmanFilter filter_temperature_sensor_hoi_ve;
	SimpleKalmanFilter filter_temperature_sensor_dau_day;
}Filter_Temperature_Sensors;
////////////////////////////////////////////////////////////////////////////////
typedef struct{
	SimpleKalmanFilter filter_high_pressure_sensor;
	SimpleKalmanFilter filter_low_pressure_sensor;
}Filter_Pressure_Sensors;




extern Temperature_Sensors temperature_sensors;
extern Pressure_Sensors pressure_sensors;

extern ADC_Temperature_Sensors adc_temperature_sensors;
extern ADC_Pressure_Sensors adc_pressure_sensors;

extern Filter_Temperature_Sensors filter_temperature_sensors;
extern Filter_Pressure_Sensors filter_pressure_sensors;

extern uint16_t adc_vref;
extern float vref;


void Filter_Input_Init(void);
void Calcular_Input(ADC_HandleTypeDef* hadc1);
#endif /* INC_INPUT_PARAMETERS_H_ */
