/*
 * SimpleKalmanFilter_v2.h
 *
 *  Created on: May 29, 2025
 *      Author: PC
 */

#ifndef INC_SIMPLEKALMANFILTER_V2_H_
#define INC_SIMPLEKALMANFILTER_V2_H_
#include "stm32h5xx.h"
#include "math.h"
typedef struct {
    float err_measure;
    float err_estimate;
    float q;
    float current_estimate;
    float last_estimate;
    float kalman_gain;
} SimpleKalmanFilter;

void 	SimpleKalmanFilterInit(SimpleKalmanFilter *filter, float mea_e, float est_e, float q);
uint16_t 	SimpleKalmanFilter_updateEstimate(SimpleKalmanFilter *filter, float mea);
void 	SimpleKalmanFilter_setMeasurementError(SimpleKalmanFilter *filter, float mea_e);
void 	SimpleKalmanFilter_setEstimateError(SimpleKalmanFilter *filter, float est_e);
void 	SimpleKalmanFilter_setProcessNoise(SimpleKalmanFilter *filter, float q);
float 	SimpleKalmanFilter_getKalmanGain(SimpleKalmanFilter *filter);
#endif /* INC_SIMPLEKALMANFILTER_V2_H_ */
