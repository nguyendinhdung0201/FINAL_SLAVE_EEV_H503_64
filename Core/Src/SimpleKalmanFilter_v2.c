/*
 * SimpleKalmanFilter_v2.c
 *
 *  Created on: May 29, 2025
 *      Author: PC
 */
#include "SimpleKalmanFilter_v2.h"

void SimpleKalmanFilterInit(SimpleKalmanFilter *filter, float mea_e, float est_e, float q) {
    filter->err_measure = mea_e;
    filter->err_estimate = est_e;
    filter->q = q;
}

uint16_t SimpleKalmanFilter_updateEstimate(SimpleKalmanFilter *filter, float mea) {
    filter->kalman_gain = filter->err_estimate / (filter->err_estimate + filter->err_measure);
    filter->current_estimate = filter->last_estimate + filter->kalman_gain * (mea - filter->last_estimate);
    filter->err_estimate = (1.0f - filter->kalman_gain) * filter->err_estimate + fabsf(filter->last_estimate - filter->current_estimate) * filter->q;
    filter->last_estimate = filter->current_estimate;
    return (uint16_t)fmaxf(0.0f, fminf(filter->current_estimate, 65535.0f));
}

void SimpleKalmanFilter_setMeasurementError(SimpleKalmanFilter *filter, float mea_e) {
    filter->err_measure = mea_e;
}

void SimpleKalmanFilter_setEstimateError(SimpleKalmanFilter *filter, float est_e) {
    filter->err_estimate = est_e;
}

void SimpleKalmanFilter_setProcessNoise(SimpleKalmanFilter *filter, float q) {
    filter->q = q;
}

float SimpleKalmanFilter_getKalmanGain(SimpleKalmanFilter *filter) {
    return filter->kalman_gain;
}
