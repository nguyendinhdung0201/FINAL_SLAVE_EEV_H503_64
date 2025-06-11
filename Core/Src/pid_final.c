/*
 * pid_final.c
 *
 *  Created on: Feb 24, 2025
 *      Author: PC
 */
#include "pid_final.h"
#include "math.h"
#include "SimpleKalmanFilter_v2.h"
SimpleKalmanFilter output_filter;
void PID_Init(PID_TypeDef *pid, float Kp, float Ts, float setpoint) {
    pid->T = Ts;
    pid->Kp = Kp;                          // P = 3%
    pid->Ki = (pid->Kp / 20.0f)*pid->T;    // Ti = 20s
    pid->Kd = (pid->Kp * 4.0f)/pid->T;     // Td = 4s

    pid->setpoint = setpoint;

    // Khởi tạo các giá trị
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->prev_output = 0.0f;

    // Giới hạn output và tích phân
    pid->max_output = 60.0f;
    pid->min_output = -60.0f;
    pid->max_integral = 100.0f;      // Giới hạn chống tích phân
}
float PID_Calculate(PID_TypeDef *pid, float measurement) {
//    // Tính sai số
    float error = pid->setpoint - measurement;
    float P = pid->Kp * error;

    // Thành phần I (với chống tích phân)
    pid->integral += error * pid->T;
    pid->integral = fmaxf(-pid->max_integral, fminf(pid->integral, pid->max_integral));
    float I = pid->Ki * pid->integral;
    float D = pid->Kd * ((error - pid->prev_error) / pid->T);
    float output = P + I + D;
    // Cập nhật các giá trị cho lần sau
    output = fmaxf(pid->min_output, fminf(output, pid->max_output));
    pid->prev_error = error;
    pid->prev_output = output;
    if (fabsf(output) < 0.2f){
        return output > 0.0f ? 0.2f : -0.2f;
    }
    return output;
}


