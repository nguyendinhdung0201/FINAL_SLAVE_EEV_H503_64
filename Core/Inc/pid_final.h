/*
 * pid_final.h
 *
 *  Created on: Feb 24, 2025
 *      Author: PC
 */
#ifndef PID_FINAL_H_
#define PID_FINAL_H_
typedef struct {
    float Kp;        // Hệ số tỷ lệ
    float Ki;        // Hệ số tích phân (Kp/Ti)
    float Kd;        // Hệ số vi phân (Kp*Td)
    float setpoint;  // Giá trị đặt
    float prev_error;// Sai số trước đó
    float integral;  // Tích phân sai số
    float prev_output;// Giá trị điều khiển trước đó
    float T;         // Thời gian lấy mẫu (s)
    float max_output;// Giới hạn trên output
    float min_output;// Giới hạn dưới output
    float max_integral;// Giới hạn tích phân
} PID_TypeDef;
void PID_Init(PID_TypeDef *pid, float Kp, float Ts, float setpoint);
float PID_Calculate(PID_TypeDef *pid, float measurement);
#endif /* INC_PID_FINAL_H_ */
