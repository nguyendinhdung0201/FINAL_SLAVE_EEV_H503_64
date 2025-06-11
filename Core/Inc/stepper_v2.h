/*
 * stepper_v2.h
 *
 *  Created on: Nov 14, 2024
 *      Author: PC
 */

#ifndef STEPPER_V2_H_
#define STEPPER_V2_H_
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
// Cấu trúc để lưu thông tin chân GPIO
typedef struct {
    GPIO_TypeDef* PORT_IN1;
    uint16_t PIN_IN1;
    GPIO_TypeDef* PORT_IN2;
    uint16_t PIN_IN2;
    GPIO_TypeDef* PORT_IN3;
    uint16_t PIN_IN3;
    GPIO_TypeDef* PORT_IN4;
    uint16_t PIN_IN4;
} StepperPins;

// Cấu trúc để quản lý trạng thái động cơ
typedef struct {
    StepperPins pins;
    uint32_t step_delay;     // Độ trễ giữa các bước (microseconds)
    int32_t target_steps;    // Số bước cần di chuyển
    int32_t current_step;    // Vị trí hiện tại
    uint8_t current_phase;   // Pha hiện tại (0-7 cho full step)
    uint32_t last_step_time; // Thời điểm bước cuối cùng
    uint8_t is_moving;       // Trạng thái chuyển động
    uint8_t direction;       // Hướng quay (0: CW, 1: CCW)
} Stepper;

// Khởi tạo động cơ
void Stepper_Init(Stepper* stepper, StepperPins pins);

// Đặt tốc độ (rpm)
void Stepper_SetSpeed(Stepper* stepper, uint32_t rpm);

// Di chuyển số bước nhất định
void Stepper_Move(Stepper* stepper, int32_t steps);

// Xử lý nonblocking trong main loop
void Stepper_Run(Stepper* stepper);

// Kiểm tra trạng thái chuyển động
uint8_t Stepper_IsMoving(Stepper* stepper);
uint8_t Stepper_State(Stepper* stepper);
// Dừng động cơ
void Stepper_Stop(Stepper* stepper);

void Stepper_Hold(Stepper* stepper);

#endif /* INC_STEPPER_V2_H_ */
