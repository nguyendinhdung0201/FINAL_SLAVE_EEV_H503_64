/*
 * stepper_v2.c
 *
 *  Created on: Nov 14, 2024
 *      Author: PC
 */
#include "stepper_v2.h"
//// Mảng các bước cho chế độ full step
/*const uint8_t STEP_SEQUENCE[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};*/
const uint8_t STEP_SEQUENCE[8][4] = {
    {0, 1, 1, 1},
    {0, 0, 1, 1},
    {1, 0, 1, 1},
    {1, 0, 0, 1},
    {1, 1, 0, 1},
    {1, 1, 0, 0},
    {1, 1, 1, 0},
    {0, 1, 1, 0}
};

// Hàm nội bộ để điều khiển các chân
static void Stepper_SetPhase(Stepper* stepper, uint8_t phase) {
    HAL_GPIO_WritePin(stepper->pins.PORT_IN1, stepper->pins.PIN_IN1, STEP_SEQUENCE[phase][0]);
    HAL_GPIO_WritePin(stepper->pins.PORT_IN2, stepper->pins.PIN_IN2, STEP_SEQUENCE[phase][1]);
    HAL_GPIO_WritePin(stepper->pins.PORT_IN3, stepper->pins.PIN_IN3, STEP_SEQUENCE[phase][2]);
    HAL_GPIO_WritePin(stepper->pins.PORT_IN4, stepper->pins.PIN_IN4, STEP_SEQUENCE[phase][3]);
}

void Stepper_Init(Stepper* stepper, StepperPins pins) {
    stepper->pins = pins;
    stepper->step_delay = 10;  // Mặc định 2ms/bước
    stepper->target_steps = 0;
    stepper->current_step = 0;
    stepper->current_phase = 0;
    stepper->last_step_time = 0;
    stepper->is_moving = 0;
    stepper->direction = 0;

    // Đặt tất cả các chân về 0
    Stepper_SetPhase(stepper, 0);
}
void Stepper_SetSpeed(Stepper* stepper, uint32_t rpm) {
    // Chuyển đổi rpm thành microseconds/step
    // 1 vòng = 4096 bước (với full step gồm 8 pha)
    stepper->step_delay = rpm;
}

void Stepper_Move(Stepper* stepper, int32_t steps) {
    stepper->target_steps = steps;
    stepper->current_step = 0;
    stepper->direction = (steps >= 0) ? 1 : 0;
    if(stepper->target_steps != 0){
        stepper->is_moving = 1;
    }
    stepper->last_step_time = HAL_GetTick();  // Chuyển đổi ms sang us
}

void Stepper_Run(Stepper* stepper) {
	if(stepper->direction == 1 && step_position == 0){
      	Stepper_Stop(stepper);
        return;
    }
    if(stepper->direction == 0 && step_position == 500){
    	Stepper_Stop(stepper);
        return;
    }
    if (!stepper->is_moving) return;
    if(stepper->target_steps == 0) return;


        if (stepper->direction == 0) {
            stepper->current_phase = (stepper->current_phase + 1) % 8;
        } else {
            stepper->current_phase = (stepper->current_phase + 7) % 8;
        }
        // Thực hiện bước
        Stepper_SetPhase(stepper, stepper->current_phase);
        stepper->current_step++;
        int16_t next_step = step_position + (stepper->direction ? -1 : 1);
        if (next_step >= 0 && next_step <= 500) {
            step_position = next_step;
        }
        percent_step = (step_position/500.0f)*100.0f;
        // Kiểm tra xem đã hoàn thành chưa
        if (stepper->current_step >= abs(stepper->target_steps)) {
                 last_time_step = HAL_GetTick();
            	 stepper->is_moving = 0;
        }
}

uint8_t Stepper_IsMoving(Stepper* stepper) {
    return stepper->is_moving;
}
// direction = 0: OPEN
// direction = 1: CLOSE
uint8_t Stepper_State(Stepper* stepper){
	return stepper->direction;
}

void Stepper_Stop(Stepper* stepper) {
    stepper->is_moving = 0;
    // Tắt tất cả các cuộn dây
    HAL_GPIO_WritePin(stepper->pins.PORT_IN1, stepper->pins.PIN_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(stepper->pins.PORT_IN2, stepper->pins.PIN_IN2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(stepper->pins.PORT_IN3, stepper->pins.PIN_IN3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(stepper->pins.PORT_IN4, stepper->pins.PIN_IN4, GPIO_PIN_SET);
}


void Stepper_Hold(Stepper* stepper) {
    stepper->is_moving = 0;
    // Tắt tất cả các cuộn dây
    Stepper_SetPhase(stepper, stepper->current_phase);
}


























