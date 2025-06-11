/*
 * R507_temp_pressure.c
 *
 *  Created on: Nov 15, 2024
 *      Author: PC
 */
#include "R507_temp_pressure.h"

float R507_GetTemperature(float pressure) {
    // Tìm khoảng áp suất phù hợp trong bảng
    for (int i = 0; i < R507_TABLE_SIZE - 1; i++) {
        // Nếu áp suất bằng với áp suất trong bảng, trả về nhiệt độ ngay
        if (pressure == R507_TABLE[i].pressure) {
             return R507_TABLE[i].temperature_R507;
        }

        if (pressure > R507_TABLE[i].pressure &&
            pressure < R507_TABLE[i + 1].pressure) {
            // Nội suy tuyến tính
            float p1 = R507_TABLE[i].pressure;
            float p2 = R507_TABLE[i + 1].pressure;
            float t1 = R507_TABLE[i].temperature_R507;
            float t2 = R507_TABLE[i + 1].temperature_R507;

            return t1 + (pressure - p1) * (t2 - t1) / (p2 - p1);
        }
    }
    // Nếu áp suất nằm ngoài bảng, trả về giá trị gần nhất
    if (pressure < R507_TABLE[0].pressure)
        return R507_TABLE[0].temperature_R507;
    return R507_TABLE[R507_TABLE_SIZE - 1].temperature_R507;
}

