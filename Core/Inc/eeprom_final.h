/*
 * eeprom_final.h
 *
 *  Created on: Jun 6, 2025
 *      Author: PC
 */

#ifndef INC_EEPROM_FINAL_H_
#define INC_EEPROM_FINAL_H_
#include "stm32h5xx_hal.h" // Đảm bảo đúng dòng STM32 của bạn
#include <stdbool.h>
#include <stddef.h> // Cho size_t
#include <stdint.h>
#include <stdio.h> // Cho printf (nếu dùng làm backend log)

/*=========================================================================
    USER DEFINES (Cấu hình bởi người dùng)
    -----------------------------------------------------------------------*/

// Chọn loại EEPROM bạn đang dùng để xác định PAGE_SIZE và MAX_MEM_ADDR
// Chỉ bỏ comment một dòng tương ứng

// --- Cấu hình cho M24C64 ---
#define M24C64    // 64 Kbit = 8192 bytes, Page size = 32 bytes

// --- Bỏ comment các dòng AT24Cxx khác nếu không dùng ---
// #define AT24C01 // 1 Kbit = 128 bytes, Page size = 8 bytes
// #define AT24C02   // 2 Kbit = 256 bytes, Page size = 8 bytes
// #define AT24C04   // 4 Kbit = 512 bytes, Page size = 16 bytes
// #define AT24C08   // 8 Kbit = 1024 bytes, Page size = 16 bytes
// #define AT24C16   // 16 Kbit = 2048 bytes, Page size = 16 bytes


// --- Tự động định nghĩa kích thước dựa trên lựa chọn ---
#ifdef AT24C01
    #define CURRENT_EEPROM_PAGE_SIZE      8
    #define CURRENT_EEPROM_MAX_MEM_ADDR   0x7F  // 128 Bytes (0 to 127)
    #define CURRENT_EEPROM_NAME           "AT24C01"
#elif defined(AT24C02)
    #define CURRENT_EEPROM_PAGE_SIZE      8
    #define CURRENT_EEPROM_MAX_MEM_ADDR   0xFF  // 256 Bytes (0 to 255)
    #define CURRENT_EEPROM_NAME           "AT24C02"
#elif defined(AT24C04)
    #define CURRENT_EEPROM_PAGE_SIZE      16
    #define CURRENT_EEPROM_MAX_MEM_ADDR   0x1FF // 512 Bytes (0 to 511)
    #define CURRENT_EEPROM_NAME           "AT24C04"
#elif defined(AT24C08)
    #define CURRENT_EEPROM_PAGE_SIZE      16
    #define CURRENT_EEPROM_MAX_MEM_ADDR   0x3FF // 1024 Bytes (0 to 1023)
    #define CURRENT_EEPROM_NAME           "AT24C08"
#elif defined(AT24C16)
    #define CURRENT_EEPROM_PAGE_SIZE      16
    #define CURRENT_EEPROM_MAX_MEM_ADDR   0x7FF // 2048 Bytes (0 to 2047)
    #define CURRENT_EEPROM_NAME           "AT24C16"
#elif defined(M24C64) // Thêm cấu hình cho M24C64
    #define CURRENT_EEPROM_PAGE_SIZE      32
    #define CURRENT_EEPROM_MAX_MEM_ADDR   0x1FFF // 8192 Bytes (0 to 8191)
    #define CURRENT_EEPROM_NAME           "M24C64"
#else
    #error "Please define the EEPROM type (e.g., M24C64 or AT24C01) in eeprom_at24cxx.h"
#endif

// Địa chỉ I2C 7-bit mặc định của thiết bị (thường là 0x50 khi A0=A1=A2=0)
// M24C64 cũng thường dùng 0x50 nếu các chân địa chỉ được nối đất.
// Kiểm tra datasheet và cấu hình phần cứng của bạn!
#define EEPROM_DEFAULT_7BIT_ADDR   0x50

// Timeout cho các hoạt động I2C (ms)
#define EEPROM_I2C_TIMEOUT_MS        60
// Timeout chờ EEPROM sẵn sàng sau khi ghi (ms) - M24C64 có t_W điển hình 5ms
#define EEPROM_WRITE_CYCLE_TIMEOUT_MS  10
// Số lần lỗi I2C liên tiếp tối đa trước khi thử reset bus
#define EEPROM_I2C_RESET_THRESHOLD 3
/*=========================================================================
    ENUMS AND STRUCTS
    (Giữ nguyên phần này, đổi tên nếu muốn thống nhất)
    -----------------------------------------------------------------------*/
typedef enum {
    EEPROM_OK = 0,
    EEPROM_ERROR_GENERAL,
    EEPROM_ERROR_TIMEOUT,
    EEPROM_ERROR_BUSY,
    EEPROM_ERROR_NACK,
    EEPROM_ERROR_BUS_FAULT,
    EEPROM_ERROR_ADDR_OOR,
    EEPROM_ERROR_INIT_FAILED,
    EEPROM_ERROR_PARAM,
    EEPROM_ERROR_NOT_READY,
} EEPROM_Status_t; // Đổi tên từ AT24CXX_Status_t

typedef struct {
    I2C_HandleTypeDef   *i2c_handle;
    uint16_t            device_address_8bit;
    uint16_t            max_mem_address;
    uint8_t             page_size;
    bool                initialized;
    uint8_t             i2c_error_count;
    uint16_t            mem_addr_size_hal; // I2C_MEMADD_SIZE_8BIT hoặc I2C_MEMADD_SIZE_16BIT
} EEPROM_Handle_t; // Đổi tên từ AT24CXX_Handle_t

/*=========================================================================
    FUNCTION PROTOTYPES
    =========================================================================*/

EEPROM_Status_t EEPROM_Init(EEPROM_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t device_7bit_addr);
EEPROM_Status_t EEPROM_IsDeviceReady(EEPROM_Handle_t *dev, uint32_t trials);

EEPROM_Status_t EEPROM_WriteByte(EEPROM_Handle_t *dev, uint16_t mem_addr, uint8_t data);
EEPROM_Status_t EEPROM_ReadByte(EEPROM_Handle_t *dev, uint16_t mem_addr, uint8_t *p_data);

EEPROM_Status_t EEPROM_WriteBuffer(EEPROM_Handle_t *dev, uint16_t mem_addr, const uint8_t *p_data, size_t len);
EEPROM_Status_t EEPROM_ReadBuffer(EEPROM_Handle_t *dev, uint16_t mem_addr, uint8_t *p_data, size_t len);

EEPROM_Status_t EEPROM_WriteUInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, uint16_t value);
EEPROM_Status_t EEPROM_ReadUInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, uint16_t *p_value);

EEPROM_Status_t EEPROM_WriteInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, int16_t value);
EEPROM_Status_t EEPROM_ReadInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, int16_t *p_value);

EEPROM_Status_t EEPROM_EraseChip(EEPROM_Handle_t *dev, uint8_t erase_val);

void EEPROM_ResetI2CBus(EEPROM_Handle_t *dev);
#endif /* INC_EEPROM_FINAL_H_ */
