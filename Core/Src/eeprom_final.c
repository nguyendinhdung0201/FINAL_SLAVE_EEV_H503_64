/*
 * eeprom_final.c
 *
 *  Created on: Jun 6, 2025
 *      Author: PC
 */
#include "eeprom_final.h"
#include "string.h"
#include "main.h"
// --- Hàm xử lý lỗi I2C nội bộ và quyết định reset ---
static EEPROM_Status_t _EEPROM_HandleHALStatus(EEPROM_Handle_t *dev, HAL_StatusTypeDef hal_status, uint16_t mem_addr, bool is_write_op) {
    if (hal_status == HAL_OK) {
        dev->i2c_error_count = 0; // Reset bộ đếm lỗi nếu thành công
        return EEPROM_OK;
    }
    dev->i2c_error_count++;
    uint32_t i2c_error_code = HAL_I2C_GetError(dev->i2c_handle);

    printLOGDATA("[EEPROM] [WARN] I2C %s op failed. Addr=0x%04X, HAL_Status=%d, I2C_Error=0x%lX, ErrCnt=%u\r\n",
                   is_write_op ? "write" : "read", mem_addr, hal_status, i2c_error_code, dev->i2c_error_count);

    EEPROM_Status_t eep_status;
    switch (hal_status) {
        case HAL_TIMEOUT:
            eep_status = EEPROM_ERROR_TIMEOUT;
            break;
        case HAL_BUSY:
            eep_status = EEPROM_ERROR_BUSY;
            break;
        default: // HAL_ERROR
            eep_status = EEPROM_ERROR_GENERAL;
            if (i2c_error_code & HAL_I2C_ERROR_AF) { // Acknowledge Failure
            	printLOGDATA("[EEPROM] [ERROR] NACK received. EEPROM busy or not present. Addr=0x%04X\r\n", mem_addr);
                eep_status = EEPROM_ERROR_NACK;
            }
            if (i2c_error_code & HAL_I2C_ERROR_BERR) { // Bus Error
            	printLOGDATA("[EEPROM] [ERROR] I2C Bus Error (BERR) detected. Addr=0x%04X\r\n", mem_addr);
                eep_status = EEPROM_ERROR_BUS_FAULT;
            }
            if (i2c_error_code & HAL_I2C_ERROR_ARLO) { // Arbitration Lost
            	printLOGDATA("[EEPROM] [ERROR] I2C Arbitration Lost (ARLO) detected. Addr=0x%04X\r\n", mem_addr);
                eep_status = EEPROM_ERROR_BUS_FAULT;
            }
            if (i2c_error_code & HAL_I2C_ERROR_OVR) { // Overrun/Underrun
            	printLOGDATA("[EEPROM] [ERROR] I2C Overrun/Underrun (OVR) detected. Addr=0x%04X\r\n", mem_addr);
                eep_status = EEPROM_ERROR_BUS_FAULT; // Coi như lỗi bus
            }
            break;
    }

    // Quyết định reset bus I2C
    if ((eep_status == EEPROM_ERROR_BUS_FAULT || eep_status == EEPROM_ERROR_TIMEOUT || eep_status == EEPROM_ERROR_NACK /*NACK liên tục cũng là vấn đề*/) &&
        dev->i2c_error_count >= EEPROM_I2C_RESET_THRESHOLD) {
    	printLOGDATA("[EEPROM] [ERROR] I2C error threshold (%u) reached. Attempting I2C bus reset.\r\n", EEPROM_I2C_RESET_THRESHOLD);
    	EEPROM_ResetI2CBus(dev);
    }
    return eep_status;
}

// --- Chờ EEPROM hoàn thành chu trình ghi nội bộ (ACK polling) ---
static EEPROM_Status_t _EEPROM_WaitForWriteCompletion(EEPROM_Handle_t *dev) {
    HAL_StatusTypeDef hal_status;
    uint32_t start_tick = HAL_GetTick();

    printLOGDATA("[EEPROM] [DEBUG] Waiting for EEPROM write completion...\r\n");
    do {
        // Sử dụng device_address_8bit đã được dịch trái
        hal_status = HAL_I2C_IsDeviceReady(dev->i2c_handle, dev->device_address_8bit, 1, 5); // Timeout ngắn cho mỗi lần thử
        if (hal_status == HAL_OK) {
        	printLOGDATA("[EEPROM] [DEBUG] EEPROM write completed (ACK received).\r\n");
            dev->i2c_error_count = 0; // Sẵn sàng -> reset error count
            return EEPROM_OK;
        }
        if ((uint32_t)((HAL_GetTick() - start_tick)) > EEPROM_WRITE_CYCLE_TIMEOUT_MS) {
           	printLOGDATA("[EEPROM] [WARN] Timeout (%lu ms) waiting for write completion. DevAddr=0x%02X\r\n",
                                   EEPROM_WRITE_CYCLE_TIMEOUT_MS, dev->device_address_8bit);
            // Không trực tiếp gọi _handle_i2c_hal_status vì đây là lỗi logic của EEPROM không phản hồi,
            // không phải lỗi giao tiếp I2C trực tiếp trong khi chờ.
            // Nhưng vẫn tăng error count và có thể trigger reset nếu lặp lại.
            dev->i2c_error_count++;
             if (dev->i2c_error_count >= EEPROM_I2C_RESET_THRESHOLD) {
            	printLOGDATA("[EEPROM] [CRITICAL] EEPROM not ready threshold reached after write. Attempting I2C bus reset.\r\n");
            	EEPROM_ResetI2CBus(dev);
            }
            return EEPROM_ERROR_NOT_READY;
        }
        HAL_Delay(1); // Chờ 1ms rồi thử lại
    } while (true); // Vòng lặp sẽ thoát bởi return ở trên hoặc timeout
}


EEPROM_Status_t EEPROM_Init(EEPROM_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t device_7bit_addr) {
    if (dev == NULL || hi2c == NULL) {
        return EEPROM_ERROR_PARAM;
    }

    dev->i2c_handle = hi2c;
    dev->device_address_8bit = (uint16_t)(device_7bit_addr << 1);
    // Sử dụng các define mới từ file .h
    dev->page_size = CURRENT_EEPROM_PAGE_SIZE;
    dev->max_mem_address = CURRENT_EEPROM_MAX_MEM_ADDR;
    dev->initialized = false;
    dev->i2c_error_count = 0;

    // Xác định kích thước địa chỉ bộ nhớ HAL cần
    if (dev->max_mem_address <= 0xFF) { // AT24C01, AT24C02
        dev->mem_addr_size_hal = I2C_MEMADD_SIZE_8BIT;
    } else { // AT24C04 trở lên, và M24C64
        dev->mem_addr_size_hal = I2C_MEMADD_SIZE_16BIT;
    }

    // CURRENT_EEPROM_NAME đã được define trong .h
    printLOGDATA("[EEPROM] [INFO] Init: Type=%s, MaxAddr=0x%X, PageSize=%u, I2C_Addr7=0x%X, HAL_MemAddrSize=%s.\r\n",
                        CURRENT_EEPROM_NAME,
                        dev->max_mem_address, dev->page_size, device_7bit_addr,
                        (dev->mem_addr_size_hal == I2C_MEMADD_SIZE_8BIT) ? "8BIT" : "16BIT");

    HAL_StatusTypeDef hal_status = HAL_I2C_IsDeviceReady(dev->i2c_handle, dev->device_address_8bit, 2, EEPROM_I2C_TIMEOUT_MS);
    if (hal_status == HAL_OK) {
        dev->initialized = true;
        printLOGDATA("[EEPROM] [INFO] Init: Device detected. I2C_Addr8=0x%02X.\r\n", dev->device_address_8bit);
        return EEPROM_OK;
    } else {
    	printLOGDATA("[EEPROM] [ERROR] Init: Device not detected. I2C_Addr8=0x%02X, HAL_Status=%d.\r\n",
                       dev->device_address_8bit, hal_status);
        // Gọi _handle_i2c_hal_status (đã đổi tên nếu cần)
    	_EEPROM_HandleHALStatus(dev, hal_status, 0xFFFF, false);
        return EEPROM_ERROR_INIT_FAILED;
    }
}


EEPROM_Status_t EEPROM_IsDeviceReady(EEPROM_Handle_t *dev, uint32_t trials) {
    if (!dev || !dev->i2c_handle) return EEPROM_ERROR_PARAM;

    printLOGDATA("[EEPROM] [INFO] IsDeviceReady: Checking. Addr8=0x%02X, Trials=%lu...\r\n", dev->device_address_8bit, trials);
    HAL_StatusTypeDef hal_status = HAL_I2C_IsDeviceReady(dev->i2c_handle, dev->device_address_8bit, trials, EEPROM_I2C_TIMEOUT_MS);

    if (hal_status == HAL_OK) {
    	printLOGDATA("[EEPROM] [INFO] IsDeviceReady: Device is ready. Addr8=0x%02X.\r\n", dev->device_address_8bit);
        dev->initialized = true;
        dev->i2c_error_count = 0;
        return EEPROM_OK;
    } else {
    	printLOGDATA("[EEPROM] [WARN] IsDeviceReady: Device not ready. Addr8=0x%02X, HAL_Status=%d.\r\n",
                       dev->device_address_8bit, hal_status);
        return _EEPROM_HandleHALStatus(dev, hal_status, 0xFFFF, false);
    }
}



EEPROM_Status_t EEPROM_WriteByte(EEPROM_Handle_t *dev, uint16_t mem_addr, uint8_t data) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if (mem_addr > dev->max_mem_address) {
    	printLOGDATA("[EEPROM] [ERROR] WriteByte: Address out of range. Addr=0x%04X, MaxAddr=0x%04X.\r\n", mem_addr, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }

    printLOGDATA("[EEPROM] [DEBUG] WriteByte: Writing Data=0x%02X to Addr=0x%04X...\r\n", data, mem_addr);
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write(dev->i2c_handle,
                                                   dev->device_address_8bit,
                                                   mem_addr,
                                                   dev->mem_addr_size_hal,
                                                   &data,
                                                   1,
                                                   EEPROM_I2C_TIMEOUT_MS);

    EEPROM_Status_t eep_status = _EEPROM_HandleHALStatus(dev, hal_status, mem_addr, true);
    if (eep_status != EEPROM_OK) {
    	printLOGDATA("[EEPROM] [ERROR] WriteByte: HAL_I2C_Mem_Write failed. Addr=0x%04X, Data=0x%02X, Status=%d\r\n",
    	                       mem_addr, data, eep_status);
        return eep_status;
    }

    eep_status = _EEPROM_WaitForWriteCompletion(dev);
    if (eep_status == EEPROM_OK) {
    	printLOGDATA("[EEPROM] [DEBUG] WriteByte: Success. Addr=0x%04X, Data=0x%02X.\r\n", mem_addr, data);
    } else {
    	printLOGDATA("[EEPROM] [WARN] WriteByte: Not ready after write. Addr=0x%04X, Data=0x%02X.\r\n", mem_addr, data);
    }
    return eep_status;
}



EEPROM_Status_t EEPROM_ReadByte(EEPROM_Handle_t *dev, uint16_t mem_addr, uint8_t *p_data) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if (p_data == NULL) return EEPROM_ERROR_PARAM;
    if (mem_addr > dev->max_mem_address) {
    	printLOGDATA("[EEPROM] [ERROR] ReadByte: Address out of range. Addr=0x%04X, MaxAddr=0x%04X.\r\n", mem_addr, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }

    printLOGDATA("[EEPROM] [DEBUG] ReadByte: Reading from Addr=0x%04X...\r\n", mem_addr);
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(dev->i2c_handle,
                                                  dev->device_address_8bit,
                                                  mem_addr,
                                                  dev->mem_addr_size_hal,
                                                  p_data,
                                                  1,
                                                  EEPROM_I2C_TIMEOUT_MS);

    EEPROM_Status_t eep_status = _EEPROM_HandleHALStatus(dev, hal_status, mem_addr, false);
    if (eep_status == EEPROM_OK) {
    	printLOGDATA("[EEPROM] [DEBUG] ReadByte: Success. Addr=0x%04X, Data=0x%02X.\r\n", mem_addr, *p_data);
    } else {
    	printLOGDATA("[EEPROM] [ERROR] ReadByte: HAL_I2C_Mem_Read failed. Addr=0x%04X, Status=%d\r\n", mem_addr, eep_status);
    }
    return eep_status;
}



EEPROM_Status_t EEPROM_WriteBuffer(EEPROM_Handle_t *dev, uint16_t mem_addr, const uint8_t *p_data, size_t len) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if (p_data == NULL || len == 0) return EEPROM_ERROR_PARAM;
    if ((mem_addr + len -1) > dev->max_mem_address) {
    	printLOGDATA("[EEPROM] [ERROR] WriteBuffer: Address range out of bounds. Addr=0x%04X, Len=%u, MaxAddr=0x%04X.\r\n",
                        mem_addr, mem_addr + len - 1, (unsigned int)len, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }

    printLOGDATA("[EEPROM] [INFO] WriteBuffer: Writing %u bytes to Addr=0x%04X...\r\n", (unsigned int)len, mem_addr);

    HAL_StatusTypeDef hal_status;
    EEPROM_Status_t eep_status;
    uint16_t current_addr = mem_addr;
    const uint8_t *current_data_ptr = p_data;
    size_t remaining_len = len;

    while (remaining_len > 0) {
        uint16_t offset_in_page = current_addr % dev->page_size;
        size_t bytes_to_write_this_page = dev->page_size - offset_in_page;
        if (bytes_to_write_this_page > remaining_len) {
            bytes_to_write_this_page = remaining_len;
        }

        printLOGDATA("[EEPROM] [DEBUG] WriteBuffer: Page write. Addr=0x%04X, Len=%u, PageOffset=%u\r\n", (unsigned int)bytes_to_write_this_page, current_addr, offset_in_page);
        hal_status = HAL_I2C_Mem_Write(dev->i2c_handle,
                                       dev->device_address_8bit,
                                       current_addr,
                                       dev->mem_addr_size_hal,
                                       (uint8_t*)current_data_ptr,
                                       bytes_to_write_this_page,
                                       EEPROM_I2C_TIMEOUT_MS);

        eep_status = _EEPROM_HandleHALStatus(dev, hal_status, current_addr, true);
        if (eep_status != EEPROM_OK) {
        	printLOGDATA("[EEPROM] [ERROR] WriteBuffer: Page write failed during multi-byte op. Addr=0x%04X, Status=%d\r\n", current_addr, eep_status);
            return eep_status;
        }

        eep_status = _EEPROM_WaitForWriteCompletion(dev);
        if (eep_status != EEPROM_OK) {
        	printLOGDATA("[EEPROM] [ERROR] WriteBuffer: Not ready after page write. Addr=0x%04X.\r\n", current_addr);
            return eep_status;
        }

        current_addr += bytes_to_write_this_page;
        current_data_ptr += bytes_to_write_this_page;
        remaining_len -= bytes_to_write_this_page;
    }

    printLOGDATA("[EEPROM] [INFO] WriteBuffer: Success. %u bytes written to Addr=0x%04X.\r\n", (unsigned int)len, mem_addr);
    return EEPROM_OK;
}


EEPROM_Status_t EEPROM_ReadBuffer(EEPROM_Handle_t *dev, uint16_t mem_addr, uint8_t *p_data, size_t len) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if (p_data == NULL || len == 0) return EEPROM_ERROR_PARAM;
    if ((mem_addr + len -1) > dev->max_mem_address) {
    	printLOGDATA("[EEPROM] [ERROR] ReadBuffer: Address range out of bounds. Addr=0x%04X, Len=%u, MaxAddr=0x%04X.\r\n",
                        mem_addr, mem_addr + len - 1, (unsigned int)len, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }

    printLOGDATA("[EEPROM] [INFO] ReadBuffer: Reading %u bytes from Addr=0x%04X...\r\n", (unsigned int)len, mem_addr);
    uint32_t read_timeout = EEPROM_I2C_TIMEOUT_MS + (len / dev->page_size) * 5;

    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(dev->i2c_handle,
                                                  dev->device_address_8bit,
                                                  mem_addr,
                                                  dev->mem_addr_size_hal,
                                                  p_data,
                                                  len,
                                                  read_timeout);

    EEPROM_Status_t eep_status = _EEPROM_HandleHALStatus(dev, hal_status, mem_addr, false);
    if (eep_status == EEPROM_OK) {
    	printLOGDATA("[EEPROM] [INFO] ReadBuffer: Success. %u bytes read from Addr=0x%04X.\r\n", (unsigned int)len, mem_addr);
    } else {
    	printLOGDATA("[EEPROM] [ERROR] ReadBuffer: Failed. Addr=0x%04X, Len=%u, Status=%d\r\n", (unsigned int)len, mem_addr, eep_status);
    }
    return eep_status;
}


EEPROM_Status_t EEPROM_WriteUInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, uint16_t value) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if ((mem_addr + 1) > dev->max_mem_address) {
    	printLOGDATA("[EEPROM] [ERROR] WriteUInt16: Address out of range for 2 bytes. Addr=0x%04X, MaxAddr=0x%04X.\r\n",
    	                       mem_addr, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(value & 0xFF);        // LSB
    buffer[1] = (uint8_t)((value >> 8) & 0xFF); // MSB
    printLOGDATA("[EEPROM] [DEBUG] WriteUInt16: Writing Val=%u (0x%02X,0x%02X LSB-first) to Addr=0x%04X...\r\n",
                     value, buffer[0], buffer[1], mem_addr);
    return EEPROM_WriteBuffer(dev, mem_addr, buffer, 2);
}

EEPROM_Status_t EEPROM_ReadUInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, uint16_t *p_value) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if (p_value == NULL) return EEPROM_ERROR_PARAM;
    if ((mem_addr + 1) > dev->max_mem_address) {
     	printLOGDATA("[EEPROM] [ERROR] ReadUInt16: Address out of range for 2 bytes. Addr=0x%04X, MaxAddr=0x%04X.\r\n",
                           mem_addr, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }
    uint8_t buffer[2];
    printLOGDATA("[EEPROM] [DEBUG] ReadUInt16: Reading from Addr=0x%04X...\r\n", mem_addr);
    EEPROM_Status_t status = EEPROM_ReadBuffer(dev, mem_addr, buffer, 2);
    if (status == EEPROM_OK) {
        *p_value = ((uint16_t)buffer[1] << 8) | buffer[0];
        printLOGDATA("[EEPROM] [DEBUG] ReadUInt16: Success. Addr=0x%04X, Val=%u (0x%02X,0x%02X LSB-first).\r\n",
                               mem_addr, *p_value, buffer[0], buffer[1]);
    }
    return status;
}

EEPROM_Status_t EEPROM_WriteInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, int16_t value) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if ((mem_addr + 1) > dev->max_mem_address) {
    	printLOGDATA("[EEPROM] [ERROR] WriteInt16: Address out of range for 2 bytes. Addr=0x%04X, MaxAddr=0x%04X.\r\n",
    	                       mem_addr, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(value & 0xFF);        // LSB
    buffer[1] = (uint8_t)((value >> 8) & 0xFF); // MSB
    printLOGDATA("[EEPROM] [DEBUG] WriteInt16: Writing Val=%d (0x%02X,0x%02X LSB-first) to Addr=0x%04X...\r\n",
                     value, buffer[0], buffer[1], mem_addr);
    return EEPROM_WriteBuffer(dev, mem_addr, buffer, 2);
}

EEPROM_Status_t EEPROM_ReadInt16(EEPROM_Handle_t *dev, uint16_t mem_addr, int16_t *p_value) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;
    if (p_value == NULL) return EEPROM_ERROR_PARAM;
    if ((mem_addr + 1) > dev->max_mem_address) {
    	printLOGDATA("[EEPROM] [ERROR] ReadInt16: Address out of range for 2 bytes. Addr=0x%04X, MaxAddr=0x%04X.\r\n",
    	                       mem_addr, dev->max_mem_address);
        return EEPROM_ERROR_ADDR_OOR;
    }
    uint8_t buffer[2];
    printLOGDATA("[EEPROM] [DEBUG] ReadInt16: Reading from Addr=0x%04X...\r\n", mem_addr);
    EEPROM_Status_t status = EEPROM_ReadBuffer(dev, mem_addr, buffer, 2);
    if (status == EEPROM_OK) {
        *p_value = ((int16_t)buffer[1] << 8) | buffer[0];
         printLOGDATA("[EEPROM] [DEBUG] ReadInt16: Success. Addr=0x%04X, Val=%d (0x%02X,0x%02X LSB-first).\r\n",
                            mem_addr, *p_value, buffer[0], buffer[1]);
    }
    return status;
}

EEPROM_Status_t EEPROM_EraseChip(EEPROM_Handle_t *dev, uint8_t erase_val) {
    if (!dev || !dev->initialized) return EEPROM_ERROR_INIT_FAILED;

    printLOGDATA("[EEPROM] [INFO] EraseChip: Starting chip erase. Value=0x%02X, MaxAddr=0x%04X, PageSize=%u.\r\n",
                       erase_val, dev->max_mem_address, dev->page_size);

    uint8_t page_buffer[CURRENT_EEPROM_PAGE_SIZE]; // Use defined page size for stack buffer
    memset(page_buffer, erase_val, dev->page_size);

    EEPROM_Status_t status = EEPROM_OK;
    for (uint32_t addr = 0; addr <= dev->max_mem_address; ) {
        size_t bytes_to_write_this_op = dev->page_size;
        if (addr + dev->page_size > (dev->max_mem_address + 1)) {
            bytes_to_write_this_op = (dev->max_mem_address + 1) - addr;
        }

        if (bytes_to_write_this_op == 0) break;

        printLOGDATA("[EEPROM] [DEBUG] EraseChip: Erasing page. Addr=0x%04lX, Len=%u, Val=0x%02X.\r\n", addr, (unsigned int)bytes_to_write_this_op, erase_val);
        status = EEPROM_WriteBuffer(dev, (uint16_t)addr, page_buffer, bytes_to_write_this_op);
        if (status != EEPROM_OK) {
        	printLOGDATA("[EEPROM] [ERROR] EraseChip: Page write failed. Addr=0x%04lX, Status=%d.\r\n", addr, status);
            return status;
        }
        addr += bytes_to_write_this_op;
    }

    printLOGDATA("[EEPROM] [INFO] EraseChip: Chip erase completed successfully.\r\n");
    return EEPROM_OK;
}

void EEPROM_ResetI2CBus(EEPROM_Handle_t *dev) {
    if (dev == NULL || dev->i2c_handle == NULL) {
    	printLOGDATA("[EEPROM] [ERROR] ResetI2CBus: Null pointer argument (dev or i2c_handle).\r\n");
        return;
    }

    printLOGDATA("[EEPROM] [INFO] ResetI2CBus: Attempting I2C peripheral reset for I2C_Addr8=0x%02X.\r\n", dev->device_address_8bit);

    if (HAL_I2C_DeInit(dev->i2c_handle) != HAL_OK) {
    	printLOGDATA("[EEPROM] [ERROR] ResetI2CBus: HAL_I2C_DeInit failed.\r\n");
        dev->initialized = false;
        return;
    }

    HAL_Delay(10); // Short delay before re-initialization

    I2C1_Reinit();

//    if (HAL_I2C_Init(dev->i2c_handle) != HAL_OK) {
//    	printf("HAL_I2C_Init failed after DeInit during bus reset.");
//        dev->initialized = false;
//    } else {
//    	printf("I2C peripheral reset and re-initialized successfully.");
//        dev->i2c_error_count = 0;
//        // Re-check device readiness after reset
//        HAL_StatusTypeDef hal_status = HAL_I2C_IsDeviceReady(dev->i2c_handle, dev->device_address_8bit, 2, EEPROM_I2C_TIMEOUT_MS);
//        if (hal_status == HAL_OK) {
//            dev->initialized = true;
//            printf("EEPROM re-detected after I2C bus reset.");
//        } else {
//            dev->initialized = false;
//            printf("EEPROM still not detected after I2C bus reset. HAL Status: %d", hal_status);
//        }
//    }
    printLOGDATA("[EEPROM] [INFO] ResetI2CBus: I2C peripheral re-initialized. Verifying device...\r\n");
    dev->i2c_error_count = 0; // Reset error count after attempting recovery
    HAL_StatusTypeDef hal_status = HAL_I2C_IsDeviceReady(dev->i2c_handle, dev->device_address_8bit, 2, EEPROM_I2C_TIMEOUT_MS);
    if (hal_status == HAL_OK) {
        dev->initialized = true; // Crucial: mark as initialized again
        printLOGDATA("[EEPROM] [INFO] ResetI2CBus: EEPROM re-detected successfully. Addr8=0x%02X.\r\n", dev->device_address_8bit);
    } else {
        dev->initialized = false;
        printLOGDATA("[EEPROM] [ERROR] ResetI2CBus: EEPROM still not detected after reset. Addr8=0x%02X, HAL_Status=%d.\r\n",
                       dev->device_address_8bit, hal_status);
    }
}
