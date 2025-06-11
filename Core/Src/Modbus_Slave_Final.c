/*
 * Modbus_Slave_Final.c
 *
 *  Created on: May 22, 2025
 *      Author: PC
 */
/* Includes ------------------------------------------------------------------*/
#include "Modbus_Slave_Final.h" // Bao gồm header của chính thư viện này
#include <string.h>           // Cần cho memcpy và memset
#include <stdbool.h>          // Cần cho kiểu bool có giá trị true/false
/* Private Defines ---------------------------------------------------------*/
// Định nghĩa các hằng số nội bộ nếu cần (hiện tại không có)
// Kích thước khung Modbus RTU tối thiểu (Địa chỉ Slave + FC + CRC)
#define MODBUS_MIN_FRAME_SIZE 4
// Kích thước khung Modbus RTU tối thiểu cho các chức năng với địa chỉ và số lượng/giá trị (Địa chỉ + FC + Địa chỉ(2) + Số lượng/Giá trị(2) + CRC)
#define MODBUS_MIN_REQ_RESP_SIZE 8
/**
 * @brief Đọc giá trị 16-bit từ buffer theo thứ tự byte Big-Endian (MSB first).
 * @param data Con trỏ tới buffer (mảng uint8_t không đổi).
 * @param index Chỉ số bắt đầu của byte cao (MSB) trong buffer.
 * @return Giá trị uint16_t đã đọc.
 */
static inline uint16_t Modbus_ReadU16_BE(const uint8_t* data, uint16_t index) {
    return (((uint16_t)data[index] << 8) | (uint16_t)data[index + 1]);
}
/**
 * @brief Ghi giá trị 16-bit vào buffer theo thứ tự byte Big-Endian (MSB first).
 * @param buffer Con trỏ tới buffer (mảng uint8_t) sẽ ghi vào.
 * @param index Chỉ số bắt đầu để ghi byte cao (MSB).
 * @param value Giá trị uint16_t cần ghi.
 */
static inline void Modbus_WriteU16_BE(uint8_t* buffer, uint16_t index, uint16_t value) {
    buffer[index]     = (uint8_t)(value >> 8); // Ghi MSB
    buffer[index + 1] = (uint8_t)(value & 0xFF); // Ghi LSB
}

/* Private Variables -------------------------------------------------------*/
/**
 * @brief Bảng tra cứu CRC16 Modbus (đa thức 0xA001 - tương đương 0x8005 đảo bit).
 */
static const uint16_t modbus_crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* Private Function Prototypes ---------------------------------------------*/
// Khai báo các hàm nội bộ (static) để cấu trúc code rõ ràng hơn.

// --- Hàm trợ giúp ---
static uint16_t Modbus_CRC16_Table(const uint8_t* data, uint16_t length);
static void Modbus_SendResponse(ModbusHandle* modbus, uint16_t length);
static void Modbus_SendExceptionResponse(ModbusHandle* modbus, uint8_t functionCode, uint8_t exceptionCode);
static inline bool Modbus_GetBit(const uint8_t* data, uint16_t bit_index);
static inline void Modbus_SetBit(uint8_t* data, uint16_t bit_index, bool value);

// --- Hàm xử lý cho từng Function Code ---
static void Modbus_HandleReadCoils(ModbusHandle* modbus, uint16_t address, uint16_t quantity);
static void Modbus_HandleReadDiscrete(ModbusHandle* modbus, uint16_t address, uint16_t quantity);
static void Modbus_HandleReadHolding(ModbusHandle* modbus, uint16_t address, uint16_t quantity);
static void Modbus_HandleReadInput(ModbusHandle* modbus, uint16_t address, uint16_t quantity);
static void Modbus_HandleWriteSingleCoil(ModbusHandle* modbus, uint16_t address, uint16_t value, bool is_broadcast);
static void Modbus_HandleWriteSingleReg(ModbusHandle* modbus, uint16_t address, uint16_t value, bool is_broadcast);
static void Modbus_HandleWriteMultipleCoils(ModbusHandle* modbus, uint16_t address, uint16_t quantity, bool is_broadcast);
static void Modbus_HandleWriteMultipleRegs(ModbusHandle* modbus, uint16_t address, uint16_t quantity, bool is_broadcast);


/* Private Helper Functions ------------------------------------------------*/

/**
 * @brief Tính toán giá trị CRC16 Modbus sử dụng bảng tra cứu.
 * @param data Con trỏ tới mảng dữ liệu cần tính CRC.
 * @param length Số lượng byte trong mảng dữ liệu.
 * @return Giá trị CRC16 (16-bit), lưu ý thứ tự byte trong giá trị trả về
 *         là Little-Endian (byte thấp trước, byte cao sau trong thanh ghi 16-bit).
 */
static uint16_t Modbus_CRC16_Table(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF; // Giá trị khởi tạo CRC
    uint8_t lut_index;

    while (length--) {
        // XOR byte dữ liệu hiện tại với byte thấp của CRC để lấy index cho bảng tra cứu
        lut_index = (crc ^ *data++) & 0xFF;
        // Dịch phải CRC 8 bit và XOR với giá trị từ bảng tra cứu
        crc = (crc >> 8) ^ modbus_crc_table[lut_index];
    }
    // Kết quả crc bây giờ chứa giá trị CRC16 đúng theo chuẩn Modbus RTU.
    // Khi gửi đi, byte thấp (crc & 0xFF) sẽ được gửi trước, byte cao (crc >> 8) gửi sau.
    return crc;
}

/**
 * @brief Gửi gói tin phản hồi Modbus qua UART bằng DMA.
 * @param modbus Con trỏ tới cấu trúc ModbusHandle.
 * @param length Độ dài của dữ liệu trong `modbus->txBuffer` (CHƯA bao gồm 2 byte CRC).
 * @note Hàm này sẽ tính CRC, thêm CRC vào cuối `txBuffer`, cập nhật trạng thái
 *       thành TRANSMITTING và gọi HAL_UART_Transmit_DMA.
 */
static void Modbus_SendResponse(ModbusHandle* modbus, uint16_t length) {
    // Kiểm tra xem độ dài phản hồi có vượt quá kích thước buffer không (trừ 2 byte cho CRC)
    if (length > (MODBUS_TX_BUFFER_SIZE - 2)) {
        // Lỗi nghiêm trọng: Phản hồi quá dài.
        // Không gửi gì cả và quay lại trạng thái sẵn sàng.
        // Nên log lỗi này để debug.
        modbus->state = MODBUS_STATE_IDLE;
        return;
    }

    // Tính toán CRC cho phần dữ liệu (length bytes) trong txBuffer
    uint16_t crc = Modbus_CRC16_Table(modbus->txBuffer, length);

    // Thêm CRC vào cuối buffer (byte thấp trước, byte cao sau)
    modbus->txBuffer[length]     = (uint8_t)(crc & 0xFF);
    modbus->txBuffer[length + 1] = (uint8_t)((crc >> 8) & 0xFF);

    // Chỉ chuyển sang TRANSMITTING nếu chưa phải
    // (để tránh gọi HAL_UART_Transmit_DMA nhiều lần nếu có lỗi logic)
    if (modbus->state == MODBUS_STATE_PROCESSING) {
		modbus->state = MODBUS_STATE_TRANSMITTING;
		// Bắt đầu truyền dữ liệu (dữ liệu + 2 byte CRC) bằng DMA
		if (HAL_UART_Transmit_DMA(modbus->huart, modbus->txBuffer, length + 2) != HAL_OK) {
			// Lỗi khi bắt đầu truyền DMA!
			// Đây là lỗi nghiêm trọng, cần xử lý (vd: log lỗi, thử lại?, reset state).
			// Quan trọng là phải đưa state về IDLE để tránh bị kẹt.
			modbus->state = MODBUS_STATE_IDLE;
		}
	    // Trạng thái sẽ về IDLE trong TxCpltCallback nếu truyền thành công
    }else{
        // Lỗi logic: Gọi SendResponse khi state không phải là PROCESSING
         modbus->state = MODBUS_STATE_IDLE; // Cố gắng phục hồi về IDLE
    }
    // Lưu ý: Trạng thái sẽ được reset về IDLE trong Modbus_UartTxCpltCallback
    // khi quá trình truyền DMA hoàn tất thành công.
}

/**
 * @brief Gửi gói tin phản hồi ngoại lệ (Exception Response) Modbus.
 * @param modbus Con trỏ tới cấu trúc ModbusHandle.
 * @param functionCode Mã chức năng gốc từ yêu cầu của Master gây ra lỗi.
 * @param exceptionCode Mã ngoại lệ Modbus (xem MODBUS_EXCEPTION_...).
 * @note Hàm này xây dựng gói tin lỗi gồm 3 byte:
 *       [Slave Address] [Function Code + 0x80] [Exception Code]
 *       và gửi đi bằng Modbus_SendResponse (sẽ tự thêm CRC).
 */
static void Modbus_SendExceptionResponse(ModbusHandle* modbus, uint8_t functionCode, uint8_t exceptionCode) {

	 // KHÔNG gửi exception cho broadcast request
	if (modbus->rxBuffer[0] == 0) {
		modbus->state = MODBUS_STATE_IDLE; // Chỉ cần quay lại IDLE
		return;
	}

    // Kiểm tra buffer đủ chứa 3 byte dữ liệu + 2 byte CRC không
    if (MODBUS_TX_BUFFER_SIZE < 5) {
         modbus->state = MODBUS_STATE_IDLE; // Không đủ buffer, không gửi được lỗi
         // Log lỗi này.
         return;
    }

    // Lấy địa chỉ Slave từ gói tin yêu cầu gốc (đã lưu trong rxBuffer)
    modbus->txBuffer[0] = modbus->rxBuffer[0];
    // Set bit cao nhất của Function Code để báo lỗi
    modbus->txBuffer[1] = functionCode | 0x80;
    // Ghi mã lỗi cụ thể
    modbus->txBuffer[2] = exceptionCode;

    // Gửi phản hồi lỗi (3 bytes data)
    // SendResponse sẽ tự thêm CRC và quản lý state
    Modbus_SendResponse(modbus, 3);
}

/**
 * @brief Lấy giá trị của một bit cụ thể trong mảng byte.
 * @param data Con trỏ tới mảng byte chứa dữ liệu bit (đọc const).
 * @param bit_index Chỉ số của bit cần lấy (tính từ 0).
 * @return `true` nếu bit là 1, `false` nếu bit là 0.
 */
static inline bool Modbus_GetBit(const uint8_t* data, uint16_t bit_index) {
    // Xác định byte chứa bit và vị trí bit trong byte
    uint16_t byte_index = bit_index / 8;
    uint8_t bit_offset = bit_index % 8;
    // Đọc byte, dịch phải để bit cần lấy về vị trí LSB, và AND với 1
    return (data[byte_index] >> bit_offset) & 0x01;
}

/**
 * @brief Thiết lập giá trị của một bit cụ thể trong mảng byte.
 * @param data Con trỏ tới mảng byte chứa dữ liệu bit (sẽ được sửa đổi).
 * @param bit_index Chỉ số của bit cần thiết lập (tính từ 0).
 * @param value Giá trị cần thiết lập (`true` để set thành 1, `false` để clear thành 0).
 */
static inline void Modbus_SetBit(uint8_t* data, uint16_t bit_index, bool value) {
    uint16_t byte_index = bit_index / 8;
    uint8_t bit_offset = bit_index % 8;
    // Tạo mặt nạ (mask) cho bit cần thao tác
    uint8_t mask = (1 << bit_offset);

    if (value) {
        // Set bit: Dùng phép OR với mask
        data[byte_index] |= mask;
    } else {
        // Clear bit: Dùng phép AND với NOT của mask
        data[byte_index] &= ~mask;
    }
}


/* Static Handler Functions - Xử lý các Function Code cụ thể -------------*/
// Các hàm này được gọi bởi Modbus_ProcessData dựa trên function code nhận được.
// Chúng chịu trách nhiệm kiểm tra tham số, đọc/ghi dữ liệu vào vùng nhớ
// của Slave (coils, registers,...) và chuẩn bị buffer phản hồi (txBuffer).

/**
 * @brief FC 0x01: Xử lý yêu cầu đọc Coils.
 */
static void Modbus_HandleReadCoils(ModbusHandle* modbus, uint16_t address, uint16_t quantity) {
    // 1. Kiểm tra số lượng (Quantity): 1 đến 2000
    if (quantity == 0 || quantity > 2000) {
        Modbus_SendExceptionResponse(modbus, READ_COILS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    // 2. Kiểm tra phạm vi địa chỉ (Address Range)
    uint32_t end_address = (uint32_t)address + quantity; // Dùng uint32_t để tránh tràn số khi cộng
    if (address >= MAX_COILS || end_address > MAX_COILS) {
         Modbus_SendExceptionResponse(modbus, READ_COILS, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
         return;
    }

    // 3. Tính số byte dữ liệu cần gửi (CoilData)
    uint8_t CoilData = (quantity + 7) / 8; // Chia cho 8 và làm tròn lên

    // 4. Kiểm tra buffer gửi có đủ chỗ không
    // Cần: ID(1) + FC(1) + ByteCount(1) + CoilData + 2 (CRC)
    if ((3 + CoilData + 2) > (MODBUS_TX_BUFFER_SIZE)) {
        // Lỗi nội bộ: Buffer không đủ lớn -> Gửi lỗi Slave Failure
        Modbus_SendExceptionResponse(modbus, READ_COILS, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }

    // 5. Chuẩn bị phần đầu của buffer phản hồi
    modbus->txBuffer[0] = modbus->rxBuffer[0]; // Slave Address (lấy từ request)
    modbus->txBuffer[1] = READ_COILS;          // Function Code
    modbus->txBuffer[2] = CoilData;            // Số byte dữ liệu Coil theo sau

    // 6. Xóa vùng dữ liệu Coil trong buffer phản hồi (quan trọng!)
    memset(&modbus->txBuffer[3], 0, CoilData);

    // 7. Đọc và đóng gói dữ liệu Coils vào txBuffer
    for (uint16_t i = 0; i < quantity; i++) {
        // Đọc trạng thái coil từ bộ đệm `modbus->coils`
        if (Modbus_GetBit(modbus->coils, address + i)) {
            // Nếu coil ON (true), set bit tương ứng trong buffer phản hồi
            Modbus_SetBit(&modbus->txBuffer[3], i, true);
        }
        // Không cần làm gì nếu coil OFF vì buffer đã được memset về 0.
    }
    // 8. Gửi phản hồi ID(1) + FC(1) + ByteCount(1) + CoilData(N) + CRCLo(1) + CRCHi(1)
    // CoilData(N) = (quantity + 7) / 8
    // Hàm Modbus_SendResponse sẽ tự thêm CRC
    Modbus_SendResponse(modbus, 3 + CoilData);
}

/**
 * @brief FC 0x02: Xử lý yêu cầu đọc Discrete Inputs (Đọc Ngõ Vào Rời Rạc)
 *        Logic tương tự FC 0x01 nhưng đọc từ `modbus->discreteInputs`.
 */
static void Modbus_HandleReadDiscrete(ModbusHandle* modbus, uint16_t address, uint16_t quantity) {
	// 1. Kiểm tra số lượng (Quantity): 1 đến 2000
    if (quantity == 0 || quantity > 2000) {
        Modbus_SendExceptionResponse(modbus, READ_DISCRETE, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    // 2. Kiểm tra phạm vi địa chỉ (Address Range)
    uint32_t end_address = (uint32_t)address + quantity;
    if (address >= MAX_DISCRETE || end_address > MAX_DISCRETE) {
         Modbus_SendExceptionResponse(modbus, READ_DISCRETE, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
         return;
    }
    // 3. Tính số byte dữ liệu cần gửi (InputData)
    uint8_t InputData = (quantity + 7) / 8;

    // 4. Kiểm tra buffer gửi có đủ chỗ không
    // Cần: ID(1) + FC(1) + ByteCount(1) + InputData + 2 (CRC)
    if ((3 + InputData) > (MODBUS_TX_BUFFER_SIZE - 2)) {
        Modbus_SendExceptionResponse(modbus, READ_DISCRETE, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }

    // 5. Chuẩn bị phần đầu của buffer phản hồi
    modbus->txBuffer[0] = modbus->rxBuffer[0]; // Slave Address (lấy từ request)
    modbus->txBuffer[1] = READ_DISCRETE;       // Function Code
    modbus->txBuffer[2] = InputData;           // Số byte dữ liệu Input theo sau

    // 6. Xóa vùng dữ liệu Input trong buffer phản hồi (quan trọng!)
    memset(&modbus->txBuffer[3], 0, InputData);

    // 7. Đọc và đóng gói dữ liệu InputData vào txBuffer
    for (uint16_t i = 0; i < quantity; i++) {
        if (Modbus_GetBit(modbus->discreteInputs, address + i)) {
             Modbus_SetBit(&modbus->txBuffer[3], i, true);
        }
    }
    // 8. Gửi phản hồi ID(1) + FC(1) + ByteCount(1) + InputData(N) + CRCLo(1) + CRCHi(1)
    // InputData(N) = (quantity + 7) / 8
    // Hàm Modbus_SendResponse sẽ tự thêm CRC
    Modbus_SendResponse(modbus, 3 + InputData);
}

/**
 * @brief FC 0x03: Xử lý yêu cầu đọc Holding Registers (Đọc Thanh Ghi Lưu Trữ)
 */
static void Modbus_HandleReadHolding(ModbusHandle* modbus, uint16_t address, uint16_t quantity) {
    // 1. Kiểm tra số lượng (Quantity): 1 đến 125
    if (quantity == 0 || quantity > 125) {
        Modbus_SendExceptionResponse(modbus, READ_HOLDING, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    // 2. Kiểm tra phạm vi địa chỉ
     uint32_t end_address = (uint32_t)address + quantity;
    if (address >= MAX_HOLDING_REGS || end_address > MAX_HOLDING_REGS) {
        Modbus_SendExceptionResponse(modbus, READ_HOLDING, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }

    // 3. Tính số byte dữ liệu (Byte Count = số lượng * 2 bytes/register)
    uint8_t RegN = quantity * 2;

    // 4. Kiểm tra buffer gửi
    if ((3 + RegN + 2) > (MODBUS_TX_BUFFER_SIZE)) {
        Modbus_SendExceptionResponse(modbus, READ_HOLDING, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }

    // 5. Chuẩn bị header phản hồi
    modbus->txBuffer[0] = modbus->rxBuffer[0]; // Slave Address
    modbus->txBuffer[1] = READ_HOLDING;        // Function Code
    modbus->txBuffer[2] = RegN;                // Số byte dữ liệu register theo sau

    // 6. Đọc và đóng gói dữ liệu Holding Registers (Big-Endian) vào txBuffer
    for (uint16_t i = 0; i < quantity; i++) {
        // Đọc giá trị từ mảng holdingRegs
        uint16_t regValue = modbus->holdingRegs[address + i];
        // Ghi vào buffer (đảm bảo thứ tự Big-Endian)
        Modbus_WriteU16_BE(modbus->txBuffer, 3 + i * 2, regValue);
    }

    // 7. Gửi phản hồi ID(1) + FC(1) + ByteCount(1) + Reg1Hi(1) + Reg1Lo(1) + ... + RegNHi(1) + RegNLo(1) + CRCLo(1) + CRCHi(1)
    // RegN = quantity*2
    // Hàm Modbus_SendResponse sẽ tự thêm CRC
    Modbus_SendResponse(modbus, 3 + RegN);
}

/**
 * @brief FC 0x04: Xử lý yêu cầu đọc Input Registers (Đọc Thanh Ghi Ngõ Vào)
 *        Logic tương tự FC 0x03 nhưng đọc từ `modbus->inputRegs`.
 */
static void Modbus_HandleReadInput(ModbusHandle* modbus, uint16_t address, uint16_t quantity) {
	// 1. Kiểm tra số lượng (Quantity): 1 đến 125
    if (quantity == 0 || quantity > 125) {
        Modbus_SendExceptionResponse(modbus, READ_INPUT, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    // 2. Kiểm tra phạm vi địa chỉ
    uint32_t end_address = (uint32_t)address + quantity;
    if (address >= MAX_INPUT_REGS || end_address > MAX_INPUT_REGS) {
        Modbus_SendExceptionResponse(modbus, READ_INPUT, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }
    // 3. Tính số byte dữ liệu (Byte Count = số lượng * 2 bytes/register)
    uint8_t RegN = quantity * 2;
    if ((3 + RegN + 2) > (MODBUS_TX_BUFFER_SIZE)) {
        Modbus_SendExceptionResponse(modbus, READ_INPUT, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }
    // 5. Chuẩn bị header phản hồi
    modbus->txBuffer[0] = modbus->rxBuffer[0];
    modbus->txBuffer[1] = READ_INPUT;
    modbus->txBuffer[2] = RegN;

    // 6. Đọc và đóng gói dữ liệu Input Registers (Big-Endian) vào txBuffer
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t regValue = modbus->inputRegs[address + i];
        Modbus_WriteU16_BE(modbus->txBuffer, 3 + i * 2, regValue);
    }

    // 7. Gửi phản hồi ID(1) + FC(1) + ByteCount(1) + Reg1Hi(1) + Reg1Lo(1) + ... + RegNHi(1) + RegNLo(1) + CRCLo(1) + CRCHi(1)
    // RegN = quantity*2
    // Hàm Modbus_SendResponse sẽ tự thêm CRC
    Modbus_SendResponse(modbus, 3 + RegN);
}

/**
 * @brief FC 0x05: Xử lý yêu cầu ghi một Coil.
 *        Phản hồi là echo (lặp lại) của yêu cầu.
 */
static void Modbus_HandleWriteSingleCoil(ModbusHandle* modbus, uint16_t address, uint16_t value, bool is_broadcast) {
    // 1. Kiểm tra địa chỉ
    if (address >= MAX_COILS) {
        Modbus_SendExceptionResponse(modbus, WRITE_SINGLE_COIL, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }
    // 2. Kiểm tra giá trị ghi (phải là 0xFF00 cho ON hoặc 0x0000 cho OFF)
    if (value != 0xFF00 && value != 0x0000) {
        Modbus_SendExceptionResponse(modbus, WRITE_SINGLE_COIL, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    // 3. Thực hiện ghi vào bộ đệm coils
    Modbus_SetBit(modbus->coils, address, (value == 0xFF00));



    // ** Chỉ gửi phản hồi nếu KHÔNG phải broadcast **
   if (!is_broadcast) {
	   // 4. Chuẩn bị phản hồi (là bản sao của 6 byte đầu của yêu cầu)
	   if (6 > (MODBUS_TX_BUFFER_SIZE - 2)) {
			Modbus_SendExceptionResponse(modbus, WRITE_SINGLE_COIL, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE); return;
	   }
	   // 5. Gửi phản hồi ID(1) + FC(1) + AddrHi(1) + AddrLo(1) + ValueHi(1) + ValueLo(1) + CRCLo(1) + CRCHi(1)
	   // Hàm Modbus_SendResponse sẽ tự thêm CRC
	   // Phản hồi là một bản sao (echo) của yêu cầu.
	   memcpy(modbus->txBuffer, modbus->rxBuffer, 6);
	   Modbus_SendResponse(modbus, 6);
   } else {
	   // Nếu là broadcast, chỉ cần quay lại IDLE
	   modbus->state = MODBUS_STATE_IDLE;
   }
}

/**
 * @brief FC 0x06: Xử lý yêu cầu ghi một Holding Register.
 *        Phản hồi là echo (lặp lại) của yêu cầu.
 */
static void Modbus_HandleWriteSingleReg(ModbusHandle* modbus, uint16_t address, uint16_t value, bool is_broadcast) {
    // 1. Kiểm tra địa chỉ
    if (address >= MAX_HOLDING_REGS) {
        Modbus_SendExceptionResponse(modbus, WRITE_SINGLE_REG, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }

    // 2. Thực hiện ghi vào bộ đệm holdingRegs
    modbus->holdingRegs[address] = value;


    // ** Chỉ gửi phản hồi nếu KHÔNG phải broadcast **
	  if (!is_broadcast) {
		  // 3. Chuẩn bị phản hồi (là bản sao của 6 byte đầu của yêu cầu)
		  if (6 > (MODBUS_TX_BUFFER_SIZE - 2)) {
			   Modbus_SendExceptionResponse(modbus, WRITE_SINGLE_REG, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE); return;
		  }
	      // 4. Gửi phản hồi ID(1) + FC(1) + AddrHi(1) + AddrLo(1) + ValueHi(1) + ValueLo(1) + CRCLo(1) + CRCHi(1)
	      // Hàm Modbus_SendResponse sẽ tự thêm CRC
	      // Phản hồi là một bản sao (echo) của yêu cầu.
		  memcpy(modbus->txBuffer, modbus->rxBuffer, 6);
		  Modbus_SendResponse(modbus, 6);
	  } else {
		  // Nếu là broadcast, chỉ cần quay lại IDLE
		  modbus->state = MODBUS_STATE_IDLE;
	  }
}

/**
 * @brief FC 0x0F: Xử lý yêu cầu ghi nhiều Coils.
 *        Phản hồi chỉ chứa địa chỉ bắt đầu và số lượng đã ghi.
 */
static void Modbus_HandleWriteMultipleCoils(ModbusHandle* modbus, uint16_t address, uint16_t quantity, bool is_broadcast) {
    // 1. Kiểm tra số lượng (Quantity): 1 đến 1968
    if (quantity == 0 || quantity > 1968) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_COILS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    // 2. Kiểm tra phạm vi địa chỉ
    uint32_t end_address = (uint32_t)address + quantity;
    if (address >= MAX_COILS || end_address > MAX_COILS) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_COILS, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }

    // 3. Kiểm tra độ dài frame tối thiểu
    // Cần: ID(1) + FC(1) + StartAddrHi(1) + StartAddrLo(1) + QtyHi(1) + QtyLo(1) + ByteCount(1) + CoilData_min(1) + CRCLo(1) + CRCHi(1) = 10
    if (modbus->rxCount < 10) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_COILS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    // 4. Lấy Byte Count từ yêu cầu (byte thứ 7, index 6)
    uint8_t byteCount = modbus->rxBuffer[6];

    // 5. Kiểm tra sự nhất quán giữa Quantity và Byte Count
    uint8_t expectedByteCount = (quantity + 7) / 8;
    if (byteCount == 0 || byteCount != expectedByteCount) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_COILS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    // 6. Kiểm tra tổng độ dài frame nhận được có khớp không
    // Độ dài mong đợi = Header(7) + Data(byteCount) + CRC(2)
    if (modbus->rxCount != (7 + byteCount + 2)) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_COILS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    // 7. Thực hiện ghi dữ liệu từ rxBuffer (bắt đầu từ index 7) vào bộ đệm coils
    const uint8_t* coilData = &modbus->rxBuffer[7]; // Con trỏ tới đầu dữ liệu coil trong rxBuffer
    for (uint16_t i = 0; i < quantity; i++) {
        bool coilValue = Modbus_GetBit(coilData, i); // Lấy bit thứ i từ dữ liệu nhận được
        Modbus_SetBit(modbus->coils, address + i, coilValue); // Ghi vào mảng coils của slave
    }

    // ** Chỉ gửi phản hồi nếu KHÔNG phải broadcast **
    if (!is_broadcast) {
        // 8. Chuẩn bị phản hồi: Addr(1) FC(1) StartAddr(2) Quantity(2) = 6 bytes
        if (6 > (MODBUS_TX_BUFFER_SIZE - 2)) {
            Modbus_SendExceptionResponse(modbus, WRITE_MULTI_COILS, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE); return;
        }
        modbus->txBuffer[0] = modbus->rxBuffer[0];   // Slave Address
        modbus->txBuffer[1] = WRITE_MULTI_COILS;     // Function Code
        Modbus_WriteU16_BE(modbus->txBuffer, 2, address);  // Start Address (ghi lại từ request)
        Modbus_WriteU16_BE(modbus->txBuffer, 4, quantity); // Quantity (ghi lại từ request)
        // 9. Gửi phản hồi ID(1) + FC(1) + StartAddrHi(1) + StartAddrLo(1) + QtyHi(1) + QtyLo(1) + CRCLo(1) + CRCHi(1)
        // Hàm Modbus_SendResponse sẽ tự thêm CRC
        Modbus_SendResponse(modbus, 6);
    } else {
        modbus->state = MODBUS_STATE_IDLE;
    }
}


/**
 * @brief FC 0x10: Xử lý yêu cầu ghi nhiều Holding Registers.
 *        Phản hồi chỉ chứa địa chỉ bắt đầu và số lượng đã ghi.
 */
static void Modbus_HandleWriteMultipleRegs(ModbusHandle* modbus, uint16_t address, uint16_t quantity, bool is_broadcast) {
    // 1. Kiểm tra số lượng (Quantity): 1 đến 123
    if (quantity == 0 || quantity > 123) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    // 2. Kiểm tra phạm vi địa chỉ
    uint32_t end_address = (uint32_t)address + quantity;
    if (address >= MAX_HOLDING_REGS || end_address > MAX_HOLDING_REGS) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_REGS, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }

    // 3. Kiểm tra độ dài frame tối thiểu
    // Cần: ID(1) + FC(1) + StartAddrHi(1) + StartAddrLo(1) + QtyHi(1) + QtyLo(1) + ByteCount(1) + RegData_min(2) + CRCLo(1) + CRCHi(1) = 11
    if (modbus->rxCount < 11) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    // 4. Lấy Byte Count từ yêu cầu (index 6)
    uint8_t byteCount = modbus->rxBuffer[6];

    // 5. Kiểm tra sự nhất quán giữa Quantity và Byte Count
    if (byteCount == 0 || byteCount != quantity * 2) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    // 6. Kiểm tra tổng độ dài frame nhận được
    if (modbus->rxCount != (7 + byteCount + 2)) {
        Modbus_SendExceptionResponse(modbus, WRITE_MULTI_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    // 7. Thực hiện ghi dữ liệu từ rxBuffer (bắt đầu từ index 7) vào holdingRegs
    const uint8_t* regData = &modbus->rxBuffer[7]; // Con trỏ tới đầu dữ liệu register trong rxBuffer
    for (uint16_t i = 0; i < quantity; i++) {
        // Đọc giá trị register 16-bit từ buffer request (Big-Endian)
        uint16_t regValue = Modbus_ReadU16_BE(regData, i * 2);
        // (Tùy chọn) Kiểm tra giá trị `regValue` nếu cần.
        modbus->holdingRegs[address + i] = regValue; // Ghi vào mảng holdingRegs
    }

    // Sao chéo vào 1 buffer khi có lệnh ghi khẩn cấp từ master
//    if(address == 0){
//    	modbus->emergency_write_from_master = 1;
//    	memcpy(modbus->holdingRegs_emergency_cpy,modbus->holdingRegs, quantity);
//    }
   	modbus->emergency_write_from_master = 1;
   	modbus->holdingRegs_emergency_cpy[0] = modbus->holdingRegs[30];
   	modbus->holdingRegs_emergency_cpy[1] = modbus->holdingRegs[31];

    // ** Chỉ gửi phản hồi nếu KHÔNG phải broadcast **
   if (!is_broadcast) {
	   // 8. Chuẩn bị phản hồi: Addr(1) FC(1) StartAddr(2) Quantity(2) = 6 bytes
	   if (6 > (MODBUS_TX_BUFFER_SIZE - 2)) {
		   Modbus_SendExceptionResponse(modbus, WRITE_MULTI_REGS, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE); return;
	   }
	    modbus->txBuffer[0] = modbus->rxBuffer[0];   // Slave Address
	    modbus->txBuffer[1] = WRITE_MULTI_REGS;      // Function Code
	    Modbus_WriteU16_BE(modbus->txBuffer, 2, address);   // Start Address
	    Modbus_WriteU16_BE(modbus->txBuffer, 4, quantity);  // Quantity
        // 9. Gửi phản hồi ID(1) + FC(1) + StartAddrHi(1) + StartAddrLo(1) + QtyHi(1) + QtyLo(1) + CRCLo(1) + CRCHi(1)
        // Hàm Modbus_SendResponse sẽ tự thêm CRC
	    Modbus_SendResponse(modbus, 6);
   } else {
	   modbus->state = MODBUS_STATE_IDLE;
   }
}


/* Public Functions --------------------------------------------------------*/

/**
 * @brief Khởi tạo module Modbus Slave.
 */
HAL_StatusTypeDef Modbus_Init(ModbusHandle* modbus, UART_HandleTypeDef* huart, IRQn_Type uart_irqn)
{
    // Kiểm tra tham số đầu vào cơ bản
    if (modbus == NULL || huart == NULL) {
        // Lỗi nghiêm trọng, không thể khởi tạo.
        // Có thể assert(0) hoặc xử lý lỗi khác.
    	return HAL_ERROR; // Tham số không hợp lệ
    }

    // Lưu con trỏ UART handle
    modbus->huart = huart;
    modbus->uart_irqn = uart_irqn;
    // Đặt trạng thái ban đầu là sẵn sàng
    modbus->state = MODBUS_STATE_IDLE;
    // Reset bộ đếm byte nhận
    modbus->rxCount = 0;

#if MODBUS_PROCESS_IN_MAIN_LOOP == 1
    modbus->frame_ready_for_processing = false; // Khởi tạo cờ báo
#endif

    // Khởi tạo vùng nhớ dữ liệu (dùng critical section "mini" bằng ngắt toàn cục nếu cần)
    // Hoặc tốt hơn là đảm bảo Init không bị ngắt UART gọi vào
    __disable_irq(); // Tạm thời vô hiệu hóa tất cả ngắt để đảm bảo an toàn khi init
    memset(modbus->coils, 0, sizeof(modbus->coils));
    memset(modbus->discreteInputs, 0, sizeof(modbus->discreteInputs));
    memset(modbus->holdingRegs, 0, sizeof(modbus->holdingRegs));
    memset(modbus->inputRegs, 0, sizeof(modbus->inputRegs));
    memset(modbus->holdingRegs_emergency_cpy, 0, sizeof(modbus->holdingRegs_emergency_cpy));
    __enable_irq(); // Kích hoạt lại ngắt

    memset(modbus->rxBuffer, 0, MODBUS_RX_BUFFER_SIZE);
    memset(modbus->txBuffer, 0, MODBUS_TX_BUFFER_SIZE);

    //Reset cờ báo trường hợp đặc biệt(có lệnh ghi khẩn cấp từ master)
    modbus->emergency_write_from_master = 0;


    // Bắt đầu nhận dữ liệu UART bằng DMA với chế độ Idle Line detection
    // Chế độ này sẽ kích hoạt callback HAL_UARTEx_RxEventCallback khi không có
    // dữ liệu mới đến trong một khoảng thời gian nhất định (thường là 1 frame time),
    // báo hiệu kết thúc một gói tin Modbus RTU.
    // Bắt đầu nhận DMA với Idle Line detection
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(modbus->huart, modbus->rxBuffer, MODBUS_RX_BUFFER_SIZE);
    // Chỉ kích hoạt ngắt NVIC nếu critical section được BẬT VÀ DMA bắt đầu OK
    // Chỉ kích hoạt ngắt NVIC nếu một IRQ hợp lệ được cung cấp VÀ DMA bắt đầu OK
#if MODBUS_USE_CRITICAL_SECTION == 1
    if (status == HAL_OK && modbus->uart_irqn != MODBUS_IRQN_NONE) {
        HAL_NVIC_EnableIRQ(modbus->uart_irqn);
    }
#endif
    if (status != HAL_OK) {
        // Lỗi khi bắt đầu DMA Receive
        modbus->state = MODBUS_STATE_IDLE; // Vẫn ở IDLE nhưng có lỗi
    }
    return status; // Trả về kết quả của việc khởi tạo DMA
}

/**
 * @brief Khởi tạo lại sau lỗi module Modbus Slave.
 */
HAL_StatusTypeDef Modbus_ReInit(ModbusHandle* modbus, UART_HandleTypeDef* huart, IRQn_Type uart_irqn)
{
    // Kiểm tra tham số đầu vào cơ bản
    if (modbus == NULL || huart == NULL) {
        // Lỗi nghiêm trọng, không thể khởi tạo.
        // Có thể assert(0) hoặc xử lý lỗi khác.
    	return HAL_ERROR; // Tham số không hợp lệ
    }

    // Lưu con trỏ UART handle
    modbus->huart = huart;
    modbus->uart_irqn = uart_irqn;
    // Đặt trạng thái ban đầu là sẵn sàng
    modbus->state = MODBUS_STATE_IDLE;
    // Reset bộ đếm byte nhận
    modbus->rxCount = 0;

#if MODBUS_PROCESS_IN_MAIN_LOOP == 1
    modbus->frame_ready_for_processing = false; // Khởi tạo cờ báo
#endif

    modbus->emergency_write_from_master = 0;
    memset(modbus->rxBuffer, 0, MODBUS_RX_BUFFER_SIZE);
    memset(modbus->txBuffer, 0, MODBUS_TX_BUFFER_SIZE);

    //Reset cờ báo trường hợp đặc biệt(có lệnh ghi khẩn cấp từ master)
    modbus->emergency_write_from_master = 0;

    // Bắt đầu nhận dữ liệu UART bằng DMA với chế độ Idle Line detection
    // Chế độ này sẽ kích hoạt callback HAL_UARTEx_RxEventCallback khi không có
    // dữ liệu mới đến trong một khoảng thời gian nhất định (thường là 1 frame time),
    // báo hiệu kết thúc một gói tin Modbus RTU.
    // Bắt đầu nhận DMA với Idle Line detection
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(modbus->huart, modbus->rxBuffer, MODBUS_RX_BUFFER_SIZE);
    // Chỉ kích hoạt ngắt NVIC nếu critical section được BẬT VÀ DMA bắt đầu OK
    // Chỉ kích hoạt ngắt NVIC nếu một IRQ hợp lệ được cung cấp VÀ DMA bắt đầu OK
#if MODBUS_USE_CRITICAL_SECTION == 1
    if (status == HAL_OK && modbus->uart_irqn != MODBUS_IRQN_NONE) {
        HAL_NVIC_EnableIRQ(modbus->uart_irqn);
    }
#endif
    if (status != HAL_OK) {
        // Lỗi khi bắt đầu DMA Receive
        modbus->state = MODBUS_STATE_IDLE; // Vẫn ở IDLE nhưng có lỗi
    }
    return status; // Trả về kết quả của việc khởi tạo DMA
}



#if MODBUS_PROCESS_IN_MAIN_LOOP == 1
/**
 * @brief Hàm thăm dò (polling) để xử lý dữ liệu Modbus khi được cấu hình xử lý trong main loop.
 */
void Modbus_Poll(ModbusHandle* modbus)
{
    // Chỉ thực hiện khi có cờ báo và trạng thái đang là PROCESSING (chờ xử lý)
    if (modbus->frame_ready_for_processing && modbus->state == MODBUS_STATE_PROCESSING)
    {
        // Xóa cờ báo ngay để cho phép nhận frame mới (nếu đến nhanh)
        // Tránh trường hợp gọi Modbus_ProcessData nhiều lần cho cùng 1 frame
        modbus->frame_ready_for_processing = false;

        // Gọi hàm xử lý chính
        Modbus_ProcessData(modbus);
        // Modbus_ProcessData sẽ chuyển state sang TRANSMITTING hoặc IDLE sau khi xử lý xong
    }
}
#endif


/**
 * @brief Xử lý gói tin Modbus đã nhận được trong rxBuffer.
 */
void Modbus_ProcessData(ModbusHandle* modbus) {
    // Kiểm tra trạng thái đầu vào tùy theo chế độ
#if MODBUS_PROCESS_IN_MAIN_LOOP == 0
    // Chế độ Callback: Phải bắt đầu từ IDLE
    if (modbus->state != MODBUS_STATE_IDLE) {
        return; // Đang bận xử lý hoặc lỗi logic
    }
#else
    // Chế độ Main Loop: Phải bắt đầu từ PROCESSING
    if (modbus->state != MODBUS_STATE_PROCESSING) {
         // Nếu state không phải PROCESSING mà cờ lại bật -> Lỗi logic, reset cờ
        if(modbus->frame_ready_for_processing) {
            modbus->frame_ready_for_processing = false;
        }
        return; // Không ở trạng thái chờ xử lý
    }
    // State đã là PROCESSING, không cần set lại
#endif

    // 1. Kiểm tra độ dài tối thiểu của frame Modbus RTU
    // Tối thiểu phải có: SlaveAddr(1) + FC(1) + CRC(2) = 4 bytes
    if (modbus->rxCount < MODBUS_MIN_FRAME_SIZE) {
        // Frame quá ngắn -> không phải Modbus hợp lệ -> Bỏ qua.
    	modbus->state = MODBUS_STATE_IDLE;
        return;
    }

    // 2. Kiểm tra địa chỉ Slave Address (byte đầu tiên)
    if (modbus->rxBuffer[0] != SLAVE_ADDRESS && modbus->rxBuffer[0] != 0) { // Địa chỉ 0 là broadcast
        // Không phải gói tin cho Slave này (và không phải broadcast) -> Bỏ qua.
    	modbus->state = MODBUS_STATE_IDLE;
        return;
    }


    // Nếu là broadcast (địa chỉ 0), Slave KHÔNG được gửi phản hồi.
    // Cần thêm logic để xử lý broadcast nếu muốn hỗ trợ ghi broadcast (FC 05, 06, 0F, 10).
    bool is_broadcast = (modbus->rxBuffer[0] == 0);


    // 3. Kiểm tra CRC16
    // Lấy CRC nhận được từ 2 byte cuối của rxBuffer (Little Endian trên dây)
    uint16_t receivedCrc = ((uint16_t)modbus->rxBuffer[modbus->rxCount - 1] << 8) |
                            (uint16_t)modbus->rxBuffer[modbus->rxCount - 2];
    // Tính CRC cho dữ liệu nhận được (trừ 2 byte CRC cuối)
    uint16_t calculatedCrc = Modbus_CRC16_Table(modbus->rxBuffer, modbus->rxCount - 2);

    // So sánh CRC (cả hai đều ở dạng Little Endian trong thanh ghi)
    if (receivedCrc != calculatedCrc) {
        // Lỗi CRC -> Bỏ qua gói tin.
        // Có thể đếm số lần lỗi CRC để chẩn đoán.
    	modbus->state = MODBUS_STATE_IDLE;
        return;
    }

    // --- Nếu đến đây: Địa chỉ đúng (hoặc broadcast), CRC đúng ---

    // Đặt trạng thái là đang xử lý để ngăn việc xử lý frame khác chồng chéo
    modbus->state = MODBUS_STATE_PROCESSING;

    // 4. Phân tích Function Code và các tham số chung
    uint8_t functionCode = modbus->rxBuffer[1];
    uint16_t address = 0;           // Thường là địa chỉ bắt đầu
    uint16_t quantity_or_value = 0; // Thường là số lượng hoặc giá trị ghi

    // Hầu hết các function code phổ biến (01-06, 0F, 10) đều có ít nhất 4 byte tham số sau FC.
    // Tổng frame tối thiểu cho các FC này là 1+1+4+2 = 8 bytes.
    if (modbus->rxCount >= MODBUS_MIN_REQ_RESP_SIZE) {
        address           = Modbus_ReadU16_BE(modbus->rxBuffer, 2); // Đọc địa chỉ (index 2, 3)
        quantity_or_value = Modbus_ReadU16_BE(modbus->rxBuffer, 4); // Đọc số lượng/giá trị (index 4, 5)
    } else {
        // Nếu frame ngắn hơn 8 bytes nhưng FC nằm trong nhóm phổ biến -> Lỗi dữ liệu
        if ((functionCode >= READ_COILS && functionCode <= WRITE_SINGLE_REG) ||
            functionCode == WRITE_MULTI_COILS || functionCode == WRITE_MULTI_REGS)
        {
            Modbus_SendExceptionResponse(modbus, functionCode, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
            // SendExceptionResponse sẽ tự xử lý state nếu không phải broadcast
            if(is_broadcast) modbus->state = MODBUS_STATE_IDLE;
            return; // Kết thúc xử lý
        }
        // Các FC khác (nếu hỗ trợ) có thể có cấu trúc ngắn hơn, không cần xử lý ở đây.
    }

    // 5. Gọi handler tương ứng
      switch (functionCode) {
          // --- Read Functions (Luôn gửi phản hồi nếu request hợp lệ) ---
          case READ_COILS:
              if(!is_broadcast) Modbus_HandleReadCoils(modbus, address, quantity_or_value);
              else modbus->state = MODBUS_STATE_IDLE; // Read broadcast không hợp lệ
              break;
          case READ_DISCRETE:
               if(!is_broadcast) Modbus_HandleReadDiscrete(modbus, address, quantity_or_value);
               else modbus->state = MODBUS_STATE_IDLE;
               break;
          case READ_HOLDING:
              if(!is_broadcast) Modbus_HandleReadHolding(modbus, address, quantity_or_value);
              else modbus->state = MODBUS_STATE_IDLE;
              break;
          case READ_INPUT:
              if(!is_broadcast) Modbus_HandleReadInput(modbus, address, quantity_or_value);
              else modbus->state = MODBUS_STATE_IDLE;
              break;

          // --- Write Functions (Xử lý broadcast bên trong handler) ---
          case WRITE_SINGLE_COIL:
              Modbus_HandleWriteSingleCoil(modbus, address, quantity_or_value, is_broadcast);
              break;
          case WRITE_SINGLE_REG:
              Modbus_HandleWriteSingleReg(modbus, address, quantity_or_value, is_broadcast);
              break;
          case WRITE_MULTI_COILS:
              Modbus_HandleWriteMultipleCoils(modbus, address, quantity_or_value, is_broadcast);
              break;
          case WRITE_MULTI_REGS:
              Modbus_HandleWriteMultipleRegs(modbus, address, quantity_or_value, is_broadcast);
              break;
          default:
              // Function code không được hỗ trợ
              Modbus_SendExceptionResponse(modbus, functionCode, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
              if(is_broadcast) modbus->state = MODBUS_STATE_IDLE; // Set về IDLE nếu là broadcast lỗi FC
              break;
      }
    // Lưu ý quan trọng:
    // - Nếu hàm handler gọi Modbus_SendResponse hoặc Modbus_SendExceptionResponse,
    //   trạng thái sẽ được chuyển thành MODBUS_STATE_TRANSMITTING.
    // - Nếu hàm handler không gửi phản hồi (ví dụ: lỗi nội bộ không gửi được, hoặc xử lý broadcast),
    //   trạng thái CÓ THỂ vẫn là MODBUS_STATE_PROCESSING. Cần đảm bảo các handler
    //   hoặc phần gọi handler phải đưa state về IDLE hoặc TRANSMITTING.
    //   Trong thiết kế hiện tại, tất cả các nhánh đều dẫn đến SendResponse/SendException hoặc
    //   set state về IDLE (cho broadcast không hợp lệ), nên state sẽ được quản lý đúng.
}

/**
 * @brief Callback khi UART nhận xong dữ liệu (sử dụng Idle Line + DMA).
 */
void Modbus_UartRxCpltCallback(ModbusHandle* modbus, uint16_t Size) {
    // 1. Cập nhật số byte đã nhận
    modbus->rxCount = Size;

    // 2. Chỉ xử lý nếu đang ở trạng thái IDLE
    if (modbus->state == MODBUS_STATE_IDLE) {
        if (Size > 0) { // Chỉ xử lý nếu thực sự có dữ liệu
#if MODBUS_PROCESS_IN_MAIN_LOOP == 0
            // Chế độ Callback: Gọi xử lý ngay
            Modbus_ProcessData(modbus);
#else
            // Chế độ Main Loop: Đặt cờ và chuyển state chờ xử lý
            modbus->state = MODBUS_STATE_PROCESSING; // Giữ buffer, chờ main loop
            modbus->frame_ready_for_processing = true;   // Báo cho Modbus_Poll
#endif

        }
        // Modbus_ProcessData sẽ thay đổi state thành PROCESSING hoặc TRANSMITTING nếu hợp lệ,
        // hoặc giữ nguyên IDLE nếu gói tin bị bỏ qua.
    } else {
        // Đang bận (PROCESSING hoặc TRANSMITTING), bỏ qua frame này.
        // Việc này là bình thường nếu Master gửi liên tục mà Slave chưa xử lý xong.
        // Có thể đếm số lần bỏ qua để theo dõi.
    	 modbus->rxCount = 0; // Reset count để tránh xử lý dữ liệu cũ/không hoàn chỉnh
    }

    // 3. **QUAN TRỌNG:** Khởi động lại việc nhận DMA cho frame tiếp theo NGAY LẬP TỨC.
    // Dù frame vừa nhận có hợp lệ hay không, dù Slave có đang bận hay không,
    // vẫn phải luôn sẵn sàng để nhận frame kế tiếp.
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(modbus->huart, modbus->rxBuffer, MODBUS_RX_BUFFER_SIZE);
    if (status != HAL_OK) {
        // Lỗi nghiêm trọng: Không thể khởi động lại DMA Receive!
        // Cần có cơ chế phục hồi lỗi ở đây (vd: reset Modbus, báo lỗi hệ thống).
        // Có thể set state về IDLE để thử lại ở lần sau? Hoặc một trạng thái lỗi riêng.
        modbus->state = MODBUS_STATE_IDLE; // Tạm thời quay về IDLE
        // Nên có log hoặc cờ báo lỗi.
#if MODBUS_PROCESS_IN_MAIN_LOOP == 1
        modbus->frame_ready_for_processing = false; // Reset cờ
#endif
    }
}

/**
 * @brief Callback khi UART truyền xong dữ liệu bằng DMA.
 */
void Modbus_UartTxCpltCallback(ModbusHandle* modbus) {
    // Nếu trạng thái đang là TRANSMITTING (đang gửi phản hồi),
    if (modbus->state == MODBUS_STATE_TRANSMITTING) {
        modbus->state = MODBUS_STATE_IDLE;
    }
}
void Modbus_HAL_ErrorCallback(ModbusHandle* modbus, UART_HandleTypeDef* huart) {
	    if (huart == modbus->huart){
		// Quan trọng: Phải xóa các cờ lỗi trong thanh ghi trạng thái UART
		// Để ngăn chặn ngắt lỗi lặp lại hoặc trạng thái treo.
		__HAL_UART_CLEAR_PEFLAG(huart);   // Parity Error
		__HAL_UART_CLEAR_FEFLAG(huart);   // Framing Error
		__HAL_UART_CLEAR_NEFLAG(huart);   // Noise Error
		__HAL_UART_CLEAR_OREFLAG(huart);  // Overrun Error
		__HAL_UART_CLEAR_IDLEFLAG(huart); // Cân nhắc nếu gặp vấn đề với Idle Line sau lỗi

		// Đặt lại trạng thái Modbus về IDLE và reset bộ đếm
		modbus->state = MODBUS_STATE_IDLE;

#if MODBUS_PROCESS_IN_MAIN_LOOP == 1
        // Reset cờ báo nếu có lỗi
        modbus->frame_ready_for_processing = false;
#endif

		modbus->rxCount = 0;
        // Cố gắng hủy bỏ thao tác nhận hiện tại (nếu có) và khởi động lại DMA nhận
        // HAL_UART_AbortReceive() có thể cần thiết tùy thuộc vào trạng thái lỗi
        HAL_UART_AbortReceive(huart); // Thử dừng nhận hiện tại
        HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(modbus->huart, modbus->rxBuffer, MODBUS_RX_BUFFER_SIZE);
        // Kích hoạt lại ngắt NVIC chỉ khi cần và có thể
#if MODBUS_USE_CRITICAL_SECTION == 1
        if (status == HAL_OK && modbus->uart_irqn != MODBUS_IRQN_NONE) {
             HAL_NVIC_EnableIRQ(modbus->uart_irqn);
        }
#endif
	  }
}



/**
 * @brief Callback khi UART truyền xong, nhận xong hoặc có lỗ bằng DMA.
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == modbus_slave.huart)
    {
        Modbus_UartRxCpltCallback(&modbus_slave, Size);
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == modbus_slave.huart)
    {
    	Modbus_UartTxCpltCallback(&modbus_slave);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	Modbus_HAL_ErrorCallback(&modbus_slave, &huart1);
}
*/

