/*
 * Modbus_Slave_Final.h
 *
 *  Created on: May 22, 2025
 *      Author: PC
 */

#ifndef INC_MODBUS_SLAVE_FINAL_H_
#define INC_MODBUS_SLAVE_FINAL_H_
/* Configuration Defines */
/** @defgroup Modbus_Config Cấu hình Modbus - Các giá trị người dùng cần tùy chỉnh */
/** @{ */

/**
 * @brief Lựa chọn nơi xử lý khung tin Modbus nhận được.
 *        - Đặt là 0: Xử lý trực tiếp trong Modbus_UartRxCpltCallback (ngữ cảnh callback/ngắt).
 *                     Ưu điểm: Phản hồi nhanh hơn.
 *                     Nhược điểm: Có thể chiếm thời gian xử lý trong ngắt, ảnh hưởng đến các tác vụ thời gian thực khác.
 *        - Đặt là 1: Xử lý trong hàm Modbus_Poll() được gọi từ vòng lặp chính.
 *                     Ưu điểm: Giảm tải xử lý trong ngắt, phù hợp nếu việc xử lý Modbus phức tạp hoặc có nhiều tác vụ khác.
 *                     Nhược điểm: Độ trễ phản hồi phụ thuộc vào tần suất gọi Modbus_Poll().
 */
#define MODBUS_PROCESS_IN_MAIN_LOOP   0 // <-- CHỌN 0 hoặc 1 TẠI ĐÂY
/**
 * @brief Bật/Tắt cơ chế Critical Section tự động của thư viện Modbus.
 *        - Đặt là 1: BẬT. Các hàm Modbus_Enter/ExitCriticalSection sẽ
 *          disable/enable ngắt UART tương ứng (nếu IRQn hợp lệ được cung cấp).
 *          Yêu cầu cung cấp IRQn hợp lệ trong Modbus_Init.
 *        - Đặt là 0: TẮT. Các hàm Modbus_Enter/ExitCriticalSection sẽ được
 *          biên dịch thành không làm gì cả (no-op) -> Không có chi phí thực thi.
 *          Người dùng tự quản lý critical section nếu cần. Không cần cung cấp IRQn.
 */
#define MODBUS_USE_CRITICAL_SECTION 1 // <-- Đặt là 1 hoặc 0 TẠI ĐÂY
/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h" // Thay stm32f3xx bằng dòng chip STM32 bạn đang dùng (vd: stm32f1xx, stm32f4xx)
#include <stdint.h>        // Định nghĩa các kiểu số nguyên chuẩn (uint8_t, uint16_t, ...)
#include <stddef.h>        // Định nghĩa kiểu size_t và NULL
#include <string.h>        // Cung cấp các hàm xử lý chuỗi và bộ nhớ (memset, memcpy)
#include <stdbool.h>       // Định nghĩa kiểu boolean (true, false)

/* Configuration Defines ---------------------------------------------------*/
/** @defgroup Modbus_Config Cấu hình Modbus - Các giá trị người dùng cần tùy chỉnh */
/** @{ */
#define SLAVE_ADDRESS         0x04  /*!< Địa chỉ (ID) của thiết bị Slave này trên mạng Modbus. */
#define MODBUS_RX_BUFFER_SIZE 256   /*!< Kích thước bộ đệm nhận dữ liệu Modbus (byte). Nên đủ lớn cho frame dài nhất (FC 0F/10). */
#define MODBUS_TX_BUFFER_SIZE 256   /*!< Kích thước bộ đệm truyền dữ liệu Modbus (byte). Nên đủ lớn cho phản hồi dài nhất (FC 01/02/03/04). */

// Kích thước các vùng nhớ dữ liệu Modbus của Slave
// Lưu ý: Địa chỉ Modbus bắt đầu từ 1, nhưng trong mảng C bắt đầu từ 0.
// Ví dụ: Coil 1 tương ứng với modbus->coils bit 0, Holding Register 40001 tương ứng với modbus->holdingRegs[0].
#define MAX_COILS             128   /*!< Số lượng Coils tối đa (00001 - 00128). Kích thước mảng coils sẽ là MAX_COILS/8. */
#define MAX_DISCRETE          128   /*!< Số lượng Discrete Inputs tối đa (10001 - 10128). Kích thước mảng discreteInputs sẽ là MAX_DISCRETE/8. */
#define MAX_HOLDING_REGS      100   /*!< Số lượng Holding Registers tối đa (40001 - 40100). Kích thước mảng holdingRegs sẽ là MAX_HOLDING_REGS. */
#define MAX_INPUT_REGS        100   /*!< Số lượng Input Registers tối đa (30001 - 30100). Kích thước mảng inputRegs sẽ là MAX_INPUT_REGS. */
/** @} */ // End of Modbus_Config

/* Modbus Constants --------------------------------------------------------*/
/** @defgroup Modbus_Function_Codes Mã chức năng Modbus (Function Codes) */
/** @{ */
#define READ_COILS            0x01  /*!< Đọc trạng thái nhiều Coils (Output Bits). */
#define READ_DISCRETE         0x02  /*!< Đọc trạng thái nhiều Discrete Inputs (Input Bits). */
#define READ_HOLDING          0x03  /*!< Đọc giá trị nhiều Holding Registers (Output Registers). */
#define READ_INPUT            0x04  /*!< Đọc giá trị nhiều Input Registers (Input Registers). */
#define WRITE_SINGLE_COIL     0x05  /*!< Ghi trạng thái một Coil. */
#define WRITE_SINGLE_REG      0x06  /*!< Ghi giá trị một Holding Register. */
#define WRITE_MULTI_COILS     0x0F  /*!< Ghi trạng thái nhiều Coils. */
#define WRITE_MULTI_REGS      0x10  /*!< Ghi giá trị nhiều Holding Registers. */
/** @} */

/** @defgroup Modbus_Exception_Codes Mã ngoại lệ Modbus (Exception Codes) */
/** @{ */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION     0x01 /*!< Mã chức năng không hợp lệ hoặc không được Slave hỗ trợ. */
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS      0x02 /*!< Địa chỉ dữ liệu yêu cầu không hợp lệ (ngoài phạm vi cho phép). */
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE   0x03 /*!< Giá trị dữ liệu trong yêu cầu không hợp lệ (vd: số lượng quá lớn, giá trị ghi không hợp lệ). */
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE 0x04 /*!< Lỗi không thể phục hồi xảy ra trong Slave khi xử lý yêu cầu. */
// Các mã lỗi khác ít phổ biến hơn có thể được thêm vào nếu cần.
/** @} */ // End of Modbus_Constants

/* Type Definitions --------------------------------------------------------*/
/**
 * @brief Enum định nghĩa các trạng thái hoạt động của Modbus Slave.
 *        Giúp quản lý việc xử lý yêu cầu và tránh xung đột.
 */
typedef enum {
    MODBUS_STATE_IDLE,          /*!< Trạng thái nghỉ, sẵn sàng nhận yêu cầu mới. */
    MODBUS_STATE_RECEIVING,     /*!< (Có thể dùng nếu không dùng Idle Line) Đang trong quá trình nhận frame. */
    MODBUS_STATE_PROCESSING,    /*!< Đã nhận xong frame, đang kiểm tra và xử lý yêu cầu. */
    MODBUS_STATE_TRANSMITTING   /*!< Đang truyền gói tin phản hồi đi. */
} ModbusState;
/**
 * @brief Giá trị đặc biệt cho IRQn_Type để chỉ báo không sử dụng hoặc không cung cấp IRQn.
 *        Sử dụng giá trị này khi gọi Modbus_Init nếu MODBUS_USE_CRITICAL_SECTION = 0,
 *        hoặc nếu = 1 nhưng bạn muốn tạm thời bỏ qua việc quản lý ngắt bởi thư viện.
 */
#define MODBUS_IRQN_NONE ((IRQn_Type)-128) // Giá trị âm không dùng cho IRQ hợp lệ
/**
 * @brief Cấu trúc chính quản lý toàn bộ trạng thái và dữ liệu của Modbus Slave.
 *        Mỗi instance UART Modbus sẽ cần một biến thuộc kiểu này.
 */
typedef struct {
    /* --- Phần cứng và Trạng thái --- */
    UART_HandleTypeDef* huart;      /*!< Con trỏ tới handle UART HAL được sử dụng cho Modbus. */

    IRQn_Type           uart_irqn; /*!< Số hiệu ngắt UART */

#if MODBUS_PROCESS_IN_MAIN_LOOP == 1
    /**
     * @brief Cờ báo hiệu có frame Modbus chờ xử lý bởi Modbus_Poll().
     *        Chỉ được sử dụng khi MODBUS_PROCESS_IN_MAIN_LOOP = 1.
     *        volatile vì được ghi trong callback và đọc trong main loop.
     */
    volatile bool       frame_ready_for_processing;
#endif

    volatile ModbusState state;     /*!< Trạng thái hoạt động hiện tại (volatile vì có thể thay đổi bởi interrupt/DMA). */

    /* --- Bộ đệm Nhận (Receive) --- */
    uint8_t             rxBuffer[MODBUS_RX_BUFFER_SIZE]; /*!< Bộ đệm lưu trữ frame Modbus nhận được từ Master. */
    volatile uint16_t   rxCount;    /*!< Số byte đã nhận được trong `rxBuffer` (volatile vì được cập nhật bởi DMA/Interrupt). */

    /* --- Bộ đệm Truyền (Transmit) --- */
    uint8_t             txBuffer[MODBUS_TX_BUFFER_SIZE]; /*!< Bộ đệm dùng để xây dựng frame phản hồi gửi cho Master. */
    // Không cần txCount vì độ dài được truyền trực tiếp vào hàm HAL_UART_Transmit_DMA.

    /* --- Modbus Data Maps (Slave Memory) --- */
      // !! CẢNH BÁO CONCURRENCY !!
      // Nếu ứng dụng chính (ví dụ: vòng lặp while(1), các task RTOS khác)
      // cần đọc/ghi trực tiếp vào các mảng dữ liệu này, hãy đảm bảo
      // có cơ chế bảo vệ để tránh tranh chấp dữ liệu (race condition)
      // với các hàm xử lý Modbus (có thể chạy ngầm trong ngữ cảnh ngắt/callback).
      // Các cơ chế có thể dùng:
      // - Critical Sections: Tạm thời vô hiệu hóa ngắt (ví dụ: __disable_irq(), __enable_irq())
      //   khi ứng dụng chính truy cập. Chỉ dùng cho các thao tác rất nhanh.
      // - Mutex/Semaphore (nếu dùng RTOS): Bảo vệ truy cập vào toàn bộ cấu trúc dữ liệu
      //   hoặc các vùng nhớ cụ thể.
      // Input Registers và Discrete Inputs thường được cập nhật bởi ứng dụng chính.
      // Holding Registers và Coils thường được cập nhật bởi cả ứng dụng và Modbus Master.

    /* --- Vùng nhớ dữ liệu Modbus của Slave --- */
    // Kích thước mảng được tính toán để chứa đủ số lượng bit/register đã định nghĩa.
    // Sử dụng MASK để đảm bảo đủ byte cho số lượng bit lẻ.
    #define COIL_BUFFER_SIZE      (MAX_COILS / 8 + ((MAX_COILS % 8) ? 1 : 0))
    #define DISC_BUFFER_SIZE      (MAX_DISCRETE / 8 + ((MAX_DISCRETE % 8) ? 1 : 0))

    uint8_t             coils[COIL_BUFFER_SIZE];          /*!< Mảng lưu trạng thái Coils (1 bit/coil). */
    uint8_t             discreteInputs[DISC_BUFFER_SIZE]; /*!< Mảng lưu trạng thái Discrete Inputs (1 bit/input). */
    uint16_t            holdingRegs[MAX_HOLDING_REGS];    /*!< Mảng lưu giá trị Holding Registers (16-bit/register). */
    uint16_t            inputRegs[MAX_INPUT_REGS];        /*!< Mảng lưu giá trị Input Registers (16-bit/register). */

/*-----Trường hợp đặc biệt(có lệnh ghi khẩn cấp từ master)-----*/
    volatile uint8_t emergency_write_from_master;
    uint16_t holdingRegs_emergency_cpy[MAX_HOLDING_REGS];
} ModbusHandle;

/* Public Function Prototypes ----------------------------------------------*/
/** @defgroup Modbus_Public_Functions Các hàm công khai của thư viện Modbus Slave */
/**
 * @brief Khởi tạo module Modbus Slave.
 * @param modbus Con trỏ tới cấu trúc ModbusHandle.
 * @param huart Con trỏ tới UART_HandleTypeDef đã cấu hình.
 * @param uart_irqn Số hiệu ngắt (IRQ Number) của UART (ví dụ: USART2_IRQn).
 *                  - BẮT BUỘC cung cấp IRQn hợp lệ nếu MODBUS_USE_CRITICAL_SECTION = 1.
 *                  - Nên truyền MODBUS_IRQN_NONE nếu MODBUS_USE_CRITICAL_SECTION = 0.
 * @return HAL_StatusTypeDef: HAL_OK nếu khởi tạo thành công.
 */
HAL_StatusTypeDef Modbus_Init(ModbusHandle* modbus, UART_HandleTypeDef* huart, IRQn_Type uart_irqn);
/* Public Function Prototypes ----------------------------------------------*/
/** @defgroup Modbus_Public_Functions Các hàm công khai của thư viện Modbus Slave */
/**
 * @brief Khởi tạo module Modbus Slave.
 * @param modbus Con trỏ tới cấu trúc ModbusHandle.
 * @param huart Con trỏ tới UART_HandleTypeDef đã cấu hình.
 * @param uart_irqn Số hiệu ngắt (IRQ Number) của UART (ví dụ: USART2_IRQn).
 *                  - BẮT BUỘC cung cấp IRQn hợp lệ nếu MODBUS_USE_CRITICAL_SECTION = 1.
 *                  - Nên truyền MODBUS_IRQN_NONE nếu MODBUS_USE_CRITICAL_SECTION = 0.
 * @return HAL_StatusTypeDef: HAL_OK nếu khởi tạo thành công.
 */
HAL_StatusTypeDef Modbus_ReInit(ModbusHandle* modbus, UART_HandleTypeDef* huart, IRQn_Type uart_irqn);
/**
 * @brief Hàm xử lý chính cho gói tin Modbus đã nhận được.
 * @param modbus Con trỏ tới cấu trúc ModbusHandle chứa dữ liệu trong `rxBuffer` và `rxCount`.
 * @note Hàm này **CHỈ** nên được gọi từ `Modbus_UartRxCpltCallback` sau khi nhận xong frame
 *       và nếu trạng thái Modbus đang là IDLE.
 *       Nó thực hiện các bước:
 *       1. Kiểm tra độ dài tối thiểu.
 *       2. Kiểm tra địa chỉ Slave.
 *       3. Kiểm tra CRC16.
 *       4. Nếu hợp lệ, chuyển trạng thái sang PROCESSING.
 *       5. Phân tích Function Code và các tham số.
 *       6. Gọi hàm xử lý (handler) tương ứng cho Function Code đó.
 *       7. Nếu không hợp lệ hoặc có lỗi, có thể không làm gì hoặc chuẩn bị gửi Exception Response.
 */
#if MODBUS_PROCESS_IN_MAIN_LOOP == 1
/**
 * @brief Hàm thăm dò (polling) để xử lý dữ liệu Modbus.
 * @param modbus Con trỏ tới cấu trúc ModbusHandle.
 * @note Hàm này PHẢI được gọi lặp đi lặp lại từ vòng lặp chính (hoặc task riêng)
 *       khi MODBUS_PROCESS_IN_MAIN_LOOP được đặt là 1.
 *       Nó kiểm tra cờ báo và gọi Modbus_ProcessData nếu có frame chờ xử lý.
 */
void Modbus_Poll(ModbusHandle* modbus);
#endif

void Modbus_ProcessData(ModbusHandle* modbus);

/**
 * @brief Callback xử lý sự kiện UART nhận dữ liệu hoàn tất (sử dụng Idle Line detection).
 * @param modbus Con trỏ tới cấu trúc ModbusHandle tương ứng với UART đã nhận dữ liệu.
 * @param Size Số lượng byte thực sự đã nhận được trong frame vừa kết thúc.
 * @note Hàm này **PHẢI** được gọi từ bên trong callback `HAL_UARTEx_RxEventCallback` của HAL,
 *       sau khi kiểm tra đúng `huart` instance.
 *       Nó thực hiện:
 *       1. Cập nhật `modbus->rxCount` với `Size`.
 *       2. Nếu trạng thái đang là IDLE, gọi `Modbus_ProcessData` để xử lý frame.
 *       3. **QUAN TRỌNG:** Khởi động lại ngay lập tức việc nhận DMA (`HAL_UARTEx_ReceiveToIdle_DMA`)
 *          để sẵn sàng cho frame tiếp theo, bất kể frame hiện tại có hợp lệ hay không.
 */
void Modbus_UartRxCpltCallback(ModbusHandle* modbus, uint16_t Size);

/**
 * @brief Callback xử lý sự kiện UART truyền dữ liệu hoàn tất bằng DMA.
 * @param modbus Con trỏ tới cấu trúc ModbusHandle tương ứng với UART vừa truyền xong.
 * @note Hàm này **PHẢI** được gọi từ bên trong callback `HAL_UART_TxCpltCallback` của HAL,
 *       sau khi kiểm tra đúng `huart` instance.
 *       Nó thực hiện:
 *       1. Nếu trạng thái đang là TRANSMITTING, chuyển trạng thái về IDLE, báo hiệu Modbus đã sẵn sàng
 *          cho yêu cầu tiếp theo.
 */
void Modbus_UartTxCpltCallback(ModbusHandle* modbus);

/**
 * @brief Callback xử lý lỗi UART HAL.
 * @param modbus Con trỏ tới ModbusHandle tương ứng.
 * @param huart Con trỏ tới UART_HandleTypeDef gây lỗi.
 * @note **NÊN** gọi hàm này từ `HAL_UART_ErrorCallback` khi `huart` khớp.
 *       Hàm này cố gắng xóa lỗi và khởi động lại việc nhận DMA.
 */
void Modbus_HAL_ErrorCallback(ModbusHandle* modbus, UART_HandleTypeDef* huart);


/* Inline Critical Section Functions ---------------------------------------*/

/**
 * @brief Bắt đầu Critical Section (nếu được cấu hình).
 *        - Nếu MODBUS_USE_CRITICAL_SECTION = 1 và IRQn hợp lệ: Disable ngắt UART.
 *        - Nếu MODBUS_USE_CRITICAL_SECTION = 0 hoặc IRQn không hợp lệ: Không làm gì.
 * @param modbus Con trỏ tới ModbusHandle.
 */
static inline void Modbus_EnterCriticalSection(ModbusHandle* modbus) {
#if MODBUS_USE_CRITICAL_SECTION == 1
    // Chỉ thực hiện nếu con trỏ hợp lệ và IRQn đã được cung cấp (khác NONE)
    if (modbus != NULL && modbus->uart_irqn != MODBUS_IRQN_NONE) {
        HAL_NVIC_DisableIRQ(modbus->uart_irqn);
    }
#else
    // Nếu không dùng critical section, hàm này không làm gì cả.
    // (void)modbus; để tránh cảnh báo 'unused parameter' khi biên dịch.
    (void)modbus;
#endif
}

/**
 * @brief Kết thúc Critical Section (nếu được cấu hình).
 *        - Nếu MODBUS_USE_CRITICAL_SECTION = 1 và IRQn hợp lệ: Enable lại ngắt UART.
 *        - Nếu MODBUS_USE_CRITICAL_SECTION = 0 hoặc IRQn không hợp lệ: Không làm gì.
 * @param modbus Con trỏ tới ModbusHandle.
 */
static inline void Modbus_ExitCriticalSection(ModbusHandle* modbus) {
#if MODBUS_USE_CRITICAL_SECTION == 1
    // Chỉ thực hiện nếu con trỏ hợp lệ và IRQn đã được cung cấp (khác NONE)
    if (modbus != NULL && modbus->uart_irqn != MODBUS_IRQN_NONE) {
        HAL_NVIC_EnableIRQ(modbus->uart_irqn);
    }
#else
    // Nếu không dùng critical section, hàm này không làm gì cả.
    (void)modbus;
#endif
}
#endif /* INC_MODBUS_SLAVE_FINAL_H_ */
