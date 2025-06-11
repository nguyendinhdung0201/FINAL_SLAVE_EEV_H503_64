/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdarg.h"
#include "eeprom_final.h"
#include "Modbus_Slave_Final.h"
#include "Input_parameters.h"
#include "pid_final.h"
#include "stepper_v2.h"
#include "R507_temp_pressure.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef handle_GPDMA1_Channel2;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */
ModbusHandle modbus_slave;
EEPROM_Handle_t hEEPROM_final;
PID_TypeDef pid;
Stepper motor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Biến theo dõi tình trạng hoạt động của van tiết lưu
volatile int16_t step_position = 0;
volatile float percent_step = 0;
volatile uint8_t state_motor_step;
volatile uint32_t last_time_step;


// Biến tính toán PID cho số bước điều khiển van tiết lưu
volatile uint8_t count = 0;
volatile float output_pid = 0.0f;
volatile int16_t stepp_count = 0;
#define BUFFER_SIZE 64
volatile int16_t stepp_buffer[BUFFER_SIZE];  // Mảng lưu trữ dữ liệu
volatile uint8_t buffer_index = 0;           // Chỉ mục cho mảng

/*================================================ Hàm Log dữ liệu hoạt động của chương trình =======================================*/
void printLOGDATA(const char *fmt, ...) {
    char temp[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);

    if (len > 0) {
        // Đảm bảo độ dài không vượt quá giới hạn
        if (len > sizeof(temp)) len = sizeof(temp);
        HAL_UART_Transmit(&huart3, (uint8_t *)temp, len, 60);
    }
}
void I2C1_Reinit(void){
	MX_I2C1_Init();
}
/*================================================ Hàm Log dữ liệu hoạt động của chương trình =======================================*/


/*================================================ Hàm tính toán độ quá nhiệt =======================================*/
volatile float Saturation_temperature;
volatile float delta_temperatute;
void superheat_value(){
	Saturation_temperature = R507_GetTemperature(pressure_sensors.low_pressure_sensor);
	delta_temperatute = temperature_sensors.hoi_ve - Saturation_temperature;
}
/*================================================ Hàm tính toán độ quá nhiệt =======================================*/


/*================================================ Hàm tính toán để quyết định số bước điều khiển van tiết lưu =======================================*/
static int16_t find_max_value() {
	    if (BUFFER_SIZE <= 0) return 0;
	    int16_t max_abs_value = stepp_buffer[0];
	    int16_t abs_max = max_abs_value < 0 ? -max_abs_value : max_abs_value;
	        uint8_t i = 1;
	        for (; i + 3 < BUFFER_SIZE; i += 4) {
	            // Xử lý 4 phần tử mỗi lần
	            int16_t current0 = stepp_buffer[i];
	            int16_t abs0 = current0 < 0 ? -current0 : current0;
	            int update0 = (abs0 > abs_max) || (abs0 == abs_max && current0 > max_abs_value);
	            max_abs_value = update0 ? current0 : max_abs_value;
	            abs_max = update0 ? abs0 : abs_max;

	            int16_t current1 = stepp_buffer[i + 1];
	            int16_t abs1 = current1 < 0 ? -current1 : current1;
	            int update1 = (abs1 > abs_max) || (abs1 == abs_max && current1 > max_abs_value);
	            max_abs_value = update1 ? current1 : max_abs_value;
	            abs_max = update1 ? abs1 : abs_max;

	            int16_t current2 = stepp_buffer[i + 2];
	            int16_t abs2 = current2 < 0 ? -current2 : current2;
	            int update2 = (abs2 > abs_max) || (abs2 == abs_max && current2 > max_abs_value);
	            max_abs_value = update2 ? current2 : max_abs_value;
	            abs_max = update2 ? abs2 : abs_max;

	            int16_t current3 = stepp_buffer[i + 3];
	            int16_t abs3 = current3 < 0 ? -current3 : current3;
	            int update3 = (abs3 > abs_max) || (abs3 == abs_max && current3 > max_abs_value);
	            max_abs_value = update3 ? current3 : max_abs_value;
	            abs_max = update3 ? abs3 : abs_max;
	        }
	        return max_abs_value;
}
void control_stepper(){
	uint32_t current_time = HAL_GetTick();
	float error = fabsf(pid.setpoint - delta_temperatute);
	static const float k = 0.6f;
	static const uint32_t time_min = 500, time_max = 10000;
	uint32_t delay_time = time_min + (uint32_t)((time_max - time_min) / (1.0f + k * error));
	if(((uint32_t)(current_time - last_time_step) >= delay_time) && Stepper_IsMoving(&motor) == 0){
		int16_t output_final = find_max_value();
		Stepper_Move(&motor, output_final);
		volatile int16_t *ptr = stepp_buffer;
		for (int i = 0; i < BUFFER_SIZE; i += 8) {
		    *ptr++ = 0; *ptr++ = 0; *ptr++ = 0; *ptr++ = 0;
		    *ptr++ = 0; *ptr++ = 0; *ptr++ = 0; *ptr++ = 0;
		}
	}
}
/*================================================ Hàm tính toán để quyết định số bước điều khiển van tiết lưu =======================================*/



/*================================================ Hàm tính toán để quyết định thay đổi superheat setpoint =======================================*/
void convert_setpoint(){
    static const struct {
        float pressure_threshold;
        float target_temp_diff;
    } lookup_table[] = {
//        { 23.0f, 5.0f },
//        { 20.0f, 7.0f },
//        { 17.0f, 9.0f },
//        { 15.0f, 11.0f},
//        { 0.0f,  13.0f}
        { 23.0f, 5.0f },
        { 20.0f, 7.0f },
        { 17.0f, 9.0f },
        { 15.0f, 11.0f},
        { 0.0f,  13.0f}
    };
    static float last_setpoint = -1;  // Lưu giá trị cũ
    float new_setpoint = pid.setpoint;
    static uint8_t waiting = 0;
    static uint32_t change_time = 0;
    GPIO_PinState state_pin_relay = HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin);
    static uint8_t last_state = 1;
    static uint8_t is_waiting = 0;
    static uint32_t time_changed = 0;
    // Lấy giá trị áp suất
    float high_pressure = pressure_sensors.high_pressure_sensor;

    if(last_state == 0 && state_pin_relay == 1){
    	time_changed = HAL_GetTick();
    	is_waiting = 1;
    }
    last_state = state_pin_relay;

     // Duyệt bảng để tìm giá trị TARGET_TEMP_DIFF phù hợp
    if(state_pin_relay == 1){
    	 uint32_t current_time = HAL_GetTick();
    	 if(!is_waiting || ((uint32_t)(current_time - time_changed) >= 16000)){
             is_waiting = 0;
			 for (int i = 0; i < sizeof(lookup_table) / sizeof(lookup_table[0]); i++) {
				 if (high_pressure >= lookup_table[i].pressure_threshold) {
						 new_setpoint = lookup_table[i].target_temp_diff;
						 break;  // Thoát kh�?i vòng lặp sau khi tìm thấy ngưỡng phù hợp
				 }
			 }
		}
    }
     if(new_setpoint != last_setpoint){
    	 if(!waiting){
    		 change_time = HAL_GetTick();
    		 waiting = 1;
    	 }
    	 uint32_t curent_time_2 = HAL_GetTick();
    	 if((uint32_t)(curent_time_2 - change_time) >= 8000){
    		 pid.setpoint = new_setpoint;
    		 last_setpoint = new_setpoint;
    		 waiting = 0;
    	 }
     }else{
    	 waiting = 0;
     }
}
/*================================================ Hàm tính toán để quyết định thay đổi superheat setpoint =======================================*/


/*================================================ Hàm chính điều khiển van tiết lưu =======================================*/
// �?ịnh nghĩa các trạng thái
typedef enum {
    STATE_INIT,         // Trạng thái khởi tạo ban đầu
    STATE_CLOSING,      // �?óng van (500 bước)
    STATE_OPENING,      // Mở van (200 bước)
    STATE_CONTROL_EEV,      // Ch�? 15s
    STATE_IDLE_EEV
} SystemState;
// Biến toàn cục
SystemState current_state_eev = STATE_INIT;
uint32_t timer = 0;
uint8_t init_done = 0;      // C�? đánh dấu đã hoàn thành khởi tạo
void control_EEV(){
	static uint8_t check_run_defrost = 0;
	GPIO_PinState pin_state_run = HAL_GPIO_ReadPin(RUN_GPIO_Port, RUN_Pin);
	GPIO_PinState pin_state_run_defrost = HAL_GPIO_ReadPin(RUN_Defrost_GPIO_Port, RUN_Defrost_Pin);
	switch(pin_state_run_defrost){
	case GPIO_PIN_SET:
		if(check_run_defrost == 0 && !Stepper_IsMoving(&motor)){
			check_run_defrost = 1;
//        	step_position = 0;
//			percent_step = 0.0f;
//			Stepper_Move(&motor, -500);
			step_position = (step_position - 60 > 0) ? step_position - 60 : 0;
        	percent_step = (step_position/500.0f)*100.0f;
            Stepper_Move(&motor, -(500-step_position));
            current_state_eev = STATE_OPENING;
		}
		break;
	case GPIO_PIN_RESET:
		check_run_defrost = 0;
		   // Xử lý theo trạng thái hiện tại
		    switch(current_state_eev) {
		        case STATE_INIT:
		            // Luôn bắt đầu bằng việc đóng 500 bước khi khởi tạo
		            if(!Stepper_IsMoving(&motor)) {
		            	step_position = 500;
		            	percent_step = 100.0f;
		                Stepper_Move(&motor, 500);
//		            	step_position = 0;
//		            	percent_step = 0.0f;
//		                Stepper_Move(&motor, -500);
		                current_state_eev = STATE_CLOSING;
		                init_done = 0;  // �?ánh dấu chưa hoàn thành khởi tạo
		            }
		            break;

		        case STATE_CLOSING:
		            if(!Stepper_IsMoving(&motor)) {
		                if(!init_done) {
		                    // Nếu đây là lần đóng đầu tiên (khởi tạo)
		                    init_done = 1;
		                    // Nếu chân input đang ở mức cao, chuyển sang mở
		                    if(pin_state_run == GPIO_PIN_SET) {
		                        Stepper_Move(&motor, -250);
		                        current_state_eev = STATE_OPENING;
		                    } else {
		                    	current_state_eev = STATE_IDLE_EEV;  // Ch�? mức cao
		                    }
		                } else {
		                    // Nếu đây là lần đóng trong quá trình hoạt động bình thư�?ng
		                	current_state_eev = STATE_IDLE_EEV;
		                }
		            }
		            break;

		        case STATE_IDLE_EEV:
		        	if(!Stepper_IsMoving(&motor)){
		        		if(pin_state_run == GPIO_PIN_SET){
		        		    Stepper_Move(&motor, -250);
		        		    current_state_eev = STATE_OPENING;
		        		}
		        	}
		            break;

		        case STATE_OPENING:
		            if(!Stepper_IsMoving(&motor)) {
		                // �?ã mở xong, bắt đầu ch�? 15s
		                timer = HAL_GetTick();
		                current_state_eev = STATE_CONTROL_EEV;
		            }
		            break;

		        case STATE_CONTROL_EEV:
		            if(HAL_GetTick() - timer >= 8000) {
		                control_stepper();
		            }
		            // Nếu trong quá trình ch�? mà chân input chuyển thành mức thấp
		            if(pin_state_run == GPIO_PIN_RESET) {
		            	step_position = (step_position + 60 < 500) ? step_position + 60 : 500;
		            	percent_step = (step_position/500.0f)*100.0f;
		                Stepper_Move(&motor, step_position);
		                current_state_eev = STATE_CLOSING;
		            }
		            break;
		    }
	  break;
	}
}
/*================================================ Hàm chính điều khiển van tiết lưu =======================================*/


/*================================================ Hàm điều khiển dịch phụ làm mát đầu đẩy =======================================*/
int16_t nhiet_do_bat_lam_mat = 85;
int16_t nhiet_do_tat_lam_mat = 75;
void lam_mat_dau_day(){
	if(temperature_sensors.dau_day >= (float)(nhiet_do_bat_lam_mat)){
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
	}else if(temperature_sensors.dau_day <= (float)(nhiet_do_tat_lam_mat)){
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
	}
}
/*================================================ Hàm điều khiển dịch phụ làm mát đầu đẩy =======================================*/


void EWDG_Refresh(){
	HAL_GPIO_TogglePin(EWDG_GPIO_Port, EWDG_Pin);
}


/*================================================ Hàm xử lý dữ liệu giao tiếp ngoại vi =======================================*/
void modbus_communication(){
	modbus_slave.holdingRegs[0] = (uint16_t)(pressure_sensors.high_pressure_sensor*100.0f);
	modbus_slave.holdingRegs[1] = (uint16_t)(pressure_sensors.low_pressure_sensor*100.0f);
	modbus_slave.holdingRegs[2] = (int16_t)(temperature_sensors.dau_day*10.0f);
	modbus_slave.holdingRegs[3] = (int16_t)(temperature_sensors.hoi_ve*10.0f);
	modbus_slave.holdingRegs[4] = (int16_t)(Saturation_temperature*10.0f);
	modbus_slave.holdingRegs[5] = (int16_t)(delta_temperatute*10.0f);
	modbus_slave.holdingRegs[6] = (uint16_t)(pid.setpoint*10.0f);
	modbus_slave.holdingRegs[7] = (uint16_t)(percent_step*10.0f);
	modbus_slave.holdingRegs[8] = (uint16_t)(current_state_eev);
	modbus_slave.holdingRegs[9] = (uint16_t)(vref*100.0f);
//	modbus_slave.holdingRegs[8] = (uint16_t)(HAL_GPIO_ReadPin(RUN_GPIO_Port, RUN_Pin));
//	modbus_slave.holdingRegs[9] = (uint16_t)(HAL_GPIO_ReadPin(RUN_Defrost_GPIO_Port, RUN_Defrost_Pin));
}

typedef struct {
    int16_t* value_ptr;     // Con trỏ trực tiếp đến biến int16_t trong RAM
    uint16_t eeprom_addr;   // Địa chỉ EEPROM
} param_config_int16_t;
static void write_param_to_eeprom(
                                int16_t* current_ram_value_ptr,
                                uint16_t new_modbus_value, // Dữ liệu Modbus gốc là uint16_t
                                uint16_t addr) {
    if (current_ram_value_ptr == NULL) {
        return;
    }
    int16_t new_typed_value = (int16_t)new_modbus_value; // Ép kiểu giá trị Modbus sang int16_t

    if (*current_ram_value_ptr != new_typed_value) {
        *current_ram_value_ptr = new_typed_value;
        EEPROM_WriteInt16(&hEEPROM_final, addr, new_typed_value);
    }
}
void Data_Write(ModbusHandle* modbus){
	if(modbus_slave.emergency_write_from_master == 1){
		param_config_int16_t params[] = {
	      {&nhiet_do_bat_lam_mat, 0},    // Chỉ cần thêm &
	      {&nhiet_do_tat_lam_mat, 2},     // để lấy địa chỉ của biến
	   };
	  for (uint8_t i = 0; i < sizeof(params) / sizeof(params[0]); i++) {
		   write_param_to_eeprom(params[i].value_ptr, modbus_slave.holdingRegs_emergency_cpy[i], params[i].eeprom_addr);
	  }
	  modbus_slave.emergency_write_from_master = 0;
	}
}
/*================================================ Hàm xử lý dữ liệu giao tiếp ngoại vi =======================================*/




/*==================================================================================================*/
/* Restart system UART(Modbus) if error occurs */
static void Restart_UART1_DMA_Modbus_Simple(void)
{
    HAL_UART_DMAStop(&huart1);
    HAL_UART_Abort(&huart1);

    HAL_UART_DeInit(&huart1);
    HAL_Delay(10);

    MX_USART1_UART_Init();
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    Modbus_ReInit(&modbus_slave, &huart1, USART1_IRQn);
}
volatile uint32_t lastvalidIDTime = 0;
void reset_UART_DMA(){
	  if((uint32_t)(HAL_GetTick() - lastvalidIDTime) >= 10000){
		 printLOGDATA("[WARN] [UART] Modbus timeout detected. Attempting UART DMA restart...\r\n");
		 Restart_UART1_DMA_Modbus_Simple();
		 lastvalidIDTime = HAL_GetTick();
  }
}
/*==================================================================================================*/


/*================================================ ADC DMA =================================================*/
volatile uint16_t adc_buffer[5];
volatile uint8_t isADCFinish = 0;
volatile uint32_t time_refresh_adc_dma = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	isADCFinish = 1;
	time_refresh_adc_dma = HAL_GetTick();
}
/*==================================================================================================*/
/* Restart system ADC if error occurs */
static void Restart_ADC1_DMA_Simple(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DMA_Abort(&handle_GPDMA1_Channel2);

    HAL_ADC_DeInit(&hadc1);
    HAL_Delay(10); // Đợi một chút cho ổn định

    MX_ADC1_Init();
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOS | ADC_FLAG_EOC | ADC_FLAG_OVR | ADC_FLAG_AWD1 | ADC_FLAG_AWD2 | ADC_FLAG_AWD3 | ADC_FLAG_JEOS | ADC_FLAG_JEOC);

    isADCFinish = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 5);
}
void Reset_ADC_DMA(){
     uint32_t current_time =  HAL_GetTick();
     if((uint32_t)(current_time - time_refresh_adc_dma) >= 6000){
    	 printLOGDATA("[WARN] [ADC] ADC timeout detected. Attempting ADC DMA restart...\r\n");
    	 Restart_ADC1_DMA_Simple();
   		 time_refresh_adc_dma = HAL_GetTick();
     }
}
/*================================================ ADC DMA =================================================*/
float voltage_hoive;
float voltage_dauday;
float voltage_pl;
float voltage_ph;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  EEPROM_Init(&hEEPROM_final, &hi2c1, EEPROM_DEFAULT_7BIT_ADDR);
  EEPROM_ReadInt16(&hEEPROM_final, 0, &nhiet_do_bat_lam_mat);
  EEPROM_ReadInt16(&hEEPROM_final, 2, &nhiet_do_tat_lam_mat);

  StepperPins pins = {
  		  .PORT_IN1 = STEPPER_1_GPIO_Port, .PIN_IN1 = STEPPER_1_Pin,
  		  .PORT_IN2 = STEPPER_2_GPIO_Port, .PIN_IN2 = STEPPER_2_Pin,
  		  .PORT_IN3 = STEPPER_3_GPIO_Port, .PIN_IN3 = STEPPER_3_Pin,
  		  .PORT_IN4 = STEPPER_4_GPIO_Port, .PIN_IN4 = STEPPER_4_Pin
  };
  Stepper_Init(&motor, pins);
  PID_Init(&pid, 0.03f, 0.12f, 11.0f);

//  ADC_Init(&hadc1);
  Filter_Input_Init();
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 5);


  Modbus_Init(&modbus_slave, &huart1, USART1_IRQn);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(isADCFinish == 1){
		isADCFinish = 0;
		Calcular_Input(&hadc1);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buffer, 5);
	  }
//	  Calcular_Input(&hadc1);
//	  voltage_hoive = adc_temperature_sensors.ADC_hoi_ve*vref/4095.0f;
//	  voltage_dauday = adc_temperature_sensors.ADC_dau_day*vref/4095.0f;
//	  voltage_pl = adc_pressure_sensors.ADC_low_pressure*vref/4095.0f;
//	  voltage_ph = adc_pressure_sensors.ADC_high_pressure*vref/4095.0f;

	  superheat_value();
	  lam_mat_dau_day();
	  convert_setpoint();
	  control_EEV();

	  modbus_communication();
	  Data_Write(&modbus_slave);

	  reset_UART_DMA();
	  HAL_IWDG_Refresh(&hiwdg);
	  EWDG_Refresh();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 0;
  hiwdg.Init.Reload = 2999;
  hiwdg.Init.EWI = 0;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1279;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EWDG_GPIO_Port, EWDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEPPER_1_Pin|STEPPER_2_Pin|STEPPER_3_Pin|STEPPER_4_Pin
                          |RELAY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : EWDG_Pin */
  GPIO_InitStruct.Pin = EWDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EWDG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RUN_Defrost_Pin RUN_Pin */
  GPIO_InitStruct.Pin = RUN_Defrost_Pin|RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STEPPER_1_Pin STEPPER_2_Pin STEPPER_3_Pin STEPPER_4_Pin
                           RELAY_Pin */
  GPIO_InitStruct.Pin = STEPPER_1_Pin|STEPPER_2_Pin|STEPPER_3_Pin|STEPPER_4_Pin
                          |RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOB, STEPPER_1_Pin|STEPPER_2_Pin|STEPPER_3_Pin|STEPPER_4_Pin
                          |RELAY_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOB, RELAY_Pin, GPIO_PIN_RESET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
    	lastvalidIDTime = HAL_GetTick();
    	Modbus_UartTxCpltCallback(&modbus_slave);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	Modbus_HAL_ErrorCallback(&modbus_slave, &huart1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
    	count ++;
    	if(count >= 3){
            output_pid = PID_Calculate(&pid, delta_temperatute);
            float step_val = output_pid * 5.0f;
            stepp_count = (int16_t)step_val;
            // Lưu giá trị vào mảng, chỉ mục quay vòng khi đạt BUFFER_SIZE
            stepp_buffer[buffer_index] = stepp_count;
            buffer_index = (buffer_index + 1) % BUFFER_SIZE;
    	    count = 0;
    	}
        Stepper_Run(&motor);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
