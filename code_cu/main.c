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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_stm32.h"
#include "bno055.h"
#include "stdio.h"
#include "ILI9341.h"
#include "pca9685.h"
#include "math.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bno055_vector_t d;
extern const uint8_t LOGO[];
extern const uint32_t LOGO_size;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern int current_error_x;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVOMIN  125 // ~0.6ms (Góc 0)
#define SERVOMAX  490 // ~2.4ms (Góc 180)


typedef struct{
	int goc_ht[2];
	int goc_trc[2];
	int delta_goc[2];
	int goc_tong[2];
	int goc_offset[2];
	int goc_ok[2];
}IMU;
IMU BNO055={};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t TxData[8] = {100,100,100,100,100,100,100,100};
uint32_t TxMailbox;
CAN_TxHeaderTypeDef TxHeader;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
volatile float adc_list[11];
volatile uint8_t channel=0;
uint32_t adc;
uint8_t check_hc12=0;
uint8_t db=10;
uint8_t pre_check=0, flag_ok=0;
float X,Y;
float pii= 3.141592;
float pi4= 0.7854;
float pi34=2.3562;
uint8_t rx_hc12[5]={};
uint8_t id_hc12;
uint8_t hc12;
uint8_t pre_bt3=0;
float rad_dh=0;
float add_rad=0;
int goc_quay=0;
float goc_bu=0;
uint8_t i2c_devices[10];
uint8_t device_count = 0;

/* --- Speed ramp: tang/giam toc muot (tu Zephyr) --- */
#define RAMP_STEP 5.0f
float cur_spd[4] = {100.0f, 100.0f, 100.0f, 100.0f};

float ramp_toward(float current, float target, float step) {
	if (current < target) {
		current += step;
		if (current > target) current = target;
	} else if (current > target) {
		current -= step;
		if (current < target) current = target;
	}
	return current;
}

/* --- Digital Input states (active-low: nhan=0, tha=1) --- */
uint8_t dig_in[10] = {0};

/* --- Digital Output states --- */
uint8_t dig_out[12] = {0};

/* Pin mapping cho 12 digital output */
typedef struct { GPIO_TypeDef *port; uint16_t pin; } GpioPin_t;
static const GpioPin_t out_pins[12] = {
	{GPIOB, GPIO_PIN_1},  // o1
	{GPIOE, GPIO_PIN_7},  // o2
	{GPIOE, GPIO_PIN_8},  // o3
	{GPIOE, GPIO_PIN_9},  // o4
	{GPIOE, GPIO_PIN_10}, // o5
	{GPIOE, GPIO_PIN_11}, // o6
	{GPIOE, GPIO_PIN_12}, // o7
	{GPIOE, GPIO_PIN_13}, // o8
	{GPIOE, GPIO_PIN_14}, // o9
	{GPIOE, GPIO_PIN_15}, // o10
	{GPIOD, GPIO_PIN_14}, // o11
	{GPIOD, GPIO_PIN_15}, // o12
};

void Read_Digital_Inputs(void) {
	dig_in[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10); // in1
	dig_in[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11); // in2
	dig_in[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12); // in3
	dig_in[3] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);  // in4
	dig_in[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);  // in5
	dig_in[5] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);  // in6
	dig_in[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);  // in7
	dig_in[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);  // in8
	dig_in[8] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);  // in9
	dig_in[9] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10); // in10
}

void Write_Digital_Outputs(void) {
	for (int i = 0; i < 12; i++) {
		HAL_GPIO_WritePin(out_pins[i].port, out_pins[i].pin,
				dig_out[i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void Update_Display_Data(void) {
//	if(ready==1){
//			char buffer[50];
//			snprintf(buffer, sizeof(buffer), "%d %d %d %d %d %d %d %d %d %d %d",
//							 sensor_error[10], sensor_error[9], sensor_error[8],
//							 sensor_error[7], sensor_error[6], sensor_error[5],
//							 sensor_error[4], sensor_error[3], sensor_error[2],
//							 sensor_error[1], sensor_error[0]);
//			ST7735_WriteString(5, 10, buffer, Font_7x10, BLACK, WHITE);
//
//			char buff[30];
//			sprintf(buff, "ER: %-4d", er);
//			ST7735_WriteString(5, 25, buff, Font_7x10, BLACK, WHITE);
//			sprintf(buff, "STATE: %-7s", (robot.sw_start==0) ? "STOP" : "GO!!");
//			ST7735_WriteString(5, 65, buff, Font_7x10, BLACK, WHITE);
//			sprintf(buff, "MODE: %-5s", (robot.sw_mode==0) ? "LEARN" : "RUN");
//			ST7735_WriteString(5, 45, buff, Font_7x10, BLACK, WHITE);
//			sprintf(buff, "STEP: %-2d", robot.map_index);
//			ST7735_WriteString(5, 90, buff, Font_7x10, BLACK, WHITE);
//			sprintf(buff, "DIST: %-5ld", robot.sdist);
////			sprintf(buff, "IMU: %-4d", goc_z);
//
//			ST7735_WriteString(5, 110, buff, Font_7x10, BLACK, WHITE);
//			sprintf(buff, "SPM: %d", special_point_mode);
//			ST7735_WriteString(80, 110, buff, Font_7x10, BLACK, WHITE);
//		}
//		else{
//			if (robot.btn_map == BTN_HIGH && robot.pre_btn_map == BTN_LOW) robot.map=1-robot.map;
//			robot.pre_btn_map=robot.btn_map;
//			if (robot.btn_ok == BTN_HIGH && robot.pre_btn_ok == BTN_LOW) ready=1;
//			ST7735_WriteString(80, 90, robot.map==0 ? "SAN: RED " : "SAN: BLUE", Font_7x10, robot.map==0 ? RED : BLUE, WHITE);
//			ST7735_WriteString(90, 50, "SO_FLY", Font_11x18, robot.map==0 ? RED:BLUE, WHITE);
//			ST7735_DrawRectangle(0, 0, 160, 40, robot.map==0 ? RED:BLUE);
//			ST7735_DrawRectangle(0, 40, 160, 41, robot.map==0 ? RED:BLUE);
//			ST7735_DrawRectangle(0, 81, 160, 47, robot.map==0 ? RED:BLUE);
//			Select_Robot_Map(robot.map);
//		}

}
void Read_IMU(){
	d = bno055_getVectorEuler();
	BNO055.goc_ht[0] = d.x;
	BNO055.delta_goc[0] = BNO055.goc_ht[0] - BNO055.goc_trc[0];
	if (BNO055.delta_goc[0] > 180) BNO055.delta_goc[0] -= 360;
	if (BNO055.delta_goc[0] < -180) BNO055.delta_goc[0] += 360;
	BNO055.goc_tong[0] += BNO055.delta_goc[0];
	BNO055.goc_trc[0] = BNO055.goc_ht[0];

	BNO055.goc_ht[1] = d.z;
	BNO055.delta_goc[1] = BNO055.goc_ht[1] - BNO055.goc_trc[1];
	if (BNO055.delta_goc[1] > 180) BNO055.delta_goc[1] -= 360;
	if (BNO055.delta_goc[1] < -180) BNO055.delta_goc[1] += 360;
	BNO055.goc_tong[1] += BNO055.delta_goc[1];
	BNO055.goc_trc[1] = BNO055.goc_ht[1];
}




void Channel(uint8_t channel)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Bit 0
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Bit 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Bit 2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (channel & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Bit 3
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_list[channel] = adc;
	channel++;

	if (channel > 3) {
		channel = 0;
	}

	Channel(channel);
//	for(int i=0;i<300;i++);
	HAL_ADC_Start_DMA(&hadc1, &adc, 1);
}
uint8_t Chuan_hoa(float x){
	if(x>=255) x=255;
	if(x<=0) x=0;
	return (uint8_t)(round(x));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM8) {

		if(check_hc12 != pre_check){
			flag_ok=0;
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		}
		else{
			flag_ok=1;
			X=0;
			Y=0;
		}
		pre_check=check_hc12;

	}
	if (htim->Instance == TIM1) {
		Read_IMU();
		Read_Digital_Inputs();

		uint8_t btn3 = rx_hc12[2];
		uint8_t btn2 = rx_hc12[3];

		if(flag_ok == 0) { /* === CONNECTED === */

			/* --- THO THUT LEN XUONG voi limit switch --- */
			/* Active-low: dig_in[x]==0 khi cham limit, ==1 khi chua cham */
			if (btn3 == 12 && dig_in[4] == 1 && dig_in[2] == 1) {
				TxData[0] = 150; TxData[1] = 0;     // UP + THO
			} else if (btn3 == 3 && dig_in[1] == 1 && dig_in[3] == 1) {
				TxData[0] = 50;  TxData[1] = 200;   // DN + THUT
			} else if (btn3 == 4 && dig_in[4] == 1) {
				TxData[0] = 200; TxData[1] = 100;   // THO
			} else if (btn3 == 1 && dig_in[3] == 1) {
				TxData[0] = 0;   TxData[1] = 100;   // THUT
			} else if (btn3 == 8 && dig_in[2] == 1) {
				TxData[0] = 100; TxData[1] = 0;     // LEN
			} else if (btn3 == 2 && dig_in[1] == 1) {
				TxData[0] = 100; TxData[1] = 200;   // XUONG
			} else {
				TxData[0] = 100; TxData[1] = 100;   // DUNG
			}

			/* --- Huong di chuyen (btn2) --- */
			if(btn2 == 4) add_rad += pii/4;
			else if(btn2 == 2) add_rad -= pii/4;

			rad_dh += (add_rad - rad_dh) * 0.1;

			/* --- Goc quay (clamp +-20) --- */
			goc_quay = BNO055.goc_tong[0] + goc_bu - (int)(rad_dh/pii*180);
			if(goc_quay > 20) goc_quay = 20;
			else if(goc_quay < -20) goc_quay = -20;

			/* --- Dong hoc mecanum 4 banh --- */
			float goc_rad = ((goc_quay + BNO055.goc_tong[0] + goc_bu) * pii) / 180;
			float target[4];
			target[0] = X*sin(goc_rad-pi4)  + Y*cos(goc_rad-pi4)  + 100 + goc_quay;
			target[1] = X*sin(goc_rad+pi4)  + Y*cos(goc_rad+pi4)  + 100 + goc_quay;
			target[2] = X*sin(goc_rad+pi34) + Y*cos(goc_rad+pi34) + 100 + goc_quay;
			target[3] = X*sin(goc_rad-pi34) + Y*cos(goc_rad-pi34) + 100 + goc_quay;

			/* --- Speed ramp: tang/giam toc muot --- */
			for (int i = 0; i < 4; i++) {
				cur_spd[i] = ramp_toward(cur_spd[i], target[i], RAMP_STEP);
				TxData[2+i] = Chuan_hoa(cur_spd[i]);
			}

		} else { /* === MAT KET NOI === */
			TxData[0] = 100;
			TxData[1] = 100;
			/* Ramp ve 100 (neutral) - van muot */
			for (int i = 0; i < 4; i++) {
				cur_spd[i] = ramp_toward(cur_spd[i], 100.0f, RAMP_STEP);
				TxData[2+i] = Chuan_hoa(cur_spd[i]);
			}
			goc_quay = 0;
			add_rad = 0;
			rad_dh = 0;
			rx_hc12[2] = 0; /* reset btn */
			rx_hc12[3] = 0;
			TxData[7] = 100;
		}
		TxData[6] = (flag_ok == 0) ? db : 0;
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

		/* --- Ghi digital output --- */
		Write_Digital_Outputs();
	}
}
void I2C1_Scan(void) {
    // Reset mảng và biến đếm trước khi quét
    device_count = 0;
    for(int i = 0; i < 10; i++) i2c_devices[i] = 0;

    for (uint16_t i = 1; i < 128; i++) {
        // Kiểm tra thiết bị có phản hồi (ACK) hay không
        // i << 1 vì thư viện HAL sử dụng địa chỉ 8-bit (7-bit address + R/W bit)
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 2) == HAL_OK) {
            if (device_count < 10) {
                i2c_devices[device_count] = (uint8_t)(i << 1);
                device_count++;
            } else {
                // Đã đầy mảng 10 phần tử, dừng quét để bảo vệ bộ nhớ
                break;
            }
        }
    }
}
HAL_StatusTypeDef HC12_Send_AT(char *cmd, char *expected, uint32_t timeout) {
    uint8_t rx_tmp[16];
    uint8_t byte;
    uint8_t idx = 0;
    memset(rx_tmp, 0, sizeof(rx_tmp));

    HAL_UART_Abort(&huart3);
    __HAL_UART_FLUSH_DRREGISTER(&huart3);

    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), 100);
    // ✅ KHÔNG delay ở đây - bắt đầu nhận ngay lập tức

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout && idx < 15) {
        if (HAL_UART_Receive(&huart3, &byte, 1, 50) == HAL_OK) {
            rx_tmp[idx++] = byte;

            // Debug: hiện từng byte
            char dbg[32];
            sprintf(dbg, "RX[%d]:%02X '%c'   ", idx-1, byte, (byte>=32&&byte<127)?byte:'.');

            if (strstr((char*)rx_tmp, expected) != NULL) {
                return HAL_OK;
            }
        }
    }
    return HAL_ERROR;
}

void HC12_Setup_Routine(void) {
    uint32_t bauds[] = {9600, 115200, 19200, 38400, 4800, 2400}; // ✅ 9600 lên đầu
    uint8_t found = 0;


    for (uint8_t i = 0; i < 6; i++) {
        huart3.Instance = USART3;
        huart3.Init.BaudRate = bauds[i];
        huart3.Init.WordLength = UART_WORDLENGTH_8B;
        huart3.Init.StopBits = UART_STOPBITS_1;
        huart3.Init.Parity = UART_PARITY_NONE;
        huart3.Init.Mode = UART_MODE_TX_RX;
        huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart3.Init.OverSampling = UART_OVERSAMPLING_16;
        HAL_UART_Init(&huart3);
        HAL_Delay(50);

        // ✅ Chỉ gửi 1 lần duy nhất qua HC12_Send_AT
        if (HC12_Send_AT("AT\r\n", "OK", 500) == HAL_OK) {
            found = 1;
            char msg[30];
            sprintf(msg, "Found @ %lu", bauds[i]);
            break;
        }
    }

    if (found) {
        HC12_Send_AT("AT+B115200\r\n", "OK", 500);
        HAL_Delay(200); // ✅ Chờ module reboot sau khi đổi baud
        HC12_Send_AT("AT+C059\r\n", "OK", 500);

        huart3.Init.BaudRate = 11520000;
        HAL_UART_Init(&huart3);

    }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART3)
	{
		if(hc12 == 255)
		{
			id_hc12 = 0;
		}
		else
		{
			rx_hc12[id_hc12++] = hc12;
			if(id_hc12 == 5)
			{
				id_hc12 = 0;
				check_hc12=rx_hc12[4];
				if(flag_ok==0){
					Y=rx_hc12[0]-100;
					X=-rx_hc12[1]+100;
				}
				else{
					X=0;
					Y=0;
				}

			}
		}
		HAL_UART_Receive_DMA(&huart3, &hc12, 1);
	}
}
uint16_t AngleToPWM(uint8_t angle) {
    if (angle > 180) angle = 180;
    // Sử dụng tính toán số thực để giữ độ chính xác trước khi ép kiểu
    float pulse = (float)SERVOMIN + ((float)angle * (SERVOMAX - SERVOMIN) / 180.0f);
    return (uint16_t)pulse;
}
void SetServoAngle_1(uint8_t num, float angle) {
    uint16_t pwmValue = AngleToPWM(angle);
    PCA9685_SetPWM(&hi2c2,PCA9685_I2C_ADDRESS_1,num, 0, pwmValue);
}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init(&hspi1);
  ILI9341_FillRect(WHITE, 0, 0, 320, 240);
  ILI9341_DrawRect(0, 0, 319, 80, RED);
  ILI9341_DrawRect(0, 80, 319, 80, RED);
  ILI9341_DrawRect(0, 160, 319, 80, RED);

  ILI9341_DrawRect(100, 150, 50, 50, GREEN);
	LCD_PrintString(0, 0, "Vu Duc Du", BLACK);
	Channel(channel);
//	HC12_Setup_Routine();
	HAL_ADC_Start_DMA(&hadc1, &adc, 1);
	HAL_Delay(2000);
//  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
	  Error_Handler();
  }
  I2C1_Scan();

  TxHeader.StdId = 0x123;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
  HAL_UART_Receive_DMA(&huart3, &hc12, 1);
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();
  PCA9685_Init(&hi2c2, PCA9685_I2C_ADDRESS_1);
  PCA9685_SetPWMFreq(&hi2c2,PCA9685_I2C_ADDRESS_1, 50.0);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	/* --- DEBUG INPUT --- 
  	 * Ban mo "Live Expressions" trong STM32CubeIDE va add bien `dig_in` 
  	 * de xem mang 10 phan tu thay doi khi nhan/tha cong tac hanh trinh.
  	 */
  	Read_Digital_Inputs();
  	HAL_Delay(50);
  	
  	// SetServoAngle_1(0, 90);
  	// HAL_Delay(3000);
  	// SetServoAngle_1(0, 0);
  	// HAL_Delay(3000);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 143;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2879;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, led_pc13_Pin|led_pc14_Pin|led_pc15_Pin|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, s0_Pin|s1_Pin|s2_Pin|s3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|o1_Pin|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, o2_Pin|o3_Pin|o4_Pin|o5_Pin
                          |o6_Pin|o7_Pin|o8_Pin|o9_Pin
                          |o10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, o11_Pin|o12_Pin|MH_RST_Pin|MH_CS_Pin
                          |MH_DC_Pin, GPIO_PIN_RESET);

  /* === NEW: Digital Inputs voi chan moi === */
  /* in1(PC10), in2(PC11), in3(PC12) */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* in4(PD7) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* in5(PB4), in6(PB5), in7(PB8), in8(PB9) */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* in9(PE0), in10(PE10) + btn1(PE4), btn2(PE5) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_10 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        uint32_t isrflags = READ_REG(huart->Instance->SR); // Với dòng F4 dùng SR
        if ((isrflags & UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
        }
        if ((isrflags & UART_FLAG_FE) != RESET)
        {
            __HAL_UART_CLEAR_FEFLAG(huart);
        }

        // Quan trọng: Phải khởi động lại DMA sau khi lỗi
        HAL_UART_Receive_DMA(&huart3, &hc12, 1);
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
#ifdef USE_FULL_ASSERT
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
