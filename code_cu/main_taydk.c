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
#include <ST7735.h>
#include <math.h>
#include <GFX_FUNCTIONS.h>
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "PS2_F103.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Size 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int a,b,pre_a, pre_b,ofs_a,ofs_b;
int dc775[4], dc887[4];
uint8_t fl=0,fl_pre=0,fl_but1=0;
uint8_t cnt_rot=0;
float tmp_joy[2]={0,0};
uint8_t joy_but[2]={0,0};
uint8_t fc15,f_prec15=100,fc14,fc14_pre=100;
float cnt_goc_tmp=127;
uint8_t cnt_goc=127;
uint8_t flag_first=0;
int flag_bf3=0;
//uint32_t adc;
uint8_t cnt_rot_pre=0;
uint8_t cnt_hc=0;
uint8_t b0_pre=0;
int x775,x887;
uint8_t rx_hc, buf_hc[5],index_hc=0,tx_hc[6]={255,0,0,0,0,0};
Data du;
uint8_t flag_gui=0;
uint8_t tx[Size];
uint8_t tx_dma[Size];  // Buffer riêng cho DMA, tránh ghi đè khi đang truyền
uint8_t k=0;
uint8_t Indexx=0,dataa;
uint8_t Buffer[9];
uint8_t cntttt=0;
int xungmt887,xungmt775,xung775_pre=999,xung887_pre=999;
uint8_t flag_esp32=0;
int8_t cnt_bf3=0;
char bf8[5],bf7[5];
uint8_t cnt=0;
uint8_t rx_hc_at[64];      // Buffer nhận phản hồi AT bằng DMA
volatile uint8_t at_flag = 0; // Cờ báo hiệu đã nhận được phản hồi
uint16_t at_rx_size = 0;   // Độ dài dữ liệu nhận được
volatile uint8_t flag_update_lcd = 0;  // Flag để main loop cập nhật LCD
int16_t prev_arrow_x = 64, prev_arrow_y = 51; // Compass arrow previous endpoint
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
            ST7735_WriteString(5, 80, dbg, Font_7x10, WHITE, BLACK);

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

    ST7735_WriteString(5, 60, "HC12: Searching...", Font_7x10, YELLOW, BLACK);

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
            ST7735_WriteString(5, 60, msg, Font_7x10, GREEN, BLACK);
            break;
        }
    }

    if (found) {
        HC12_Send_AT("AT+B115200\r\n", "OK", 500);
        HAL_Delay(200); // ✅ Chờ module reboot sau khi đổi baud
        HC12_Send_AT("AT+C059\r\n", "OK", 500);

        huart3.Init.BaudRate = 115200; // 11520000
        HAL_UART_Init(&huart3);

        ST7735_WriteString(5, 70, "HC12: CONFIG DONE", Font_7x10, GREEN, BLACK);
    } else {
        ST7735_WriteString(5, 60, "HC12: NO RESPONSE", Font_7x10, RED, BLACK);
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
	  PS2_Update();
	  if(du.button[0]==16){
		  tmp_joy[0]-=0.5;
		  if(tmp_joy[0]<=-100)tmp_joy[0]=-100;
	  }
	  else if(du.button[0]==64){
		  tmp_joy[0]+=0.5;
		  if(tmp_joy[0]>=100)tmp_joy[0]=100;
	  }
	  else if(du.button[0]==128){
		  tmp_joy[1]-=0.5;
		  if(tmp_joy[1]<=-50)tmp_joy[1]=-50;
	  }
	  else if(du.button[0]==32){
		  tmp_joy[1]+=0.5;
		  if(tmp_joy[1]>=50)tmp_joy[1]=50;
	  }
	  else{
		  tmp_joy[0]=0;
		  tmp_joy[1]=0;

	  }


	  tx[3]=du.button[1];
	  cnt_rot_pre=cnt_rot;

	  // === Chức năng quay motor chip ID 2 ===
	  // L1 (button[1]==4): gửi lệnh quay thuận cho chip ID 2
	  // R1 (button[1]==8): gửi lệnh quay ngược cho chip ID 2

tx[0] = du.address;  // Địa chỉ bình thường (255)

	  // Fix overflow: clamp giá trị trước khi gán vào uint8_t
	  int16_t val_x = (int16_t)du.data[0] + (int16_t)tmp_joy[0];
	  int16_t val_y = (int16_t)du.data[1] + (int16_t)tmp_joy[1];
	  if(val_x < 0) val_x = 0;
	  if(val_x > 255) val_x = 255;
	  if(val_y < 0) val_y = 0;
	  if(val_y > 255) val_y = 255;
	  tx[1] = (uint8_t)val_x;
	  tx[2] = (uint8_t)val_y;

	  tx[4]=du.button[0];
	  cnt++;
	  if(cnt>=250) cnt=0;
	  tx[5] = cnt;

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  // Fix DMA BUSY: kiểm tra UART sẵn sàng + dùng buffer riêng
	  if(huart3.gState == HAL_UART_STATE_READY){
//		  memcpy(tx_dma, tx, Size);
		  HAL_UART_Transmit_DMA(&huart3, tx, Size);
	  }

	  flag_update_lcd = 1;  // Để main loop cập nhật LCD, không làm trong ISR
	}


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
  HAL_Delay(1000);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();HAL_Delay(1000);
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init(0);
  	fillScreen(BLACK);
  	ST7735_SetRotation(0);

  	// ── Title Bar (y=0, h=16) ──
  	ST7735_FillRectangle(0, 0, 128, 16, BLUE);
  	ST7735_WriteString(25, 3, "RBC DKH UTC", Font_7x10, WHITE, BLUE);

  	// ── Compass Section (y=17, h=68) ──
  	drawRoundRect(2, 17, 124, 68, 3, CYAN);
  	drawCircle(64, 51, 23, WHITE);
  	drawCircle(64, 51, 22, WHITE);
  	// N=Tien, S=Lui, E=Phai, W=Trai
  	ST7735_WriteString(60, 19, "N", Font_7x10, RED, BLACK);
  	ST7735_WriteString(60, 75, "S", Font_7x10, WHITE, BLACK);
  	ST7735_WriteString(90, 47, "E", Font_7x10, WHITE, BLACK);
  	ST7735_WriteString(31, 47, "W", Font_7x10, WHITE, BLACK);
  	fillCircle(64, 51, 2, YELLOW);

  	// ── X/Y Section (y=87, h=18) ──
  	drawRoundRect(2, 87, 60, 18, 3, CYAN);
  	drawRoundRect(66, 87, 60, 18, 3, CYAN);
  	ST7735_WriteString(8, 91, "X:", Font_7x10, GREEN, BLACK);
  	ST7735_WriteString(72, 91, "Y:", Font_7x10, GREEN, BLACK);

  	// ── Button Section (y=107, h=18) ──
  	drawRoundRect(2, 107, 124, 18, 3, CYAN);
  	ST7735_WriteString(6, 111, "L1", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(24, 111, "R1", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(42, 111, "L2", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(60, 111, "R2", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(82, 111, "UP", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(100, 111, "DN", Font_7x10, 0x4208, BLACK);

  	// ── Button Section 2 (y=127, h=17) ──
  	drawRoundRect(2, 127, 124, 17, 3, CYAN);
  	ST7735_WriteString(4, 130, "Tr", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(19, 130, "Ci", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(34, 130, "Cr", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(49, 130, "Sq", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(64, 130, "LT", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(79, 130, "RT", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(94, 130, "ST", Font_7x10, 0x4208, BLACK);
  	ST7735_WriteString(109, 130, "SE", Font_7x10, 0x4208, BLACK);

  	// ── Mode Section (y=146, h=14) ──
  	drawRoundRect(2, 146, 124, 14, 3, CYAN);
  	ST7735_WriteString(8, 148, "MODE:", Font_7x10, YELLOW, BLACK);
//  	HAL_Delay(500);       // Đợi module HC-12 ổn định nguồn
//		HC12_Setup_Routine(); // Chạy cấu hình AT
  	PS2_Init(&htim4,&du);
  	// Khởi tạo giá trị an toàn trước khi bật ISR
  	du.address = 255;
  	du.data[0] = 100;  // Center (50+50)
  	du.data[1] = 100;
  	du.button[0] = 0;
  	du.button[1] = 0;
  	HAL_Delay(500);
  	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Cập nhật LCD trong main loop thay vì ISR (tránh ISR quá nặng)
	  if(flag_update_lcd){
		  flag_update_lcd = 0;
		  char buff[20];

		  // ── Compass Arrow ──
		  // X giam=tien(N/up), X tang=lui(S/down)
		  // Y tang=phai(E/right), Y giam=trai(W/left)
		  drawLine(64, 51, prev_arrow_x, prev_arrow_y, BLACK); // Erase old

		  int16_t joy_x = (int16_t)tx[1] - 100; // X: neg=tien, pos=lui
		  int16_t joy_y = (int16_t)tx[2] - 100; // Y: neg=trai, pos=phai
		  if(joy_x > 100) joy_x = 100; if(joy_x < -100) joy_x = -100;
		  if(joy_y > 100) joy_y = 100; if(joy_y < -100) joy_y = -100;
		  // Map: joy_y -> screen X (phai/trai), joy_x -> screen Y (lui/tien)
		  int16_t new_ax = 64 + (joy_y * 19) / 100;
		  int16_t new_ay = 51 + (joy_x * 19) / 100;

		  if(joy_x > 5 || joy_x < -5 || joy_y > 5 || joy_y < -5) {
			  drawLine(64, 51, new_ax, new_ay, GREEN); // Draw new
		  }
		  fillCircle(64, 51, 2, YELLOW); // Redraw center dot
		  prev_arrow_x = new_ax;
		  prev_arrow_y = new_ay;

		  // ── X, Y Values ──
		  sprintf(buff, "%-3d", tx[1]);
		  ST7735_WriteString(24, 91, buff, Font_7x10, WHITE, BLACK);
		  sprintf(buff, "%-3d", tx[2]);
		  ST7735_WriteString(88, 91, buff, Font_7x10, WHITE, BLACK);

		  // ── Button Indicators ──
		  uint16_t col;
		  col = (tx[3] & 4)  ? GREEN : 0x4208;  // L1
		  ST7735_WriteString(6, 111, "L1", Font_7x10, col, BLACK);
		  col = (tx[3] & 8)  ? GREEN : 0x4208;  // R1
		  ST7735_WriteString(24, 111, "R1", Font_7x10, col, BLACK);
		  col = (tx[3] & 1)  ? GREEN : 0x4208;  // L2
		  ST7735_WriteString(42, 111, "L2", Font_7x10, col, BLACK);
		  col = (tx[3] & 2)  ? GREEN : 0x4208;  // R2
		  ST7735_WriteString(60, 111, "R2", Font_7x10, col, BLACK);
		  col = (tx[4] & 16) ? GREEN : 0x4208;  // UP
		  ST7735_WriteString(82, 111, "UP", Font_7x10, col, BLACK);
		  col = (tx[4] & 64) ? GREEN : 0x4208;  // DN
		  ST7735_WriteString(100, 111, "DN", Font_7x10, col, BLACK);

		  // ── Button Row 2: Tr Ci Cr Sq LT RT ST SE ──
		  col = (tx[3] & 16)  ? GREEN : 0x4208;  // Triangle
		  ST7735_WriteString(4, 130, "Tr", Font_7x10, col, BLACK);
		  col = (tx[3] & 32)  ? GREEN : 0x4208;  // Circle
		  ST7735_WriteString(19, 130, "Ci", Font_7x10, col, BLACK);
		  col = (tx[3] & 64)  ? GREEN : 0x4208;  // Cross
		  ST7735_WriteString(34, 130, "Cr", Font_7x10, col, BLACK);
		  col = (tx[3] & 128) ? GREEN : 0x4208;  // Square
		  ST7735_WriteString(49, 130, "Sq", Font_7x10, col, BLACK);
		  col = (tx[4] & 128) ? GREEN : 0x4208;  // Left
		  ST7735_WriteString(64, 130, "LT", Font_7x10, col, BLACK);
		  col = (tx[4] & 32)  ? GREEN : 0x4208;  // Right
		  ST7735_WriteString(79, 130, "RT", Font_7x10, col, BLACK);
		  col = (tx[4] & 8)   ? GREEN : 0x4208;  // Start
		  ST7735_WriteString(94, 130, "ST", Font_7x10, col, BLACK);
		  col = (tx[4] & 1)   ? GREEN : 0x4208;  // Select
		  ST7735_WriteString(109, 130, "SE", Font_7x10, col, BLACK);

		  // ── Mode ──
		  if(flag_esp32 == 0){
			  ST7735_WriteString(48, 148, "HC12  ", Font_7x10, GREEN, BLACK);
		  } else {
			  ST7735_WriteString(48, 148, "ESPNOW", Font_7x10, CYAN, BLACK);
		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 143;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO_PS2_Pin|CS_PS2_Pin|CLK_PS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_MH_Pin|CS_MH_Pin|A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DI_PS2_Pin */
  GPIO_InitStruct.Pin = DI_PS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DI_PS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_PS2_Pin CS_PS2_Pin CLK_PS2_Pin */
  GPIO_InitStruct.Pin = DO_PS2_Pin|CS_PS2_Pin|CLK_PS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_MH_Pin CS_MH_Pin A0_Pin */
  GPIO_InitStruct.Pin = RST_MH_Pin|CS_MH_Pin|A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
