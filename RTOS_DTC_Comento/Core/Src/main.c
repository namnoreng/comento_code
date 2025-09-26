/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (초급: RTOS 제거, 단일 루프)
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"   // HAL 전반 포함 (stm32f4xx_hal.h)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// PMIC Fault 레지스터 (데이터시트 없으므로 예시)
typedef enum {
  REG_UV      = 0x10, // Under-Voltage status
  REG_OC      = 0x11, // Over-Current status
  REG_OV      = 0x12, // Over-Voltage status
  REG_VOUT    = 0x20  // Vout 설정 레지스터 (예시)
} PMIC_Reg;

// Fault 데이터(8바이트) 예시: bit 필드 + raw 동시 활용
typedef union {
  uint8_t raw[8];
  struct {
    uint8_t UV:1;
    uint8_t OC:1;
    uint8_t OV:1;
    uint8_t rsv:5;
    uint8_t pad[7];
  } bits;
} FaultData;

// 간단 DTC 구조 (예: 2바이트 코드 + 상태)
#pragma pack(push, 1)
typedef struct {
  uint16_t code;    // 예: 0x1234
  uint8_t  status;  // 0=pass, 1=fault active
} DTC_t;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PMIC I2C 7-bit 주소 (예: 0x40) -> HAL에서는 8-bit로 <<1 필요
#define PMIC_SLAVE_ADDR   (0x40U << 1)

// EEPROM (일반 SPI 시퀀스 예시: 25xx 계열 가정)
#define EEPROM_CS_PORT    GPIOB
#define EEPROM_CS_PIN     GPIO_PIN_0
#define CMD_WREN          0x06
#define CMD_WRDI          0x04
#define CMD_RDSR          0x05
#define CMD_WRSR          0x01
#define CMD_READ          0x03
#define CMD_WRITE         0x02

// CAN 파라미터
#define CAN_TX_STDID      0x123

// UART 로그 버퍼 크기
#define LOG_BUF_SZ        64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CS_LOW()   HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
static FaultData g_fault;
static DTC_t     g_dtc   = {0x1234, 0};  // 예시 DTC
static uint8_t   g_uart_logbuf[LOG_BUF_SZ];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);

/* USER CODE BEGIN PFP */
// PMIC
static HAL_StatusTypeDef PMIC_ReadFaultReg(PMIC_Reg reg, FaultData *out);
static HAL_StatusTypeDef PMIC_SetVout_mV(uint16_t mv); // 예시 변환 함수

// EEPROM (SPI, 폴링)
static void EEPROM_WriteEnable(void);
static uint8_t EEPROM_ReadStatus(void);
static void EEPROM_WaitWriteComplete(void);
static void EEPROM_WriteBytes(uint16_t addr, const uint8_t *data, uint16_t len);
static void EEPROM_ReadBytes(uint16_t addr, uint8_t *data, uint16_t len);

// CAN
static void CAN_SendBytes(const uint8_t *data, uint8_t len);

// UART
static void UART_Log(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ---- PMIC ----
static HAL_StatusTypeDef PMIC_ReadFaultReg(PMIC_Reg reg, FaultData *out)
{
  if (!out) return HAL_ERROR;
  memset(out->raw, 0, sizeof(out->raw));
  // 예: status 1바이트만 사용
  return HAL_I2C_Mem_Read(&hi2c1, PMIC_SLAVE_ADDR, (uint16_t)reg,
                          I2C_MEMADD_SIZE_8BIT, out->raw, 1, 100);
}

// 예시: Vout(mV) -> 코드 변환 (데이터시트 기반으로 수정하세요)
// 가정: 5mV/LSB, base 0mV (단순 예시)
static HAL_StatusTypeDef PMIC_SetVout_mV(uint16_t mv)
{
  uint8_t vcode = (uint8_t)(mv / 5U); // 5mV 단위
  return HAL_I2C_Mem_Write(&hi2c1, PMIC_SLAVE_ADDR, (uint16_t)REG_VOUT,
                           I2C_MEMADD_SIZE_8BIT, &vcode, 1, 100);
}

// ---- EEPROM (SPI) ----
static void EEPROM_WriteEnable(void)
{
  uint8_t cmd = CMD_WREN;
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  CS_HIGH();
}

static uint8_t EEPROM_ReadStatus(void)
{
  uint8_t cmd = CMD_RDSR, sr = 0;
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  HAL_SPI_Receive(&hspi1, &sr, 1, 100);
  CS_HIGH();
  return sr;
}

static void EEPROM_WaitWriteComplete(void)
{
  // 통상 SR bit0 (WIP) = 1이면 write 진행 중
  while (EEPROM_ReadStatus() & 0x01U) { /* busy polling */ }
}

static void EEPROM_WriteBytes(uint16_t addr, const uint8_t *data, uint16_t len)
{
  uint8_t hdr[3] = { CMD_WRITE, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF) };

  EEPROM_WriteEnable();
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, hdr, 3, 100);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, 100);
  CS_HIGH();
  EEPROM_WaitWriteComplete();
}

static void EEPROM_ReadBytes(uint16_t addr, uint8_t *data, uint16_t len)
{
  uint8_t hdr[3] = { CMD_READ, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF) };
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, hdr, 3, 100);
  HAL_SPI_Receive(&hspi1, data, len, 100);
  CS_HIGH();
}

// ---- CAN ----
static void CAN_SendBytes(const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef txh = {0};
  uint32_t mailbox = 0;

  txh.StdId = CAN_TX_STDID;
  txh.IDE   = CAN_ID_STD;
  txh.RTR   = CAN_RTR_DATA;
  txh.DLC   = (len > 8) ? 8 : len;

  HAL_CAN_AddTxMessage(&hcan1, &txh, (uint8_t*)data, &mailbox);
}

// ---- UART ----
static void UART_Log(const char *s)
{
  if (!s) return;
  HAL_UART_Transmit(&huart4, (uint8_t*)s, (uint16_t)strlen(s), 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */
  // CAN 시작
  HAL_CAN_Start(&hcan1);

  // 예시: PMIC Vout을 5000mV로 설정 (데이터시트에 맞게 조정)
  PMIC_SetVout_mV(5000);

  UART_Log("Boot OK\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1) I2C: PMIC Fault 레지스터 읽기 (UV 예시)
    if (PMIC_ReadFaultReg(REG_UV, &g_fault) == HAL_OK) {
      if (g_fault.bits.UV) {
        // Fault 발생 → DTC 활성화 후 EEPROM 저장
        g_dtc.status = 1;
        EEPROM_WriteBytes(0x0000, (uint8_t*)&g_dtc, sizeof(g_dtc));
      } else {
        g_dtc.status = 0;
        EEPROM_WriteBytes(0x0000, (uint8_t*)&g_dtc, sizeof(g_dtc));
      }
    }

    // 2) SPI: EEPROM에서 DTC 다시 읽기
    DTC_t dtc_read = {0};
    EEPROM_ReadBytes(0x0000, (uint8_t*)&dtc_read, sizeof(dtc_read));

    // 3) CAN: DTC 송출 (2+1=3바이트)
    CAN_SendBytes((uint8_t*)&dtc_read, sizeof(dtc_read));

    // 4) UART: 주기 로그
    int n = snprintf((char*)g_uart_logbuf, LOG_BUF_SZ,
                     "System OK | DTC=0x%04X, st=%d\r\n",
                     dtc_read.code, dtc_read.status);
    HAL_UART_Transmit(&huart4, g_uart_logbuf, (uint16_t)n, 100);

    HAL_Delay(1000); // 1초 주기
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
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
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  sConfig.Channel      = ADC_CHANNEL_2;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance             = I2C2;
  hi2c2.Init.ClockSpeed      = 100000;
  hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1     = 0;
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2     = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance               = SPI2;
  hspi2.Init.Mode              = SPI_MODE_MASTER;
  hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi2.Init.NSS               = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
  huart4.Instance        = UART4;
  huart4.Init.BaudRate   = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits   = UART_STOPBITS_1;
  huart4.Init.Parity     = UART_PARITY_NONE;
  huart4.Init.Mode       = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK) { Error_Handler(); }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  // 필요시 NVIC 우선순위 조정
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  // 나머지 스트림은 필요 시 Enable
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  // PB0: EEPROM CS, PB1: 여유, PB2: OD 예시
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;      // CS 포함
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;                 // Open-Drain 예시
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  Error Handler
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
