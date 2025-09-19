/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
 /* ECU 제어 시스템 시퀀스 정리 (전체 구조 설명)
====================================================================================
① MCU Power-On → BootROM → Reset_Handler → SystemInit() → HAL_Init() → Clock Init
② MX_GPIO/DMA/ADC/CAN/I2C/SPI/UART 초기화
③ HAL_CAN_Start() / HAL_CAN_ActivateNotification() 활성화 (수신 인터럽트 준비)
④ SPI EEPROM → EEPROM_ReadDTC() 호출 (이전 DTC 복구)
⑤ FreeRTOS osKernelInitialize()
⑥ FreeRTOS Mutex 생성 (CommMutexHandleHandle)
⑦ FreeRTOS Queue 생성 (CanQueueHandle)
⑧ FreeRTOS Task 생성 (I2CTask → SPITask → CANTask → UARTTask)
⑨ osKernelStart() → 스케줄러 시작
⑩ 각 Task 별 역할 수행:
    - I2CTask: PMIC I2C 0x07 FAULT_STATUS1 레지스터 주기 확인
    - SPITask: EEPROM에 DTC 주기적 백업
    - CANTask: CAN 수신 데이터 Queue 처리 → OBD2/UDS 명령 응답
    - UARTTask: 상태 출력 디버깅
⑪ UV Fault 감지 시 DTC 활성화 → EEPROM 기록
⑫ OBD2, UDS CAN 진단 요청 시 현재 DTC 응답
====================================================================================*/

/*
=========================================================================================================================
Full ECU DTC 관리 시스템 - STM32F413ZHTx 부팅부터 Task 실행까지 상세 동작 시퀀스 설명
=========================================================================================================================

### [0] STM32F413ZHTx BootROM → Reset Flow 상세 시퀀스

① 시스템 전원 인가 (3.3V VDD / VDDIO → 내부 LDO로 CORE 1.2V 공급 시작)

② STM32F413ZHTx 내부 BootROM 실행 (핸드오프 순서)
    - Option Bytes → Boot Pin → 부팅 모드 결정 (일반적으로 Flash 부팅)

③ Reset_Handler() 진입 (Vector Table 기준 초기 핸들러)

④ 초기화 루틴 (CRT Startup) 실행:
   - 초기 MSP (Main Stack Pointer) 설정 (SP ← __StackTop 주소)
   - 벡터 테이블 설정 (SCB->VTOR 설정 → Flash 0x0800 0000)
   - .data 섹션 초기화 (Flash에서 RAM으로 초기 데이터 복사)
   - .bss 섹션 초기화 (RAM의 초기화되지 않은 변수 0으로 클리어)
   - SystemInit() 함수 호출 → 클럭/PLL 설정
   - main() 함수로 분기

=========================================================================================================================

### [1] main() 함수 시스템 초기화

① HAL_Init() → HAL Library 전체 초기화 (SysTick, NVIC, 기본 IRQ, HAL State)

② SystemClock_Config() → 내부 HSI 16MHz 클럭 → SYSCLK 설정
    (PLL 사용 안함 → HSI 직접 SYSCLK 소스로 사용)

③ 각 주변장치 Peripheral 초기화

- MX_GPIO_Init() → 모든 입출력 핀 모드 설정 (SPI/I2C/UART/CAN CS 핀 포함)
- MX_DMA_Init() → DMA 스트림 활성화 (SPI/I2C 연동)
- MX_ADC1_Init() → ADC 설정 (현재 미사용)
- MX_CAN1_Init() → CAN 통신 초기화 (OBD2/UDS)
- MX_I2C1_Init() → PMIC I2C 통신용 초기화 (100kHz)
- MX_SPI1_Init() → EEPROM SPI 통신용 초기화 (EEPROM 25LC256)
- MX_UART4_Init() → 디버깅 UART 설정 (115200bps)

=========================================================================================================================

### [2] EEPROM → DTC 복구

- SPI1 이용 → EEPROM_ReadDTC() 함수 호출
- EEPROM (25LC256) 0x0000 주소에서 DTC Table 복원
- 전원 끊김 후에도 기존 Fault 이력 유지

=========================================================================================================================

### [3] FreeRTOS RTOS 초기화

① osKernelInitialize() 호출 → RTOS 커널 준비
② Mutex 생성: CommMutexHandleHandle
③ Queue 생성: CanQueueHandle (CAN RX 메시지 수신 큐)

④ Task 생성:
- StartDefaultTask (idle 용)
- StartI2CTask (PMIC 상태 모니터링)
- StartSPITask (EEPROM 기록)
- StartCANTask (CAN 명령 파싱)
- StartUARTTask (디버깅)

⑤ osKernelStart() → FreeRTOS 스케줄러 시작

=========================================================================================================================

### [4] RTOS Task 상세 동작 시퀀스

┌────────────────────────────────────────────────────────────────────────────────┐
│ Task Name │ 주기 │ 주요 동작 내용 │ Mutex 사용 │
├───────────┼──────┼──────────────────────────────────────┼─────────────┤
│ I2CTask   │ 500ms│ PMIC 0x07 (FAULT_STATUS1) 읽기 │ I2C 보호 │
│ SPITask   │ 5s   │ EEPROM 주기적 DTC 백업 │ SPI 보호 │
│ CANTask   │ 이벤트│ CAN Queue Polling → 명령 파싱 │ CAN 보호 │
│ UARTTask  │ 1s   │ 디버깅 메시지 출력 │ UART 보호 │
│ defaultTask│ idle│ 기본 유휴 루프 │ (미사용) │
└────────────────────────────────────────────────────────────────────────────────┘

=========================================================================================================================

### [5] PMIC (MP5475GU) UV Fault 감지

① I2CTask 주기마다 I2C1 통해 0x60 주소 → 0x07 레지스터 읽기

② faultReg & 0x01 → UV_FAULT_A (Under Voltage A) 발생 확인

③ 신규 Fault 발생시 DTC 활성화 → EEPROM 기록

※ 반복적으로 같은 Fault가 발생해도 EEPROM에는 1회만 기록 (기록 횟수 보호)

=========================================================================================================================

### [6] CAN 수신 처리 흐름 (Queue 기반)

① CAN 인터럽트 발생 (RX_FIFO0_MSG_PENDING)

② HAL_CAN_RxFifo0MsgPendingCallback() ISR 실행

③ HAL_CAN_GetRxMessage()로 Frame 수신

④ osMessageQueuePut() 통해 CanQueueHandle 큐에 메시지 삽입

⑤ CANTask에서 osMessageQueueGet() → 큐 Polling → 명령 파싱 시작

=========================================================================================================================

### [7] CAN 명령 → OBD2 / UDS DTC 파싱

① OBD2 규격:
- 요청 프레임: 0x7DF → 응답: 0x7E8
- Read DTC: PID 03 (0x43)
- Clear DTC: PID 04 (0x04)

② UDS 규격:
- Read DTC: SID $19 (0x19)
- Clear DTC: SID $14 (0x14)

③ Process_OBD2_Read() / Process_UDS_Read() 호출
- DTC_Table.active 에 따라 응답 구성 후 CAN 송출

④ Process_OBD2_Clear() / Process_UDS_Clear() 호출
- DTC 비활성화 → EEPROM 재기록

=========================================================================================================================

### [8] EEPROM 기록 과정 상세 (SPI1)

① EEPROM_WriteEnable() → WREN 명령 전송 (0x06)

② EEPROM Write 명령 (0x02) + 주소 0x0000 전송

③ DTC_Table 전체 구조체 SPI 전송

④ CS 핀 해제 → 기록 완료

⑤ 재부팅시 EEPROM_ReadDTC() → RAM으로 복구

=========================================================================================================================

### [9] Mutex (FreeRTOS 보호 메커니즘)

- I2C1, SPI1, UART4, CAN1 전송에 대해 Mutex 보호

① osMutexAcquire(CommMutexHandleHandle, osWaitForever)
② HAL API 호출 (통신 수행)
③ osMutexRelease(CommMutexHandleHandle)

※ 통신 자원 공유시 전형적인 RTOS 충돌 방지 기법

=========================================================================================================================
---------------------------------------------------------------------------------------

전체 통신 프로토콜 요약:

PMIC: MP5475GU (I2C: 0x60 << 1), FAULT_STATUS1 0x07
EEPROM: 25LC256 (SPI), 25LC256 명령어 (WREN, READ, WRITE)
CAN Protocol: ISO15765 (OBD2/UDS), 기본 PID/Service 사용 예시
- OBD2: SAE J1979 (PID 03/04 사용)
- UDS: ISO 14229-1 (SID $19 / $14 사용)
DTC Code 예시: 0x1234 (Brake UV Fault)

---------------------------------------------------------------------------------------

Reference (참고 URL):
1️ OBD-II 전체 설명 사이트
 https://www.obd-codes.com/

2️ UDS 프로토콜 기초 설명
 https://vector.com/uds-tutorial
 https://en.wikipedia.org/wiki/Unified_Diagnostic_Services

3️ ISO/SAE 표준 구입 사이트 (정식 국제 표준문서 필요시)
 https://www.iso.org/standard/61088.html (ISO 14229-1)
 https://www.sae.org/standards (SAE J1979, J2012)

https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_03
https://en.wikipedia.org/wiki/OBD-II_PIDs
https://en.wikipedia.org/wiki/Unified_Diagnostic_Services

- UDS: https://en.wikipedia.org/wiki/Unified_Diagnostic_Services
- OBD2: https://en.wikipedia.org/wiki/On-board_diagnostics
- EEPROM 25LC256 Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/25LC256.pdf
- PMIC MP5475GU Datasheet: https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP5475GU/
=======================================================================================
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

void EEPROM_ReadDTC(void);
// --- DTC 데이터 구조 정의 ---
typedef struct {
  uint16_t DTC_Code;              // 고장 코드 (예: C1234)
  char Description[50];           // 설명 문자열
  uint8_t active;                 // 활성화 상태 플래그
} DTC_Table_t;

DTC_Table_t DTC_Table = { 0x1234, "Brake UV Fault", 0 };

// SPI EEPROM 명령어
#define EEPROM_CMD_WREN  0x06
#define EEPROM_CMD_WRITE 0x02
#define EEPROM_CMD_READ  0x03
#define EEPROM_DTC_ADDR  0x0000

// PMIC 주소 및 레지스터
#define PMIC_I2C_ADDR  (0x60 << 1)      // MP5475GU I2C 7bit 주소
#define PMIC_FAULT_STATUS1_REG  0x07    // FAULT_STATUS1 레지스터
/* USER CODE END Includes */

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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2CTask */
osThreadId_t I2CTaskHandle;
const osThreadAttr_t I2CTask_attributes = {
  .name = "I2CTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPITask */
osThreadId_t SPITaskHandle;
const osThreadAttr_t SPITask_attributes = {
  .name = "SPITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanQueue */
osMessageQueueId_t CanQueueHandle;
const osMessageQueueAttr_t CanQueue_attributes = {
  .name = "CanQueue"
};
/* Definitions for CommMutexHandle */
osMutexId_t CommMutexHandleHandle;
const osMutexAttr_t CommMutexHandle_attributes = {
  .name = "CommMutexHandle"
};
/* USER CODE BEGIN PV */

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
void StartDefaultTask(void *argument);
void StartI2CTask(void *argument);
void StartSPITask(void *argument);
void StartCANTask(void *argument);
void StartUARTTask(void *argument);

int main(void)
{
  /* USER CODE BEGIN 1 */
  // MCU 부팅: BootROM → Reset_Handler → SystemInit() → main() 진입
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();  // HAL 라이브러리 초기화

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();  // 시스템 클럭 설정

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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
  // CAN 초기화 후 수신 인터럽트 활성화
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 부팅 시 EEPROM에 저장된 이전 DTC 정보 복구
  EEPROM_ReadDTC();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  // RTOS 커널 초기화

  /* Create the mutex(es) */
  CommMutexHandleHandle = osMutexNew(&CommMutexHandle_attributes);  // 통신 Mutex 생성

  /* Create the queue(s) */
  CanQueueHandle = osMessageQueueNew(8, 8, &CanQueue_attributes);  // CAN 수신 큐 생성

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  I2CTaskHandle = osThreadNew(StartI2CTask, NULL, &I2CTask_attributes);
  SPITaskHandle = osThreadNew(StartSPITask, NULL, &SPITask_attributes);
  CANTaskHandle = osThreadNew(StartCANTask, NULL, &CANTask_attributes);
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  /* Start scheduler */
  osKernelStart();

  /* Infinite loop */
  while (1)
  {
    // 절대 도달하지 않음 (스케줄러가 Task 관리)
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

// --- SPI EEPROM 쓰기 함수 ---
void EEPROM_WriteEnable(void) {
  uint8_t cmd = EEPROM_CMD_WREN;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void EEPROM_WriteDTC(void) {
  uint8_t cmd[3];
  EEPROM_WriteEnable();
  cmd[0] = EEPROM_CMD_WRITE;
  cmd[1] = (EEPROM_DTC_ADDR >> 8) & 0xFF;
  cmd[2] = EEPROM_DTC_ADDR & 0xFF;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, cmd, 3, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&DTC_Table, sizeof(DTC_Table), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void EEPROM_ReadDTC(void) {
  uint8_t cmd[3];
  cmd[0] = EEPROM_CMD_READ;
  cmd[1] = (EEPROM_DTC_ADDR >> 8) & 0xFF;
  cmd[2] = EEPROM_DTC_ADDR & 0xFF;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, cmd, 3, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, (uint8_t*)&DTC_Table, sizeof(DTC_Table), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

// --- CAN 수신 인터럽트 콜백 ---
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  osMessageQueuePut(CanQueueHandle, RxData, 0, 0);
}

// --- OBD2/UDS 응답 처리 ---
void Process_CAN_Response(uint8_t *data) {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t TxData[8] = {0};

  TxHeader.StdId = 0x7E8; // 응답 ID
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  // OBD2 0x43: Read DTCs
  if (data[1] == 0x43) {
    if (DTC_Table.active) {
      TxData[0] = 0x03; TxData[1] = 0x43;
      TxData[2] = (DTC_Table.DTC_Code >> 8) & 0xFF;
      TxData[3] = DTC_Table.DTC_Code & 0xFF;
    } else {
      TxData[0] = 0x01; TxData[1] = 0x43; TxData[2] = 0x00;
    }
  }
  // OBD2 0x04: Clear DTCs
  else if (data[1] == 0x04) {
    DTC_Table.active = 0;
    EEPROM_WriteDTC();
    TxData[0] = 0x01; TxData[1] = 0x44; // 응답
  }
  // UDS 0x19: Read DTCs
  else if (data[1] == 0x19) {
    if (DTC_Table.active) {
      TxData[0] = 0x03; TxData[1] = 0x59; TxData[2] = 0x02;
      TxData[3] = (DTC_Table.DTC_Code >> 8) & 0xFF;
      TxData[4] = DTC_Table.DTC_Code & 0xFF;
    } else {
      TxData[0] = 0x01; TxData[1] = 0x59; TxData[2] = 0x00;
    }
  }
  // UDS 0x14: Clear DTCs
  else if (data[1] == 0x14) {
    DTC_Table.active = 0;
    EEPROM_WriteDTC();
    TxData[0] = 0x02; TxData[1] = 0x54; // 응답
  }
  else
  {
	/* for misra code*/
  }

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 5 */
void StartI2CTask(void *argument) {
  uint8_t faultReg;
  for(;;) {
    osMutexAcquire(CommMutexHandleHandle, osWaitForever);
    HAL_I2C_Mem_Read(&hi2c1, PMIC_I2C_ADDR, PMIC_FAULT_STATUS1_REG, I2C_MEMADD_SIZE_8BIT, &faultReg, 1, HAL_MAX_DELAY);
    if (faultReg & 0x01) {
      if (DTC_Table.active == 0) {
        DTC_Table.active = 1;
        EEPROM_WriteDTC();
      }
    }
    osMutexRelease(CommMutexHandleHandle);
    osDelay(500);
  }
}

void StartSPITask(void *argument) {
  for(;;) {
    osMutexAcquire(CommMutexHandleHandle, osWaitForever);
    EEPROM_WriteDTC();
    osMutexRelease(CommMutexHandleHandle);
    osDelay(5000);
  }
}

void StartCANTask(void *argument) {
  uint8_t rxBuf[8];
  for(;;) {
    if (osMessageQueueGet(CanQueueHandle, rxBuf, NULL, osWaitForever) == osOK) {
      osMutexAcquire(CommMutexHandleHandle, osWaitForever);
      Process_CAN_Response(rxBuf);
      osMutexRelease(CommMutexHandleHandle);
    }
    osDelay(100);
  }
}

void StartUARTTask(void *argument) {
  const char msg[] = "ECU System Running\r\n";
  for(;;) {
    osMutexAcquire(CommMutexHandleHandle, osWaitForever);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    osMutexRelease(CommMutexHandleHandle);
    osDelay(1000);
  }
}
/* USER CODE END 5 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
