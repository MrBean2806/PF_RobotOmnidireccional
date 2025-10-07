/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ADIS16364.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Estructura de la trama recibida por la Nucleo desde el Controlador de Alto Nivel (CAN)
typedef struct{
    float v_x;
    float v_y;
    float w_z;
    uint8_t crc;
} s_Trama_rx;

// Union de la trama recibida por la Nucleo desde el CAN
typedef union{
    s_Trama_rx data;
    char string[16];
} u_Trama_rx;

// Estructura de la trama enviada desde la Nucleo al CAN
typedef struct{
    uint32_t start;
    uint32_t timestamp; // contador que indica el ms en que se hizo la medicion
    float u_m[4];       // velocidad angular en el eje de cada motor
    int16_t a_m[3];       // aceleraciÃ³n lineal medida en cada eje
    int16_t phi_m[3];     // giro en cada eje
    int16_t i_m;       // corriente medida en cada motor
    uint16_t v_bat;
    uint16_t status;
    uint8_t crc; // dejar el crc al final de la trama cuando el CAN utilice la verificacion por CRC
    uint8_t stop;
} s_Trama_tx;

// status: 0 -> deshabilitado, 1 -> habilitado
// Bit  0: status & (0b0000000000000001 << 0) -> Estado de la UART 
// Bit  1: status & (0b0000000000000001 << 1) -> Estado del sensor de corriente
// Bit  2: status & (0b0000000000000001 << 2) -> Estado de la IMU
// Bit  3: status & (0b0000000000000001 << 3) -> Estado de medicion de velocidad
// Bit  4: status & (0b0000000000000001 << 4) -> Estado de
// Bit  5: status & (0b0000000000000001 << 5) -> Estado de
// Bit  6: status & (0b0000000000000001 << 6) -> Estado de
// Bit  7: status & (0b0000000000000001 << 7) -> Estado de
// Bit  8: status & (0b0000000000000001 << 7) -> Estado de
// Bit  9: status & (0b0000000000000001 << 7) -> Estado de
// Bit 10: status & (0b0000000000000001 << 7) -> Estado de
// Bit 11: status & (0b0000000000000001 << 7) -> Estado de
// Bit 12: status & (0b0000000000000001 << 7) -> Estado de
// Bit 13: status & (0b0000000000000001 << 7) -> Estado de
// Bit 14: status & (0b0000000000000001 << 7) -> Estado de
// Bit 15: status & (0b0000000000000001 << 7) -> Estado de

// Union de la trama enviada desde la Nucleo al CAN
typedef union{
    s_Trama_tx data;
    char string[44];
} u_Trama_tx;

// Estructura para enviar las aceleraciones leidas en la tarea StartMeasureAccel() a la tarea StartTxTask()
typedef struct{
	int16_t phi_m[3];
	int16_t a_m[3];
}s_Aceleracion;

// Estructura para enviar las velocidades leidas en la tarea StartMeasureVel() a la tarea StartTxTask()
typedef struct{
	float v_x;
	float v_y;
	float w_z;
}s_Velocidad;


// Estructura para guardar los valores de velocidad de cada rueda
typedef struct{
	float u_1;
	float u_2;
	float u_3;
	float u_4;
}s_Velocidad_ruedas;

enum STATE { START_State = 0, RUN_State = 1 };

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId defaultTaskHandle;
osThreadId TxTaskHandle;
osThreadId RxTaskHandle;
osThreadId CinematicConvHandle;
osThreadId ControlVelHandle;
osThreadId MeasureAccelHandle;
osThreadId MeasureVelHandle;
osSemaphoreId myBinarySemAccelTxHandle;
osSemaphoreId myBinarySemVelTxHandle;
osSemaphoreId myBinarySemVelControlHandle;
osSemaphoreId myBinarySemRxConvHandle;
osSemaphoreId myBinarySemConvControlHandle;
/* USER CODE BEGIN PV */
//unsigned char str[]="Hello World!\r\n";
osMessageQId myQueueRxConvHandle;
osMessageQId myQueueConvControlHandle;
osMessageQId myQueueVelControlHandle;
osMessageQId myQueueVelTxHandle;
osMessageQId myQueueAccelTxHandle;

// Variables para medir la velocidad de cada rueda, por medio de los encoders
uint32_t contador_pasado[4];
uint32_t contador_actual[4] = {0,0,0,0};
uint32_t tiempoPulsos[4] = {0,0,0,0};
float u1_medida=0;
float u2_medida=0;
float u3_medida=0;
float u4_medida=0;
uint32_t contadorVelocidad[4] = {0,0,0,0};
uint32_t velocidadActual[4] = {0,0,0,0};
uint32_t velocidadPasada[4] = {0,0,0,0};

// status
int counter_rx_stop = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTxTask(void const * argument);
void StartRxTask(void const * argument);
void StartCinematicConv(void const * argument);
void StartControlVel(void const * argument);
void StartMeasureAccel(void const * argument);
void StartMeasureVel(void const * argument);

/* USER CODE BEGIN PFP */
int RxFlag = RESET; // Variable que le indica a la tarea StartRxTask() que se leyo un dato en la UART
int state = RUN_State;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	RxFlag = SET;
}
int counter_stop[4] = {0,0,0,0};

//Interrupcion para contar pulsos del encoder

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == Encoder1_Pin){
		if(state == RUN_State){
			counter_stop[0] = 0;
			contador_actual[0] = __HAL_TIM_GET_COUNTER(&htim2);
			if(contador_actual[0] >= contador_pasado[0]){
				tiempoPulsos[0] = contador_actual[0] - contador_pasado[0];
			}else{
				tiempoPulsos[0] = (4294967295 - contador_pasado[0]) + contador_actual[0];
			}

			u1_medida = (60.0 * 72000000.0) / (tiempoPulsos[0] * 20.0);
			contador_pasado[0] = __HAL_TIM_GET_COUNTER(&htim2);
		}
	}
	if(GPIO_Pin == Encoder2_Pin){
		if(state == RUN_State){
			counter_stop[1] = 0;
			contador_actual[1] = __HAL_TIM_GET_COUNTER(&htim2);
			if(contador_actual[1] >= contador_pasado[1]){
				tiempoPulsos[1] = contador_actual[1] - contador_pasado[1];
			}else{
				tiempoPulsos[1] = (4294967295 - contador_pasado[1]) + contador_actual[1];
			}

			u2_medida = (60.0 * 72000000.0) / (tiempoPulsos[1] * 20.0);
			contador_pasado[1] = __HAL_TIM_GET_COUNTER(&htim2);
		}
	}
	if(GPIO_Pin == Encoder3_Pin){
		if(state == RUN_State){
			counter_stop[2] = 0;
			contador_actual[2] = __HAL_TIM_GET_COUNTER(&htim2);
			if(contador_actual[2] >= contador_pasado[2]){
				tiempoPulsos[2] = contador_actual[2] - contador_pasado[2];
			}else{
				tiempoPulsos[2] = (4294967295 - contador_pasado[2]) + contador_actual[2];
			}

			u3_medida = (60.0 * 72000000.0) / (tiempoPulsos[2] * 20.0);
			contador_pasado[2] = __HAL_TIM_GET_COUNTER(&htim2);
		}
	}
	if(GPIO_Pin == Encoder4_Pin){
		if(state == RUN_State){
			counter_stop[3] = 0;
			contador_actual[3] = __HAL_TIM_GET_COUNTER(&htim2);
			if(contador_actual[3] >= contador_pasado[3]){
				tiempoPulsos[3] = contador_actual[3] - contador_pasado[3];
			}else{
				tiempoPulsos[3] = (4294967295 - contador_pasado[3]) + contador_actual[3];
			}

			u4_medida = (60.0 * 72000000.0) / (tiempoPulsos[3] * 20.0);
			contador_pasado[3] = __HAL_TIM_GET_COUNTER(&htim2);
		}
	}
}

// Declaracion de variables para leer la tension de las baterias
int raw_adc_value = 0;
uint8_t flag_adc = 0;
// Funcion que se utiliza cuando el ADC termino de convertir el valor de tension de las baterias
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	raw_adc_value = HAL_ADC_GetValue(&hadc2);
	flag_adc = 1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t crcVal;
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
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 1);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  hcrc.Instance = CRC;

  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 0x07;
  hcrc.Init.InitValue = 0x00;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

//  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
	  Error_Handler();
  }

  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);


  // Inicializo los contadores
  contador_pasado[0] = __HAL_TIM_GET_COUNTER(&htim2);
  contador_pasado[1] = __HAL_TIM_GET_COUNTER(&htim2);
  contador_pasado[2] = __HAL_TIM_GET_COUNTER(&htim2);
  contador_pasado[3] = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySemAccelTx */
  osSemaphoreDef(myBinarySemAccelTx);
  myBinarySemAccelTxHandle = osSemaphoreCreate(osSemaphore(myBinarySemAccelTx), 1);

  /* definition and creation of myBinarySemVelTx */
  osSemaphoreDef(myBinarySemVelTx);
  myBinarySemVelTxHandle = osSemaphoreCreate(osSemaphore(myBinarySemVelTx), 1);

  /* definition and creation of myBinarySemVelControl */
  osSemaphoreDef(myBinarySemVelControl);
  myBinarySemVelControlHandle = osSemaphoreCreate(osSemaphore(myBinarySemVelControl), 1);

  /* definition and creation of myBinarySemRxConv */
  osSemaphoreDef(myBinarySemRxConv);
  myBinarySemRxConvHandle = osSemaphoreCreate(osSemaphore(myBinarySemRxConv), 1);

  /* definition and creation of myBinarySemConvControl */
  osSemaphoreDef(myBinarySemConvControl);
  myBinarySemConvControlHandle = osSemaphoreCreate(osSemaphore(myBinarySemConvControl), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  // Queue RxTask to ConvDirec
  osMessageQDef(myQueueRxConv, sizeof(s_Velocidad), s_Velocidad);
  myQueueRxConvHandle = osMessageCreate(osMessageQ(myQueueRxConv), NULL);

  // Queue ConvDirec to ControlVel
  osMessageQDef(myQueueConvControl, sizeof(s_Velocidad_ruedas), s_Velocidad_ruedas);
  myQueueConvControlHandle = osMessageCreate(osMessageQ(myQueueConvControl), NULL);
  
  // Queue MeasureVel to ControlVel
  osMessageQDef(myQueueVelControl, sizeof(s_Velocidad_ruedas), s_Velocidad_ruedas);
  myQueueVelControlHandle = osMessageCreate(osMessageQ(myQueueVelControl), NULL);

  // Queue MeasureVel to TxTask
  osMessageQDef(myQueueVelTx, sizeof(s_Velocidad_ruedas), s_Velocidad_ruedas);
  myQueueVelTxHandle = osMessageCreate(osMessageQ(myQueueVelTx), NULL);

  // Queue MeasureAccel to TxTask
  osMessageQDef(myQueueAccelTx, sizeof(s_Aceleracion), s_Aceleracion);
  myQueueAccelTxHandle = osMessageCreate(osMessageQ(myQueueAccelTx), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TxTask */
  osThreadDef(TxTask, StartTxTask, osPriorityLow, 0, 128);
  TxTaskHandle = osThreadCreate(osThread(TxTask), NULL);

  /* definition and creation of RxTask */
  osThreadDef(RxTask, StartRxTask, osPriorityAboveNormal, 0, 128);
  RxTaskHandle = osThreadCreate(osThread(RxTask), NULL);

  /* definition and creation of CinematicConv */
  osThreadDef(CinematicConv, StartCinematicConv, osPriorityNormal, 0, 128);
  CinematicConvHandle = osThreadCreate(osThread(CinematicConv), NULL);

  /* definition and creation of ControlVel */
  osThreadDef(ControlVel, StartControlVel, osPriorityHigh, 0, 128);
  ControlVelHandle = osThreadCreate(osThread(ControlVel), NULL);

  /* definition and creation of MeasureAccel */
  osThreadDef(MeasureAccel, StartMeasureAccel, osPriorityBelowNormal, 0, 128);
  MeasureAccelHandle = osThreadCreate(osThread(MeasureAccel), NULL);

  /* definition and creation of MeasureVel */
  osThreadDef(MeasureVel, StartMeasureVel, osPriorityRealtime, 0, 128);
  MeasureVelHandle = osThreadCreate(osThread(MeasureVel), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3600-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 237;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 3600-1;
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
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 237;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 3600-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 237;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3600-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 237;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin Encoder3_Pin Encoder2_Pin Encoder1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|Encoder3_Pin|Encoder2_Pin|Encoder1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Encoder4_Pin */
  GPIO_InitStruct.Pin = Encoder4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Encoder4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
  /* Infinite loop */
  for(;;)
  {
	//   if ( state == RUN_State ){
//		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//   }
	  osDelay(1500);
//	  osDelay(1500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTxTask */
/**
* @brief Function implementing the TxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTxTask */
void StartTxTask(void const * argument)
{
  /* USER CODE BEGIN StartTxTask */
	s_Velocidad_ruedas velocidad_medida;
	s_Aceleracion aceleracion;
	u_Trama_tx trama_tx;
	trama_tx.data.u_m[0]   	= 0.0;
	trama_tx.data.u_m[1]   	= 0.0;
	trama_tx.data.u_m[2]   	= 0.0;
	trama_tx.data.u_m[3]   	= 0.0;
	trama_tx.data.a_m[0]   	= 0;
	trama_tx.data.a_m[1]   	= 0;
	trama_tx.data.a_m[2]   	= 0;
	trama_tx.data.phi_m[0] 	= 0;
	trama_tx.data.phi_m[1] 	= 0;
	trama_tx.data.phi_m[2] 	= 0;
	trama_tx.data.i_m   	= 0;
	trama_tx.data.v_bat    	= 0;
	trama_tx.data.status   	= 0;
	trama_tx.data.timestamp = 0;
	trama_tx.data.start     = '[';
	trama_tx.data.stop      = ']';
	trama_tx.data.crc      	= 0;


//    uint8_t crcTx = 0;


    /////////////////////////////////////////////////////////////////////////////////
    // Sensor de corriente ina219
    /////////////////////////////////////////////////////////////////////////////////
    uint8_t buffer_ina[3];
    int16_t valor_corriente;
    static const uint8_t INA219_ADDR = 0x40 << 1;    // 7-bit address
    static const uint8_t MaximumExpectedCurrent = 2; // Amper
    float CurrentLSB = MaximumExpectedCurrent / 32768.0;
    float Rshunt = 0.1; // ohm
    uint16_t Cal = (uint16_t)(0.04096 / (CurrentLSB * Rshunt));

    // Ajusto el registro de calibracion
    buffer_ina[0] = 0x05;                // Direccion del registro de calibracion
    buffer_ina[1] = (Cal & 0xFF00) >> 8; // MSByte del valor a escribir en el registro
    buffer_ina[2] = Cal & 0xFF;          // LSByte del valor a escribir en el registro
    HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, buffer_ina, 3, 100);

    HAL_I2C_Master_Receive(&hi2c1, INA219_ADDR, buffer_ina, 2, 500);
    valor_corriente = (int16_t)(buffer_ina[1]) << 8 | (buffer_ina[2]);

    // Muevo el puntero para medir corriente
    buffer_ina[0] = 0x04;
    HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, buffer_ina, 1, 100);

    // Inicializo el adc para medir la tension de las baterias
    HAL_ADC_Start_IT(&hadc2);
  /* Infinite loop */
    for(;;)
    {
        if ( state == RUN_State ){
            // Recibo velocidad medida
            if ( xQueueReceive(myQueueVelTxHandle, &velocidad_medida, 0) == pdPASS ){
                trama_tx.data.u_m[0]   = velocidad_medida.u_1;
                trama_tx.data.u_m[1]   = velocidad_medida.u_2;
                trama_tx.data.u_m[2]   = velocidad_medida.u_3;
                trama_tx.data.u_m[3]   = velocidad_medida.u_4;

                trama_tx.data.status |= 0b0000000000001000; // status velocidad medida

                // Devuelvo el semaforo, para que la tarea StartMeasureVel pueda volver a enviar un nuevo dato
                xSemaphoreGive( myBinarySemVelTxHandle ); 
            }
            else
            {
                trama_tx.data.status &= 0b1111111111110111; // status velocidad medida
            }
            

            // Recibo aceleracion medida
            if ( xQueueReceive(myQueueAccelTxHandle, &aceleracion, 0) == pdPASS ){
                //Creo la trama a transmitir
                trama_tx.data.a_m[0]   = aceleracion.a_m[0];
                trama_tx.data.a_m[1]   = aceleracion.a_m[1];
                trama_tx.data.a_m[2]   = aceleracion.a_m[2];
                trama_tx.data.phi_m[0] = aceleracion.phi_m[0];
                trama_tx.data.phi_m[1] = aceleracion.phi_m[1];
                trama_tx.data.phi_m[2] = aceleracion.phi_m[2];

                trama_tx.data.status |= 0b0000000000000100; // status IMU

                // Devuelvo el semaforo, para que la tarea StartMeasureAccel pueda volver a enviar un nuevo dato
                xSemaphoreGive( myBinarySemAccelTxHandle );
            }
            else
            {
                trama_tx.data.status &= 0b1111111111111011; // status IMU
            }
            

            // Guardo tension de las baterias en la trama a enviar
            if(flag_adc){
                trama_tx.data.v_bat = (((raw_adc_value)/4096.0)*3.3 / 0.2332)*1000;
                flag_adc= 0;
                HAL_ADC_Start_IT(&hadc2);
            }

            // Guardo corriente de las baterias en la trama a enviar
            if (HAL_I2C_Master_Receive(&hi2c1, INA219_ADDR, buffer_ina, 2, 500) == HAL_OK){
                valor_corriente = (int16_t)(buffer_ina[0]) << 8 | (buffer_ina[1]);
                valor_corriente = valor_corriente * 0.06104;
                trama_tx.data.i_m = valor_corriente;

                trama_tx.data.status |= 0b0000000000000010; // status sensor de corriente
            }
            else
            {
                trama_tx.data.status &= 0b1111111111111101; // status sensor de corriente
            }
            

            // status uart
            counter_rx_stop = counter_rx_stop + 1;
            if (counter_rx_stop >= 1200)
            {
                trama_tx.data.status &= 0b1111111111111110;
                counter_rx_stop = 1200;
            }
            else
            {
                trama_tx.data.status |= 0b0000000000000001;
            }


            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Calculo CRC. Descomentar las dos lineas posteriores cuando el CAN sea capaz de calcular el CRC al recibir la trama.
            // crcTx = HAL_CRC_Accumulate(&hcrc, (uint8_t *)trama_tx.string , 55);
            // trama_tx.data.crc = crcTx;
            // Recordar que se debe invertir el lugar del crc en la declaracion de la estructura tambien.
            // Ademas, descomentar la declaracion de crcTx antes del bucle for de esta task.
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            //Transmito la trama
            HAL_UART_Transmit_DMA(&huart3, trama_tx.string,sizeof(trama_tx.string));
        }
        osDelay(100); // 200Hz
    }
  /* USER CODE END StartTxTask */
}

/* USER CODE BEGIN Header_StartRxTask */
/**
* @brief Function implementing the RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRxTask */
void StartRxTask(void const * argument)
{
  /* USER CODE BEGIN StartRxTask */
	uint8_t crcVal=0;
	uint8_t crcRx=0;

	u_Trama_rx trama_rx;
	trama_rx.data.v_y = 0.0;
	trama_rx.data.v_x = 0.0;
	trama_rx.data.w_z = 0.0;
	trama_rx.data.crc = 0;
    s_Velocidad velocidad;
    
    // Libero semaforo, para poder tomarlo en esta tarea y volver a poder tomar cada vez que la tarea StartCinematicConv la reciba.
    xSemaphoreGive( myBinarySemRxConvHandle );

    // Activo la recepcion de la trama por UART con DMA.
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)trama_rx.string, 16);

    /* Infinite loop */
    for(;;)
    {
	    if (RxFlag){
              // Si se recibe una trama
		    if ( state == RUN_State ){
			    velocidad.v_x = trama_rx.data.v_x;
			    velocidad.v_y = trama_rx.data.v_y;
			    velocidad.w_z = trama_rx.data.w_z;
			    crcRx = (uint8_t)trama_rx.string[12];
  
			    crcVal = HAL_CRC_Accumulate(&hcrc, (uint8_t *)trama_rx.string, 12);
  
			    //if( crcVal == crcRx){
				    if( xSemaphoreTake( myBinarySemRxConvHandle, ( TickType_t ) 0 ) == pdTRUE ){
					    if(xQueueSend(myQueueRxConvHandle, &velocidad, portMAX_DELAY) == pdPASS){
                            // Debug
					    }
				    }
			    //}

                // Reinicio el registro INIT del modulo del crc para volver a calcular la proxima vez.
                // Si no se realiza, el calculo del crc comienza con el valor final del calculo anterior.
			    hcrc.Instance->INIT = 0x00000000;

                // Reseteo la flag para evitar que se entre nuevamente a este bucle sin datos nuevos recibidos.
			    RxFlag = RESET;

                // Status
                counter_rx_stop = 0;

                HAL_UART_Receive_DMA(&huart3, (uint8_t *)trama_rx.string, 16);
		    }

            ///////////////////////////////////////////////////////////////////////////
            // Seccion para utilizar si se desea que las tareas no comienzan hasta recibir cierto dato.
		    // else if ( state == START_State ){
			//     velocidad.v_x = trama_rx.data.v_x;
			//     velocidad.v_y = trama_rx.data.v_y;
			//     velocidad.w_z = trama_rx.data.w_z;
			//     crcRx = (uint8_t)trama_rx.string[6];
			//     if ( crcRx == 0xEF ){ // ejemplo de dato concreto
			//   	  state = RUN_State;
			//     }
			//     RxFlag = RESET;
		    // }
            ///////////////////////////////////////////////////////////////////////////
	    }

        osDelay(5);
    }
  /* USER CODE END StartRxTask */
}

/* USER CODE BEGIN Header_StartCinematicConv */
/**
* @brief Function implementing the CinematicConv thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCinematicConv */
void StartCinematicConv(void const * argument)
{
  /* USER CODE BEGIN StartCinematicConv */
	float l = 0.15;
	float w = 0.125;
	float r = 0.05;

	s_Velocidad velocidad_recibida;
	s_Velocidad_ruedas velocidad_convertida;

    // Devuelvo semaforo para poder tomarlo en esta tarea. Este semaforo se recibe cada vez que la tarea StartControlVel lo devuelve
	xSemaphoreGive( myBinarySemConvControlHandle );

	/* Infinite loop */
	for(;;)
	{
		if ( state == RUN_State){
			// Recibo las velocidades comandadas por el CAN, desde la tarea StartRxTask
			if ( xQueueReceive(myQueueRxConvHandle, &velocidad_recibida, 0) == pdPASS ){
				// Convierto velocidades del robot en velocidades angulares de cada rueda.
				velocidad_convertida.u_1 = ((1.0/(r*1.0))*((-l-w)*velocidad_recibida.w_z + velocidad_recibida.v_x - velocidad_recibida.v_y));
				velocidad_convertida.u_2 = ((1.0/(r*1.0))*((l+w)*velocidad_recibida.w_z + velocidad_recibida.v_x + velocidad_recibida.v_y));
				velocidad_convertida.u_3 = ((1.0/(r*1.0))*((l+w)*velocidad_recibida.w_z + velocidad_recibida.v_x - velocidad_recibida.v_y));
				velocidad_convertida.u_4 = ((1.0/(r*1.0))*((-l-w)*velocidad_recibida.w_z + velocidad_recibida.v_x + velocidad_recibida.v_y));

                // Aplico conversion de rad/seg a rpm.
				velocidad_convertida.u_1 = (velocidad_convertida.u_1 * 30.0)/3.1415;
				velocidad_convertida.u_2 = (velocidad_convertida.u_2 * 30.0)/3.1415;
				velocidad_convertida.u_3 = (velocidad_convertida.u_3 * 30.0)/3.1415;
                velocidad_convertida.u_4 = (velocidad_convertida.u_4 * 30.0)/3.1415;

                // Devuelvo el semaforo a la tarea StartRxTask, para no entrar en esta parte del codigo hasta no tener un dato nuevo
				xSemaphoreGive( myBinarySemRxConvHandle );

				// Una vez recibidas y convertidas estas velocidades, las envio a la tarea del StartControlVel
				if( xSemaphoreTake( myBinarySemConvControlHandle, ( TickType_t ) 0 ) == pdTRUE ){
					if( xQueueSend(myQueueConvControlHandle, &velocidad_convertida, portMAX_DELAY) == pdPASS ){
                        // Debug
					}
				}
			}
		}
		osDelay(5);
	}
  /* USER CODE END StartCinematicConv */
}

/* USER CODE BEGIN Header_StartControlVel */
/**
* @brief Function implementing the ControlVel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlVel */
void StartControlVel(void const * argument)
{
  /* USER CODE BEGIN StartControlVel */
//	char str[100];
	s_Velocidad_ruedas velocidad_medida;
	s_Velocidad_ruedas velocidad_ref;
	s_Velocidad_ruedas error_velocidad;
	int u_1_duty = 0;
	int u_2_duty = 0;
	int u_3_duty = 0;
	int u_4_duty = 0;

    // Parametros para realizar el control
	float Kp, Ki, Kt, T;
	float entrada_PI[4] = {0.0, 0.0, 0.0, 0.0};
	float salida_PI[4] = {0.0, 0.0, 0.0, 0.0};
	float H[4] = {0.0,0.0, 0.0, 0.0};
	float H_1[4] = {0.0,0.0,0.0,0.0};
	float I[4] = {0.0,0.0,0.0,0.0};
	float P[4] = {0.0,0.0,0.0,0.0};

	T = 1.0/1000.0;
    Kp = 0.00282970945592438;
    // Kp = 4.286;
    Ki = 7.4971179443538;
    // Ki = 306.143;
	Kt = Ki * T/2.0;

    // Inicializo las variables
	velocidad_ref.u_1 = 0.0;
	velocidad_ref.u_2 = 0.0;
	velocidad_ref.u_3 = 0.0;
	velocidad_ref.u_4 = 0.0;

	velocidad_medida.u_1 = 0.0;
	velocidad_medida.u_2 = 0.0;
	velocidad_medida.u_3 = 0.0;
	velocidad_medida.u_4 = 0.0;

    // Obtengo el valor que representa el duty maximo para cada PWM 
    uint16_t maxDuty  = htim1.Instance->ARR + 1;

    // Le asigno a cada timer el valor que deja los motores quietos.
	TIM1->CCR1  = maxDuty/2;
	TIM8->CCR1  = maxDuty/2;
	TIM15->CCR1 = maxDuty/2;
	TIM16->CCR1 = maxDuty/2;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);

    float vel_maxima = 200.0;
//	float aux;
  /* Infinite loop */
  for(;;)
  {
	if (state == RUN_State){
		// Recibo la velocidad de referencia
		if ( xQueueReceive(myQueueConvControlHandle, &velocidad_ref, 0) == pdPASS ){
			xSemaphoreGive(myBinarySemConvControlHandle);
		}


		// Recibo la velocidad medida
		if ( xQueueReceive(myQueueVelControlHandle, &velocidad_medida, 0) == pdPASS ){
            xSemaphoreGive(myBinarySemVelControlHandle);
		}

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////// Rueda 1 ///////////////////////
            //////////////////////////////////////////////////

            /////////// Sentido de giro positivo /////////////
            if (velocidad_ref.u_1 >= 0.0)
            {
                error_velocidad.u_1 = velocidad_ref.u_1 - velocidad_medida.u_1;
	        }
	        /////////// Sentido de giro negativo /////////////
	        else
	        {
	            error_velocidad.u_1 = -(velocidad_ref.u_1) - velocidad_medida.u_1;
	        }
	        // A partir de acá el control no distingue entre vel_ref positiva o negativa
	        entrada_PI[0] = error_velocidad.u_1;
	        // Bloque P:
	        P[0] = entrada_PI[0] * Kp;
	        // Anti-windup -> el acumulador (integrador del PI) solo funcionará cuando el error de velocidad sea menor a 50rpm
	        if ((error_velocidad.u_1 < 50.0) && (error_velocidad.u_1 > -50.0))
	            H[0] = entrada_PI[0] + H_1[0];
	        else
	        {
	            H[0] = 0.0;
	            H_1[0] = 0.0;
	        }
	        // Bloque I:
	        I[0] = Kt * (H[0] + H_1[0]);
	        // Sumo la parte proporcional e integral para obtener el PI completo
	        salida_PI[0] = P[0] + I[0];
	        H_1[0] = H[0];

            if (velocidad_ref.u_1 >= 0.0)
                salida_PI[0] = velocidad_ref.u_1 + salida_PI[0];
            else
                salida_PI[0] = velocidad_ref.u_1 - salida_PI[0];

            // limitador
            if (salida_PI[0] < -vel_maxima)
                salida_PI[0] = -vel_maxima;
            if (salida_PI[0] > vel_maxima)
                salida_PI[0] = vel_maxima;

            // Conversion: velocidad en rpm [(-195rpm)-(+195rpm)] -> duty [0-maxDuty]
            u_1_duty = (salida_PI[0] / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2.0);
//            u_1_duty = (velocidad_ref.u_1 / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2.0);
            if (((float)u_1_duty / (float)maxDuty < 0.6) && ((float)u_1_duty / (float)maxDuty > 0.4))
            {
                u_1_duty = maxDuty/2;
	            H_1[0] = 0.0;
            }
            else if ((velocidad_ref.u_1 < 15.0) && (velocidad_ref.u_1 > -15.0))
			{
				u_1_duty = maxDuty/2;
	            H_1[0] = 0.0;
			}
            
            TIM8->CCR1 = u_1_duty;
            //////////////////////////////////////////////////
            //////////////// Fin rueda 1 /////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////// Rueda 2 ///////////////////////
            //////////////////////////////////////////////////

            /////////// Sentido de giro positivo /////////////
            if (velocidad_ref.u_2 >= 0.0)
            {
                error_velocidad.u_2 = velocidad_ref.u_2 - velocidad_medida.u_2;
            }
            /////////// Sentido de giro negativo /////////////
            else
            {
                error_velocidad.u_2 = -(velocidad_ref.u_2) - velocidad_medida.u_2;
            }
            // A partir de acá el control no distingue entre vel_ref positiva o negativa
            entrada_PI[1] = error_velocidad.u_2;
            // Bloque P:
            P[1] = entrada_PI[1] * Kp;
            // Anti-windup -> el acumulador (integrador del PI) solo funcionará cuando el error de velocidad sea menor a 50rpm
            if ((error_velocidad.u_2 < 50.0) && (error_velocidad.u_2 > -50.0))
                H[1] = entrada_PI[1] + H_1[1];
            else
            {
                H[1] = 0.0;
                H_1[1] = 0.0;
            }
            // Bloque I:
            I[1] = Kt * (H[1] + H_1[1]);
            // Sumo la parte proporcional e integral para obtener el PI completo
            salida_PI[1] = P[1] + I[1];
            H_1[1] = H[1];


            if (velocidad_ref.u_2 >= 0.0)
                salida_PI[1] = velocidad_ref.u_2 + salida_PI[1];
            else
                salida_PI[1] = velocidad_ref.u_2 - salida_PI[1];

            // limitador
            if (salida_PI[1] < -vel_maxima)
                salida_PI[1] = vel_maxima;
            if (salida_PI[1] > vel_maxima)
                salida_PI[1] = vel_maxima;

            // Conversion: velocidad en rpm [(-195rpm)-(+195rpm)] -> duty [0-maxDuty]
            u_2_duty = (salida_PI[1] / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2.0);
//            u_2_duty = (velocidad_ref.u_2 / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2.0);
            if (((float)u_2_duty / (float)maxDuty < 0.6) && ((float)u_2_duty / (float)maxDuty > 0.4))
            {
                u_2_duty = maxDuty / 2;
	            H_1[1] = 0.0;
            }

            else if ((velocidad_ref.u_2 < 15.0) && (velocidad_ref.u_2 > -15.0))
            {
            	u_2_duty = maxDuty / 2;
	            H_1[1] = 0.0;
            }

            TIM16->CCR1 = u_2_duty;
            //////////////////////////////////////////////////
            //////////////// Fin rueda 2 /////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////// Rueda3 ///////////////////////
            //////////////////////////////////////////////////

            /////////// Sentido de giro positivo /////////////
            if (velocidad_ref.u_3 >= 0.0)
            {
                error_velocidad.u_3 = velocidad_ref.u_3 - velocidad_medida.u_3;
            }
            /////////// Sentido de giro negativo /////////////
            else
            {
                error_velocidad.u_3 = -(velocidad_ref.u_3) - velocidad_medida.u_3;
            }
            // A partir de acá el control no distingue entre vel_ref positiva o negativa
            entrada_PI[2] = error_velocidad.u_3;
            // Bloque P:
            P[2] = entrada_PI[2] * Kp;
            // Anti-windup -> el acumulador (integrador del PI) solo funcionará cuando el error de velocidad sea menor a 50rpm
            if ((error_velocidad.u_3 < 50.0) && (error_velocidad.u_3 > -50.0))
                H[2] = entrada_PI[2] + H_1[2];
            else
            {
                H[2] = 0.0;
                H_1[2] = 0.0;
            }
            // Bloque I:
            I[2] = Kt * (H[2] + H_1[2]);
            // Sumo la parte proporcional e integral para obtener el PI completo
            salida_PI[2] = P[2] + I[2];
            H_1[2] = H[2];

            if (velocidad_ref.u_3 >= 0.0)
                salida_PI[2] = velocidad_ref.u_3 + salida_PI[2];
            else
                salida_PI[2] = velocidad_ref.u_3 - salida_PI[2];

            // limitador
            if (salida_PI[2] < -vel_maxima)
                salida_PI[2] = -vel_maxima;
            if (salida_PI[2] > vel_maxima)
                salida_PI[2] = vel_maxima;

            // Conversion: velocidad en rpm [(-195rpm)-(+195rpm)] -> duty [0-maxDuty]
            u_3_duty = (salida_PI[2] / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2.0);
//            u_3_duty = (velocidad_ref.u_3 / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2./0);
            if (((float)u_3_duty / (float)maxDuty < 0.6) && ((float)u_3_duty / (float)maxDuty > 0.4))
            {
                u_3_duty = maxDuty / 2;
	            H_1[2] = 0.0;
            }
            else if ((velocidad_ref.u_3 < 15.0) && (velocidad_ref.u_3 > -15.0))
            {
            	u_3_duty = maxDuty / 2;
	            H_1[2] = 0.0;
            }
            TIM15->CCR1 = u_3_duty;
            //////////////////////////////////////////////////
            //////////////// Fin rueda 3 /////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////// Rueda 4 ///////////////////////
            //////////////////////////////////////////////////

            /////////// Sentido de giro positivo /////////////
            if (velocidad_ref.u_4 >= 0.0)
            {
                error_velocidad.u_4 = velocidad_ref.u_4 - velocidad_medida.u_4;
            }
            /////////// Sentido de giro negativo /////////////
            else
            {
                error_velocidad.u_4 = -(velocidad_ref.u_4) - velocidad_medida.u_4;
            }
            // A partir de acá el control no distingue entre vel_ref positiva o negativa
            entrada_PI[3] = error_velocidad.u_4;
            // Bloque P:
            P[3] = entrada_PI[3] * Kp;
            // Anti-windup -> el acumulador (integrador del PI) solo funcionará cuando el error de velocidad sea menor a 50rpm
            if ((error_velocidad.u_4 < 50.0) && (error_velocidad.u_4 > -50.0))
                H[3] = entrada_PI[3] + H_1[3];
            else
            {
                H[3] = 0.0;
                H_1[3] = 0.0;
            }
            // Bloque I:
            I[3] = Kt * (H[3] + H_1[3]);
            // Sumo la parte proporcional e integral para obtener el PI completo
            salida_PI[3] = P[3] + I[3];
            H_1[3] = H[3];

            if (velocidad_ref.u_4 >= 0.0)
                salida_PI[3] = velocidad_ref.u_4 + salida_PI[3];
            else
                salida_PI[3] = velocidad_ref.u_4 - salida_PI[3];

            // limitador
            if (salida_PI[3] < -vel_maxima)
                salida_PI[3] = -vel_maxima;
            if (salida_PI[3] > vel_maxima)
                salida_PI[3] = vel_maxima;

            // Conversion: velocidad en rpm [(-195rpm)-(+195rpm)] -> duty [0-maxDuty]
            u_4_duty = (salida_PI[3] / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2.0);
//            u_4_duty = (velocidad_ref.u_4 / vel_maxima) * (maxDuty / 2.0) + (maxDuty / 2.0);
            if (((float)u_4_duty / (float)maxDuty < 0.6) && ((float)u_4_duty / (float)maxDuty > 0.4))
            {
                u_4_duty = maxDuty / 2;
	            H_1[3] = 0.0;
            }
            else if ((velocidad_ref.u_4 < 15.0) && (velocidad_ref.u_4 > -15.0))
            {
            	u_4_duty = maxDuty / 2;
	            H_1[3] = 0.0;
            }
            TIM1->CCR1 = u_4_duty;
            //////////////////////////////////////////////////
            //////////////// Fin rueda 4 /////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////
        }

        ///////////////////////////////////////////////////////////////////////////
        // Seccion para utilizar si se desea que las tareas no comienzan hasta recibir cierto dato.
        // else if ( state == START_State ){
        //     entrada_PI[0] = salida_PI[0] = H[0] = H_1[0] = I[0] = P[0] = 0;
        //     entrada_PI[1] = salida_PI[1] = H[1] = H_1[1] = I[1] = P[1] = 0;
        //     entrada_PI[2] = salida_PI[2] = H[2] = H_1[2] = I[2] = P[2] = 0;
        //     entrada_PI[3] = salida_PI[3] = H[3] = H_1[3] = I[3] = P[3] = 0;
        // }
        ///////////////////////////////////////////////////////////////////////////

        osDelay(1);
  }
  /* USER CODE END StartControlVel */
}

/* USER CODE BEGIN Header_StartMeasureAccel */
/**
* @brief Function implementing the MeasureAccel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMeasureAccel */
void StartMeasureAccel(void const * argument)
{
  /* USER CODE BEGIN StartMeasureAccel */
	s_Aceleracion aceleracion;
//	char str[100];
	int16_t valor;
	uint8_t data_tx[2] ={0,0};
	uint8_t data_rx[2] ={0,0};
//	uint8_t data_aux[2]={0,0};

    ////////////////////
    // Configuracion de la imu: reiniciar, leer los valores actuales y aplicarle un offset a cada parametro.
    // ACLARACION: Si el MSB = 0 de la trama, se realiza lectura. Si MSB = 1, se realiza escritura.
    ////////////////////

    // Reseteo. Setear GLOB_CMD[1] = 1.
    data_tx[0] = 0xBE;     // byte MAS significativo del DIN para realizar "Restoring Factory Calibration"
    data_tx[1] = 0x02;     // byte MENOS significativo del DIN para realizar "Restoring Factory Calibration"
    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET); // Habilitar el chip select
    HAL_SPI_Transmit(&hspi1, data_tx, 2, 500); // Envio comando de reseteo
    HAL_Delay(100); // Espero los 50 ms que tarda el sensor en terminar el reseteo
    
    // Calibracion automatica. Setear GLOB_CMD[0] = 1.
    data_tx[0] = 0xBE;     // byte MAS significativo del DIN para realizar "Gyroscope Automatic Bias Null Calibration	"
    data_tx[1] = 0x01;     // byte MENOS significativo del DIN para realizar "Gyroscope Automatic Bias Null Calibration"
    HAL_SPI_Transmit(&hspi1, data_tx, 2, 500); // Envio comando de reseteo
    HAL_Delay(100); // Espero los 50 ms que tarda el sensor en terminar el reseteo

    // declaracion de bandera
    uint8_t flag_adis16364_error = 0;

    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET); // Deshabilitar el chip select

    xSemaphoreGive( myBinarySemAccelTxHandle );
  /* Infinite loop */
  for(;;)
  {
	  if ( state == RUN_State){
          // Leo rotaciones
          // Leo XGYRO_OUT
        data_tx[0] = ADIS16364_XGYRO_OUT;
        HAL_SPI_Transmit(&hspi1, data_tx, 2, 200);
        if (HAL_SPI_Receive(&hspi1, data_rx, 2, 200) == HAL_OK)
        {
            valor = (data_rx[0] & 0x3F) << 8 | (data_rx[1]);
            valor = (int16_t)(valor | (valor & 0x2000) << 1 | (valor & 0x2000) << 2);
            
            // Guardo valor en la estructura de aceleracion.
            aceleracion.phi_m[0] = valor;
        }
        else
        {
            flag_adis16364_error = 1;
        }

        // Leo YGYRO_OUT
        data_tx[0] = ADIS16364_YGYRO_OUT;
        HAL_SPI_Transmit(&hspi1, data_tx, 2, 200);
        if (HAL_SPI_Receive(&hspi1, data_rx, 2, 200) == HAL_OK){
            valor = (data_rx[0] & 0x3F) << 8 | (data_rx[1]);
            valor = (int16_t)(valor | (valor & 0x2000) << 1 | (valor & 0x2000) << 2);

            // Guardo valor en la estructura de aceleracion.
            aceleracion.phi_m[1] = valor;
        }
        else
        {
            flag_adis16364_error = 1;
        }

        // Leo ZGYRO_OUT
        data_tx[0] = ADIS16364_ZGYRO_OUT;
        HAL_SPI_Transmit(&hspi1, data_tx, 2, 200);
        if(HAL_SPI_Receive(&hspi1, data_rx, 2, 200) == HAL_OK)
        {
            valor = (data_rx[0] & 0x3F) << 8 | (data_rx[1]);
            valor = (int16_t)(valor | (valor & 0x2000) << 1 | (valor & 0x2000) << 2);

            // Guardo valor en la estructura de aceleracion.
            aceleracion.phi_m[2] = valor;
        }
        else
        {
            flag_adis16364_error = 1;
        }

        // Leo XACCL_OUT
        data_tx[0] = ADIS16364_XACCL_OUT;
        HAL_SPI_Transmit(&hspi1, data_tx, 2, 200);
        if(HAL_SPI_Receive(&hspi1, data_rx, 2, 200) == HAL_OK)
        {
            valor = (data_rx[0] & 0x3F) << 8 | (data_rx[1]);
            valor = (int16_t)(valor | (valor & 0x2000) << 1 | (valor & 0x2000) << 2);

            // Guardo valor en la estructura de aceleracion.
            aceleracion.a_m[0] = valor;
        }
        else
        {
            flag_adis16364_error = 1;
        }

        // Leo YACCL_OUT
        data_tx[0] = ADIS16364_YACCL_OUT;
        HAL_SPI_Transmit(&hspi1, data_tx, 2, 200);
        if(HAL_SPI_Receive(&hspi1, data_rx, 2, 200) == HAL_OK)
        {
            valor = (data_rx[0] & 0x3F) << 8 | (data_rx[1]);
            valor = (int16_t)(valor | (valor & 0x2000) << 1 | (valor & 0x2000) << 2);

            // Guardo valor en la estructura de aceleracion.
            aceleracion.a_m[1] = valor;
        }
        else
        {
            flag_adis16364_error = 1;
        }

        // Leo ZACCL_OUT
        data_tx[0] = ADIS16364_ZACCL_OUT;
        HAL_SPI_Transmit(&hspi1, data_tx, 2, 200);
        if (HAL_SPI_Receive(&hspi1, data_rx, 2, 200) == HAL_OK)
        {
            valor = (data_rx[0] & 0x3F) << 8 | (data_rx[1]);
            valor = (int16_t)(valor | (valor & 0x2000) << 1 | (valor & 0x2000) << 2);

            // Guardo valor en la estructura de aceleracion.
            aceleracion.a_m[2] = valor;
        }
        else
        {
            flag_adis16364_error = 1;
        }

        if (flag_adis16364_error == 0){
            if( xSemaphoreTake( myBinarySemAccelTxHandle, ( TickType_t ) 0 ) == pdTRUE )
            {
                //Envio las aceleraciones a la tarea de transmitir valores medidos
                if( xQueueSend(myQueueAccelTxHandle, &aceleracion, portMAX_DELAY) == pdPASS ){
                    // Debug
                }
            }
        }
	  }
    flag_adis16364_error = 0;
    osDelay(5);
  }
  /* USER CODE END StartMeasureAccel */
}

/* USER CODE BEGIN Header_StartMeasureVel */
/**
* @brief Function implementing the MeasureVel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMeasureVel */
void StartMeasureVel(void const * argument)
{
  /* USER CODE BEGIN StartMeasureVel */
	s_Velocidad_ruedas vel_medida;
	int STOP_FLAG[4] = {0,0,0,0};
	vel_medida.u_1 = 0.0;
	vel_medida.u_2 = 0.0;
	vel_medida.u_3 = 0.0;
	vel_medida.u_4 = 0.0;
//	char str[100];
	xSemaphoreGive( myBinarySemVelControlHandle );
	xSemaphoreGive( myBinarySemVelTxHandle );
  /* Infinite loop */
  for(;;)
  {
	if ( state == RUN_State){
		counter_stop[0] = counter_stop[0] + 1;
		if(counter_stop[0]>=1000){
			STOP_FLAG[0] = 1;
			counter_stop[0] = 1000;
		}
		else
			STOP_FLAG[0] = 0;

		counter_stop[1] = counter_stop[1] + 1;
		if(counter_stop[1]>=1000){
			STOP_FLAG[1] = 1;
			counter_stop[1] = 1000;
		}
		else
			STOP_FLAG[1] = 0;

		counter_stop[2] = counter_stop[2] + 1;
		if(counter_stop[2]>=1000){
			STOP_FLAG[2] = 1;
			counter_stop[2] = 1000;
		}
		else
			STOP_FLAG[2] = 0;

		counter_stop[3] = counter_stop[3] + 1;
		if(counter_stop[3]>=1000){
			STOP_FLAG[3] = 1;
			counter_stop[3] = 1000;
		}
		else
			STOP_FLAG[3] = 0;

		// Guarda velocidad de cada rueda 
		if (STOP_FLAG[0] == 1)
			vel_medida.u_1 = 0.0;
		else
			vel_medida.u_1 = u1_medida;

		if (STOP_FLAG[1] == 1)
			vel_medida.u_2 = 0.0;
		else
			vel_medida.u_2 = u2_medida;

		if (STOP_FLAG[2] == 1)
			vel_medida.u_3 = 0.0;
		else
			vel_medida.u_3 = u3_medida;

		if (STOP_FLAG[3] == 1)
			vel_medida.u_4 = 0.0;
		else
			vel_medida.u_4 = u4_medida;


		// Intento tomar el semaforo
		if( xSemaphoreTake( myBinarySemVelControlHandle, ( TickType_t ) 0 ) == pdTRUE )
		{
			// Envio velocidad a la tarea de control de motores
			if( xQueueSend(myQueueVelControlHandle, &vel_medida, portMAX_DELAY) == pdPASS ){
                // Debug
			}
		}

		// Intento tomar el semaforo
		if( xSemaphoreTake( myBinarySemVelTxHandle, ( TickType_t ) 0 ) == pdTRUE )
		{
			// Envio velocidad a la tarea que transmite los valores
			if( xQueueSend(myQueueVelTxHandle, &vel_medida, portMAX_DELAY) == pdPASS ){
                // Debug
			}
		}


	  }
	  osDelay(1);

  }
  /* USER CODE END StartMeasureVel */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
