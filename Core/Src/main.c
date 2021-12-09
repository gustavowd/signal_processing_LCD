/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "lvgl/lvgl.h"

#include "hal_stm_lvgl/tft/tft.h"
#include "hal_stm_lvgl/touchpad/touchpad.h"

#include "arm_const_structs.h"
#include "arm_math.h"

#include "levmarq.h"
#include "serial.h"
#include <stdio.h>
#include <stdlib.h>
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

/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
osThreadId_t CalculationTaskHandle;
const osThreadAttr_t CalculationTask_attributes = {
  .name = "CalculationTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);

void CalculationTask(void *argument);

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim8;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  /* USER CODE BEGIN 2 */
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LCDTask */
  LCDTaskHandle = osThreadNew(StartDefaultTask, NULL, &LCDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  CalculationTaskHandle = osThreadNew(CalculationTask, NULL, &CalculationTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 14062;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

// Declares a queue structure for the UART
xQueueHandle qADC;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	uint32_t buf = 2;
	signed portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	xQueueSendToBackFromISR(qADC, &buf, &pxHigherPriorityTaskWoken);
	if (pxHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD();
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	uint32_t buf = 1;
	signed portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	xQueueSendToBackFromISR(qADC, &buf, &pxHigherPriorityTaskWoken);
	if (pxHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD();
	}
}

uint16_t adcBuffer[3072];
uint16_t Buffer1[1536];
uint16_t Buffer2[1536];

#define N_MEASUREMENTS_COS 3072
#define N_MEASUREMENTS 395
#define N_PARAMS 3

float t_data[N_MEASUREMENTS] = { 21.6,  22. ,  22.4,  22.4,  22.5,  22.4,  22.2,  22.3,  22.4,\
                                  22.1,  21.8,  21.6,  22.1,  22. ,  21.7,  21.7,  21.5,  21.5,\
                                  21.4,  21.6,  21.7,  21.8,  21.9,  22. ,  21.9,  22.2,  22.3,\
                                  22.3,  22.4,  22.4,  22.4,  23. ,  23.6,  24.6,  24.3,  24.3,\
                                  24.2,  25. ,  25. ,  25.2,  25.4,  26. ,  26.3,  26.3,  26. ,\
                                  26.1,  25.9,  25.7,  26.1,  26. ,  26.3,  26.4,  26.4,  26.4,\
                                  26.5,  26.8,  26.8,  26.9,  27.2,  27.8,  27.9,  28.2,  28.1,\
                                  28.1,  28. ,  28. ,  28.3,  28.4,  28.8,  29.7,  30. ,  30.1,\
                                  29.8,  30.4,  30.9,  30.6,  30.7,  30.6,  30.3,  30.4,  30.8,\
                                  31.3,  31.3,  31.4,  31.2,  31.4,  31.5,  31.6,  31.8,  32.3,\
                                  32.2,  32.7,  32.4,  32.3,  32.7,  33. ,  33. ,  33.1,  33.2,\
                                  33.3,  33.6,  34. ,  33.9,  34.1,  34.2,  34.4,  34.7,  35.1,\
                                  35.2,  34.9,  35.2,  35.3,  35.1,  35.3,  36. ,  35.9,  35.9,\
                                  36.4,  36.9,  36.5,  36.4,  36.9,  36.9,  36.9,  37.4,  37. ,\
                                  36.9,  36.4,  37. ,  37.1,  37. ,  37.1,  37.1,  37.6,  37.8,\
                                  38. ,  38.7,  39. ,  39.2,  39.5,  40.1,  40.4,  40.6,  40.2,\
                                  40.1,  40.3,  40.4,  40.7,  40.9,  40.5,  40.7,  41.5,  41.4,\
                                  41.3,  40.7,  40.5,  40.8,  41.2,  41.2,  41. ,  41.3,  41.5,\
                                  41.7,  42.1,  42.1,  42.3,  42.2,  41.8,  42. ,  42.4,  42.6,\
                                  42.6,  42.3,  42.4,  42.7,  43.1,  42.9,  42.8,  42.8,  43.1,\
                                  43.4,  44. ,  43.9,  43.9,  43.8,  43.9,  44.4,  44.6,  45.1,\
                                  45.2,  45.2,  45.2,  45.4,  45.7,  45.3,  45.1,  45.1,  45.6,\
                                  45.9,  45.9,  45.9,  46. ,  46.2,  46.2,  46.3,  46.6,  46.9,\
                                  46.9,  47.2,  47.6,  47.5,  47.7,  47.6,  47.7,  48. ,  47.8,\
                                  47.9,  48. ,  48.2,  48. ,  48. ,  48.1,  48.6,  48.6,  49.2,\
                                  48.9,  48.8,  49.2,  49.4,  49.2,  49. ,  49.3,  49.8,  50.3,\
                                  50.7,  50.9,  50.8,  50.6,  50.4,  50.7,  50.9,  51. ,  51.4,\
                                  51.4,  51.8,  51.8,  51.6,  52.3,  52.7,  53.3,  53.1,  54. ,\
                                  53.3,  53.3,  53.6,  53.2,  53. ,  53.1,  53.4,  53.5,  53.6,\
                                  53.9,  53.7,  53.9,  53.9,  52.8,  53.1,  53.1,  53.3,  53.4,\
                                  53.6,  54. ,  54.3,  54.2,  54.4,  54.7,  54.6,  56.5,  56.4,\
                                  55.7,  55.8,  55.9,  56.2,  56.2,  56.3,  56.3,  56.5,  56.9,\
                                  57. ,  57.4,  57.9,  58.2,  57.6,  57.5,  57.9,  58.4,  58.9,\
                                  59.1,  58.4,  59. ,  62.4,  57.6,  56.6,  58.6,  59. ,  59. ,\
                                  59. ,  59.3,  59.5,  59. ,  59.4,  59.2,  59.1,  60. ,  59.8,\
                                  60.3,  60.8,  60.5,  60.4,  61.5,  60.7,  60.7,  61. ,  60.9,\
                                  61. ,  61.3,  61.7,  61.5,  61.4,  61.8,  61.9,  62.6,  62.3,\
                                  62.1,  62. ,  62.3,  62.4,  62.4,  61.7,  62.7,  63.2,  62.8,\
                                  62.9,  63. ,  63.6,  63.9,  63.3,  63.7,  63.5,  63.3,  63.9,\
                                  64. ,  63.8,  63.9,  64.5,  64.3,  63.8,  63.5,  64. ,  64.3,\
                                  65.2,  65.7,  65.9,  65.6,  64.7,  65. ,  65.1,  65.8,  66.2,\
                                  66.6,  66.3,  66. ,  65.5,  66.2,  66.4,  66.8,  67.4,  67.3,\
                                  67. ,  66.8,  66.4,  67.3,  66.9,  67.6,  72.1,  66.6,  66.2,\
                                  67.1,  70. ,  68.8,  68.1,  67.8,  67.3,  67.4,  66.2 };

float params[N_PARAMS] = {200, 20, 0.001}; // Initial values of parameters

float params_cos[N_PARAMS] = {0.8, 370.0, 3.0};
float params_cos2[N_PARAMS] = {0.8, 370.0, 3.0};

LMstat lmstat;

/* @brief   Function, describes Newton law of heating/cooling
 *
 * @usage   par[0] - temperature of heater,
 *          par[1] - initial temperature of water,
 *          par[2] - heat transmission coefficient
 *
 * @par     input of Newton Law:
 * @x       samplenumber
 * @fdata   additional data, not used
 *
 * @return  temperature at the time x
 */
float newton_func(float *par, int x, void *fdata)
{
    return par[0] + (par[1] - par[0]) * expf( -par[2]*x);
}

/*
 * @brief   Gradient function for Newton law of heating
 */
void gradient(float *g, float *par, int x, void *fdata)
{
    g[0] = 1.0 - expf(-par[2] * x);
    g[1] = expf(-par[2] * x);
    g[2] = -x * (par[1] - par[0]) * expf(-par[2] * x);
}

float cos_func(float *par, int x, void *fdata){
    return par[0] * cosf(par[1] * 0.00006510417 * x + par[2]);
}

void gradient_cos(float *g, float *par, int x, void *fdata){
    g[0] = cosf(par[1] * 0.00006510417 * x + par[2]);
    g[1] = -x * par[0] * 0.00006510417 * sinf(par[1] * x + par[2]);
    g[2] = -par[0] * sinf(par[1] * 0.00006510417 * x + par[2]);
}

/*
 * @brief  Function for prediction of time, when target temperature will be reached
 *
 * @par    Parameters from Newton equation
 * @temp   Target temperature
 * @return Number of sample
 */
int temp_to_time(float *par, float temp)
{
    return -(1/par[2]) * logf((temp - par[0])/(par[1] - par[0]));
}


#include "levmar.h"
#define DBL_RAND_MAX (float)(RAND_MAX)
#define INIT_RANDOM(seed) srandom((int)xTaskGetTickCount()) // seed unused
//#define INIT_RANDOM(seed) srandom((int)GETPID()) // seed unused

/* Gaussian noise with mean m and variance s, uses the Box-Muller transformation */
float gNoise(float m, float s)
{
	float r1, r2, val;

  r1=((float)random())/DBL_RAND_MAX;
  r2=((float)random())/DBL_RAND_MAX;

  val=sqrtf(-2.0*logf(r1))*cosf(2.0*M_PI*r2);

  val=s*val+m;

  return val;
}

/* model to be fitted to measurements: x_i = p[0]*exp(-p[1]*i) + p[2], i=0...n-1 */
void expfunc(float *p, float *x, int m, int n, void *data)
{
register int i;

  for(i=0; i<n; ++i){
    x[i]=p[0]*expf(-p[1]*i) + p[2];
  }
}

float signal_y2[N_MEASUREMENTS_COS];
void cos_func2(float *p, float *x, int m, int n, void *data){
	register int i;
    float Ts = *((float *)data);
	  for(i=0; i<n; ++i){
	    x[i]= p[0] * cosf(p[1]*Ts*i + p[2]);
	  }
}

void jaccosfunc(float *p, float *jac, int m, int n, void *data)
{
register int i, j;

  /* fill Jacobian row by row */
  for(i=j=0; i<n; ++i){
    jac[j++]=cosf(p[1]*i);
    jac[j++]=p[0]*i*cosf(p[1]*i);
    jac[j++]=1.0;
  }
}

/* Jacobian of expfunc() */
void jacexpfunc(float *p, float *jac, int m, int n, void *data)
{
register int i, j;

  /* fill Jacobian row by row */
  for(i=j=0; i<n; ++i){
    jac[j++]=expf(-p[1]*i);
    jac[j++]=-p[0]*i*expf(-p[1]*i);
    jac[j++]=1.0;
  }
}

#define CONS_T_AM  0.00006510416666666657f    // Ts = 1/15360

float lm_func_cos(float *p, int x, void *dt){
	// f = A*cos(wn*Ts*x+phi)
    return p[0]*arm_cos_f32(p[1]*(float)CONS_T_AM*x+p[2]);
}

void lm_grad_cos(float *g, float *p, int x, void *dt){
	/*
		df/d(A)   = cos(wn*Ts*x+phi)
		df/d(wn)  = -Ts*x*A*sin(wn*Ts*x+phi)
		df/d(phi) = -A*sin(wn*Ts*x+phi)
	*/
    g[0] = arm_cos_f32(p[1] * (float)CONS_T_AM * x + p[2]);
    g[1] = -(float)CONS_T_AM*x*p[0]*arm_sin_f32(p[1]*(float)CONS_T_AM*x+p[2]);
    g[2] = -p[0]*arm_sin_f32(p[1]*(float)CONS_T_AM*x+p[2]);
}



//##################################################################################################
// %%%%% %%%%% %%%%% LEVENBERG-MARQUARDT
#define CONS_PI_3  1.04719755120341065132    // pi/3
//#define CONS_T_AM  0.00006510416666666657    // Ts = 1/15360
#define CONS_FS_AM 15360                     // Fs = 15360

#define N_CICLOS 12                        	// N_CICLOS   = Fn/Fiec = 60Hz/5Hz
//#define N_MEASUREMENTS (N_CICLOS * 256)     // N_AMOSTRAS = N_CICLOS*Fs/Fn
#define N_PARAMS 3                          // p[0] = A, p[1] = wn, p[2] = phi
#define N_FFT 2048							// N_FFT <= N_MEASUREMENTS
#define N_1_FFT 0.00048828125				// N_1_FFT = 1 / N_FFT
#define N_ATOMS	50

#define I_TOL 0.001 /* smallest value allowed in cholesky_decomp() */
#define I_MAX 50

#define VERBOSE 2

// Variaveis para FFT
arm_rfft_fast_instance_f32 rffti;
uint32_t maxindex;
float32_t maxvalue;
float32_t signal_o1[N_MEASUREMENTS_COS], signal_o2[N_FFT];
float32_t realFFT[N_FFT/2], imagFFT[N_FFT/2], anglFFT[N_FFT/2];
float32_t A_fft, f_fft, phi_fft, phi_fft_d;

// Variaveis para Levenberg-Marquardt
LMstat lmstat;
uint32_t n_iterations, cont;
float32_t params[N_PARAMS], A_atom[N_ATOMS], wn_atom[N_ATOMS], phi_atom[N_ATOMS];
float32_t signal_ss[N_MEASUREMENTS_COS], signal_temp;
float32_t res_sum, res_n[N_ATOMS];
float32_t lm_dif;

float work_buffer[4*N_MEASUREMENTS_COS+4*N_PARAMS + N_MEASUREMENTS_COS*N_PARAMS + N_PARAMS*N_PARAMS];

void CalculationTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t buf = 0;
	char buffer[256];

    float opts[LM_OPTS_SZ], info[LM_INFO_SZ];


	qADC = xQueueCreate(1, sizeof(uint32_t));

	//vTaskDelay(500);
	UARTInit();
	UARTPutString(STDOUT, "\033[2J\033[H",0);

	volatile float32_t bkp = 0;
	float32_t Ts  = (float32_t)CONS_T_AM;
	float32_t wn_1 = 2.0*(float32_t)M_PI*0.4*60.0*(float32_t)CONS_T_AM;
	float32_t wn_2 = 2.0*(float32_t)M_PI*0.8*60.0*(float32_t)CONS_T_AM;
	float32_t wn_3 = 2.0*(float32_t)M_PI*1.0*60.0*(float32_t)CONS_T_AM;
	float32_t wn_4 = 2.0*(float32_t)M_PI*1.6*60.0*(float32_t)CONS_T_AM;
	float32_t wn_5 = 2.0*(float32_t)M_PI*4.4*60.0*(float32_t)CONS_T_AM;
	float32_t wn_6 = 2.0*(float32_t)M_PI*6.4*60.0*(float32_t)CONS_T_AM;
	float32_t wn_7 = 2.0*(float32_t)M_PI*9.8*60.0*(float32_t)CONS_T_AM;
	float32_t wn_8 = 2.0*(float32_t)M_PI*11.8*60.0*(float32_t)CONS_T_AM;
	float32_t phi = 2.0*(float32_t)CONS_PI_3;
	float32_t phi_n_d = 0;

    for(uint32_t t=0; t<(uint32_t)N_MEASUREMENTS_COS; t++){
        signal_ss[t] = 0.0;
        signal_y2[t] =  0.030*arm_cos_f32(wn_1*t + phi+1) +	// atom = 2 	& 	wn_1 = 150.796463	& 	phi+1 = 3.09439516
						0.024*arm_cos_f32(wn_2*t + phi+2) +	// atom = 6		& 	wn_2 = 301.519226	& 	phi+2 = 4.09439516
						1.000*arm_cos_f32(wn_3*t + phi+3) +	// atom = 1		& 	wn_3 = 376.99115	& 	phi+3 = 5.09439516
						0.023*arm_cos_f32(wn_4*t + phi+4) +	// atom = 5 	& 	wn_4 = 603.185852	& 	phi+4 = 6.09439516
						0.029*arm_cos_f32(wn_5*t + phi+5) +	// atom = 4		& 	wn_5 = 1658.76099	& 	phi+5 = 7.09439516
						0.030*arm_cos_f32(wn_6*t + phi+6) +	// atom = 3		& 	wn_6 = 2412.74341	& 	phi+6 = 8.09439516
						0.003*arm_cos_f32(wn_7*t + phi+7) +	// atom = 9		& 	wn_7 = 3694.51343	& 	phi+7 = 9.09439516
						0.004*arm_cos_f32(wn_8*t + phi+8);	// atom = 7		& 	wn_8 = 4448.49561	& 	phi+8 = 10.09439516
        													// atom = 8 ??
    }

    cont=0;
    lm_dif=1;
    levmarq_init(&lmstat);
	arm_rfft_fast_init_f32(&rffti, N_FFT);
    opts[0]=LM_INIT_MU; opts[1]=1E-5; opts[2]=1E-5; opts[3]=1E-5;
    opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used


	#if VERBOSE == 2
	UARTPutString(STDOUT, "Biblioteca levmarq\n\r\n\r", 0);
	#endif
    volatile TickType_t start_time2 = xTaskGetTickCount();

	do{
		volatile TickType_t start_time = xTaskGetTickCount();
		// Copia o sinal para vetor "signal_o1" da FFT
		for(uint32_t t=0; t<(uint32_t)N_MEASUREMENTS_COS; t++){
			signal_o1[t] = signal_y2[t];
		}
		TickType_t stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Copia do sinal\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Realiza a FFT e exporta para o vetor "signal_o2"
		start_time = xTaskGetTickCount();
		arm_rfft_fast_f32(&rffti, signal_o1, signal_o2, 0);
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "FFT\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Extrai a parte real e imaginaria da FFT
		start_time = xTaskGetTickCount();
		for (uint32_t t=0; t<(uint32_t)(N_FFT/2-1); t++) {
			realFFT[t] = signal_o2[t*2];
			imagFFT[t] = signal_o2[(t*2)+1];
			//anglFFT[t] = atan2f(imagFFT[t], realFFT[t]);
		}
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Real e imaginario\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Processa o modulo das partes reais e imaginarias
		start_time = xTaskGetTickCount();
		arm_cmplx_mag_f32(signal_o2, signal_o2, (uint32_t)((float32_t)N_FFT*0.5));
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Calculo do módulo\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif
		signal_o2[0] = 0; 	// Garante parte DC nula

		// Extrai o maior valor e indice
		start_time = xTaskGetTickCount();
		arm_max_f32(&signal_o2[1], (uint32_t)((float32_t)N_FFT*0.5-1), &maxvalue, &maxindex);
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Extrai o maior valor\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif
		maxindex++;

		// Estrai os parametros  da FFT
		A_fft = maxvalue * 0.001;	//x1000
		f_fft = (float32_t)maxindex * (float32_t)CONS_FS_AM * (float32_t)N_1_FFT;
		phi_fft = atan2f(imagFFT[maxindex], realFFT[maxindex]);
		phi_fft_d = 180*(float32_t)M_1_PI*phi_fft;

		// Parametros iniciais
		params[0] = A_fft;
		params[1] = 2.0*(float32_t)M_PI*f_fft;
		params[2] = phi_fft;

		// Levenberg-Marquardt com os parametros da FFT
		start_time = xTaskGetTickCount();
		n_iterations = levmarq(N_PARAMS, params, N_MEASUREMENTS_COS, signal_y2, NULL, &lm_func_cos, &lm_grad_cos, NULL, &lmstat);
	    //int ret;
	    //ret=slevmar_dif(cos_func2, params, signal_y2, N_PARAMS, N_MEASUREMENTS_COS, 1000, opts, info, work_buffer, NULL, &Ts); // without Jacobian
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "levmarq\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Guardar parametros
		A_atom[cont+1] = params[0];
		wn_atom[cont+1] = params[1];
		phi_atom[cont+1] = params[2];
		#if VERBOSE == 2
		sprintf(buffer, "Atomo %d -  A: %f, Wn: %f, Ph: %f\n\r", cont+1, params[0], params[1], params[2]);
		UARTPutString(STDOUT, buffer, 0);
		#endif


		// Verifica angulo radiano em graus
		phi_n_d = 180*(float32_t)M_1_PI*params[2];
		while(phi_n_d <= 0){
			phi_n_d += (float32_t)360.0;
		}

		bkp=1;

		//Atualiza e reconstroi o sinal
		start_time = xTaskGetTickCount();
		for(uint32_t t=0; t<(uint32_t)N_MEASUREMENTS_COS; t++){
					signal_temp = params[0]*arm_cos_f32(params[1]*Ts*t+params[2]);
					signal_y2[t] -= signal_temp;
					signal_ss[t] += signal_temp;
		}
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Atualiza e reconstroi o sinal\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Atualiza o residuo
		start_time = xTaskGetTickCount();
		arm_power_f32(signal_y2, (uint32_t)N_MEASUREMENTS_COS, &res_sum);
		arm_sqrt_f32(res_sum, &res_n[cont]);
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Atualiza o residuo\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Atualiza a diferença
		if(cont > 1){
			lm_dif = res_n[cont-1] - res_n[cont];
		}

		cont++; // Contador de atomos
		#if VERBOSE == 1
		sprintf(buffer, "Iteração %d\n\r\n\r", (int)cont);
		UARTPutString(STDOUT, buffer, 0);
		#endif
	}while(lm_dif > (float32_t)I_TOL);
	volatile TickType_t stop_time2 = xTaskGetTickCount();
	#if VERBOSE == 2
	sprintf(buffer, "\n\rNumero de iteração %d\n\r", (int)cont);
	UARTPutString(STDOUT, buffer, 0);
	sprintf(buffer, "Total calculation time (ms): %d\n\r\n\r", (int)(stop_time2 - start_time2));
	UARTPutString(STDOUT, buffer, 0);
	#endif

	//#####################################################################################################
	//#####################################################################################################
    for(uint32_t t=0; t<(uint32_t)N_MEASUREMENTS_COS; t++){
        signal_ss[t] = 0.0;
        signal_y2[t] =  0.030*arm_cos_f32(wn_1*t + phi+1) +	// atom = 2 	& 	wn_1 = 150.796463	& 	phi+1 = 3.09439516
						0.024*arm_cos_f32(wn_2*t + phi+2) +	// atom = 6		& 	wn_2 = 301.519226	& 	phi+2 = 4.09439516
						1.000*arm_cos_f32(wn_3*t + phi+3) +	// atom = 1		& 	wn_3 = 376.99115	& 	phi+3 = 5.09439516
						0.023*arm_cos_f32(wn_4*t + phi+4) +	// atom = 5 	& 	wn_4 = 603.185852	& 	phi+4 = 6.09439516
						0.029*arm_cos_f32(wn_5*t + phi+5) +	// atom = 4		& 	wn_5 = 1658.76099	& 	phi+5 = 7.09439516
						0.030*arm_cos_f32(wn_6*t + phi+6) +	// atom = 3		& 	wn_6 = 2412.74341	& 	phi+6 = 8.09439516
						0.003*arm_cos_f32(wn_7*t + phi+7) +	// atom = 9		& 	wn_7 = 3694.51343	& 	phi+7 = 9.09439516
						0.004*arm_cos_f32(wn_8*t + phi+8);	// atom = 7		& 	wn_8 = 4448.49561	& 	phi+8 = 10.09439516
        													// atom = 8 ??
    }

    cont=0;
    lm_dif=1;
	arm_rfft_fast_init_f32(&rffti, N_FFT);
    opts[0]=LM_INIT_MU; opts[1]=1E-5; opts[2]=1E-5; opts[3]=1E-5;
    opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used


	#if VERBOSE == 2
	UARTPutString(STDOUT, "Biblioteca slevmar_dif\n\r\n\r", 0);
	#endif
    start_time2 = xTaskGetTickCount();

	do{
		volatile TickType_t start_time = xTaskGetTickCount();
		// Copia o sinal para vetor "signal_o1" da FFT
		for(uint32_t t=0; t<(uint32_t)N_MEASUREMENTS_COS; t++){
			signal_o1[t] = signal_y2[t];
		}
		volatile TickType_t stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Copia do sinal\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Realiza a FFT e exporta para o vetor "signal_o2"
		start_time = xTaskGetTickCount();
		arm_rfft_fast_f32(&rffti, signal_o1, signal_o2, 0);
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "FFT\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Extrai a parte real e imaginaria da FFT
		start_time = xTaskGetTickCount();
		for (uint32_t t=0; t<(uint32_t)(N_FFT/2-1); t++) {
			realFFT[t] = signal_o2[t*2];
			imagFFT[t] = signal_o2[(t*2)+1];
			//anglFFT[t] = atan2f(imagFFT[t], realFFT[t]);
		}
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Real e imaginario\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Processa o modulo das partes reais e imaginarias
		start_time = xTaskGetTickCount();
		arm_cmplx_mag_f32(signal_o2, signal_o2, (uint32_t)((float32_t)N_FFT*0.5));
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Calculo do módulo\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif
		signal_o2[0] = 0; 	// Garante parte DC nula

		// Extrai o maior valor e indice
		start_time = xTaskGetTickCount();
		arm_max_f32(&signal_o2[1], (uint32_t)((float32_t)N_FFT*0.5-1), &maxvalue, &maxindex);
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Extrai o maior valor\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif
		maxindex++;

		// Estrai os parametros  da FFT
		A_fft = maxvalue * 0.001;	//x1000
		f_fft = (float32_t)maxindex * (float32_t)CONS_FS_AM * (float32_t)N_1_FFT;
		phi_fft = atan2f(imagFFT[maxindex], realFFT[maxindex]);
		phi_fft_d = 180*(float32_t)M_1_PI*phi_fft;

		// Parametros iniciais
		params[0] = A_fft;
		params[1] = 2.0*(float32_t)M_PI*f_fft;
		params[2] = phi_fft;

		// Levenberg-Marquardt com os parametros da FFT
		start_time = xTaskGetTickCount();
		//n_iterations = levmarq(N_PARAMS, params, N_MEASUREMENTS_COS, signal_y2, NULL, &lm_func_cos, &lm_grad_cos, NULL, &lmstat);
	    int ret;
	    ret=slevmar_dif(cos_func2, params, signal_y2, N_PARAMS, N_MEASUREMENTS_COS, 1000, opts, info, work_buffer, NULL, &Ts); // without Jacobian
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "levmarq\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Guardar parametros
		A_atom[cont+1] = params[0];
		wn_atom[cont+1] = params[1];
		phi_atom[cont+1] = params[2];
		#if VERBOSE == 2
		sprintf(buffer, "Atomo %d -  A: %f, Wn: %f, Ph: %f\n\r", cont+1, params[0], params[1], params[2]);
		UARTPutString(STDOUT, buffer, 0);
		#endif


		// Verifica angulo radiano em graus
		phi_n_d = 180*(float32_t)M_1_PI*params[2];
		while(phi_n_d <= 0){
			phi_n_d += (float32_t)360.0;
		}

		bkp=1;

		//Atualiza e reconstroi o sinal
		start_time = xTaskGetTickCount();
		for(uint32_t t=0; t<(uint32_t)N_MEASUREMENTS_COS; t++){
					signal_temp = params[0]*arm_cos_f32(params[1]*Ts*t+params[2]);
					signal_y2[t] -= signal_temp;
					signal_ss[t] += signal_temp;
		}
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Atualiza e reconstroi o sinal\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Atualiza o residuo
		start_time = xTaskGetTickCount();
		arm_power_f32(signal_y2, (uint32_t)N_MEASUREMENTS_COS, &res_sum);
		arm_sqrt_f32(res_sum, &res_n[cont]);
		stop_time = xTaskGetTickCount();
		#if VERBOSE == 1
		UARTPutString(STDOUT, "Atualiza o residuo\n\r", 0);
		sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
		UARTPutString(STDOUT, buffer, 0);
		#endif

		// Atualiza a diferença
		if(cont > 1){
			lm_dif = res_n[cont-1] - res_n[cont];
		}

		cont++; // Contador de atomos
		#if VERBOSE == 1
		sprintf(buffer, "Iteração %d\n\r\n\r", (int)cont);
		UARTPutString(STDOUT, buffer, 0);
		#endif
	}while(lm_dif > (float32_t)I_TOL);
	stop_time2 = xTaskGetTickCount();
	#if VERBOSE == 2
	sprintf(buffer, "\n\rNumero de iteração %d\n\r", (int)cont);
	UARTPutString(STDOUT, buffer, 0);
	sprintf(buffer, "Total calculation time (ms): %d\n\r\n\r", (int)(stop_time2 - start_time2));
	UARTPutString(STDOUT, buffer, 0);
	#endif
	//#####################################################################################################
	//#####################################################################################################


	while(1){
	    osDelay(1000);
	}
#if 0
    int n_iterations;
    char buffer[256];
#if 0

    levmarq_init(&lmstat);
    TickType_t start_time = xTaskGetTickCount();
    n_iterations = levmarq(N_PARAMS, params, N_MEASUREMENTS, t_data, NULL, &newton_func, &gradient, NULL, &lmstat);
    TickType_t stop_time = xTaskGetTickCount();
    sprintf(buffer, "**************** End of calculation ***********************\n\r");
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "N iterations: %d\n\r", n_iterations);
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "T_heater: %f, T_0: %f, k: %f\n\r", params[0], params[1], params[2]);
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "**************** Interpolation test ***********************\n\r");
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "Search for temp 70 degrees\n\r");
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "Result: %d sample\n\r", temp_to_time(params, 50.0));
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
    UARTPutString(STDOUT, buffer, 0);
#endif

    float A=0.9, f=62.5, Ts, wn, phi;

 	Ts  =   1.0 / 15360.0; //0.000260417
 	wn  =   2*(float)M_PI*f*Ts;
 	phi =   2*(float)M_PI*0.033;

 	for(int t=0; t<N_MEASUREMENTS_COS; t++){
 		signal_y2[t] =  A*cosf(wn*t+phi) + gNoise(0.0, 0.01);
 	}


#if 0
    levmarq_init(&lmstat);
    volatile TickType_t start_time = xTaskGetTickCount();
    n_iterations = levmarq(N_PARAMS, params_cos, N_MEASUREMENTS_COS, signal_y2, NULL, &lm_func_cos, &lm_grad_cos, NULL, &lmstat);
    volatile TickType_t stop_time = xTaskGetTickCount();
    sprintf(buffer, "**************** End of calculation with levmarq ***********************\n\r");
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "N iterations: %d\n\r", n_iterations);
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "A: %f, Wn: %f, phi: %f\n\r", params_cos[0], params_cos[1], params_cos[2]);
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "**************** Interpolation test ***********************\n\r");
    UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "Search for cos function \n\r");
    UARTPutString(STDOUT, buffer, 0);
    //sprintf(buffer, "Result: %d sample\n\r", temp_to_time(params, 50.0));
    //UARTPutString(STDOUT, buffer, 0);
    sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
    UARTPutString(STDOUT, buffer, 0);
#endif


    const int n=40, m=3; // 40 measurements, 3 parameters
    float p[m], x[n], opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    register int i;
    int ret;

      /* generate some measurement using the exponential model with
       * parameters (5.0, 0.1, 1.0), corrupted with zero-mean
       * Gaussian noise of s=0.1
       */
      INIT_RANDOM(0);
#if 0
      for(i=0; i<n; ++i)
        x[i]=(5.0*expf(-0.1*i) + 1.0) + gNoise(0.0, 0.1);

      /* initial parameters estimate: (1.0, 0.0, 0.0) */
      p[0]=1.0; p[1]=0.0; p[2]=0.0;

      /* optimization control parameters; passing to levmar NULL instead of opts reverts to defaults */
      opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
      opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used

      /* invoke the optimization function */
      TickType_t start_time = xTaskGetTickCount();
      ret=slevmar_der(expfunc, jacexpfunc, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic Jacobian
      //ret=slevmar_dif(expfunc, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // without Jacobian
      TickType_t stop_time = xTaskGetTickCount();
      sprintf(buffer, "Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n\r", info[5], info[6], info[1], info[0]);
      UARTPutString(STDOUT, buffer, 0);
      sprintf(buffer, "Best fit parameters: %.7g %.7g %.7g\n\r", p[0], p[1], p[2]);
      UARTPutString(STDOUT, buffer, 0);
      sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time - start_time));
      UARTPutString(STDOUT, buffer, 0);
#endif

      /* initial parameters estimate: (1.0, 0.0, 0.0) */
      //p[0]=1.0; p[1]=0.0; p[2]=0.0;

      /* optimization control parameters; passing to levmar NULL instead of opts reverts to defaults */
      //opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
      //opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used

      opts[0]=LM_INIT_MU; opts[1]=1E-4; opts[2]=1E-4; opts[3]=1E-4;
      opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used

      /* invoke the optimization function */
      volatile TickType_t start_time2 = xTaskGetTickCount();
      //start_time = xTaskGetTickCount();
      //ret=slevmar_der(expfunc, jacexpfunc, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic Jacobian


      /* invoke the optimization function */
      //ret=dlevmar_der(expfunc, jacexpfunc, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic Jacobian
      //ret=dlevmar_dif(expfunc, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // without Jacobian
      ret=slevmar_dif(cos_func2, params_cos2, signal_y2, N_PARAMS, N_MEASUREMENTS_COS, 1000, opts, info, work_buffer, NULL, &Ts); // without Jacobian
      volatile TickType_t stop_time2 = xTaskGetTickCount();
      //stop_time = xTaskGetTickCount();
      sprintf(buffer, "**************** End of calculation with slevmar_dif ***********************\n\r");
      UARTPutString(STDOUT, buffer, 0);
      sprintf(buffer, "Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n\r", info[5], info[6], info[1], info[0]);
      UARTPutString(STDOUT, buffer, 0);
      sprintf(buffer, "Best fit parameters: %.7g %.7g %.7g\n\r", params_cos[0], params_cos[1]/(2*M_PI), params_cos[2]);
      UARTPutString(STDOUT, buffer, 0);
      sprintf(buffer, "Calculation time (ms): %d\n\r\n\r", (int)(stop_time2 - start_time2));
      UARTPutString(STDOUT, buffer, 0);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 3072);
	HAL_TIM_Base_Start(&htim8);

	while (1)
	{
		xQueueReceive(qADC, &buf, portMAX_DELAY);
		switch(buf){
			case 1:
				// Os dados estão entre adcBuffer[0] e adcBuffer[1535]
				for (int i=0; i<1536;i++){
					Buffer1[i] = adcBuffer[i];
					adcBuffer[i] = 0;
				}
				for (int i=1536; i<3072;i++){
					Buffer2[i-1536] = adcBuffer[i];
				}
				buf = 0;
				break;
			case 2:
				// Os dados estão entre adcBuffer[1536] e adcBuffer[3071]
				for (int i=0; i<1536;i++){
					Buffer1[i] = adcBuffer[i];
				}
				for (int i=1536; i<3072;i++){
					Buffer2[i-1536] = adcBuffer[i];
				}
				buf = 0;
				break;
			default:
				break;
		}
		// Para FFT
		//arm_cfft_f32(&arm_cfft_sR_f32_len256,FasesAC_ReIm_R,0,1);
		//arm_cmplx_mag_f32(FasesAC_ReIm_R,FasesAC_mod_R,256);
		//arm_scale_f32(FasesAC_mod_R,(1/numero_pontos), FasesAC_mod_R,numero_pontos);
		//harmonics_R_phase[k] = atan2(FasesAC_ReIm_R[2*k+5],FasesAC_ReIm_R[2*k+4]);
	}
#endif
  /* USER CODE END 5 */
}

void lv_gui(void);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the LCDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    lv_init();

    tft_init();
    touchpad_init();

    lv_gui();

    while (1)
    {
        osDelay(5);
        lv_task_handler();
    }
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
