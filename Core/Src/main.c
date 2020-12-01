/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"
#include "delay.h"
#include "display.h"
#include "ds18b20.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DEBUG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* Definitions for Task_center */
osThreadId_t Task_centerHandle;
const osThreadAttr_t Task_center_attributes = {
    .name = "Task_center",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};
/* Definitions for Task_key_scan */
osThreadId_t Task_key_scanHandle;
const osThreadAttr_t Task_key_scan_attributes = {
    .name = "Task_key_scan",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* Definitions for Task_NTC_scan */
osThreadId_t Task_NTC_scanHandle;
const osThreadAttr_t Task_NTC_scan_attributes = {
    .name = "Task_NTC_scan",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* Definitions for Task_led */
osThreadId_t Task_ledHandle;
const osThreadAttr_t Task_led_attributes = {
    .name = "Task_led",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* Definitions for Task_display */
osThreadId_t Task_displayHandle;
const osThreadAttr_t Task_display_attributes = {
    .name = "Task_display",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* Definitions for Task_output */
osThreadId_t Task_outputHandle;
const osThreadAttr_t Task_output_attributes = {
    .name = "Task_output",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM8_Init(void);
void Task_center_f(void *argument);
void Task_key_scan_f(void *argument);
void Task_NTC_scan_f(void *argument);
void Task_led_f(void *argument);
void Task_display_f(void *argument);
void Task_output_f(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
#ifdef DEBUG
int fputc(int ch, FILE *f) {
  uint8_t temp[1] = {ch};
  HAL_UART_Transmit(&huart1, temp, 1, 2);
  return 0;
}
#endif
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

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
  /* creation of Task_center */
  Task_centerHandle = osThreadNew(Task_center_f, NULL, &Task_center_attributes);

  /* creation of Task_key_scan */
  Task_key_scanHandle =
      osThreadNew(Task_key_scan_f, NULL, &Task_key_scan_attributes);

  /* creation of Task_NTC_scan */
  Task_NTC_scanHandle =
      osThreadNew(Task_NTC_scan_f, NULL, &Task_NTC_scan_attributes);

  /* creation of Task_led */
  Task_ledHandle = osThreadNew(Task_led_f, NULL, &Task_led_attributes);

  /* creation of Task_display */
  Task_displayHandle =
      osThreadNew(Task_display_f, NULL, &Task_display_attributes);

  /* creation of Task_output */
  Task_outputHandle = osThreadNew(Task_output_f, NULL, &Task_output_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {
  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
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
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_center_f */
/**
 * @brief  Function implementing the Task_center thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_center_f */
data_t data;
void Task_center_f(void *argument) {
  /* USER CODE BEGIN 5 */
  osKernelLock();
#ifdef DEBUG
  printf("center initing..\r\n");
#endif
  uint32_t time = 0;
  data.mode = relax_mode;
  data.timing_mode = zero;
  data.display_mode = work;
  data.key = key_work_mode;
  data.key_sign = 0;
  osKernelUnlock();
  /* Infinite loop */
  while (1) {
#ifdef DEBUG
    if (data.timing_mode != zero) printf("%d\r\n", time / 1000);
#endif
    uint32_t tick = osKernelGetTickCount();
    if (data.timing_mode != zero && time <= 0) {
      data.timing_mode = zero;
      data.mode = relax_mode;
    }
    if (time > 0) time -= 20;
    if (data.key_sign) /* 有按键按下 */
    {
      data.key_sign = 0;
      switch (data.key) {
        case key_work_mode:
          if (data.timing_mode != zero) switch (data.mode) {
              case relax_mode:
                data.mode = sleep_mode;
                break;
              case sleep_mode:
                data.mode = nature_mode;
                break;
              case nature_mode:
                data.mode = common_mode;
                break;
              case common_mode:
                data.mode = sleep_mode;
                break;
            }
          break;
        case key_stop:
          data.mode = relax_mode;
          data.timing_mode = zero;
          break;
        case key_show:
          switch (data.display_mode) {
            case work:
              data.display_mode = temperature;
              break;
            case temperature:
              data.display_mode = work;
              break;
          }
          break;
        case key_timing_mode:
          data.display_mode = work;
          switch (data.timing_mode) {
            case zero:
              data.timing_mode = one;
              time = 1000 * 60;
              if (data.mode == relax_mode) data.mode = sleep_mode;
#ifdef DEBUG
              printf("zero\r\n");
#endif
              break;
            case one:
              data.timing_mode = two;
#ifdef DEBUG
              printf("one\r\n");
#endif
              if (data.mode == relax_mode) data.mode = sleep_mode;
              time = 2000 * 60;
              break;
            case two:
              data.timing_mode = zero;
#ifdef DEBUG
              printf("two\r\n");
#endif
              if (data.mode != relax_mode) data.mode = relax_mode;
              time = 0;
              break;
          }
          break;
      };
    }
    // do something
    switch (data.mode) {
      case relax_mode:
        data.pwm = 0;
        data.led1 = 0;
        data.led2 = 0;
        data.led3 = 0;
        break;
      case sleep_mode:
        data.pwm = 200;
        data.led1 = 1;
        data.led2 = 0;
        data.led3 = 0;
        break;
      case nature_mode:
        data.pwm = 300;
        data.led1 = 0;
        data.led2 = 1;
        data.led3 = 0;
        break;
      case common_mode:
        data.pwm = 700;
        data.led1 = 0;
        data.led2 = 0;
        data.led3 = 1;
        break;
    }
    if (data.display_mode == work) {
      switch (data.mode)  //工作模式选择
      {
        case relax_mode:
          data.display.a = 'r';
          data.display.b = 'r';
          data.display.c = 'r';
          data.display.d = 'r';
          data.display.e = 'r';
          data.display.f = 'r';
          data.display.g = 'r';
          data.display.h = 'r';
          break;
        case sleep_mode:  //工作模式为睡眠风
          data.display.a = '-';
          data.display.b = '1';
          data.display.c = '-';
          data.display.d = 'r';
          data.display.e = '0';
          data.display.f = ((time / 100000) % 10) + 48;
          data.display.g = ((time / 10000) % 10) + 48;
          data.display.h = ((time / 1000) % 10) + 48;
          break;
        case nature_mode:  //工作模式为自然风
          data.display.a = '-';
          data.display.b = '2';
          data.display.c = '-';
          data.display.d = 'r';
          data.display.e = '0';
          data.display.f = ((time / 100000) % 10) + 48;
          data.display.g = ((time / 10000) % 10) + 48;
          data.display.h = ((time / 1000) % 10) + 48;
          break;
        case common_mode:  //工作模式为常风
          data.display.a = '-';
          data.display.b = '3';
          data.display.c = '-';
          data.display.d = 'r';
          data.display.e = '0';
          data.display.f = ((time / 100000) % 10) + 48;
          data.display.g = ((time / 10000) % 10) + 48;
          data.display.h = ((time / 1000) % 10) + 48;
          break;
      }
    }
    if (data.display_mode == temperature) {
      data.display.a = '-';
      data.display.b = '4';
      data.display.c = '-';
      data.display.d = 'r';
      data.display.e = 'r';
      data.display.f = ((data.ntc / 100) % 10) + 48;
      data.display.g = ((data.ntc / 10) % 10) + 48;
      data.display.h = 'c';
    }
    tick += 20;
    osDelayUntil(tick);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_key_scan_f */
/**
 * @brief Function implementing the Task_key_scan thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_key_scan_f */
void Task_key_scan_f(void *argument) {
/* USER CODE BEGIN Task_key_scan_f */
#ifdef DEBUG
  osKernelLock();
  printf("key scan initing..\r\n");
  osKernelUnlock();
#endif
  uint32_t tick = osKernelGetTickCount();
  int sign1 = 0, sign2 = 0, sign3 = 0, sign4 = 0, a;
  /* Infinite loop */
  while (1) {
    a = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
    if (a && sign1 > 0) sign1--;
    if (!a && sign1 == 0) {
      data.key = key_work_mode;
      sign1 = 30;
      data.key_sign++;
#ifdef DEBUG
      printf("input\r\n");
#endif
    }
    a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    if (a && sign2 > 0) sign2--;
    if (!a && sign2 == 0) {
      data.key = key_timing_mode;
      sign2 = 30;
      data.key_sign++;
#ifdef DEBUG
      printf("input\r\n");
#endif
    }
    a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    if (a && sign3 > 0) sign3--;
    if (!a && sign3 == 0) {
      data.key = key_stop;
      sign3 = 30;
      data.key_sign++;
#ifdef DEBUG
      printf("input\r\n");
#endif
    }
    a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    if (a && sign4 > 0) sign4--;
    if (!a && sign4 == 0) {
      data.key = key_show;
      sign4 = 30;
      data.key_sign++;
#ifdef DEBUG
      printf("input\r\n");
#endif
    }
    tick += 3;
    osDelayUntil(tick);
  }
  /* USER CODE END Task_key_scan_f */
}

/* USER CODE BEGIN Header_Task_NTC_scan_f */
/**
 * @brief Function implementing the Task_NTC_scan thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_NTC_scan_f */
void Task_NTC_scan_f(void *argument) {
  /* USER CODE BEGIN Task_NTC_scan_f */
  delay_init(72);
#ifdef DEBUG
  osKernelLock();
  printf("NTC scan initing..\r\n");
  osKernelUnlock();
#endif
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  while (1) {
    data.ntc = DS18B20_Get_Temp();
    tick += 40;
    osDelayUntil(tick);
  }
  /* USER CODE END Task_NTC_scan_f */
}

/* USER CODE BEGIN Header_Task_led_f */
/**
 * @brief Function implementing the Task_led thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_led_f */
void Task_led_f(void *argument) {
/* USER CODE BEGIN Task_led_f */
#ifdef DEBUG
  osKernelLock();
  printf("led initing..\r\n");
  osKernelUnlock();
#endif
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  while (1) {
    if (data.led1)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    if (data.led2)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    if (data.led3)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    tick += 20;
    osDelayUntil(tick); 
  }
  /* USER CODE END Task_led_f */
}

/* USER CODE BEGIN Header_Task_display_f */
/**
 * @brief Function implementing the Task_display thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_display_f */
void Task_display_f(void *argument) {
/* USER CODE BEGIN Task_display_f */
#ifdef DEBUG
  osKernelLock();
  printf("display initing..\r\n");
  osKernelUnlock();
#endif
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  uint8_t sum = 0;
  while (1) {
    switch (sum) {
      case 0:
        Smg_Select('a', data.display.a);
        break;
      case 1:
        Smg_Select('b', data.display.b);
        break;
      case 2:
        Smg_Select('c', data.display.c);
        break;
      case 3:
        Smg_Select('d', data.display.d);
        break;
      case 4:
        Smg_Select('e', data.display.e);
        break;
      case 5:
        Smg_Select('f', data.display.f);
        break;
      case 6:
        Smg_Select('g', data.display.g);
        break;
      case 7:
        Smg_Select('h', data.display.h);
        break;
      default:
        Smg_Select('a', data.display.a);
        sum = 0;
        break;
    }
    sum++;
    tick += 2;
    osDelayUntil(tick);
  }
  /* USER CODE END Task_display_f */
}

/* USER CODE BEGIN Header_Task_output_f */
/**
 * @brief Function implementing the Task_output thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_output_f */
void Task_output_f(void *argument) {
/* USER CODE BEGIN Task_output_f */
#ifdef DEBUG
  osKernelLock();
  printf("output initing..\r\n");
  osKernelUnlock();
#endif
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  uint16_t pwm = 0;
  while (1) {
    if (pwm != data.pwm) {
      pwm = data.pwm;
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm);
    }
    // pwm_control(); TODO
    tick += 20;
    osDelayUntil(tick);
  }
  /* USER CODE END Task_output_f */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
