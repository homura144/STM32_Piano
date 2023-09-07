/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
const unsigned int music1[3][97] = {
        {600, 300, 300, 600, 600, 600, 300, 300, 600, 600, 600, 600, 300, 300, 600, 600, 300, 300, 900, 600, 600, 300, 300, 600, 600, 600, 300, 300, 600, 600, 600, 300, 300, 600, 600, 600, 300, 300, 600, 900, 300, 600, 600, 600, 300, 900, 900, 300, 600, 600, 600, 300, 300, 900, 900, 300, 600, 600, 600, 300, 300, 900, 900, 300, 600, 600, 600, 300, 300, 900, 600, 900, 600, 600, 900, 600, 600, 600, 600, 600, 600, 600, 900, 600, 900, 600, 600, 900, 600, 600, 600, 600, 600, 600, 300, 300, 900},
        {2,   2,   2,   2,   2,   2,   3,   2,   2,   2,   2,   2,   2,   2,   2,   1,   2,   2,   2,   2,   2,   2,   3,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   2,   3,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   1,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,   3,   3,   1,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,   2},
        {1,   1,   3,   5,   5,   6,   1,   6,   5,   5,   3,   3,   5,   3,   1,   6,   1,   2,   5,   6,   6,   6,   1,   5,   3,   2,   3,   2,   1,   2,   5,   5,   4,   5,   6,   6,   7,   6,   5,   1,   1,   6,   1,   5,   6,   5,   6,   6,   5,   3,   2,   2,   3,   5,   1,   1,   1,   3,   2,   3,   2,   1,   6,   6,   5,   3,   2,   3,   2,   1,   1,   1,   0,   6,   6,   0,   5,   5,   6,   5,   2,   3,   5,   1,   1,   0,   6,   6,   0,   5,   5,   6,   5,   2,   3,   2,   1}
},
        music2[3][66] = {
        {405, 203, 0, 405, 405, 203, 405, 203, 203, 203, 203, 203, 203, 810, 608, 203, 203, 203, 203, 203, 405, 203, 405, 405, 203, 810, 203, 203, 405, 203, 405, 405, 203, 203, 810, 203, 405, 203, 405, 405, 203, 405, 203, 203, 203, 203, 203, 203, 203, 608, 405, 203, 203, 203, 608, 203, 405, 203, 405, 203, 405, 405, 405, 405, 405, 1013},
        {2,   2,   2, 2,   2,   2,   2,   2,   2,   2,   1,   1,   2,   2,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   1,   1,   1,   2,   2,   2,   2,   1,   2,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   1,   1,   2,   2,   1,   2,   2,   2,   2,   1,   2,   2,   2,   1,   2,   2,   2,   2,   2,   2},
        {3,   3,   2, 5,   2,   2,   2,   2,   3,   2,   7,   7,   1,   1,   0,   7,   3,   5,   5,   7,   6,   5,   3,   1,   7,   7,   0,   2,   2,   3,   1,   7,   2,   2,   2,   0,   3,   3,   2,   5,   2,   2,   2,   2,   3,   2,   7,   7,   1,   1,   0,   7,   7,   3,   3,   0,   4,   3,   4,   0,   7,   6,   5,   4,   3,   4}
},
        music3[3][86] = {
        {215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 429, 215, 215, 644, 644, 429, 215, 215, 429, 215, 215, 644, 644, 429, 215, 215, 429, 215, 215, 644, 1073, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644},
        {2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,    2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2},
        {1,   4,   5,   5,   6,   6,   6,   6,   6,   6,   6,   6,   5,   5,   5,   5,   5,   5,   5,   5,   4,   4,   4,   4,   4,   4,   4,   4,   1,   1,   1,   1,   1,   4,   5,   5,   6,   6,   6,   6,   6,   6,   1,   1,   6,   6,   6,   6,   6,   5,   4,   5,   6,   6,   6,   6,   0,   5,   6,   5,   4,   4,   4,   4,   0,   5,   6,   5,   4,   4,   4,   4,    6,   6,   6,   6,   5,   1,   6,   6,   5,   5,   6,   6,   6,   6}
},
        pwm_freq[3][7] = {
        {131, 147, 165, 175, 196, 220, 247},
        {262, 294, 330, 349, 392, 440, 494},
        {523, 587, 659, 698, 784, 880, 988}
};
const unsigned char midi_freq[3][7] = {
        {48, 50, 52, 53, 55, 57, 59},
        {60, 62, 64, 65, 67, 69, 71},
        {72, 74, 76, 77, 79, 81, 83}
},
        length[3] = {97, 66, 86},
        tube_data[5] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99};

const unsigned int *music[3][3] = {
        {music1[0], music1[1], music1[2]},
        {music2[0], music2[1], music2[2]},
        {music3[0], music3[1], music3[2]}
};

unsigned char midi_event[3] = {0x90, 60, 0x4f},
        buffer1, buffer2[2];
unsigned int pitch = 0, tone = 0, mode = 0, remote_mode = 0, music_type = 0, music_position = 0, music_start = 0, receive = 0;
unsigned int duration = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void direct_play();

void direct_broadcast();

void remoteControl();

void change_LED();

void play_pwm();

void play_midi() ;

void choose_music(int n);

void broadcast_music();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim3);//打开定时中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        //弹奏
        if (mode == 1 || mode == 2) {
            direct_play();
        }

        //播放
        if (mode == 3 || mode == 4) {
            direct_broadcast();
        }

        //远程控制
        if (mode == 5) {
            remoteControl();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1699;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 PC3 PC4
                           PC5 PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {//按钮的回调函数

    if (GPIO_PIN == GPIO_PIN_13) {//按下蓝色按钮
        if (mode == 5) {
            mode = 0;
        } else {
            mode++;
        }
        change_LED();
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (mode == 5 && (remote_mode == '3' || remote_mode == '4')) {
        receive = 1;
    }

}

void change_data(char data) {//改变数码管显示

    for (unsigned char i = 0; i < 8; i++)//控制8个数码管
    {
        if ((data & GPIO_PIN_7) == 0)//根据data的最高位，使与DS连接的引脚输出相应电平
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    //在与SHCP连接的引脚上输出上升沿信号
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

        data <<= 1;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  //在与STCP连接的引脚上输出信号
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

}

void change_LED() {//改变LED显示
    //全部熄灭
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

    //根据状态点亮LED
    if (mode == 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    } else if (mode == 2) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    } else if (mode == 3) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    } else if (mode == 4) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    } else if (mode == 5) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    }

}

void direct_play() {//在扩展板上弹奏

    while (mode == 1 || mode == 2) {

        if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)) {//查询音高
            pitch = 0;
        } else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)) {//注意这个按钮是按下为高电平
            pitch = 2;
        } else {
            pitch = 1;
        }

        if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)) {//查询音调
            tone = 0;
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {
            tone = 1;
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)) {
            tone = 2;
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)) {
            tone = 3;
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)) {
            tone = 4;
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)) {
            tone = 5;
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)) {
            tone = 6;
        } else {
            return;
        }

        if (mode == 1) {//pwm输出
            duration = 100;
            play_pwm();
        } else if (mode == 2) {//midi输出
            duration = 200;//延时长一点，避免连续播放时出现杂音
            play_midi();
        }
    }

}

void play_pwm() {//利用蜂鸣器发声

    if (pitch == -1 && tone == -1) {//休止符
        HAL_Delay(duration);
    } else if (pitch >= 0 && pitch < 3 && tone >= 0 && tone < 7) {//改变时钟的ARR，CCR1，输出频率给蜂鸣器
        TIM3->ARR = 100000 / pwm_freq[pitch][tone] - 1;
        TIM3->CCR1 = TIM3->ARR * 0.95;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_Delay(duration);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    }
}

void play_midi() {//利用autopiano发声

    if (pitch == -1 && tone == -1) {
        HAL_Delay(duration);
    }
    if (pitch >= 0 && pitch < 3 && tone >= 0 && tone < 7) {
        midi_event[1] = midi_freq[pitch][tone];
        //按下琴键
        midi_event[0] = 0x90;
        HAL_UART_Transmit(&huart1, midi_event, 3, 0xffff);
        //延时
        HAL_Delay(duration);
        //释放琴键
        midi_event[0] = 0x80;
        HAL_UART_Transmit(&huart1, midi_event, 3, 0xffff);
    }

}

void direct_broadcast() {//在扩展板上控制播放

    //显示音乐信息
    change_data(tube_data[0]);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    //初始化播放信息
    music_type = -1;
    music_position = 0;
    music_start = 0;

    while (mode == 3 || mode == 4) {

        if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)) {//查询操作
            choose_music(1);
            change_data(tube_data[1]);
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {
            choose_music(2);
            change_data(tube_data[2]);
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)) {
            choose_music(3);
            change_data(tube_data[3]);
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)) {//暂停
            music_start = 0;
            change_data(tube_data[4]);
        } else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)) { //继续
            music_start = 1;
            change_data(tube_data[music_type + 1]);
        }

        broadcast_music();

    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

}

void broadcast_music() {
    if (music_start) {
        duration = music[music_type][0][music_position];
        pitch = music[music_type][1][music_position] - 1;
        tone = music[music_type][2][music_position] - 1;

        if (mode == 3) {//pwm输出
            play_pwm();
        } else if (mode == 4) {//midi输出
            play_midi();
        } else if (mode == 5) {
            if (remote_mode == '3') {//pwm输出
                play_pwm();
            } else if (remote_mode == '4') {//midi输出
                play_midi();
            }
        }

        music_position += 1;//播放下一个音符

        if (music_position == length[music_type]) {//播放结束，重置
            music_position = 0;
            music_start = 0;
            change_data(tube_data[0]);
        }
    }
}

void choose_music(int n) {
    music_type = n - 1;
    music_position = 0;
    music_start = 1;
}

void remoteControl() {

    while (mode == 5) {

        HAL_UART_Transmit(&hlpuart1, "Please choose mode:\r\n", 22, 0xffff);
        remote_mode = '\0';

        HAL_UART_Receive_IT(&hlpuart1, &remote_mode, 1);

        while (mode == 5 && remote_mode == '\0');//查询是否已选择远程模式

        if (remote_mode == '0') {//退出远程控制模式
            mode = 0;
            change_LED();
        } else if (remote_mode == '1' || remote_mode == '2') {//进入远程弹奏模式

            if (remote_mode == '1') {
                HAL_UART_Transmit(&hlpuart1, "Enter play-pwm mode\r\n", 22, 0xffff);
            } else {
                HAL_UART_Transmit(&hlpuart1, "Enter play-midi mode\r\n", 23, 0xffff);
            }

            while (mode == 5) {

                HAL_UART_Receive(&hlpuart1, buffer2, 2, 0xffff);//接收字符
                HAL_UART_Transmit(&hlpuart1, buffer2, 2, 0xffff);//显示刚才的输入
                HAL_UART_Transmit(&hlpuart1, "\r\n", 2, 0xffff);//换行

                if (buffer2[0] == '0' && buffer2[1] == '0') {//退出弹奏模式
                    break;
                } else {
                    pitch = buffer2[0] - '1';
                    tone = buffer2[1] - '1';
                }

                if (remote_mode == '1') {//蜂鸣器发声
                    duration = 100;
                    play_pwm();
                } else if (remote_mode == '2') {//自由钢琴发声
                    duration = 200;
                    play_midi();
                }
            }
        } else if (remote_mode == '3' || remote_mode == '4') {//进入远程播放模式

            if (remote_mode == '3') {
                HAL_UART_Transmit(&hlpuart1, "Enter broadcast-pwm mode\r\n", 27, 0xffff);
            } else {
                HAL_UART_Transmit(&hlpuart1, "Enter broadcast-midi mode\r\n", 28, 0xffff);
            }

            //显示音乐信息
            change_data(tube_data[0]);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

            //初始化播放信息
            music_type = -1;
            music_position = 0;
            music_start = 0;
            receive = 0;

            while (mode == 5) {

                HAL_UART_Receive_IT(&hlpuart1, &buffer1, 1);//中断型串口通信，避免堵塞音乐播放

                if (receive) {//接收到指令
                    if (buffer1 == '0') {//退出播放模式
                        HAL_UART_AbortReceive_IT(&hlpuart1);//停止当前的串口通信中断，防止在退出播放模式之后还接收到指令
                        break;
                    }
                    if (buffer1 >= '1' && buffer1 <= '3') {//选择歌曲
                        choose_music(buffer1 - '0');
                        change_data(tube_data[music_type + 1]);
                    } else if (buffer1 == '4') {//暂停
                        music_start = 0;
                        change_data(tube_data[4]);
                    } else if (buffer1 == '5') { //继续
                        music_start = 1;
                        change_data(tube_data[music_type + 1]);
                    }

                    receive = 0;//重置，等待下一条指令
                }

                broadcast_music();

            }

            HAL_UART_AbortReceive_IT(&hlpuart1);//停止当前的串口通信中断
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        }

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
    while (1) {
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
