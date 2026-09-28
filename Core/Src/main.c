/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <math.h>

#include "nRF24_driver.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
SPI_HandleTypeDef*  hspi_bno  = &hspi1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TODO: Still need to properly test the rotary encoder!
int16_t pulse_count;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//  GPIO_PinState A_status = HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin);
//  GPIO_PinState B_status = HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin);
//
//  if ((GPIO_Pin == ROT_A_Pin) == (A_status == B_status))
//	pulse_count++;
//  else
//    pulse_count--;

  if (GPIO_Pin == BNO_IRQ_Pin) {
    BNO_IRQcallback();
  }
}


const float MAX_PWM_CRR = 5000.0f;

void setMotorOutput(float v) {
  int16_t motor_output = (int16_t)(MAX_PWM_CRR*v);

  // Input clipping and taking absolute
  if (v > 1.0f || v < -1.0f)
    motor_output = MAX_PWM_CRR;
  else if (v < 0.0f)
    motor_output = -motor_output;

  // Set the CRR value
  TIM4->CCR2 = motor_output;

  // Set direction pins
  if (v > 0.0f) {
    HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_RESET);
  }
  else {
    HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_SET);
  }
}

void setupNRF24() {
  // Turn off module while configuring it.
  writeNRF24register(N24_CONFIG,    0b01111001); // Mask all interrupts, enable 1B CRC, keep power down, act as PTX

  // Setup extra features. TODO: The toggleNRF24extraFeatures function doesn't seem to have an effect... find out why?
  toggleNRF24extraFeatures();
  writeNRF24register(N24_FEATURE,   0b00000111); // Enable all acknowledge payload features
  writeNRF24register(N24_DYNPD,     0b00000001); // Enable dynamic payload in pipe 0

  // Miscellaneous configuration settings.
  writeNRF24register(N24_EN_AA,     0b00000001); // Only enable auto acknowledge on pipe 0
  writeNRF24register(N24_EN_RXADDR, 0b00000001); // Only enable reception on pipe 0
  writeNRF24register(N24_SETUP_AW,  0b00000011); // Set 5 byte address width
  writeNRF24register(N24_SETUP_RETR,0b00010101); // 5 retransmissions, each spaced 500us apart
  writeNRF24register(N24_RF_CH,     0b00000010); // Set frequency channel to 2... this is the datasheet's default, so I'll trust it for now
  writeNRF24register(N24_RF_SETUP,  0b00000111); // 1Mbps tx rate, 0dBm tx gain, setup LNA gain (idk what that means)

  // Setup TX and RX addresses.
  const uint8_t device_addr[5] = {0b01101001, 0b10010110, 0b10010110, 0b01101001, 0b10101010};
  setNRF24TXaddr(device_addr, 5);    // Configure TX address
  setNRF24RXaddr(device_addr, 5, 0); // Configure RX address for pipe 0 to be the same as TX address

  // Flush the buffers
  flushNRF24buffers();

  // Final configuration, and turn on the module
  writeNRF24register(N24_CONFIG, 0b01111011); // Power up the nRF24
  //TODO: Footnote d on Table 24 (pg. 58) says that DPL needs to be activated on both PTX and PRX
  //      if you want to have payloads on the ACK packet. Configure this!

  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
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
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


//  // Start the nRF24
//  setupNRF24();

  // Stop motor, then start PWM generation
  setMotorOutput(0.0f);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);


  // Start the BNO085
  BNO_startup();


  // Then start the rotation vector output
  BNO_startRotationVector();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t last_recv = 0.0f;
  uint16_t ack_number = 0;
  uint8_t payload[4];

  while (1)
  {
//    uint8_t fifo_status = readNRF24register(N24_FIFO_STATUS);
//
//    if ((fifo_status & 0b00000001) == 0) {
//      // Set new acknowledge payload
//      ++ack_number;
//      payload[0] = (pulse_count & 0xff00) >> 8;
//      payload[1] = pulse_count & 0x00ff;
//      payload[2] = (ack_number & 0xff00) >> 8;
//      payload[3] = ack_number & 0x00ff;
//      writeNRF24ackPayload((uint8_t*)payload, 8, 0);
//
//      // Reset pulse_count to get a relative measurement
//      pulse_count = 0;
//
//      // Read buffer to clear it
//      uint8_t recv_buf[32];
//      readNRF24payload(recv_buf, 8);
//
//      // Recover value from buffer
//      last_recv = recv_buf[0] * 0x100 + recv_buf[1];
//
//      setMotorOutput(last_recv / 3000.0f);
//
//      // Clear RX_DR interrupt
//      writeNRF24register(N24_STATUS, 0b01000000);
//
//
//      HAL_GPIO_TogglePin(BLINK_GPIO_Port, BLINK_Pin);
//    }

    HAL_Delay(2);
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BNO_WAKE_Pin|BNO_CSN_Pin|NRF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BNO_RESET_Pin|NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOT_bwd_Pin|MOT_fwd_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLINK_Pin */
  GPIO_InitStruct.Pin = BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLINK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROT_A_Pin ROT_B_Pin */
  GPIO_InitStruct.Pin = ROT_A_Pin|ROT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BNO_WAKE_Pin BNO_RESET_Pin BNO_CSN_Pin NRF_CE_Pin
                           NRF_CSN_Pin */
  GPIO_InitStruct.Pin = BNO_WAKE_Pin|BNO_RESET_Pin|BNO_CSN_Pin|NRF_CE_Pin
                          |NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_IRQ_Pin */
  GPIO_InitStruct.Pin = BNO_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNO_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_bwd_Pin MOT_fwd_Pin */
  GPIO_InitStruct.Pin = MOT_bwd_Pin|MOT_fwd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
