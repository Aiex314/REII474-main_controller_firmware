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
#include "bno_driver.h"
#include "motor_controller.h"

#include "message_protocol.h"

#include "lookup_tables.h"
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
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
SPI_HandleTypeDef*  hspi_bno  = &hspi1;

// ======================== CONSTANTS =======================
// Conversion constants
#define PI 3.14159265358979323
const float DEG_TO_RAD = PI/180.0;
const float RAD_TO_DEG = 180.0/PI;
// Maximum angle to activate linear controller in
const float ANGLE_LIM  = 10.0*DEG_TO_RAD;
// Linear region offset (radians)
const float LINEAR_OFFSET = 3.8*DEG_TO_RAD;


#define nA4  440.00
#define nB4  493.88
#define nC4  523.25
#define nD4  587.33
#define nE4  659.26
#define nF4  698.46
#define nG4  783.99
#define nAs4 466.16
#define nCs4 554.37
#define nDs4 622.25
#define nFs4 739.99
#define nGs4 830.61
#define nA5  880.00
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void setMotorOutput(float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Raw outputs of IMU
float yaw, yaw_raw;
float dyaw, dyaw_raw, dyaw_smooth;
float pitch, roll;
float accuracy;
// Rotary encoder output
int16_t pulse_count = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == BNO_IRQ_Pin) {
    float yaw_prev = yaw;
    float yaw_raw_prev = yaw_raw;

    BNO_IRQcallback();
    BNO_getYPR(&yaw, &pitch, &roll);
    BNO_getAccuracy(&accuracy);

    // Correct for nonlinearity.....
    yaw_raw = yaw;
    yaw = motorLinearizationLookup(yaw);

    // Calculate raw dyaw for linear region
    dyaw_raw = (yaw_raw_prev - yaw_prev)*100.0;

    // Yes, this is sketch.
    // The angle output runs from -pi to pi radians. 0 radians
    // corresponds to the upright position, and pi/-pi is the
    // downwards position. Due to this, the angle output is dis-
    // continuous when the pendulum points exactly downwards.
    //
    // To fix this, I check how big the calculated angular velocity
    // is, and if it exceeds pi radians per timestep (100 radians/s)
    // then we assume that the zero point was crossed and subtract 2*pi
    // from the result.

    static float last_iir_output = 0.0;

    if (yaw - yaw_prev > PI)
      dyaw = (yaw - yaw_prev - 2*PI)*100.0;
    else if (yaw - yaw_prev < -PI)
      dyaw = (yaw - yaw_prev + 2*PI)*100.0;
    else
      dyaw = (yaw - yaw_prev)*100.0;

    last_iir_output = 0.90 * last_iir_output + 0.10 * dyaw;
    dyaw_smooth = last_iir_output;

    return;
  }
  else {
    GPIO_PinState A_status = HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin);
    GPIO_PinState B_status = HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin);

    if ((GPIO_Pin == ROT_A_Pin) == (A_status == B_status))
      pulse_count++;
    else
      pulse_count--;
  }
}

RWP_MODE rwp_mode = RWP_MODE_HALTED;
RWP_DATA_KIND rwp_data = SENSOR_DATA;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  // 200Hz timer (for control and comms)
  if (htim == &htim5) {
    // Control variables
    static double a_p, v_f, v_p;
    static double KE_p, KE_f, PE_p;
    static double motor_output  = 0.0;


    /* ============== MOTOR CONTROL CODE ============= */

    // Calculate the current state (all rad or rad/s)
    v_f  = (400*3.14159265)/(1024.00)*pulse_count;
    a_p  = yaw;
    v_p  = dyaw;

    // Motor control value
    float ctrl_value = 0.0;

    // If we're within 10 degrees of the setpoint, activate the linear controller
    if ((a_p+0.069 < ANGLE_LIM) && (a_p+0.069 > -ANGLE_LIM)) {
      yaw_raw += 0.04;
      ctrl_value = -(100*yaw_raw + 0*dyaw_raw - 0*v_f);
    }
    // Otherwise, activate the swingup controller
    else {
      const float J_f = 0.0005825; // Reaction wheel moment of inertia as kg/m^2
//      const float J_p = 0.0256;    // Pendulum arm moment of inertia as kg/m^2
      const float J_p = 0.0200;    // Pendulum arm moment of inertia as kg/m^2
      const float K_g = 1.2255;    // Potential energy constant


      static double prev_output = 0.0;

      KE_p = v_p*v_p*J_p/2.0;
      KE_f = dyaw_smooth*dyaw_smooth*J_f/2.0;
      PE_p = K_g * (cos(a_p)-1);
      double total_energy = KE_p + KE_f + PE_p;

      // IIR lowpass filter :3
      double output = 0.2*total_energy + 0.8*prev_output;
      prev_output = output;

      ctrl_value = -(float)(output * dyaw_smooth * 100);
    }

    // Clip the control value
    if (ctrl_value > 0.7) ctrl_value = 0.7;
    else if (ctrl_value < -0.7) ctrl_value = -0.7;

    // Update motor output
    if (rwp_mode == RWP_MODE_AUTO) {
      // Set the motor output to the calculated control value if in auto mode
      setMotorOutput(ctrl_value);
    }
    else if (rwp_mode == RWP_MODE_MANUAL) {
      // Set motor output to user input if in manual mode
      setMotorOutput(motor_output);
    }
    else {
      setMotorOutput(0);
    }


    /* ============== COMMUNICATIONS CODE ============= */

    // If there's data available from the nRF24, get it!
    uint8_t fifo_status = readNRF24register(N24_FIFO_STATUS);
    if ((fifo_status & 0b00000001) == 0) {

      // Read the received packet into a frame get its contents
      RMT_MESSAGE_FRAME rmt_message;
      readNRF24payload(rmt_message.bytes, 16);
      int16_t raw_value = rmt_message.control_msg.pot_value;
      motor_output = raw_value/2048.0f;
      // and update the mode based on received packet
      rwp_mode = rmt_message.control_msg.req_mode;
      rwp_data = rmt_message.control_msg.req_data_type;

      // Clear, rebuild and set new acknowledge payload
      RWP_MESSAGE_FRAME rwp_message;
      memset(rwp_message.bytes, 0x00, 32);
      rwp_message.sensor_data.data_type = rwp_data;
      rwp_message.sensor_data.cur_mode = rwp_mode;
      if (rwp_data == SENSOR_DATA) {
        rwp_message.sensor_data.yaw = RAD_TO_DEG * yaw;
        rwp_message.sensor_data.pitch = RAD_TO_DEG * pitch;
        rwp_message.sensor_data.roll = RAD_TO_DEG * roll;
        rwp_message.sensor_data.rotary_encoder = pulse_count;
      }
      else {
        uint8_t control_mode = (a_p < ANGLE_LIM && a_p > -ANGLE_LIM) ? 1 : 0;
        rwp_message.control_data.control_mode = control_mode;
        rwp_message.control_data.ctrl_value   = ctrl_value;
        if (control_mode == 1) {
          rwp_message.control_data.v_a = yaw;
          rwp_message.control_data.v_b = dyaw;
          rwp_message.control_data.v_c = v_f;
        }
        else {
          rwp_message.control_data.v_a = PE_p;
          rwp_message.control_data.v_b = KE_p;
          rwp_message.control_data.v_c = KE_f;
        }
      }
      writeNRF24ackPayload(rwp_message.bytes, 16, 0);

      // Clear RX_DR interrupt
      writeNRF24register(N24_STATUS, 0b01000000);
    }

    // Reset pulse_count to get a relative measurement
    pulse_count = 0;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // Stop motor, then start PWM generation
  setMotorOutput(0.0f);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  // Startup tune
  HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_SET);
  setMotorBeep(nA4);
  HAL_Delay(120);
  setMotorBeep(nCs4);
  HAL_Delay(120);
  setMotorBeep(nE4);
  HAL_Delay(120);
  setMotorBeep(nA5);
  HAL_Delay(120);
  stopBeep();
  HAL_Delay(240);
  HAL_GPIO_WritePin(MOT_fwd_GPIO_Port, MOT_fwd_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOT_bwd_GPIO_Port, MOT_bwd_Pin, GPIO_PIN_RESET);
  setMotorBeep(nA5);
  HAL_Delay(120);
  setMotorBeep(nE4);
  HAL_Delay(120);
  setMotorBeep(nCs4);
  HAL_Delay(120);
  setMotorBeep(nA4);
  HAL_Delay(120);
  stopBeep();

  // Start the nRF24
  NRF24_init();

  // Start the BNO085
  BNO_startup();

  // Start checking the nRF24 after setting everything else up
  HAL_TIM_Base_Start_IT(&htim5);

  // Then start the rotation vector output
  BNO_startRotationVector();



  /* USER CODE END 2 */

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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 499;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNO_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROT_A_Pin ROT_B_Pin */
  GPIO_InitStruct.Pin = ROT_A_Pin|ROT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_bwd_Pin MOT_fwd_Pin */
  GPIO_InitStruct.Pin = MOT_bwd_Pin|MOT_fwd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
