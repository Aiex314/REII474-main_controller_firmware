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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
SPI_HandleTypeDef*  hspi_bno  = &hspi2;
TIM_HandleTypeDef*  htim_mot  = &htim4;

// ======================== CONSTANTS =======================
// pi
#define PI 3.14159265358979323
// Angle conversion constants
const float DEG_TO_RAD = PI/180.0;
const float RAD_TO_DEG = (180.0/PI);
const float RDPS_TO_RPM = 60/(2*PI);

// Maximum controller output
#define MAX_CTRL 1.0
// Controller frequency
#define CTRL_FREQ 100.0
// Hysteresis limits for linear controller
const float LIM_LIN_START_ANGLE  = 3.5*DEG_TO_RAD;
const float LIM_LIN_START_SPEED  = 0.1;
const float LIM_LIN_STOP_ANGLE = 7.5*DEG_TO_RAD;
const float LIM_LIN_STOP_SPEED = 999.0;
// Yaw offset (radians)
float YAWR_OFFSET = 3.30*DEG_TO_RAD;
float YAW_OFFSET  = 0;
// Linear region sensitivity
float LINEAR_SENSITIVITY = 0.718;

// Rotary encoder pulses per rotation, and conversion constant
#define ROTARY_ENC_PPR 8192.0
const float ROT_E_TO_ROT_VEL = 2.0*PI*CTRL_FREQ/ROTARY_ENC_PPR;

// Physical parameters of RWP
const float J_f = 0.000058;   // Reaction wheel moment of inertia as kg/m^2
const float J_p = 0.0210;     // Pendulum arm moment of inertia as kg/m^2
const float K_g = 1.3726;     // Potential energy constant
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void setMotorOutput(float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Raw outputs of IMU
float yaw, yaw_raw;
float dyaw, dyaw_raw, dyaw_smooth;
float pitch, roll;
// Rotary encoder output
int16_t pulse_count = 0;

// Wireless comms synchronization crap
uint8_t last_cmd_counter = 0, last_imu_counter = 0;

// There are two sources of pin interrupts. These are the IMU's IRQ and the rotary encoder.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // IMU interrupt request
  if (GPIO_Pin == BNO_IRQ_Pin) {
    // Yes, this is sketch.
    //
    // The IMU measurement is very non-linear, so I put it through
    // a lookup table to linearise. This introduces a lot of noise,
    // so an IIR is used to denoise. The phase lag this introduces
    // is too much for the linear region, so a separate set of raw
    // values is maintained as well...
    //
    // Next,
    // the angle output runs from -pi to pi radians. 0 radians corr-
    // esponds to the upright position, and pi/-pi is the downwards
    // position. Due to this, the angle output is discontinuous when
    // the pendulum points exactly downwards. This  breaks numeric
    // differentiation, giving massive spikes for dyaw. To fix this,
    // I check how big the calculated angular velocity is, and if it
    // exceeds +-pi radians per timestep (100 radians/s) then we ass-
    // ume that the zero point was crossed and subtract/add 2*pi from -
    // the result before using it to calculate dyaw.
    //
    // It's hacky, but I've tested it...

    // Save previously measured values
    float yaw_prev = yaw, yaw_raw_prev = yaw_raw;

    // Update for synchronisation
    last_imu_counter = 0;

    // Get the raw IMU values
    BNO_IRQcallback();
    BNO_getYPR(&yaw_raw, &pitch, &roll);

    // Calculate raw dyaw for linear region
    // This is only used when yaw is smol, so adjusting for
    // the discontinuity is not necessary
    dyaw_raw = (yaw_raw - yaw_raw_prev)*100.0;

    // Correct the plain yaw for nonlinearity
    yaw = motorLinearizationLookup(yaw_raw);

    // Calculate plain dyaw, being aware of the discontinuity
    if (yaw - yaw_prev > PI)
      dyaw = (yaw - yaw_prev - 2*PI)*100.0;
    else if (yaw - yaw_prev < -PI)
      dyaw = (yaw - yaw_prev + 2*PI)*100.0;
    else
      dyaw = (yaw - yaw_prev)*100.0;

    // IIR filter for dyaw
    static float last_iir_output = 0.0;
    last_iir_output = 0.80 * last_iir_output + 0.20 * dyaw;
    dyaw_smooth = last_iir_output;

    return;
  }
  // Rotary encoder tick
  else {
    // Get the rotary encoder pin levels
    GPIO_PinState A_status = HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin);
    GPIO_PinState B_status = HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin);

    // If pin A triggered, the pins are the same when rotating CCW
    // If pin B triggered, the pins are different when rotating CCW
    // If the pins have the opposite sense to the above, rotation is CW
    // This can elegantly be expressed as the below
    if ((GPIO_Pin == ROT_A_Pin) == (A_status == B_status))
      pulse_count++;
    else
      pulse_count--;
  }
}

RWP_MODE rwp_mode = RWP_MODE_HALTED;
RWP_DATA_KIND rwp_data = SENSOR_DATA;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  // Control variables
  static float  user_input = 0.0;
  static double a_p, a_pl, v_f, v_p, v_pl;
  static double KE_p, KE_f, PE_p, total_energy;
  static float  motor_output = 0.0;
  static uint8_t  auto_mode  = 0;

  // Communication variables
  static uint8_t missed_packets = 0;

  // 100Hz timer for control
  if (htim == &htim5) {
    /* ============== MOTOR CONTROL CODE ============= */

    // Calculate the current state (all rad or rad/s)
    a_p  = yaw - YAW_OFFSET;             // Linearised theta
    a_pl = yaw_raw - YAWR_OFFSET;        // Raw theta
    v_p  = dyaw_smooth;                  // Linearised omega_p
    v_pl = dyaw_raw;                     // Raw omega_p
    v_f  = -ROT_E_TO_ROT_VEL*pulse_count; // Reaction wheel vel
    float v_f_abs = v_f + v_p; // Absolute velocity of reaction wheel

    // System energy estimation (all J)
    KE_p = v_p*v_p*J_p/2.0;
    KE_f = v_f*v_f*J_f/2.0;
    PE_p = K_g * (cos(a_p)-1);
    total_energy = KE_p + KE_f + PE_p;

    // Reset pulse_count to get a relative measurement
    pulse_count = 0;

    // Speed and angle hysteresis for linear controller
    if (auto_mode == 1) {
      // If outside of speed or angle limits, disable linear controller
      if (a_pl > LIM_LIN_STOP_ANGLE || a_pl < -LIM_LIN_STOP_ANGLE || v_p > LIM_LIN_STOP_SPEED || v_p < -LIM_LIN_STOP_SPEED)
        auto_mode = 0;
    }
    else {
      // If inside of speed and angle limits, enable linear controller
      if (a_pl > -LIM_LIN_START_ANGLE && a_pl < LIM_LIN_START_ANGLE && v_p > -LIM_LIN_STOP_SPEED && v_p < LIM_LIN_STOP_SPEED)
        auto_mode = 1;
    }


    if (rwp_mode == RWP_MODE_CALIBRATION) {
      YAWR_OFFSET = yaw_raw;
      YAW_OFFSET  = yaw;
      motor_output = 0.2*v_f;
    }


    // If in manual mode, set the output to the potentiometer value
    if (rwp_mode == RWP_MODE_MANUAL)
      motor_output = user_input;
    // If in halt mode, set the output to zero
    else if (rwp_mode == RWP_MODE_HALTED)
      motor_output = 0;
    // If the pendulum state is within acceptable bounds, use linear controller
    else if (rwp_mode == RWP_MODE_AUTO && auto_mode == 1)
      motor_output = -(28.28*a_pl + 4.03*v_pl + 0.0317*v_f);
//      motor_output = -(100*a_pl + 16*v_pl - 0.03*v_f);
    // Otherwise, activate the swingup controller
    else if  (rwp_mode == RWP_MODE_AUTO && auto_mode != 1) {
      // IIR lowpass filter :3
      static double prev_output = 0.0;
      double output = 0.3*total_energy + 0.7*prev_output;
      prev_output = output;

      motor_output = -(float)(output * v_p);
    }
    // If the rotor lock mode is enabled, resist any rotation of the rotor
    else
      motor_output = 0.1*v_f;
//
    // Clip the control value
    if (motor_output > MAX_CTRL) motor_output = MAX_CTRL;
    else if (motor_output < -MAX_CTRL) motor_output = -MAX_CTRL;

    // Update motor output
    setMotorOutput(motor_output);
  }
  // 1kHz timer for communications
  else if (htim == &htim9) {
    /* ============== COMMUNICATIONS CODE ============= */
    // Increment synchronization counters
//    ++last_imu_counter;
    ++last_cmd_counter;

    // If the last or next IMU irq is less than 20ms away, cancel the poll
//    if (last_imu_counter < 3 || last_imu_counter > 7) return;

    // If there's data available from the nRF24, get it!
    uint8_t fifo_status = readNRF24register(N24_FIFO_STATUS);
    if ((fifo_status & 0b00000001) == 0) {
      HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

      // Packet has been received, update appropriate variables
      last_cmd_counter = 0;
      missed_packets = 0;

      // Read the received packet into a frame get its contents
      RMT_MESSAGE_FRAME rmt_message;
      readNRF24payload(rmt_message.bytes, 16);
      int16_t raw_value = rmt_message.control_msg.pot_value;
      user_input = raw_value/2048.0f;
      // and update the mode based on received packet
      rwp_mode = rmt_message.control_msg.req_mode;
      rwp_data = rmt_message.control_msg.req_data_type;

      // Clear, rebuild and set new acknowledge payload
      RWP_MESSAGE_FRAME rwp_message;
      memset(rwp_message.bytes, 0x00, 32);
      rwp_message.sensor_data.data_type = rwp_data;
      rwp_message.sensor_data.cur_mode = rwp_mode;
      if (last_imu_counter > 10) {
        memset(rwp_message.bytes, 0xff, 32);
      }
      else if (rwp_data == SENSOR_DATA) {
        rwp_message.sensor_data.yaw   = RAD_TO_DEG * yaw;
        rwp_message.sensor_data.pitch = RAD_TO_DEG * pitch;
        rwp_message.sensor_data.roll  = RAD_TO_DEG * roll;
        rwp_message.sensor_data.RPM   = RDPS_TO_RPM * v_f;
      }
      else {
        uint8_t control_mode = auto_mode;
        if (rwp_mode != RWP_MODE_AUTO) control_mode = 2;
        rwp_message.control_data.control_mode = control_mode;
        rwp_message.control_data.ctrl_value   = motor_output;
        rwp_message.control_data.v_a = a_p;
        rwp_message.control_data.v_b = v_p;
        rwp_message.control_data.v_c = v_f;
        rwp_message.control_data.v_d = PE_p;
        rwp_message.control_data.v_e = KE_p;
        rwp_message.control_data.v_f = KE_f;
      }
      writeNRF24ackPayload(rwp_message.bytes, 32, 0);

      // Clear RX_DR interrupt
      writeNRF24register(N24_STATUS, 0b01000000);

      HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_SET);
    }

    // Check for missed packets (10 consecutive polls with no packet)
    if (last_cmd_counter > 2) {
      last_cmd_counter = 0;

      // If we miss three packets in a row, stop
      if (missed_packets >= 3) {
        rwp_mode = RWP_MODE_HALTED;

        missed_packets = ((missed_packets-9)%25)+10;

        // Blink if there is a consistent wireless comms failure
        if (missed_packets == 10) HAL_GPIO_TogglePin(BLINK_GPIO_Port, BLINK_Pin);
      }
      else {
         ++missed_packets;
      }
    }
  }
  else if (htim == htim_mot) {
    if (next_sample() == 0)
      HAL_TIM_Base_Stop_IT(htim_mot);
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
  MX_TIM9_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // Stop motor, then start PWM generation
  setMotorOutput(0.0f);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // Start the nRF24
  NRF24_init();
  HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim9);

  // Play a startup tune
  playStartupTune();

  // Start the BNO085
  BNO_startup();

  // Then start the rotation vector output
  BNO_startRotationVector();

  // Finally, enable the motor controller after setting everything else up
  HAL_TIM_Base_Start_IT(&htim5);




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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim5.Init.Period = 999;
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
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 499;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, BNO_CSN_Pin|BNO_WAKE_Pin|NRF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BNO_RESET_Pin|MOT_fwd_Pin|MOT_bwd_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLINK_Pin */
  GPIO_InitStruct.Pin = BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLINK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROT_A_Pin ROT_B_Pin */
  GPIO_InitStruct.Pin = ROT_A_Pin|ROT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BNO_CSN_Pin BNO_RESET_Pin BNO_WAKE_Pin NRF_CSN_Pin
                           MOT_fwd_Pin MOT_bwd_Pin */
  GPIO_InitStruct.Pin = BNO_CSN_Pin|BNO_RESET_Pin|BNO_WAKE_Pin|NRF_CSN_Pin
                          |MOT_fwd_Pin|MOT_bwd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_IRQ_Pin */
  GPIO_InitStruct.Pin = BNO_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNO_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
