/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HydraSense Smart Unit — Main application entry point
  *                   STM32F030K6T6 @ 48 MHz
  *
  *  Two build modes (select with the BRINGUP_TEST_MODE compile define):
  *    - BRINGUP_TEST_MODE defined  -> RGB + NTC + TDS + HX711 self-test,
  *                                    values exposed in g_test (Live Expr).
  *    - BRINGUP_TEST_MODE undefined-> full application (App_Init / App_Run).
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include "ws2812b.h"
#include "buzzer.h"
#include "hx711.h"
#include "tds_sensor.h"
#include "ntc_temp.h"
#include "battery.h"
#include "bma253.h"
#include "ble_jdy29.h"
#include "ble_protocol.h"
#include "rtc_manager.h"
#include "data_storage.h"
#include "device_state.h"
#include "app_logic.h"
#include "bringup_test.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* App workflow state is now compact and private inside app_logic.c. */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Defined in app_logic.c. Re-arms the JDY BLE UART's 1-byte IT reception
 * after a UART error aborted it (see HAL_UART_ErrorCallback below). Declared
 * here so main.c compiles without editing the CubeMX-owned header; you may
 * instead move this prototype into app_logic.h if you prefer. */
extern void App_BLE_RxRecover(void);
/* USER CODE END PFP */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  /* Calibrate ADC (important for accurate sensor readings on F030) */
  HAL_ADCEx_Calibration_Start(&hadc);

#ifdef BRINGUP_TEST_MODE
  /* ── HARDWARE BRING-UP MODE ──────────────────────────────────────────
   * Skips the full app/BLE stack and runs the RGB + NTC + TDS + HX711
   * self-test. Add 'g_test' to STM32CubeIDE -> Live Expressions to watch
   * every value update live over SWD. */
  BringUp_Init(&hadc, &htim1);
  BringUp_RunOnce();      /* deterministic RGB colour walk + first reads */
#else
  /* ── FULL APPLICATION ────────────────────────────────────────────────
   * Initialises every driver + state machine. */
  App_Init(&hadc,
           &hi2c1,
           &huart1,
           &htim1,   /* WS2812B — TIM1 CH1 DMA */
           &htim3);  /* Buzzer   — TIM3 CH2 PWM */
#endif

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
#ifdef BRINGUP_TEST_MODE
    BringUp_RunLoop();
#else
    App_Run();
#endif
    /* USER CODE END 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  */
static void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  *
  * NOTE: CubeMX generates TIM1 here in slave/external-clock mode because of the
  * .ioc setup. That is corrected at runtime inside WS2812B_Init() (it clears
  * SMCR and sets PSC/ARR for 800 kHz). Leaving this CubeMX block untouched
  * keeps the project regenerable from the .ioc.
  */
static void MX_TIM1_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM3 Initialization Function
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, HX711_SCK_Pin|TDS_DRIVE_Pin, GPIO_PIN_RESET);

  /* RTC 1 Hz tick (PA11) is the only rising-edge EXTI on port A now.
   * PA0 (was BMA253 MOTION_INT) is left as a plain input: the IMU motion
   * interrupt has been removed — drink detection is weight-based and the
   * BLE link-state line replaces it as the only asynchronous wake source. */
  GPIO_InitStruct.Pin = RTC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA0 former MOTION_INT — now an idle input with pull-down so the unused
   * BMA253 INT1 line cannot float and generate spurious activity. */
  GPIO_InitStruct.Pin = MOTION_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* BLE_STATE (PA15, JDY-23 link line) — interrupt on BOTH edges so a
   * connect (rising) or disconnect (falling) is delivered as an event
   * instead of being polled. This is the "Bluetooth packet" wake source
   * that took over the EXTI slot previously used by the IMU. */
  GPIO_InitStruct.Pin = BLE_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HX711_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HX711_SCK_Pin|TDS_DRIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CHRG_STAT_Pin|STDBY_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Physical button SW1 on PB3 — active LOW, internal pull-up. Polled in the
   * app for short / long(3s) / very-long(10s) press per the PRD reset spec. */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

/* ── Application-wide HAL callbacks ───────────────────────────────────────
 * These forward hardware interrupts to the right driver. They live here (not
 * in a driver file) so CubeMX-generated sections stay untouched.
 * app_logic.c exposes tiny ISR forwarders instead of a public context.
 * In BRINGUP_TEST_MODE the app state is not initialised, so the bodies are
 * compiled out — the bring-up test polls instead of using interrupts. */

#ifndef BRINGUP_TEST_MODE

/* GPIO EXTI:
 *   PA11 RTC_INT   — PCF8563 1 Hz tick.
 *   PA15 BLE_STATE — JDY-23 link up/down edge (replaces the old IMU motion
 *                    interrupt on PA0). Forwarded to the app so connect /
 *                    disconnect is handled as an event, not by polling. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == RTC_INT_Pin) {
    App_RTC_TickISR();
  } else if (GPIO_Pin == BLE_STATE_Pin) {
    App_BLE_LinkISR();
  }
}

/* UART RX complete: feed the BLE byte-framer (JDY BLE module on USART1). */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    App_BLE_RxISR();
  }
}

/* UART error: a long blocking operation (e.g. TARE, which bit-bangs the
 * HX711 for a while) can coincide with an RX overrun / noise / framing
 * error on the JDY BLE line. HAL's default behaviour on such an error is to
 * abort IT reception and NOT re-arm it — after which App_BLE_RxISR() never
 * fires again and the device silently ignores every command ("BLE stops
 * responding after TARE"). Clear the error flags and restart reception. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);   /* overrun  */
    __HAL_UART_CLEAR_NEFLAG(huart);    /* noise    */
    __HAL_UART_CLEAR_FEFLAG(huart);    /* framing  */
    __HAL_UART_CLEAR_PEFLAG(huart);    /* parity   */
    /* HAL has already set RxState = READY inside its error handling, so the
     * BLE driver can immediately re-arm HAL_UART_Receive_IT(). */
    App_BLE_RxRecover();
  }
}

#endif /* !BRINGUP_TEST_MODE */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
