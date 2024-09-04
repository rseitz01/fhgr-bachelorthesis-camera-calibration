/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include "tlc.h"
#include "timer_estimator.h"
#include "cli.h"
#include "auto_mode.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*#define MAX_LED 	16
extern bool led_data[MAX_LED];
extern int led_pwm;
extern int led_pwm_active;*/

extern USBD_HandleTypeDef hUsbDeviceFS;
extern size_t cdc_rx_buf_len;
extern char cdc_rx_buf[64];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void main_exchange(bool set, Tlc *tlc, AutoMode *auto_mode)
{
	static Tlc *tlc_stored;
	static AutoMode *auto_mode_stored;
	if(set) {
		tlc_stored = tlc;
		auto_mode_stored = auto_mode;
	} else {
		*tlc = *tlc_stored;
		*auto_mode = *auto_mode_stored;
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
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  //USBD_Clock_Config()

  //TIM2->CCR1 = 0;
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  //HAL_UART_Receive_IT(&huart2, UART2_RxBuffer, 1); 	// somehow is required to start HAL_UART_RxCpltCallback

  /* tlc enters normal mode on startup by default */
  //HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_SET); /* high LE = transfer to latch */
  HAL_GPIO_WritePin(TLC5926_OE_GPIO_Port, TLC5926_OE_Pin, GPIO_PIN_RESET); /* low OE = drive LEDs */
  HAL_GPIO_WritePin(TLC5926_SDI_GPIO_Port, TLC5926_SDI_Pin, GPIO_PIN_SET);

  HAL_Delay(200);
  HAL_GPIO_WritePin(USB_Reconnect_GPIO_Port, USB_Reconnect_Pin, GPIO_PIN_SET);

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  //HAL_Delay(500);


  /*
   * configure
   * 	arr		auto reload (=VAL =auto)
   * 	psc		prescaler (=VAL =auto)
   *	fps		target fps =VAL
   * 	can-bus
   * 	int-in	(=COMMAND)
   * 	int-out
   *
   * info
   * 	version
   * 	clock-freq
   * 	arr
   * 	psc
   * 	fps
   *	can-bus		list connected...
   *
   */

  Tlc tlc = {0};
  tlc_init(&tlc, TIM2, &htim2);

  AutoMode auto_mode = {0};

  Cli cli = {0};
  cli_init(&cli, &tlc, &auto_mode);

  main_exchange(true, &tlc, &cli);

  tlc_set_fps(&tlc, 60.0f);

#if 0
  TimerEstimator est = {0};
  uint32_t iter = timer_estimate(&est, 72000000, 1920);

  TIM2->PSC = 9; //(32)*base_MHZ_reciproc - 1; // 31(+1) => 1MHz counter
  TIM2->ARR = 1747; // !!! ARR @8MHz for -WHATEVER REASON- min=117 (regardless of PSC) !!!
  TIM2->CCR1 = TIM2->ARR/2;
#endif

  /* TEST for pwm */
#if 0
  TIM2->PSC = 0;
  TIM2->ARR = 150;
  TIM2->CCR1 = TIM2->ARR/2;

  tlc.running = true;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
#endif
  //HAL_TIM_Base_Start(&htim2);

#if 0
  int usbd_result = 0;
  do {
	  usbd_result = USBD_CDC_SetTxBuffer(&hUsbDeviceFS, "Hello\n\r", 7);
  } while(usbd_result != USBD_OK);
#endif

#if 0
  bool data[16] = {1, 1, 0, 1,
				   0, 0, 1, 0,
				   0, 0, 1, 1,
				   1, 1, 0, 1};

  tlc_set_fps(&tlc, 1.0);
  tlc_mode(&tlc, TLC_MODE_PWM);
  //memcpy(tlc.display.data, data, 16);
  tlc_run(&tlc, true);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_Delay(1000);
	  //*tlc.pwm.duty_cycle += 1;
	  //if(*tlc.pwm.duty_cycle > 16) *tlc.pwm.duty_cycle = 0;

	  if(cdc_rx_buf_len) {
		  while(CDC_Transmit_FS(cdc_rx_buf, cdc_rx_buf_len) == USBD_BUSY) {};
		  //while(CDC_Transmit_FS(CSTR("\r\nFeed..\r\n")) == USBD_BUSY) {};
		  cli_feed(&cli, cdc_rx_buf, cdc_rx_buf_len);	 // TODO:RETVAL
		  //while(CDC_Transmit_FS(CSTR("Parse..\r\n")) == USBD_BUSY) {};
		  cli_parse(&cli);	// TODO:RETVAL
		  cli_process(&cli);
		  cdc_rx_buf_len = 0;
	  }
	  auto_mode_process(&auto_mode, &tlc);

#if 0
	  uint32_t tick = HAL_GetTick();
	  if(tick >= 1250) {
		  //USBD_CDC_SetTxBuffer(&hUsbDeviceFS, "Hello\n\r", 7);
		  led_pwm = -1;
		  HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_SET); /* high LE = transfer to latch */
		  //USBD_CDC_SetTxBuffer(&hUsbDeviceFS, "Hello\n\r", 7);
		  //USBD_CDC_TransmitPacket(&hUsbDeviceFS);
		  //HAL_Delay(1000);
	  } else if(tick >= 1000) {
		  for(int i = 0; i < MAX_LED; ++i) {
			  bool val = ((TIM2->ARR>>i)&1);
			  led_data[i] = val;
		  }
		  led_pwm = 0;
	  } else if(tick >= 750) {
		  for(int i = 0; i < MAX_LED; ++i) {
			  bool val = ((TIM2->PSC>>i)&1);
			  led_data[i] = val;
		  }
		  led_pwm = 0;
	  } else if(tick >= 500) {
		  for(int i = 0; i < MAX_LED; ++i) {
			  bool val = ((72>>i)&1);
			  led_data[i] = val;
		  }
		  led_pwm = 0;
	  } else if(tick >= 250) {
		  for(int i = 0; i < MAX_LED; ++i) {
			  bool val = ((3>>i)&1);
			  led_data[i] = val;
		  }
		  led_pwm = 0;
	  } else {
		  for(int i = 0; i < MAX_LED; ++i) {
			  bool val = 1;
			  led_data[i] = val;
		  }
		  led_pwm = 0;
	  }
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TLC5926_CLK_Pin|TLC5926_SDI_Pin|TLC5926_LE_Pin|TLC5926_OE_Pin
                          |GPIO_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_Reconnect_GPIO_Port, USB_Reconnect_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TLC5926_CLK_Pin TLC5926_SDI_Pin TLC5926_LE_Pin TLC5926_OE_Pin */
  GPIO_InitStruct.Pin = TLC5926_CLK_Pin|TLC5926_SDI_Pin|TLC5926_LE_Pin|TLC5926_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_Int_Pin */
  GPIO_InitStruct.Pin = GPIO_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_Int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_Out_Pin */
  GPIO_InitStruct.Pin = GPIO_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_Reconnect_Pin */
  GPIO_InitStruct.Pin = USB_Reconnect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_Reconnect_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
