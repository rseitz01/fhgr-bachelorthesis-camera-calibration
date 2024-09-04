/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "tlc.h"
#include "auto_mode.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#define LED_MAX 					16
#define LED_MAX_IS_POWER_OF_TWO 	true
#define LED_MAX_MASK 	0x1F

#define PWM_MAX 					TLC_PWM_MAX

static int led_index = 0;
bool led_data[LED_MAX] = {
		1,0,0,1,
		0,1,1,0,
		1,1,0,0,
		0,0,1,1,
};
int led_pwm = 1;
int led_pwm_count = 0;
bool led_pwm_active = true;

bool led_oe = false;
bool led_le = false;

// TODO put this all into some struct!!!

TlcModeList led_mode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void it_mode_rider();
void it_mode_pwm();
void it_mode_display();
void it_mode_none();
void it_mode_clear();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef void (*VoidFunction)(void);

VoidFunction led_functions[TLC_MODE__COUNT] = {
		[TLC_MODE_NONE] = &it_mode_none,
		[TLC_MODE_RIDER] = &it_mode_rider,
		[TLC_MODE_PWM] = &it_mode_pwm,
		[TLC_MODE_DISPLAY] = &it_mode_display,
		[TLC_MODE_CLEAR] = &it_mode_clear,
};


void it_mode_rider()
{
	HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, led_index & 1);
	bool first = (led_index < 2);
	HAL_GPIO_WritePin(TLC5926_SDI_GPIO_Port, TLC5926_SDI_Pin, first);

	/*DRY*/
	++led_index;
#if LED_MAX_IS_POWER_OF_TWO
	led_index &= LED_MAX_MASK;
#else
	if(led_index >= 2*LED_MAX) {	// TODO: maybe possible to replace with modulo?
		led_index = 0;
	}
#endif
}

void it_mode_pwm()
{
	HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, led_index & 1);

#if 1
	bool set_to = (led_index < 2 * led_pwm);
	HAL_GPIO_WritePin(TLC5926_SDI_GPIO_Port, TLC5926_SDI_Pin, set_to);

	++led_index;
#if LED_MAX_IS_POWER_OF_TWO
	led_index &= LED_MAX_MASK;
#else
	if(led_index >= 2*LED_MAX) {
		led_index = 0;
	}
#endif
#else
	bool set_to = (led_pwm_count < led_pwm);
	HAL_GPIO_WritePin(TLC5926_SDI_GPIO_Port, TLC5926_SDI_Pin, set_to);

	++led_index;

	if(led_index >= 2*LED_MAX) {
		//led_pwm_count += LED_MAX;
		++led_pwm_count;
		led_index = 0;
		//HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_SET); /* high LE = transfer to latch */
		//HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_RESET); /* low LE = don't transfer to latch */
		if(led_pwm_count >= PWM_MAX) {
			//led_pwm_count %= PWM_MAX;
			led_pwm_count = 0;
			//HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, true); // TODO:TEMP
		}
	}
#endif
}

void it_mode_display()
{
	HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, led_index & 1);
	bool set_to = led_data[led_index >> 1];
	HAL_GPIO_WritePin(TLC5926_SDI_GPIO_Port, TLC5926_SDI_Pin, set_to);

	/*DRY*/
	++led_index;

	if(led_index >= 2*LED_MAX) {
		HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_SET); /* high LE = transfer to latch */
		HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_RESET); /* low LE = don't transfer to latch */
		led_index = 0;
		//HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, true); // TODO:TEMP
	}
}

void it_mode_none()
{
	/* do nothing */
}

void it_mode_clear()
{
	HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, led_index & 1);
	bool set_to = 0;
	HAL_GPIO_WritePin(TLC5926_SDI_GPIO_Port, TLC5926_SDI_Pin, set_to);

	/*DRY*/
	++led_index;

	if(!(led_index % (2*LED_MAX))) {
		HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_SET); /* high LE = transfer to latch */
		HAL_GPIO_WritePin(TLC5926_LE_GPIO_Port, TLC5926_LE_Pin, GPIO_PIN_RESET); /* low LE = don't transfer to latch */
		//led_index = 0;
	}
	if(led_index >= 4*LED_MAX) {
		led_index = 0;
		led_mode = TLC_MODE_NONE;
		//HAL_GPIO_WritePin(TLC5926_CLK_GPIO_Port, TLC5926_CLK_Pin, true); // TODO:TEMP
	}
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	//NVIC_SystemReset();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

	Tlc tlc = {0};
	AutoMode auto_mode = {0};
	main_exchange(false, &tlc, &auto_mode);
	auto_mode_activate(&auto_mode, tlc.freq.fps);

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_Int_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  led_functions[led_mode]();

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USB high priority global interrupt.
  */
void USB_HP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_HP_IRQn 0 */

  /* USER CODE END USB_HP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_HP_IRQn 1 */

  /* USER CODE END USB_HP_IRQn 1 */
}

/**
  * @brief This function handles USB low priority global interrupt.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
