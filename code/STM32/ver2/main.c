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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "iks4a1_motion_sensors.h"
#include "custom_bus.h"
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

/* USER CODE BEGIN PV */

int target_speed_L = 0;   // Target speed for the left motor (pulses per 25ms)
int target_speed_R = 0;   // Target speed for the right motor (pulses per 25ms)

float Kp = 12.0;          // Proportional gain
float Ki = 0.5;           // Integral gain

float calka_L = 0;        // Accumulated error (Integral) for the left motor
float calka_R = 0;        // Accumulated error (Integral) for the right motor
volatile float uchyb_L;   // Current error for the left motor
volatile float uchyb_R;   // Current error for the right motor

int16_t prev_licznik1 = 0;
int16_t prev_licznik2 = 0;
int32_t prev_czas = 0;

int robot_state = 0;      // '0'-STOP, '1'-FOREWARD, '2'-RIGHT, '3'-LEFT, '4'-BACK

int deadband = 400;       // Minimum PWM value to overcome motor static friction
float antyWindUp = 2000;  // Maximum limit for the integral term

volatile int16_t current_cnt1, current_cnt2;
volatile int16_t speed1, speed2;

volatile int in1 = 0, in2 = 0, in3 = 0, in4 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PWM CONTROL FUNCTION
// PC6=CH1, PC7=CH2 (Motor 1 - Left), PC8=CH3, PC9=CH4 (Motor 2 - Right)
void pwm(int s1_m1, int s1_m2, int s2_m1, int s2_m2) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, s1_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, s1_m2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, s2_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, s2_m2);
}

// FUNCTION TO CLAMP VALUES WITHIN A GIVEN RANGE
int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
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
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

      // SET TIM8 PERIOD
      __HAL_TIM_SET_AUTORELOAD(&htim8, 999);

      // START PWM FOR BOTH MOTORS (4 channels on TIM8)
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

      // START HARDWARE ENCODER COUNTING
      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

      // MOTORS STOPPED AT STARTUP
      pwm(0,0,0,0);

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (HAL_GetTick() - prev_czas >= 25) // Execute control loop every 25ms
	        {
	            prev_czas = HAL_GetTick();

	            // FETCH DATA FROM ENCODERS
	            current_cnt1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	            current_cnt2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

	            // CALCULATE SPEEDS
	            // Casting to int16_t ensures correct difference calculation upon counter overflow
	            speed1 = -(current_cnt1 - prev_licznik1);
	            speed2 = (current_cnt2 - prev_licznik2);

	            prev_licznik1 = current_cnt1;
	            prev_licznik2 = current_cnt2;

	            // 1. SET TARGET SPEEDS BASED ON ROBOT STATE
	            if (robot_state == 0) // STOP
				{
					target_speed_L = 0;
					target_speed_R = 0;
				}
				else if (robot_state == 1) // FOREWARD
				{
					target_speed_L = 22;
					target_speed_R = 22;
				}
				else if (robot_state == 2) // RIGHT (point turn)
				{
					target_speed_L = 22;
					target_speed_R = -22;
				}
				else if (robot_state == 3) //  LEFT (point turn)
				{
					target_speed_L = -22;
					target_speed_R = 22;
				}
				else if (robot_state == 4) // BACK
				{
					target_speed_L = -22;
					target_speed_R = -22;
				}

				// 2. PI CONTROLLER FOR LEFT MOTOR
				uchyb_L = (float)target_speed_L - (float)speed1;
				calka_L += uchyb_L;

				// Anti-windup for the left motor
				if (calka_L > antyWindUp) calka_L = antyWindUp;
				if (calka_L < -antyWindUp) calka_L = -antyWindUp;

				// Force clear the integral term when stopped
				if (robot_state == 0) calka_L = 0;

				int p1 = (int)(uchyb_L * Kp + calka_L * Ki);

				// 3. PI CONTROLLER FOR RIGHT MOTOR
				uchyb_R = (float)target_speed_R - (float)speed2;
				calka_R += uchyb_R;

				// Anti-windup for the right motor
				if (calka_R > antyWindUp) calka_R = antyWindUp;
				if (calka_R < -antyWindUp) calka_R = -antyWindUp;

				// Force clear the integral term when stopped
				if (robot_state == 0) calka_R = 0;

				int p2 = (int)(uchyb_R * Kp + calka_R * Ki);

				// 4. MAP PI OUTPUTS TO H-BRIDGE CHANNELS WITH DEADBAND COMPENSATION

				// Left motor channels (CH1, CH2)
				if (target_speed_L == 0) {
				    in1 = 0;
				    in2 = 0;
				} else if (p1 >= 0) {
				    in1 = clamp(p1 + deadband, 0, 1000); // Forward + deadband
				    in2 = 0;
				} else {
				    in1 = 0;
				    in2 = clamp(-p1 + deadband, 0, 1000); // Backward + deadband
				}

				// Right motor channels (CH3, CH4)
				if (target_speed_R == 0) {
				    in3 = 0;
				    in4 = 0;
				} else if (p2 >= 0) {
				    in3 = clamp(p2 + deadband, 0, 1000); // Forward + deadband
				    in4 = 0;
				} else {
				    in3 = 0;
				    in4 = clamp(-p2 + deadband, 0, 1000); // Backward + deadband
				}

				// Apply calculated values to the motors
				pwm(in1, in2, in3, in4);

	        }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		static uint32_t last_interrupt_time = 0;
		if (HAL_GetTick() - last_interrupt_time > 200) // Debounce check
			{
			robot_state++;

			// Reset hardware timers
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			__HAL_TIM_SET_COUNTER(&htim3, 0);

			// Reset software tracking variables
			current_cnt1 = current_cnt2 = 0;
			prev_licznik1 = prev_licznik2 = 0;

			// Reset accumulated errors on state change to prevent sudden jerks
			calka_L = 0;
			calka_R = 0;

			if (robot_state > 4) robot_state = 0;
			}
		last_interrupt_time = HAL_GetTick();
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
