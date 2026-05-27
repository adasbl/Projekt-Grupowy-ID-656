/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
volatile int16_t current_cnt1, current_cnt2;
volatile int16_t speed1, speed2;

int16_t prev_licznik1 = 0;
int16_t prev_licznik2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FUNKCJA STEROWANIA PWM
// PC6=CH1, PC7=CH2 (Silnik 1), PC8=CH3, PC9=CH4 (Silnik 2)
void pwm(int s1_m1, int s1_m2, int s2_m1, int s2_m2) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, s1_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, s1_m2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, s2_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, s2_m2);
}

// FUNKCJA OGRANICZAJĄCA WARTOŚCI DO DANEGO PRZEDZIALU
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
  MX_USART3_UART_Init(); // Zostawiamy inicjalizację sprzętową CubeMX żeby nie psuć projektu
  /* USER CODE BEGIN 2 */


  __HAL_TIM_SET_AUTORELOAD(&htim8, 999);		// ustawianie okresu

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);		// start PWM dla obu silników (4 kanały na TIM8)
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);


  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	// start sprzętowego zliczania enkoderów
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Wyrzucone nasłuchiwanie UART (HAL_UART_Receive_IT)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. POBIERANIE STANÓW ENKODERÓW
      current_cnt1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
      current_cnt2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

      speed1 = current_cnt1 - prev_licznik1;
      speed2 = -(current_cnt2 - prev_licznik2);

      prev_licznik1 = current_cnt1;
      prev_licznik2 = current_cnt2;

      // 2. SZTYWNE WYMUSZENIE RUCHU (Bez żadnej matematyki)
      // Wysterowanie na 50% mocy (500 z 1000).
      // Jeśli silniki kręcą się w złą stronę, zamień 500 z 0.
      pwm(900, 0, 900, 0);

      HAL_Delay(50);

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
// Wyrzucone funkcje HAL_UART_RxCpltCallback oraz HAL_UART_ErrorCallback
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
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
