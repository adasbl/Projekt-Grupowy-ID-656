/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Regulator PI sterowany przez ramki UART (v_lin, v_ang)
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
#include <string.h> // Do memcpy
// #include "iks4a1_motion_sensors.h" // Odkomentuj jesli faktycznie uzywasz
// #include "custom_bus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * Struktura ramki z ROS:
 * [0]   : 0xAA (start)
 * [1-4] : float linear velocity (m/s)
 * [5-8] : float angular velocity (rad/s)
 * [9]   : 0x55 (stop)
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t header;
    float v_lin;
    float v_ang;
    uint8_t footer;
} TelemetryFrame_t;
#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ZMIENNE KINEMATYCZNE - DO DOSTOSOWANIA DO FIZYCZNEGO ROBOTA
#define WHEEL_BASE_M       0.148f      // Rozstaw kół w metrach (d)
#define TICKS_PER_METER    16262.0f   // Ile impulsów enkodera odpowiada 1 metrowi
#define DT_S               0.025f    // Czas pętli sterowania (25 ms)
// #define TIMEOUT_MS         500       // Zatrzymanie silników po 500 ms bez komend z UART

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Zmienne UART
uint8_t rx_buffer[10];
uint8_t rx_index = 0;
volatile uint8_t new_data_flag = 0;
TelemetryFrame_t rxFrame;
uint32_t last_cmd_time = 0;

// Zmienne sterowania
int target_speed_L = 0;
int target_speed_R = 0;

float Kp = 12.0;
float Ki = 0.5;

float calka_L = 0;
float calka_R = 0;
volatile float uchyb_L;
volatile float uchyb_R;

int16_t prev_licznik1 = 0;
int16_t prev_licznik2 = 0;
int32_t prev_czas = 0;

int deadband = 400;
float antyWindUp = 2000;

volatile int16_t current_cnt1, current_cnt2;
volatile int16_t speed1, speed2;

volatile int in1 = 0, in2 = 0, in3 = 0, in4 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void pwm(int s1_m1, int s1_m2, int s2_m1, int s2_m2);
int clamp(int value, int min, int max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void pwm(int s1_m1, int s1_m2, int s2_m1, int s2_m2) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, s1_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, s1_m2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, s2_m1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, s2_m2);
}

int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  // Konfiguracja TIM8
  __HAL_TIM_SET_AUTORELOAD(&htim8, 999);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  // Start Enkoderów
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Silniki zatrzymane na starcie
  pwm(0,0,0,0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  // Start nasłuchiwania UART
  rx_index = 0;
  HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
  last_cmd_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. ODBIÓR I PRZELICZENIE DANYCH Z UART
      if (new_data_flag == 1)
      {
          new_data_flag = 0;
          last_cmd_time = HAL_GetTick();

          // Obliczenie prędkości liniowych dla kół (m/s) z kinematyki
          float v_L_mps = -rxFrame.v_lin + (rxFrame.v_ang * WHEEL_BASE_M / 2.0f);
          float v_R_mps = -rxFrame.v_lin - (rxFrame.v_ang * WHEEL_BASE_M / 2.0f);

          // Przeliczenie m/s na impulsy na krok czasowy 25ms
          target_speed_L = (int)(v_L_mps * TICKS_PER_METER * DT_S);
          target_speed_R = (int)(v_R_mps * TICKS_PER_METER * DT_S);
      }

      // 2. WATCHDOG BEZPIECZEŃSTWA (Zatrzymanie przy braku komunikacji)
      /*
       if (HAL_GetTick() - last_cmd_time > TIMEOUT_MS)
      {
          target_speed_L = 0;
          target_speed_R = 0;
      }
      */

      // 3. PĘTLA REGULATORA PI (25 ms)
      if (HAL_GetTick() - prev_czas >= 25)
      {
          prev_czas = HAL_GetTick();

          // POBRANIE DANYCH Z ENKODERÓW
          current_cnt1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
          current_cnt2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

          speed1 = -(current_cnt1 - prev_licznik1);
          speed2 = (current_cnt2 - prev_licznik2);

          prev_licznik1 = current_cnt1;
          prev_licznik2 = current_cnt2;

          // PI DLA LEWEGO SILNIKA
          uchyb_L = (float)target_speed_L - (float)speed1;
          calka_L += uchyb_L;

          if (calka_L > antyWindUp) calka_L = antyWindUp;
          if (calka_L < -antyWindUp) calka_L = -antyWindUp;

          if (target_speed_L == 0) calka_L = 0; // Czyszczenie całki przy postoju

          int p1 = (int)(uchyb_L * Kp + calka_L * Ki);

          // PI DLA PRAWEGO SILNIKA
          uchyb_R = (float)target_speed_R - (float)speed2;
          calka_R += uchyb_R;

          if (calka_R > antyWindUp) calka_R = antyWindUp;
          if (calka_R < -antyWindUp) calka_R = -antyWindUp;

          if (target_speed_R == 0) calka_R = 0; // Czyszczenie całki przy postoju

          int p2 = (int)(uchyb_R * Kp + calka_R * Ki);

          // MAPOWANIE NA KANAŁY MOSTKA H (z kompensacją martwej strefy)

          // Silnik Lewy
          if (target_speed_L == 0) {
              in1 = 0; in2 = 0;
          } else if (p1 >= 0) {
              in1 = clamp(p1 + deadband, 0, 1000);
              in2 = 0;
          } else {
              in1 = 0;
              in2 = clamp(-p1 + deadband, 0, 1000);
          }

          // Silnik Prawy
          if (target_speed_R == 0) {
              in3 = 0; in4 = 0;
          } else if (p2 >= 0) {
              in3 = clamp(p2 + deadband, 0, 1000);
              in4 = 0;
          } else {
              in3 = 0;
              in4 = clamp(-p2 + deadband, 0, 1000);
          }

          // Wyślij sygnały do silników
          pwm(in1, in2, in3, in4);
      }
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rx_index == 0)
        {
            if (rx_buffer[0] == 0xAA) {
                rx_index = 1;
                HAL_UART_Receive_IT(&huart3, &rx_buffer[1], 9);
            } else {
                HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
            }
        }
        else
        {
            if (rx_buffer[9] == 0x55) {
                memcpy(&rxFrame, rx_buffer, 10);
                new_data_flag = 1;
            }
            rx_index = 0;
            HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        rx_index = 0;
        HAL_UART_Receive_IT(&huart3, &rx_buffer[0], 1);
    }
}

/* * Obsługa przycisku w tej implementacji została opróżniona,
 * ponieważ przejmujemy pełne sterowanie po UART.
 * Możesz wykorzystać ten callback np. jako sprzętowy wyłącznik awaryjny (E-STOP).
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		// Miejsce na logikę E-STOP
	}
}

/* USER CODE END 4 */

/**
  * @brief  Error handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
