/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OC_BUFFER_LEN       (12)
#define OC_IDLE             (0)
#define OC_RUN              (1)
#define OC_DONE             (2)

#define APP_STATE_INIT      (0)
#define APP_STATE_TASKS     (1)

#define PULSE_TIME          (100)
#define LO_TIME             (200)
#define HI_TIME             (400)
#define START_TIME          (1000)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t txBuffer[OC_BUFFER_LEN];
static volatile uint32_t txLength = 0;

static volatile uint8_t ocState = OC_IDLE;
static uint32_t appState = APP_STATE_INIT;
static bool btnPrevState = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        switch (appState)
        {
            case APP_STATE_INIT:
            {
                if (HAL_TIM_Base_Start(&htim3) == HAL_OK)
                {
                    btnPrevState = HAL_GPIO_ReadPin( B1_GPIO_Port, B1_Pin);
                    ocState = OC_IDLE;
                    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);
                    appState = APP_STATE_TASKS;
                }
                break;
            }

            case APP_STATE_TASKS:
            {
                switch (ocState)
                {
                    case OC_IDLE:
                    {
                        bool btn = HAL_GPIO_ReadPin( B1_GPIO_Port, B1_Pin);
                        if (btn != btnPrevState)
                        {
                            HAL_Delay(100);
                            if (!btn)
                            {
                                if (!HAL_GPIO_ReadPin( B1_GPIO_Port, B1_Pin))
                                {
                                    txBuffer[0] = 0xAA;
                                    txBuffer[1] = 0x55;
                                    txLength = 12;
                                    htim3.Instance->CCR3 = htim3.Instance->CNT + 100;
                                    ocState = OC_RUN;
                                }
                            }
                            btnPrevState = btn;
                        }
                        break;
                    }

                    case OC_RUN:
                    {
                        break;
                    }

                    case OC_DONE:
                    {
                        ocState = OC_IDLE;
                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
            }

            default:
            {
                break;
            }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        if (ocState == OC_RUN)
        {
            static uint8_t step = 0;
            static uint32_t bitCounter = 0;
            uint16_t period = 0;
            uint16_t timer = htim->Instance->CNT;
            uint8_t bytePos = 0;
            uint8_t bitPos = 0;
            bool pinState = false;

            switch (step)
            {
                case 0:
                    period = PULSE_TIME;
                    pinState = true;
                    bitCounter = 0;
                    step++;
                    break;

                case 1:
                    period = START_TIME;
                    pinState = false;
                    step++;
                    break;

                case 2:
                    period = PULSE_TIME;
                    pinState = true;
                    step += bitCounter == txLength ? 2 : 1;
                    break;

                case 3:
                    bytePos = bitCounter >> 3;
                    bitPos = bitCounter & 0x07;

                    period = (1 << bitPos) & (txBuffer[bytePos]) ? HI_TIME : LO_TIME;
                    pinState = false;
                    bitCounter++;
                    step = 2;
                    break;
                case 4:
                    period = START_TIME;
                    pinState = false;
                    step++;
                    break;

                default:
                    ocState = OC_DONE;
                    break;

            }

            if (ocState == OC_DONE)
            {
                pinState = GPIO_PIN_RESET;
                step = 0;
            }
            else
            {
                htim->Instance->CCR3 = timer + period;
            }
            HAL_GPIO_WritePin(NET_TX_GPIO_Port, NET_TX_Pin, pinState ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
