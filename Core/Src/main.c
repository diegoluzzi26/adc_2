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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    MODE_ECONOMICO = 0,
    MODE_NORMAL,
    MODE_RACE,
    MODE_COUNT         
} AppMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADXL345_ADDR        (0x53 << 1)
#define ADXL_REG_DATA_FMT   0x31
#define ADXL_REG_POWER_CTL  0x2D
#define ADXL_REG_DATAX0     0x32
#define ADXL_REG_DEVID      0x00

#define ADC_MAX   4095u
#define VREF_MV   3300u

/* Botão */
#define BTN_PORT            GPIOC
#define BTN_PIN             GPIO_PIN_4
#define BTN_DEBOUNCE_MS     50u     /* tempo mínimo entre leituras válidas  */

/* LEDs */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc_value = 0;
char msg[64];
char buffer[32];

/* Máquina de estados do modo */
static AppMode_t s_mode           = MODE_NORMAL;
static uint32_t  s_btn_last_tick  = 0;
static uint8_t   s_btn_last_state = GPIO_PIN_SET;   /* pull-up > repouso = 1 */

/* Nomes dos modos*/
static const char * const MODE_NAMES[MODE_COUNT] = {
    "ECONOMICO",
    "NORMAL",
    "RACE"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef ADXL345_Init(void);
static HAL_StatusTypeDef ADXL345_Read(int16_t *ax, int16_t *ay, int16_t *az);
static void              Button_Update(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* ADXL345 */

static HAL_StatusTypeDef ADXL345_Init(void)
{
    uint8_t val;
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, ADXL_REG_DEVID,
                            I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    if (ret != HAL_OK) return ret;

    int len = snprintf(buffer, sizeof(buffer),
                       "ADXL345 DEVID: 0x%02X %s\r\n",
                       val, (val == 0xE5) ? "OK" : "ERRO!");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);

    val = 0x0B;
    ret = HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, ADXL_REG_DATA_FMT,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    if (ret != HAL_OK) return ret;

    val = 0x08;
    ret = HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, ADXL_REG_POWER_CTL,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return ret;
}

static HAL_StatusTypeDef ADXL345_Read(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, ADXL_REG_DATAX0,
                                              I2C_MEMADD_SIZE_8BIT, buf, 6, 100);
    if (ret != HAL_OK) return ret;

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
    return HAL_OK;
}

/*Máquina de estados PC4*/
static void Button_Update(void)
{
    uint8_t current = HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN);

    /* Detecta borda de descida (pull-up solto → botão pressionado) */
    if (current == GPIO_PIN_RESET && s_btn_last_state == GPIO_PIN_SET)
    {
        uint32_t now = HAL_GetTick();

        if ((now - s_btn_last_tick) >= BTN_DEBOUNCE_MS)
        {
            s_btn_last_tick = now;

            s_mode = (AppMode_t)((s_mode + 1) % MODE_COUNT);

            int len = snprintf(buffer, sizeof(buffer),
                               "[MODO] %s\r\n", MODE_NAMES[s_mode]);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
        }
    }

    s_btn_last_state = current;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  if (ADXL345_Init() != HAL_OK) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"ADXL345 falhou na inicializacao!\r\n", 34, 100);
  }

  /* Anuncia modo inicial */
  {
      int len = snprintf(buffer, sizeof(buffer),
                         "[MODO] %s\r\n", MODE_NAMES[s_mode]);
      HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t ax = 0, ay = 0, az = 0;
  uint8_t accel_ok;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*Verifica botão PC4 e atualiza modo*/
    Button_Update();

    /* --- ADC --- */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    uint32_t mvN    = (adc_value * VREF_MV + ADC_MAX / 2) / ADC_MAX;
    uint32_t v_intN = mvN / 1000u;
    uint32_t v_decN = mvN % 1000u;

    /*ADXL345*/
    accel_ok = (ADXL345_Read(&ax, &ay, &az) == HAL_OK);

    /*Serial */
    if (accel_ok) {
    if (s_mode == MODE_RACE) {
        uint32_t mv    = ((adc_value * VREF_MV + ADC_MAX / 2) / ADC_MAX*1.5);
        uint32_t v_int = mv / 1000u;
        uint32_t v_dec = mv % 1000u;
        int len = snprintf(msg, sizeof(msg),
                           "[RACE] X=%d Y=%d Z=%d ADC=%lu V=%lu.%03lu V=%lu.%03lu\r\n",
                           ax, ay, az, adc_value, v_intN, v_decN, v_int, v_dec);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 20);
    }
    else if (s_mode == MODE_ECONOMICO) {
        uint32_t mv    = ((adc_value * VREF_MV + ADC_MAX / 2) / ADC_MAX*0.5);
        uint32_t v_int = mv / 1000u;
        uint32_t v_dec = mv % 1000u;
        int len = snprintf(msg, sizeof(msg),
                           "[ECONOMICO] V=%lu.%03lu V\r\n",
                           v_int, v_dec);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 20);
    }
    else {
        int len = snprintf(msg, sizeof(msg),
                           "[NORMAL] X=%d Y=%d Z=%d ADC=%lu\r\n",
                           ax, ay, az, adc_value);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 20);
    }
} else {
    HAL_UART_Transmit(&huart1, (uint8_t*)"Erro ADXL345\r\n", 14, 20);
}
    HAL_Delay(100);
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
