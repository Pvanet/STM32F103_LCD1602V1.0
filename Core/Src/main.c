/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void Init1602(void);
void Write_Init4b_Command(uint8_t data);
void Write4b_Command(uint8_t data);
void Write_Init8b_Command(uint8_t data);
void Write4b_Data(uint8_t data);
void Strob (void);
void LCD_string (uint8_t *str);

uint16_t temperTMP37 = 0;
uint16_t settemperTMP37 = 40;
uint16_t adcVoltage;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t adcData = 0;
char trans_str[16] = {0,};
uint16_t temper = 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  Init1602();                         // инициализация для 4битного управления (можно изменить)
	uint8_t DDRAM = 0;                  // адрес для первого символа, начало экрана, первая строка
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcData, 1); // DMA вытаскивает с АЦП значение и отправляет 
	Write4b_Command(0x80 | DDRAM);
	LCD_string ("Measure Temp*C");
	DDRAM = 0x40;
  Write4b_Command(0x80 | DDRAM);
	temper = 1024;
	HAL_Delay (100); 
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Write4b_Command(0x80 | DDRAM);
		snprintf(trans_str, 9, "ADC %d\n", adcData); //упаковали всё в массив для отображения в ASCII
		LCD_string (trans_str);	
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(300);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |En_Pin|RS_Pin|RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : En_Pin RS_Pin RW_Pin */
  GPIO_InitStruct.Pin = En_Pin|RS_Pin|RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Init1602(void)
	{
		HAL_Delay(100);
/******************* Определить команду (функцию) можно по первому установленному в 1 биту, после него могут идти еще биты управления *******************/
/******************* 0011.xxxx - отправляем на старший полубайт (nibble) Fset команду включения 8битного интерфейса (просто так нужно 3 раза, не 2) *******************/
		Write_Init8b_Command (0x3); 
		Write_Init8b_Command (0x3);
		Write_Init8b_Command (0x3); 
		
/******************* 0010.xxxx - отправляем на старший полубайт (nibble) Fset команду включения 4битного интерфейса *******************/		
		Write_Init8b_Command (0x2);
		
/******************* 0010.11xx -  отправляем двумя полубайтами Fset команду включения 4битного интерфейса с настройками *******************/
	//		0010 			-  4 битный интерфейс
	//				.11xx -	 2 линии (1), размер символов 5x10 точек (1)  
		Write_Init4b_Command(0x2C);

/******************* 0000.1111 - отправляем двумя полубайтами команду "Настройка режима отображения" *******************/
//					.1111 -  определение команды в DB3 бит (1), включение отображения (1), курсор (1), квадрат мигает(1) 
		Write_Init4b_Command(0x0F);

/******************* 0001.11xx - отправляем двумя полубайтами команду "Настройка режима отображения" *******************/		
		Write_Init4b_Command(0x1C);

/******************* 0000.0001 - отправляем двумя полубайтами команду "Очистка экрана. Счетчик адреса на 0 позицию DDRAM" *******************/
		Write_Init4b_Command(0x01);

/******************* 0000.0110 - отправляем двумя полубайтами команду "	Настройка сдвига экрана и курсора" *******************/
//					.0110 -  определение команды в DB2 бит (1), инкремент счетчика адреса (1), не сдвигаем экран (0)
		Write_Init4b_Command(0x06);
	}

/***********************************************************************************************************
DB7		DB6		DB5		DB4		| DB3		DB2		DB1		DB0		Значение
	0			0			0			0		| 	0			0			0			1		Очистка экрана. Счетчик адреса на 0 позицию DDRAM
	0			0			0			0		| 	0			0			1			—		Адресация на DDRAM сброс сдвигов, Счетчик адреса на 0
	0			0			0			0		| 	0			1			I/D 	S		Настройка сдвига экрана и курсора
	0			0			0			0		| 	1			D			C			B		Настройка режима отображения
	0			0			0			1		| 	S/C		R/L		—			—		Сдвиг курсора или экрана, в зависимости от битов
	0			0			1			DL	| 	N			F			—			—		Выбор числа линий, ширины шины и размера символа
	0			1			AG		AG	| 	AG		AG		AG		AG	Переключить адресацию на SGRAM и задать адрес в SGRAM
	1			AD		AD		AD	| 	AD		AD		AD		AD	Переключить адресацию на DDRAM и задать адрес в DDRAM

	
I/D — инкремент или декремент счетчика адреса. По дефолту стоит 0 — Декремент. Т.е. каждый следующий байт будет записан в n-1 ячейку. Если поставить 1 — будет Инкремент.
S — сдвиг экрана, если поставить 1 то с каждым новым символом будет сдвигаться окно экрана, пока не достигнет конца DDRAM, 
		наверное удобно будет когда выводишь на экран здоровенную строку, на все 40 символов, чтобы не убегала за экран.
D — включить дисплей. Если поставить туда 0 то изображение исчезнет, а мы в это время можем в видеопамяти творить всякие непотребства и они не будут мозолить глаза. 
		А чтобы картинка появилась в эту позицию надо записать 1.
С — включить курсор в виде прочерка. Все просто, записали сюда 1 — включился курсор.
B — сделать курсор в виде мигающего черного квадрата.
S/C сдвиг курсора или экрана. Если стоит 0, то сдвигается курсор. Если 1, то экран. По одному разу за команду
R/L — определяет направление сдвига курсора и экрана. 0 — влево, 1 — вправо.
D/L — бит определяющий ширину шины данных. 1-8 бит, 0-4 бита
N — число строк. 0 — одна строка, 1 — две строки.
F — размер символа 0 — 5х8 точек. 1 — 5х10 точек (встречается крайне редко)
AG — адрес в памяти CGRAM
АD — адрес в памяти DDRAM
************************************************************************************************************/


/************* Стробирование. После выставления данных ****************************/
void Strob (void)
	{
	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 0);
	HAL_Delay(1);
	}
	
/************** Команда инициализации. Передача одним байтом ***********************/
void Write_Init8b_Command(uint8_t data)
	{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 0);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);
	GPIOA->BSRR = (0x0F<<16);
	GPIOA->BSRR |= data;
		Strob();
	HAL_Delay(5);
	}
	
/************** Команда инициализации. Передача двумя полубайтами ******************/
void Write_Init4b_Command(uint8_t data)
	{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 0);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);
	GPIOA->BSRR = (0x0F<<16);
	GPIOA->BSRR |= (data>>4);
		Strob();
	GPIOA->BSRR = (0x0F<<16);
	GPIOA->BSRR |= data;
		Strob();
	HAL_Delay(2);
	}

/************** Команда. Передача двумя полубайтами *********************************/	
	void Write4b_Command(uint8_t data)
	{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 0);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);
	GPIOA->BSRR = (0x0F<<16);
	GPIOA->BSRR |= (data>>4);
		Strob();
	GPIOA->BSRR = (0x0F<<16);
	GPIOA->BSRR |= data;
		Strob();
	HAL_Delay(2);
	}

/************** Данные. Передача двумя полубайтами ***********************************/	
void Write4b_Data(uint8_t data)
	{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 1);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);
	GPIOA->BSRR = (0x0F<<16);
	GPIOA->BSRR |= (data>>4);
		Strob();
	GPIOA->BSRR = (0x0F<<16);
	GPIOA->BSRR |= data;
		Strob();
	HAL_Delay(2);
	}

/************** Массив. Передача в LCD1602 ********************************************/
void LCD_string (uint8_t *str)
{
	uint8_t data = 0;
	while (*str)
	{
		data = *str++;
		Write4b_Data(data);
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
