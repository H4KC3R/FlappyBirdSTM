/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LED_MATRIX  4
#define OP_NOOP         0
#define OP_DIGIT0       1
#define OP_DIGIT1       2
#define OP_DIGIT2       3
#define OP_DIGIT3       4
#define OP_DIGIT4       5
#define OP_DIGIT5       6
#define OP_DIGIT6       7
#define OP_DIGIT7       8
#define OP_DECODEMODE   9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

typedef struct{
	uint8_t x;
	uint8_t y;
}coordinate;

int isPlaying = 1;
coordinate position = {0, OP_DIGIT7};

// row - screen number
// column - state on row
volatile uint8_t map[4][8]={
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
 };

uint8_t cell[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

char trans_str[64] = {0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// the function writes LOSE word on the LED screen
void write_Lose(){
	uint8_t buf[2];
	uint8_t L[8] = {0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x7C};
	uint8_t O[8] = {0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C};
	uint8_t S[8] = {0x78,0x04,0x04,0x04,0x38,0x40,0x40,0x3C};
	uint8_t E[8] = {0x7C,0x04,0x04,0x7C,0x04,0x04,0x04,0x7C};
	for(int i = 0; i < 8; i++){
	HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);  // pull the CS pin LOW
		buf[0] = 8 - i;
		buf[1] = E[i];
		HAL_SPI_Transmit_IT (&hspi1, buf, 2);

		buf[0] = 8 - i;
		buf[1] = S[i];
		HAL_SPI_Transmit_IT (&hspi1, buf, 2);

		buf[0] = 8 - i;
		buf[1] = O[i];
		HAL_SPI_Transmit_IT (&hspi1, buf, 2);

		buf[0] = 8 - i;
		buf[1] = L[i];
		HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_SET);  // pull the CS pin HIGH
	HAL_Delay(100);
	}
}

// the function draws obstacles
void draw_bar(){
	uint8_t buf[2];
	for(int i = 1; i < 9; i++){
		buf[0] = i;
		HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);  // pull the CS pin LOW
		for(int j = 4; j >= 1; j--){
			// if the row not null
			if(map[j-1][i-1] != 0){
				// if we reach the end of the screen, move to the next
				if((map[j-1][i-1] & 0x01) != 0){
					if(j == 1){
						map[j-1][i-1] &= (~0x01);
					}
					else{
						map[j-1][i-1] &= (~0x01);
						map[j-2][i-1] |= 0x80;
					}
				}
				else{
					// move obstacles
					uint8_t temp = map[j-1][i-1] >> 1;
					map[j-1][i-1] = temp;
				}
			}
			buf[1] = map[j-1][i-1];
			HAL_SPI_Transmit_IT (&hspi1, buf, 2);
		}
		HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_SET);  // pull the CS pin HIGH
	}
}

// the function checks collisions between bird and environment
void check_collision(){
	// the bird fell to the ground
	if((position.y - 1) < OP_DIGIT0)
		isPlaying = 0;

	// we are on the top
	if((position.y + 1) > OP_DIGIT7)
		position.y -= 1;

	// we have finally reached the screen, start from the beginning
	if(position.x + 1 >= 32)
		position.x = 0;

	// checking the collisions between bird and obstacles
	uint8_t screen = 0;
	if (position.x + 1  < 8)
		screen = 1;
	else if((position.x + 1) >= 8 && (position.x + 1) < 16)
		screen = 2;
	else if((position.x + 1) >= 16 && (position.x + 1)< 24)
		screen = 3;
	else
		screen = 4;

	uint8_t dot = map[screen-1][position.y];
	dot &= cell[(position.x + 1)%8];
	if (dot != 0){
		isPlaying = 0;
	}
}

// the function draws our bird
void draw_cell (uint8_t x, uint8_t y) {
	if(x > 31 || x < 0)
		return;
	if(y > 8 || y < 1)
		return;

	uint8_t screen = 0;
	uint8_t dot = cell[x%8];

	if (x < 8)
		screen = 1;
	else if(x >= 8 && x < 16)
		screen = 2;
	else if(x >= 16 && x < 24)
		screen = 3;
	else
		screen = 4;

	uint8_t buf[2];
	buf[0] = y;
	HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);  // pull the CS pin LOW
	for(int i = 4; i >= 1; i--){
		buf[1] = map[i-1][y-1];
		if(i == screen){
			if(map[i-1][y-1] != 0x00){
				dot = map[i-1][y-1];
				dot |= cell[x%8];
				buf[1] = dot;
			}
			else
				buf[1] = dot;
		}
		HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	}
	HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_SET);  // pull the CS pin HIGH
}

// the function clears bird's old position
void clear_cell (uint8_t x, uint8_t y){
	if(x > 31 || x < 0)
		return;
	if(y > 8 || y < 1)
		return;

	uint8_t screen = 0;
	uint8_t dot = 0x00;

	if (x < 8)
		screen = 1;
	else if(x >= 8 && x < 16)
		screen = 2;
	else if(x >= 16 && x < 24)
		screen = 3;
	else
		screen = 4;

	uint8_t buf[2];
	buf[0] = y;
	HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);  // pull the CS pin LOW
	for(int i = 4; i >= 1; i--){
		buf[1] = map[i-1][y-1];
		if(i == screen){
			if(map[i-1][y-1] != 0x00){
				dot = map[i-1][y-1];
				dot &= (~cell[x%8]);
				buf[1] = dot;
			}
			else
				buf[1] = 0x00;
		}
		HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	}
	HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_SET);  // pull the CS pin HIGH
}

// the function clears whole screen
void clear_all(){
	uint8_t buf[2];
	for(int i = 1; i < 9; i++){
		buf[0] = i;
		buf[1] = 0x00;
		HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);  // pull the CS pin LOW
		for(int j = 4; j >= 1; j--){
			HAL_SPI_Transmit_IT (&hspi1, buf, 2);
			map[j-1][i-1] = 0x00;
		}
		HAL_GPIO_WritePin (GPIOF, GPIO_PIN_12, GPIO_PIN_SET);  // pull the CS pin HIGH
	}
}

// the draws bird's next position depending on ADC data
void move(uint16_t x_raw, uint16_t y_raw){
	// parsing raw data
	if(x_raw < 1000 && y_raw < 1000){
		snprintf(trans_str, 63, "down\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t*)trans_str, strlen(trans_str), 1000);
		check_collision();
		position.x += 1;
		position.y -= 1;
		draw_cell(position.x, position.y);
	}
	else if(x_raw > 4000 && y_raw > 4000){
		snprintf(trans_str, 63, "up\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t*)trans_str, strlen(trans_str), 1000);
		check_collision();
		position.x += 1;
		position.y += 1;
		draw_cell(position.x, position.y);
	}
	else{
		snprintf(trans_str, 63, "falling\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t*)trans_str, strlen(trans_str), 1000);
		check_collision();
		position.x += 1;
		position.y -= 1;
		draw_cell(position.x, position.y);
	}
}

// initialize screen
void matrix_init(void) {
	uint8_t buf[2];
	HAL_Delay(500);
	buf[0] = OP_DISPLAYTEST;
	buf[1] = 0x00;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	buf[0] = OP_SCANLIMIT;
	buf[1] = 0x07;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	buf[0] = OP_DECODEMODE;
	buf[1] = 0x00;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	buf[0] = OP_SHUTDOWN;
	buf[1] = 0x01;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

	buf[0] = OP_INTENSITY;
	buf[1] = 0x05;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_SPI_Transmit_IT (&hspi1, buf, 2);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
}

// every 2 sec we randomly create new obstacles
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
        {
        	uint8_t temp;
        	uint32_t num = HAL_RNG_GetRandomNumber(&hrng);
        	if(num < 1431655765){
        		temp = map[3][5] | 0x80;
        		map[3][5] = temp;
        		temp = map[3][6] | 0x80;
        		map[3][6] = temp;
        		temp = map[3][7] | 0x80;
        		map[3][7] = temp;
        	}
        	else if((num >= 1431655765) & (num < 2863311530)){
        		temp = map[3][2] | 0x80;
      		 	map[3][2] = temp;
        		temp = map[3][3] | 0x80;
      		 	map[3][3] = temp;
        		temp = map[3][4] | 0x80;
      		 	map[3][4] = temp;
        	}
        	else{
        		temp = map[3][0] | 0x80;
        		map[3][0] = temp;
        		temp = map[3][1] | 0x80;
        		map[3][1] = temp;
        	}
        }
}

// restart game
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	clear_all();
	isPlaying = 1;
	position.x = 0;
	position.y = OP_DIGIT7;
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  matrix_init();

  HAL_Delay(500);

  clear_all();

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF); // очищаем флаг
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(isPlaying){
		  HAL_ADCEx_InjectedStart(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1,100);
		  // the result of polling each channel is written to its own register, and we pick it up and copy it to a variable
		  uint16_t x_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		  uint16_t y_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

		  draw_bar();
		  move(x_raw, y_raw);
		  HAL_Delay(500);
		  clear_cell(position.x, position.y);
	  	  }
	  	  else{
	  		  write_Lose();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
