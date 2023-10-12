/* USER CODE BEGIN Header */
/*******************************************************************************
  * File Name          : main.c
  * Description        : Takes a string from the user through Termite terminal and produces a morse code for that string.
  *
  * Author             : Meet Patel
  * Date               : 13th June 2021
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include<stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define morseUnit 300                     // one morseUnit = one dot
#define maxMsgLength 20                   // Maximum input string length
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// FUNCTION      : glowMorse
// DESCRIPTION   : Responsible for glowing the input string continuously in the while loop
// PARAMETERS    : char morseString[]
// RETURNS       : void
void glowMorse(char morseString[]);

// FUNCTION      : glowAlphanumeric    Example - glowD, glow4,glowI etc.
// DESCRIPTION   : Responsible for glowing the individual character of the input string. This function is called by glowMorse
//                 Due to time constraint I couldn't write morse code for all the Alphanumeric characters but managed to write for the below ones
// PARAMETERS    : none
// RETURNS       : void
void glowA();
void glowB();
void glowC();
void glowD();
void glowE();
void glowF();
void glowG();
void glowH();
void glowI();
void glowS();

// FUNCTION      : glowTerminateAndSpace
// DESCRIPTION   : This function serves two purposes.It produces appropriate morse code when input string is terminated by checking the null character '\0' as well as produces morse code for the space ' ' character.
// PARAMETERS    : none
// RETURNS       : void
void glowTerminateAndSpace();

// FUNCTION      : LD11
// DESCRIPTION   : Responsible to toggle the LED at digital GPIO_Output D11 (PB5)
// PARAMETERS    : onTime and offTime
// RETURNS       : void
void LD11(int onTime, int offTime);
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
  /* USER CODE BEGIN 2 */
     printf("Please enter the message for which you want to generate the Morse Code\n");
	 char msg[maxMsgLength];    //Array of characters with length equal to maxMsgLength which was defined in the macro
	 gets(msg);                 //Get the string through terminal
	 printf("The message is %s\n", msg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  glowMorse(msg);           //Continuously glow the morse code for the input string
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin PB5 */
  GPIO_InitStruct.Pin = LD3_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void glowMorse(char morseString[]){
	for(int i = 0; i<= maxMsgLength; i++)   //loop used to generate morse code for input string by iterating through all individual characters
	    {
	    switch(morseString[i]){             // switch condition to check for the individual character of a string and produce morse code for that particular character
	        case '\0':
	        glowTerminateAndSpace();
	        break;
	        case ' ':
	        glowTerminateAndSpace();
	    	break;
	        case 'a':
	        case 'A':
	        glowA();
	        break;
	        case 'b':
	        case 'B':
	        glowB();
			break;
	        case 'c':
	        case 'C':
	        glowC();
	        break;
	        case 'd':
	        case 'D':
	        glowD();
			break;
	        case 'e':
	        case 'E':
	        glowE();
	        break;
	        case 'f':
	        case 'F':
	        glowF();
			break;
	        case 'g':
	        case 'G':
	        glowG();
	        break;
	        case 'h':
	        case 'H':
	        glowH();
			break;
	        case 'i':
	        case 'I':
	        glowI();
	        break;
	        case 's':
	        case 'S':
 	        glowS();
	        break;
	    }
	    if(morseString[i] == '\0'){  //if there is a null character in the input string, it means the string has ended. Thus, break out of for loop
	    	break;
	    }
	    }
}

void glowA(){
	LD11(morseUnit, morseUnit);
	LD11(3 * morseUnit,3 * morseUnit);
}
void glowB(){
	LD11(3 * morseUnit, morseUnit);
	LD11(morseUnit, morseUnit);
        LD11(morseUnit, morseUnit);
	LD11(morseUnit, 3 * morseUnit);
}
void glowC(){
	LD11(3 * morseUnit, morseUnit);
	LD11(morseUnit, morseUnit);
        LD11(3 * morseUnit, morseUnit);
	LD11(morseUnit, 3 * morseUnit);
}
void glowD(){
	LD11(3 * morseUnit, morseUnit);
	LD11(morseUnit, morseUnit);
	LD11(morseUnit, 3 * morseUnit);
}
void glowE(){
	LD11(morseUnit, 3 * morseUnit);
}
void glowF(){
	LD11(morseUnit, morseUnit);
        LD11(morseUnit, morseUnit);
        LD11(3 * morseUnit, morseUnit);
	LD11(morseUnit, 3 * morseUnit);
}
void glowG(){
        LD11(3 * morseUnit, morseUnit);
        LD11(3 * morseUnit, morseUnit);
	LD11(morseUnit, 3 * morseUnit);
}
void glowH(){
	LD11(morseUnit, morseUnit);
        LD11(morseUnit, morseUnit);
        LD11(morseUnit, morseUnit);
        LD11(morseUnit, 3 * morseUnit);
}
void glowI(){
	LD11(morseUnit, morseUnit);
	LD11(morseUnit, 3 * morseUnit);
}
void glowS(){
	LD11(morseUnit, morseUnit);
	LD11(morseUnit, morseUnit);
	LD11(morseUnit, 3 * morseUnit);
}
void glowTerminateAndSpace(){
	LD11(0 * morseUnit, 4 * morseUnit);
}
void LD11(int onTime, int offTime){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(onTime);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_Delay(offTime);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
