/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "stdarg.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include "gpio.h"
#include <math.h>
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int RunState_FlagTimer = 0; //Variable to define if timer is running
int Count_BtnPress = 0; //Declare variable that will count the btn presses
int Last_BtnState = 0; //Variable to store last state of btn press
int Flag_Start = 0; //Variable to indicate if execution is running or stopped
//Variables for Logic Processing
int ChangeState = 0; //Variable to know what is being changed
int Mode = 0; //Variable to indicate current mode
double Mode_HighTime = 0.5; // Variable to store high time duration value
int Mode_Freq = 1; //Variable to store frequency value
int Mode_Duration = 0; //Variable to store runtime duration (Note: 0 is forever)
//Variables Timer BuzerLed
int HighTime = 0; //Variable to define time when value is high
int LowTime = 0; //Variable to define time when value is low
int Timer_SwapFlag = 0; //Variable to define if we are on High or Low value
int Trigger_COMMessage = 0; //Variable to Trigger COM Messages

enum
 {
 	PERIPHERAL_USART,	// UART serial device
 	PERIPHERAL_USB 		// USB serial device
 };
void myPrintf(uint16_t peripheral, char *format, ...)
{
	char buffer[64];

	va_list args;
	va_start(args, format);
	vsprintf(buffer, format, args);
	va_end(args);

	switch(peripheral)
	{
	case PERIPHERAL_USART:
		break;
	case PERIPHERAL_USB:
		CDC_Transmit_FS ((uint8_t *)buffer, strlen(buffer));
		break;
	default:
		break;
	}
}

//Callback for BTN
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (RunState_FlagTimer == false)
		{
			RunState_FlagTimer = true;// Set variable as true since timer is running
			HAL_TIM_Base_Start_IT(&htim13);//If btn pressed, start timer
		}
	Count_BtnPress++; //To increment when the btn is press, so we know how many times btn is pressed
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USB_DEVICE_Init();
  MX_ADC3_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  int Last_FlagStart = Flag_Start; //To store last flag value
  int CLK_Count = 0; //Variable to count all iterations
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	///////////////// Start Code ///////////////////
	if(Last_FlagStart == false && Flag_Start == true){
		HighTime = (1 / Mode_Freq) * 1000 * Mode_HighTime; //Calculate HighTime (Will stay at 1)
		LowTime = (1 / Mode_Freq) * 1000 * (1 - Mode_HighTime); //Calculate HighTime (Will stay at 1)
		int DurationTime = Mode_Duration * 1000;
		//HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_SET); //Turn On Buzzer
		HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_SET);// Set Buzzer On
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); //Turn OFF Emergency LED
		myPrintf(PERIPHERAL_USB, "///////////////////////////////\n\r");// Write Message to Console
		myPrintf(PERIPHERAL_USB, "RunState - Program Start\n\r");// Write Message to Console
	 }else if (Last_FlagStart == true && Flag_Start == false){
		//Reset Pins
		HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_MSB_GPIO_Port, LED_MSB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_LSB_GPIO_Port, LED_LSB_Pin, GPIO_PIN_RESET);
		ChangeState = 0; //reset change position
		myPrintf(PERIPHERAL_USB, "RunState - Program Stop\n\r");// Write Message to Console
		myPrintf(PERIPHERAL_USB, "///////////////////////////////\n\r");// Write Message to Console
	}

	if(Flag_Start == true){ //Logic for Duration
		CLK_Count += 20; //Add iteration Time, since Hal_Delay is 20 (20ms)
		if(Mode_Duration != 0){ //For when we have a defined duration time
			if (CLK_Count > Mode_Duration){//If counter is bigger then our duration, then we stop the execution
				Flag_Start = false;//set flag to false
				CLK_Count = 0;//reset
			}
		}

		//For Low_Time
		if(CLK_Count % (HighTime + LowTime) < HighTime){
			HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_SET);//Turn ON Buzzer
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); //Turn ON LED
		} else{//For High_Time
			HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_RESET);//Turn OFF Buzzer
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);//Turn OFF LED
		}

	}



	Last_FlagStart = Flag_Start; //Update Variable
	HAL_Delay(20);//sys Delay
	////////////////// End Code ////////////////////
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim == &htim13){
		// Stop SW Timer //
		HAL_TIM_Base_Stop_IT(&htim13);// Stop Timer

		// Pre-Checks //
		if (Count_BtnPress == 0)// If for some reason Btn presses are 0, we put it at 1 :)
			Count_BtnPress = 1;
		if (Count_BtnPress > 3) //if more then 3 presses we put it at 3
			Count_BtnPress = 3;

		//Read if Btn is still pressed
		GPIO_PinState PinState = HAL_GPIO_ReadPin(Btn_User_GPIO_Port, Btn_User_Pin);

		switch(Count_BtnPress){
			case 1:
				if (PinState){//if long press
					if(Flag_Start == true){//if program is running
						Flag_Start = false; //we stop it
						Timer_SwapFlag = 0;
					}else{ //else if program is stopped
						Flag_Start = true; //we start it
						Timer_SwapFlag = 0;
					}
				}
				else{
					ChangeState++; //Iterate on the modes, in a loop way (mode/hightime/freq/duration)
					ChangeState %= 4; //if we reach position 4, it goes back to 0
					//Set external lad according to the current state (state watcher)
					if(ChangeState == 0){ //For State 0 - Mode
						HAL_GPIO_WritePin(LED_MSB_GPIO_Port, LED_MSB_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(LED_LSB_GPIO_Port, LED_LSB_Pin, GPIO_PIN_RESET);
						myPrintf(PERIPHERAL_USB, "State - Set to 0 (Mode)\n\r");// Write Message to Console
					}else if(ChangeState == 1){ //For State 1 - hightime
						HAL_GPIO_WritePin(LED_MSB_GPIO_Port, LED_MSB_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(LED_LSB_GPIO_Port, LED_LSB_Pin, GPIO_PIN_SET);
						myPrintf(PERIPHERAL_USB, "State - Set to 1 (HighTime)\n\r");// Write Message to Console
					} else if(ChangeState == 2){ //For State 2 - freq
						HAL_GPIO_WritePin(LED_MSB_GPIO_Port, LED_MSB_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(LED_LSB_GPIO_Port, LED_LSB_Pin, GPIO_PIN_RESET);
						myPrintf(PERIPHERAL_USB, "State - Set to 2 (Frequency)\n\r");// Write Message to Console
					} else if(ChangeState == 3){ //For State 3 - duration
						HAL_GPIO_WritePin(LED_MSB_GPIO_Port, LED_MSB_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(LED_LSB_GPIO_Port, LED_LSB_Pin, GPIO_PIN_SET);
						myPrintf(PERIPHERAL_USB, "State - Set to 3 (Duration)\n\r");// Write Message to Console
					}
				}
				break;
			case 2:
				if(ChangeState == 0){ // To Increment the Mode
					Mode ++; //increment the mode
					Mode %= 2; // If mode is 2, we set it back to 0
					//Mode Selection
					if (Mode == 0){ //For first Mode - toque intermitente 0,5seg - para sempre
						Mode_HighTime = 0.5;
						Mode_Freq = 1;
						Mode_Duration = 0;
						Timer_SwapFlag = 0;
						// Write Message to Console
						myPrintf(PERIPHERAL_USB, "Set Mode 0 - HighTime: 0.5;Freq: 1;Duration:0(Forever) \n\r");
					} else if (Mode == 1){ //For second Mode - toque rápido 0,15seg - durante 3seg
						Mode_HighTime = 0.15;
						Mode_Freq = 1;
						Mode_Duration = 3;
						Timer_SwapFlag = 0;
						// Write Message to Console
						myPrintf(PERIPHERAL_USB, "Set Mode 1 - HighTime: 0.15;Freq: 1;Duration:3 \n\r");
					}
					Flag_Start = false; //Stop execution since mode is changed
				}else if(ChangeState == 1){ //To Increment HightTime
					Mode_HighTime += 0.1 * (1 - floor(Mode_HighTime)); //increment hightime global variable until we reach max of 1, then it stays always at 1
					// Write Message to Console
					myPrintf(PERIPHERAL_USB, "HighTime - Increased Value to: \n\r");
					//myPrintf(PERIPHERAL_USB, Mode_HighTime +"\n\r");
				}
				else if(ChangeState == 2){ //To Increment Frequency
					Mode_Freq ++; //increment freq global variable
					// Write Message to Console
					myPrintf(PERIPHERAL_USB, "Frequency - Increased Value to: \n\r");
					//myPrintf(PERIPHERAL_USB, Mode_HighTime +"\n\r");
				}

				else if(ChangeState == 3) //To Increment Duration
				{
					Mode_Duration ++; //increment duration global variable
					// Write Message to Console
					myPrintf(PERIPHERAL_USB, "Duration - Increased Value to: \n\r");
					//myPrintf(PERIPHERAL_USB, Mode_HighTime +"\n\r");
				}
				break;
			case 3:
				if(ChangeState == 0){ // To Decrease the Mode
					Mode --; //decrement the mode
					Mode %= 2; // If mode is 2, we set it back to 0
					//Mode Selection
					if (Mode == 0){ //For first Mode - toque intermitente 0,5seg - para sempre
						Mode_HighTime = 0.5;
						Mode_Freq = 1;
						Mode_Duration = 0;
						// Write Message to Console
						myPrintf(PERIPHERAL_USB, "Set Mode 0 - HighTime: 0.5;Freq: 1;Duration:0(Forever) \n\r");
					} else if (Mode == 1){ //For second Mode - toque rápido 0,15seg - durante 3seg
						Mode_HighTime = 0.15;
						Mode_Freq = 1;
						Mode_Duration = 3;
						// Write Message to Console
						myPrintf(PERIPHERAL_USB, "Set Mode 1 - HighTime: 0.15;Freq: 1;Duration:3 \n\r");
					}
					Flag_Start = false; //Stop execution since mode is changed
				}else if(ChangeState == 1){ //To Decrease HightTime
					Mode_HighTime -= 0.1 * (ceil(Mode_HighTime)); //decrease hightime global variable until we reach min of 0, then it stays always at 0
					// Write Message to Console
					myPrintf(PERIPHERAL_USB, "HighTime - Decreased Value to: \n\r");
					//myPrintf(PERIPHERAL_USB, Mode_HighTime +"\n\r");
				}
				else if(ChangeState == 2){ //To Decrease Frequency
					Mode_Freq --; //decrease freq global variable
					// Write Message to Console
					myPrintf(PERIPHERAL_USB, "Frequency - Decreased Value to: \n\r");
					//myPrintf(PERIPHERAL_USB, Mode_HighTime +"\n\r");
				}
				else if(ChangeState == 3){ //To Decrease Duration
					Mode_Duration --; //decrease duration global variable
					// Write Message to Console
					myPrintf(PERIPHERAL_USB, "Duration - Decreased Value to: \n\r");
					//myPrintf(PERIPHERAL_USB, Mode_HighTime +"\n\r");
				}
				break;
		}
		//Put flag and count back to 0 since processing is done
		Count_BtnPress = 0;
		RunState_FlagTimer = false;
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
