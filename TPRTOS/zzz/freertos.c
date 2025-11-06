/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
osThreadId Task_BuzzerHandle;
osThreadId Pross_BtnUserHandle;
osThreadId Task_LEDHandle;
osThreadId Task_SOSHandle;
osTimerId BTN_TimerHandle;
osTimerId Timer_BuzzerPWMHandle;
osTimerId Timer_BuzzerDurationHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Click_RE_Event();//Declare function for Rising Edge
/* USER CODE END FunctionPrototypes */

void Task_Buzzer_Control(void const * argument);
void Start_Pross_BtnUser_Task(void const * argument);
void Task_LED_Control(void const * argument);
void Task_SOS_Control(void const * argument);
void Pross_BTNPresses(void const * argument);
void Callback_BuzzerPWM(void const * argument);
void Callback_BuzzerDuration(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of BTN_Timer */
  osTimerDef(BTN_Timer, Pross_BTNPresses);
  BTN_TimerHandle = osTimerCreate(osTimer(BTN_Timer), osTimerPeriodic, NULL);

  /* definition and creation of Timer_BuzzerPWM */
  osTimerDef(Timer_BuzzerPWM, Callback_BuzzerPWM);
  Timer_BuzzerPWMHandle = osTimerCreate(osTimer(Timer_BuzzerPWM), osTimerPeriodic, NULL);

  /* definition and creation of Timer_BuzzerDuration */
  osTimerDef(Timer_BuzzerDuration, Callback_BuzzerDuration);
  Timer_BuzzerDurationHandle = osTimerCreate(osTimer(Timer_BuzzerDuration), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_Buzzer */
  osThreadDef(Task_Buzzer, Task_Buzzer_Control, osPriorityLow, 0, 128);
  Task_BuzzerHandle = osThreadCreate(osThread(Task_Buzzer), NULL);

  /* definition and creation of Pross_BtnUser */
  osThreadDef(Pross_BtnUser, Start_Pross_BtnUser_Task, osPriorityNormal, 0, 128);
  Pross_BtnUserHandle = osThreadCreate(osThread(Pross_BtnUser), NULL);

  /* definition and creation of Task_LED */
  osThreadDef(Task_LED, Task_LED_Control, osPriorityLow, 0, 128);
  Task_LEDHandle = osThreadCreate(osThread(Task_LED), NULL);

  /* definition and creation of Task_SOS */
  osThreadDef(Task_SOS, Task_SOS_Control, osPriorityHigh, 0, 128);
  Task_SOSHandle = osThreadCreate(osThread(Task_SOS), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Task_Buzzer_Control */
/**
  * @brief  Function implementing the Task_Buzzer thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_Buzzer_Control */
void Task_Buzzer_Control(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Task_Buzzer_Control */
  int Last_FlagStart = Flag_Start; //To store last flag value
  /* Infinite loop */
  for(;;)
  {
    if(Last_FlagStart == false && Flag_Start == true){
    	HighTime = (1 / Mode_Freq) * 1000 * Mode_HighTime; //Calculate HighTime (Will stay at 1)
    	LowTime = (1 / Mode_Freq) * 1000 * (1 - Mode_HighTime); //Calculate HighTime (Will stay at 1)
 		int DurationTime = Mode_Duration * 1000;
 		//HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_SET); //Turn On Buzzer
    	osTimerStart(Timer_BuzzerPWMHandle, HighTime); //StartTimer
    	HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_SET);// Set Buzzer On
    	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); //Turn OFF Emergency LED
    	if(Mode_Duration != 0)
    		osTimerStart(Timer_BuzzerDurationHandle, DurationTime); //StartTimer for Duration
		myPrintf(PERIPHERAL_USB, "///////////////////////////////\n\r");// Write Message to Console
		myPrintf(PERIPHERAL_USB, "RunState - Program Start\n\r");// Write Message to Console
    }else if (Last_FlagStart == true && Flag_Start == false){
		osTimerStop(Timer_BuzzerPWMHandle); //StopTimer
		osTimerStop(Timer_BuzzerDurationHandle); //Stop Timer for Duration
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
    Last_FlagStart = Flag_Start; //Update Variable
	osDelay(1);
  }
  /* USER CODE END Task_Buzzer_Control */
}

/* USER CODE BEGIN Header_Start_Pross_BtnUser_Task */
/**
* @brief Function implementing the Pross_BtnUser thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Pross_BtnUser_Task */
void Start_Pross_BtnUser_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Pross_BtnUser_Task */
  /* Infinite loop */
  for(;;)
  {
	int Curr_BtnState = HAL_GPIO_ReadPin(Btn_User_GPIO_Port, Btn_User_Pin); // Get Current Btn Stat
	if (Curr_BtnState == true && Last_BtnState == false) //If Btn is pressed and was not pressed before (raising edge triggered)
		Click_RE_Event(); //Call Event press

	if (Trigger_COMMessage == 1){ //For logic when receiving presses from COM Message
		Trigger_COMMessage = 0; //Reset
		RunState_FlagTimer = true;// Set variable as true since timer is running
		osTimerStart(BTN_TimerHandle, 50);//Start SW_Timer
	}

	Last_BtnState = Curr_BtnState; //Save curr value as last for next iteration
    osDelay(50); //Sys Delay for deboucing of Btn
  }
  /* USER CODE END Start_Pross_BtnUser_Task */
}

/* USER CODE BEGIN Header_Task_LED_Control */
/**
* @brief Function implementing the Task_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_LED_Control */
void Task_LED_Control(void const * argument)
{
  /* USER CODE BEGIN Task_LED_Control */
	char Buzzer_State; //To store Buzzer State
	int Last_FlagStart = Flag_Start; //To store last flag value
	int Counter_IntervalTime; // To count the running time, so we will trigger logic after counter has a desired value
	//int Counter_DurationTime; //
  /* Infinite loop */
  for(;;)
  {
    // For Led 1 to follow the Buzzer High Value //
	Buzzer_State = HAL_GPIO_ReadPin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin);
    if (Buzzer_State == true)
    	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); //Turn On LED
    else
    	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); //Turn OFF LED

    // Triggers only if start is activated and is running for more then 3 sec
    if(Last_FlagStart == false && Flag_Start == true)
    	Counter_IntervalTime = 0; //Reset
    else if (Flag_Start == true){
    	Counter_IntervalTime++; //Add to counter
    	if(Counter_IntervalTime >= 150 && Counter_IntervalTime <= 200){ // if we reach the 3 seconds
    		if((Counter_IntervalTime - 150) % 5 == 0)
    			HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin); //Blink the LED
    	} else if (Counter_IntervalTime >= 200){
    		Counter_IntervalTime = 0; //Reset
    		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); //Turn OFF LED
    	}

    }

    Last_FlagStart = Flag_Start; //Update Variable
	osDelay(20);
  }
  /* USER CODE END Task_LED_Control */
}

/* USER CODE BEGIN Header_Task_SOS_Control */
/**
* @brief Function implementing the Task_SOS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_SOS_Control */
void Task_SOS_Control(void const * argument)
{
  /* USER CODE BEGIN Task_SOS_Control */
  uint16_t ADC_LDRValue; //Variable to Store LDR Values
  /* Infinite loop */
  for(;;)
  {
	// READ ADC Values
	HAL_ADC_Start(&hadc3); //Start to read ADC Values
	HAL_ADC_PollForConversion(&hadc3, 20);
	ADC_LDRValue = HAL_ADC_GetValue(&hadc3);

	// Emergency Trigger //
	if (ADC_LDRValue < 600){
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); //Turn ON Emergency LED
		HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_SET); //Turn ON Buzzer
		Flag_Start = false; //execution is stopped
		// Write Message to Console
		myPrintf(PERIPHERAL_USB, "!!! SOS - Trigger Active !!!\n\r");
	}

    osDelay(20);
  }
  /* USER CODE END Task_SOS_Control */
}

/* Pross_BTNPresses function */
void Pross_BTNPresses(void const * argument)
{
  /* USER CODE BEGIN Pross_BTNPresses */
	// Stop SW Timer //
	osTimerStop(BTN_TimerHandle);// Stop Timer

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

  /* USER CODE END Pross_BTNPresses */
}

/* Callback_BuzzerPWM function */
void Callback_BuzzerPWM(void const * argument)
{
  /* USER CODE BEGIN Callback_BuzzerPWM */
	Timer_SwapFlag ++; //Increment the Swap Flag
	//Recalculate High and Low Values
	HighTime = round((1.0 / Mode_Freq) * 1000 * Mode_HighTime); //Calculate HighTime (Will stay at 1)
	LowTime = round((1.0 / Mode_Freq) * 1000 * (1 - Mode_HighTime)); //Calculate HighTime (Will stay at 1)
	//if/else logic
	if (Timer_SwapFlag%2 == 0){ //if is a pair number, the we run it as HighState
		osTimerStop(Timer_BuzzerPWMHandle); //Stop Timer
		HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_SET);//Activate Buzzer
		//HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
		Timer_SwapFlag = 0;//Reset
		osTimerStart(Timer_BuzzerPWMHandle, HighTime);//Start Timer with highValue time.
	}else{ //if is odd, then we run it as LowState
		osTimerStop(Timer_BuzzerPWMHandle); //Stop Timer
		HAL_GPIO_WritePin(Buzzer_Output_GPIO_Port, Buzzer_Output_Pin, GPIO_PIN_RESET);//Deactivate Buzzer
		//HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
		osTimerStart(Timer_BuzzerPWMHandle, LowTime);//Start Timer with highValue time.
	}
  /* USER CODE END Callback_BuzzerPWM */
}

/* Callback_BuzzerDuration function */
void Callback_BuzzerDuration(void const * argument)
{
  /* USER CODE BEGIN Callback_BuzzerDuration */
	Flag_Start = false; //execution is stopped
  /* USER CODE END Callback_BuzzerDuration */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Click_RE_Event()
{
	if (RunState_FlagTimer == false)
	{
		RunState_FlagTimer = true;// Set variable as true since timer is running
		osTimerStart(BTN_TimerHandle, 1000);//Start SW_Timer
	}
	Count_BtnPress++; //To increment when the btn is press, so we know how many times btn is pressed

}
/* USER CODE END Application */

