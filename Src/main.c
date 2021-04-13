/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cJSON.h"
#include <time.h>
#include "dwt_delay.h"
#include "AM2305.h"
#include "time.h"
#include "ee24.h"
#include "SIM8xx.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct program_t{
/* 0 */	uint8_t ID;
/* 1 */	uint8_t Month;
/* 2 */	uint8_t Day;
/* 3 */	uint8_t Hour;
/* 4 */	uint8_t Minutes;
/* 5 */	uint8_t PeriodDay;
/* 6 */	uint8_t PeriodHour; 
/* 7 */	uint8_t PeriodMinute;
/* 8 */	uint8_t Action;
/* 9 */	uint8_t has_interval;
} Program;
typedef struct ProcessProgram_t{
/* 0 */	uint8_t  ID;
/* 1 */	uint32_t duration[10];	
/* 2 */	uint8_t  Hour[10];
/* 3 */	uint8_t  Minutes[10];
/* 4 */	uint8_t  Second[10];	
/* 5 */	uint8_t  spouts[10];
/* 6 */	uint8_t  events_count;	
	
}
ProcessProgram;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// DEVICE NAME: "ABGIRI"

#define		__FIRSTTIME_PROGRAMMING__	0																						// If you are programming a board for first time, value should be 1.
#define		__SENSORS_ARE_ENABLE__		0																						// If you want to use sensors, this value should be 1.
#define		__BUTTONS_ARE_ENABLE__		1																						// If you want to use external buttons, this value should be 1.
#define		__IS_3G_MODULE__					0																						// If you use 3G module this value should be 1.
#define  	__SERIAL_NUMBER						"666"																				// Unique serial number. We use this number to get "land ID" from server.
#define		__WELCOME_TEXT						"TEST BENCH STARTED"												// The text sent via SMS after reset.

#define		PROGRAM_ID_ADDRESS				 19																					// The address of RTC backup register storing the next program ID.
#define		LAST_STATUS_ADDRESS			   10																					// The address of RTC backup register storing the last output status.
#define		MY_ID_ADDRESS							 18																					// The address of RTC backup register storing the "Land ID".
#define		PROCESS_PROGRAM_ID_ADDRESS 11																					// The address of RTC backup register storing the last process program ID.
#define		LAST_EVENT_NUMBER_ADDRESS  12																					// The address of RTC backup register storing the last event number of process program .
#define		LAST_CURRENT_TIME_STAMP		 13																					// The address of RTC backup register storing the last current time stamp.
#define		LAST_PROCESS_FLAG_STATUS	 14																					// The address of RTC backup register storing the last procees flag status.
#define   LAST_EVENT_TIME_STAMP			 17																					// The address of RTC backup register storing the last event time stamp.
#define		PROCESS_PROG_START_ADD		1000																				// The starting address of process programs in eeprom
#define		debugUART									huart4																			// Debug uart handler
#define		simUART										huart1																			// SIM800 uart handler
#define   BUTTONS_NUM								2																						//External BUTTONS number
//#define 	SERVER_IP									"37.152.181.206"													// Server IP
#define		SERVER_IP									"ldmpanel.ir"																// Server domain
//#define 	phone										"+989024264731"
//#define 	phone										"+989362117764"	// Hasan agha
//#define		phone											"+989125572257"
//#define		phone2										"+989213106616"
//#define 	phone											"+989128504339"
#define 	phone											"+989395676056"
//#define   Buttons_number						2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint8_t									isConnect=1;										// Determines the connection status. 1 means connected
uint16_t								myID = 2;												// This is the landID
char										ContentStr[size],str[size],oledStr[size];			// Two general strings used in some functions as string buffers
char										SMStext[120];										// The text we want to send
uint32_t								counter;												// Used in flow meter

RTC_TimeTypeDef					Time;														// Stores time
RTC_DateTypeDef					Date;														// Stores date
RTC_AlarmTypeDef				Alarm;													// Stores alarm setting
RTC_AlarmTypeDef				alarm_struct;										// Stores alarm setting

uint8_t									action = 1;											// This is a flag showing whether we want to send requests to server or not. This value is being changed every 3 minutes

float * 								TH;															// temp & humidity

uint32_t								AnalogSensors[3];								// ADC coupled DMA array
uint32_t								moisture[10];										// Moisture value

uint32_t								Flow1,Flow2;										// Flow values

uint8_t 								averageItterator=0;							// The itteration variable for averaging method

HAL_StatusTypeDef				Sim80x_StatusTypeDef;
uint8_t									ErrorCounter=0;									// Counts the errors returned by SIM800
char 						    		RxBuffer[size];									// A buffer for SIM800's response
uint8_t                 buttonsStatus[2]={0,0};				  //status of the 2 external buttons 
char 										outputsStatus[size];					  // used in "sim80x_Send_Status" 
char* 									outputsRssiButtonsStatus;				// include the status of outputs,RSSI value and external buttons.
char										processProgram[size];						//used in processProgramParser

//char										Buttons_processProgram[Buttons_number];
uint8_t 								LastStatus;											// Stores the last output status
uint8_t 								get_output_result;							// The result of GetOutput fnc
uint8_t									SMSisPending=0;									// It is a flag showing whether we want to send SMS or not
uint8_t									AlarmIsSet = 0;									// Shows the status of alarm. 1 means an alarm was set
uint8_t   							processProgramsAlarmIsSet=0;		//used when we want enable/disable processprogramsAlarm
uint8_t  							  eventNumber=0;									//Running process event number 
uint8_t 								buttonId;												//it is the extenal botton id that is pushed
uint8_t 								processFlag=0;									//It is set to 1 if we have a process program event to run. buttons are pushed
uint8_t									unfinishedEventFlag=0;					//It is set to 1 if we have an unfinished event 
uint8_t 								getProcessProgramsStatus=0;			//Get process program  unsuccessful /successful  flag
uint8_t									initializingFlag=0;							//set to 1 when initial setting started and then set to 0 when it done and remain 0 in all program
uint8_t									initializingDoneFlag=0;					//set to 1 when initializingFlag vlaue chnged from 1 to 0 
uint8_t									tim5CallbackCounter=0;					//a counter that used in HAL_TIM_PeriodElapsedCallback function
uint8_t									dotPointCounter=0;							// count dotpoint in initializing in HAL_TIM_PeriodElapsedCallback function
uint32_t								lastTimeStamp;									//used when read last current time  stamp from eeprom
uint32_t								lastEventTimeStamp;							//used when read last event time  stamp from eeprom

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
//void						Hal_Delay(uint32_t interval);
uint16_t					GetID(void);
uint8_t						GetOutput(void);
void							measureAndLog(void);
void 							postSensors(void);		
uint8_t						str2bin(char* input);
void							writeProg(Program* prog);
void 							writeProcessProg(ProcessProgram* processProg);
Program						readProg(uint8_t progID);
ProcessProgram    readProcessProg(uint8_t progID);
void							deleteAllProcessPrograms(void);
void							deleteProg(Program* prog);
void 							GetAllPrograms(void);
void							ApplyAction(uint8_t OutputStatus);
Program						ProgramParser(char* input_str);
ProcessProgram    processProgramParser(char* input_str);
void							PrintProgram(Program input);
void 							PrintAllProcessProgram(int i);
void							SetAlarm(RTC_HandleTypeDef* rtc);
uint8_t 					SetProcessProgramsAlarm(RTC_HandleTypeDef* rtc);
void							SetNextAlarm(RTC_HandleTypeDef* rtc);
void 							SetNextAlarm_Processprograms(RTC_HandleTypeDef* rtc);
void							updatePrograms(void);
uint8_t						GetAllProcessPrograms(void);
void							deleteAllPrograms(void);
void							PrintAllPrograms(void);
uint8_t						JSON2Str(char* result, char* raw_input, char* key);
uint8_t						JSON2int(char* result, char* raw_input, char* key);
uint8_t						JSON2Str_nested(char* result, char* raw_input, char* key_parent, char* key_child);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
	MX_TIM5_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	TH = malloc(2*sizeof(float));
	DWT_Init();
ssd1306_Init();
	
	//	HAL_RTCEx_BKUPWrite(&hrtc,1,8260);
	//	HAL_RTCEx_BKUPWrite(&hrtc,2,8500);
		
	DEBUG("\n\r************************************************");
	DEBUG("\n\r*            Petus: Baghyar - V2.2             *");
	DEBUG("\n\r************************************************");
	DEBUG("\n\r*   Programed By :   Land Development Message  *");
	DEBUG("\n\r*           Date :   1399/08/22                *");
	DEBUG("\n\r************************************************");
	DEBUG("\n\r*                                              *");
	DEBUG("\n\r*               Made in I.R.Iran               *");
	DEBUG("\n\r*                                              *");
	DEBUG("\n\r************************************************\n\r");
	
	initializingFlag=1; //initializing seting is starting
	DEBUG("\n\r Tim start IT... \n\r");
	HAL_TIM_Base_Start_IT(&htim5);
	DEBUG("\n\r    --DONE--\n\r");
#if(__FIRSTTIME_PROGRAMMING__==1)		
	//*
	DEBUG("\n\rSETTING DATE AND TIME...");	
		HAL_Delay(500);
		RTC_TimeTypeDef sTime = {0};
		RTC_DateTypeDef sDate = {0};
		sTime.Hours = 18;
		sTime.Minutes = 11;
		sTime.Seconds = 30;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
		{
		  Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
		sDate.Month = RTC_MONTH_MARCH;
		sDate.Date = 14;
		sDate.Year = 2021 - 2000;
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
		{
		  Error_Handler();
		}
	DEBUG("\n\r    --DONE--");	
	//*/
	
	//*
	DEBUG("\n\rCLEARING BACKUP REGISTERS...");
		HAL_Delay(500);
		HAL_RTCEx_BKUPWrite(&hrtc, 1, 0);										// FLOW1
		HAL_RTCEx_BKUPWrite(&hrtc, 2, 0);										//FLOW2
		HAL_RTCEx_BKUPWrite(&hrtc, PROGRAM_ID_ADDRESS, 0);
		HAL_RTCEx_BKUPWrite(&hrtc, LAST_STATUS_ADDRESS, 0);
		HAL_RTCEx_BKUPWrite(&hrtc, MY_ID_ADDRESS, 0);
	DEBUG("\n\r    --DONE--");
	//*/
	
	//*
	DEBUG("\n\rCLEARING EEPROM...");
		HAL_Delay(500); 
		if(ee24_isConnected(&hi2c1)){
			ee24_eraseChip(&hi2c1);
		}
		else{
			DEBUG("\n\r ***Connection to EEPROM failed***");
		}
	DEBUG("\n\r    --DONE--");
	//*/
	
	
	DEBUG("\n\r  <<< INITIALIZING DONE >>>\n\r");
	
	while(1);
	
#endif

	/*
	DEBUG("\n\rAPPLYING LAST OUTPUT STATUS...");
		HAL_Delay(500);
		processFlag=HAL_RTCEx_BKUPRead(&hrtc, LAST_PROCESS_FLAG_STATUS);
		LastStatus = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, LAST_STATUS_ADDRESS);	
		ApplyAction(LastStatus);
	DEBUG("\n\r    --DONE--\n\r");	
	
	DEBUG("\n\rREADING FLOW1 AND FLOW2 FROM MEMORY...");
		HAL_Delay(500);
		Flow1=HAL_RTCEx_BKUPRead(&hrtc,1);	
		Flow2=HAL_RTCEx_BKUPRead(&hrtc,2);	
	DEBUG("\n\r    --DONE--\n\r");	
	//*/
	
	//*
	DEBUG("\n\rGETTING DATE AND TIME...");
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
		snprintf(str,sizeof(str),"\n\r Time: %d/%02d/%02d %02d:%02d:%02d", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds);
		DEBUG(str);
	DEBUG("\n\r    --DONE--\n\r");
	//*/	
	
	//*
	DEBUG("\n\rACTIVATING SIM808 MODULE...\n\r");
		HAL_Delay(500);
		if(!ACKHandler()){
			isConnect = 0;
			DEBUG("\n\r SIM800 is not responding. I'm Offline now.");
		}
	DEBUG("\n\r    --DONE--\n\r");	
	//*/
	
	//*
	DEBUG("\n\rSTARTING HTTP...\n\r");	
		sim80x_HTTP_Start();
	DEBUG("\n\r    --DONE--\n\r");	
	//*/
		
/*
	DEBUG("\n\rGETTING ID BY SERIAL NUMBER...\n\r");
		if(isConnect==1)
			myID = GetID();
		
		if(myID==0)
			myID = HAL_RTCEx_BKUPRead(&hrtc, MY_ID_ADDRESS);
		else
			HAL_RTCEx_BKUPWrite(&hrtc, MY_ID_ADDRESS, myID);
		
		if(myID==0)
			isConnect=0;
		
		memset(str, NULL, size);
		snprintf(str, size, "\n\rmyID is \"%d\"", myID);
		DEBUG(str);
	DEBUG("\n\r    --DONE--\n\r");	

*/
	//*
	DEBUG("\n\rSMS SETTING...\n\r");
		SMSSetting();
	DEBUG("\n\r    --DONE--\n\r");	
	//*/
	
	//*
	
	//*/
	
	/*
	DEBUG("\n\rSENDING SMS...\n\r");
		HAL_Delay(500);
		sim80x_SendSMS(phone,__WELCOME_TEXT,6000);
	DEBUG("\n\r    --DONE--\n\r");	
	*/
	
	/*	
	DEBUG("\n\rGETTING ALL PROGRAMS FROM SERVER...\n\r");
		HAL_Delay(500);
		if(isConnect==1){
			GetAllPrograms();
			AlarmIsSet = 0;
		}	
	DEBUG("\n\r    --DONE--\n\r");	
	//*/

	/*
	DEBUG("\n\rSETTING NEXT ALARM...");
		HAL_Delay(500);
		updatePrograms();
		SetNextAlarm(&hrtc);
		AlarmIsSet = 0;
	DEBUG("\n\r    --DONE--\n\r");	
	//*/
	
	/*
	DEBUG("\n\rGETTING ALL PROCESS PROGRAMS FROM SERVER...\n\r");
		HAL_Delay(500);
		if(isConnect==1)
	 {
			if( GetAllProcessPrograms()) // get all process programs: return 1 if is successful and return 0 is unsuccessful
				DEBUG("GetAllProcessPrograms is successful ");
			else
				DEBUG("ERROR in GetAllProcessPrograms ");
		}	
	DEBUG("\n\r    --DONE--\n\r");	
	//*/
	
	//*
	/*
	DEBUG("\n\rSETTING NEXT PROCESS PROGRAM ALARM...");
		HAL_Delay(500);
		memset(str,NULL,size);
		processFlag=HAL_RTCEx_BKUPRead(&hrtc, LAST_PROCESS_FLAG_STATUS);//read last process flag from eeprom
		sprintf(str,"\n\r Last Process flag status :%d",processFlag);
		DEBUG(str);
		if(HAL_RTCEx_BKUPRead(&hrtc, LAST_PROCESS_FLAG_STATUS)==1)
		{	
			//means we have Unfinished event as a result we must apply its action and set alarm for next events.
			DEBUG("\n\r Unfinished event exist \n\r");
			unfinishedEventFlag=1;
			lastTimeStamp=HAL_RTCEx_BKUPRead(&hrtc, LAST_CURRENT_TIME_STAMP);//read last current time stamp from eeprom
			memset(str,NULL,size);
			sprintf(str,"\n\rlastTimeStamp: %s \n\r",ctime(&lastTimeStamp));
			DEBUG(str);
			HAL_Delay(50);
			lastEventTimeStamp=HAL_RTCEx_BKUPRead(&hrtc, LAST_EVENT_TIME_STAMP);//read last last event time stamp from eeprom
			memset(str,NULL,size);
			sprintf(str,"\n\rlastEventTimeStamp: %s \n\r",ctime(&lastEventTimeStamp));
			DEBUG(str);
			buttonId=	HAL_RTCEx_BKUPRead(&hrtc, PROCESS_PROGRAM_ID_ADDRESS);//read last process program id from eeprom
			eventNumber=HAL_RTCEx_BKUPRead(&hrtc, LAST_EVENT_NUMBER_ADDRESS);//read last event number from eeprom
			
			buttonsStatus[buttonId-1]=1;//change the desired button status
			memset(str,NULL,size);
			sprintf(str,"\n\rButtonsStatus: %d%d \n\r",buttonsStatus[1],buttonsStatus[0]);
			DEBUG(str);
			SetProcessProgramsAlarm(&hrtc);
		}
		else
		{
			DEBUG("\n\r We do not have any unfinished event\n\r");
			unfinishedEventFlag=0;
		}
	DEBUG("\n\r    --DONE--\n\r");
	//*/
	
	//*
	DEBUG("\n\rSTOPING HTTP...\n\r");	
		sim80x_HTTP_Stop();
	DEBUG("\n\r    --DONE--\n\r");	
	//*/
	initializingFlag=0; //initializing seting done
		
	DEBUG("\n\r  <<< INITIALIZING DONE >>>\n\r");
	HAL_Delay(1000);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Checking for sim800 errors:
		SIM800_handler();
		
		// Sending SMS in case of output changes
		if(SMSisPending==1){
			SMSisPending = 0;
			sim80x_SendSMS(phone, SMStext, 6000);	
		}
		
		#if(__SENSORS_ARE_ENABLE__==1)
			measureAndLog();  
		#endif
		
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
		HAL_RTC_GetAlarm(&hrtc, &alarm_struct, RTC_ALARM_A, RTC_FORMAT_BIN);
		
		memset(str,NULL,size);
		
		if(AlarmIsSet)
			snprintf(str,sizeof(str),"Time: %d/%02d/%02d %02d:%02d:%02d -- Next Program Alarm: %d %02d:%02d:%02d -- Next Program ID: %d\r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds, alarm_struct.AlarmDateWeekDay, alarm_struct.AlarmTime.Hours, alarm_struct.AlarmTime.Minutes, alarm_struct.AlarmTime.Seconds, HAL_RTCEx_BKUPRead(&hrtc, PROGRAM_ID_ADDRESS));
		else if(processProgramsAlarmIsSet)
			snprintf(str,sizeof(str),"Time: %d/%02d/%02d %02d:%02d:%02d -- Next Process Program Alarm: %d %02d:%02d:%02d --  Button ID: %d --Current Event Number: %d \r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds, alarm_struct.AlarmDateWeekDay, alarm_struct.AlarmTime.Hours, alarm_struct.AlarmTime.Minutes, alarm_struct.AlarmTime.Seconds, HAL_RTCEx_BKUPRead(&hrtc, PROCESS_PROGRAM_ID_ADDRESS),HAL_RTCEx_BKUPRead(&hrtc, LAST_EVENT_NUMBER_ADDRESS)+1);
				else  
			snprintf(str,sizeof(str),"Time: %d/%02d/%02d %02d:%02d:%02d -- There is no active Alarm\r\n", 2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds);
			DEBUG(str);
			HAL_Delay(2000);
		//DEBUG(buttonsStatus[0]);
		// This section runs each 5 minutes
	
	if(action){
			
		 uint32_t 							currentTimeStamp = 0;		//used when convert current time to time stamp
		 struct tm  						currentTime;
		 currentTime.tm_year	= Date.Year + 2000  - 1900;
		 currentTime.tm_mon		= Date.Month;
		 currentTime.tm_mday	= Date.Date;
		 currentTime.tm_hour	= Time.Hours;
		 currentTime.tm_min		= Time.Minutes;
		 currentTime.tm_sec		= Time.Seconds;
		 currentTime.tm_isdst	= -1;	
		 currentTimeStamp = mktime(&currentTime);//convert current time to time stamp
	 	 HAL_RTCEx_BKUPWrite(&hrtc, LAST_CURRENT_TIME_STAMP, currentTimeStamp);//stor time stamp into eeprom
			
		 sim80x_HTTP_Start();
			
//		if( GetAllProcessPrograms()) // get all process programs: return 1 if is successful and return 0 is unsuccessful
//			DEBUG("GetAllProcessPrograms is successful ");
//		else
//			DEBUG("ERROR in GetAllProcessPrograms ");
//				
		get_output_result = GetOutput();
			/*
				get_output_result has 5 values:
					-- 0: Means the connection is lost
					-- 1: Means connection is stable and there is no change in programs and outputs 
					-- 2: Means connection is stable and programs are changed
					-- 3: Means connection is stable and outputs have to be changed
					-- 4: Means connection is stable and both of programs and outputs have to be changed
			*/
		if(processFlag==0){		
				switch (get_output_result)
				{
					case 0:
						if(isConnect==1){
							DEBUG("\n\r --- OFLINE SHODAM :(( --- \n\r");
							isConnect = 0;
							updatePrograms();
							SetNextAlarm(&hrtc);
						}
						break;
						
					case 1:
						isConnect = 1;
						AlarmIsSet = 0;
						DEBUG("\n\r --- BI TAGHYEER --- \n\r");
						break;
					
					case 2:
						isConnect = 1;
						AlarmIsSet = 0;
						DEBUG("\n\r --- TAGHYEER DAR BARNAME --- \n\r");
						GetAllPrograms();
						updatePrograms();
						break;
					
					case 3:
						isConnect = 1;
						AlarmIsSet = 0;
						DEBUG("\n\r --- TAGHYEER DAR KHOROOJI --- \n\r");
						ApplyAction(str2bin(str));
						memset(SMStext, NULL, 120);
						snprintf(SMStext, 140, "Output status is: \n| %c | %c | %c | %c |%c | %c |\n\r", str[0], str[1], str[2], str[3],str[4], str[5]);
						//DEBUG(SMStext);
						//sim80x_SendSMS(phone, SMStext, 6000);	
					
						break;
					
					case 4:
						isConnect = 1;
						AlarmIsSet = 0;
						DEBUG("\n\r --- TAGHYEER DAR BARNAME VA KHOROOJI --- \n\r");
						ApplyAction(str2bin(str));
						
						memset(SMStext, NULL, 140);
						snprintf(SMStext, 140, "Output status is: \n| %c | %c | %c | %c |%c | %c |\n\r", str[0], str[1], str[2], str[3],str[4], str[5]);
						DEBUG(SMStext);
						sim80x_SendSMS(phone, SMStext, 6000);	

						GetAllPrograms();
						updatePrograms();
						break;
				 }
			 }
		
			sim80x_Send_Status(SERVER_IP); //ContentStr is the JSON contain RSSI value , OUTPUT Status and external Buttons Status that  post in server
			if(Sim80x_StatusTypeDef!=HAL_OK)//if sim800 response OK to rssi at_command  
			{				
				 DEBUG( "***\r\n");
				 DEBUG("RSSI: Can not read RSSI from sim80x\r\n");
				 DEBUG( "***\r\n");
			}

			#if(__SENSORS_ARE_ENABLE__==1)
				if(isConnect==1)
					postSensors();
			#endif
			sim80x_HTTP_Stop();
			action=0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RTC_WKUP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  /* RTC_Alarm_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* USER CODE BEGIN 4 */

/**

  * @brief  RTC wakeup interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){
  action=1;
}

/**
  * @brief  RTC alarm A interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
//	DEBUG("**************SYSTEM IS GOING TO BE RESET*************");
// 	HAL_NVIC_SystemReset();
	if(processProgramsAlarmIsSet)
		SetProcessProgramsAlarm(hrtc);	
	if(AlarmIsSet)
		SetAlarm(hrtc);
}
/**
  * @brief  RTC alarm B interrupt.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  */
void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc){
	/*
		NOTHING YET
	*/
}
/**
  * @brief  GPIO external interrupt handler.
  * @param  pin number
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	#if(__SENSORS_ARE_ENABLE__==1)
		if(GPIO_Pin == flow1_EXIT_Pin)
		{
			while(!(HAL_GPIO_ReadPin(flow1_EXIT_GPIO_Port,flow1_EXIT_Pin)))
			counter++;
			if(counter>2000)
			{
				Flow1++;
				HAL_RTCEx_BKUPWrite(&hrtc,1,Flow1);
				counter=0;
			}
		}
		
		if(GPIO_Pin == flow2_EXIT_Pin)
		{
			while(!(HAL_GPIO_ReadPin(flow2_EXIT_GPIO_Port,flow2_EXIT_Pin)))
			counter++;
			
			if(counter>40000)
			{
				Flow2++;
				HAL_RTCEx_BKUPWrite(&hrtc,2,Flow2);
				counter=0;
			}
		}
	#endif
	
	#if(__BUTTONS_ARE_ENABLE__==1)
		//while((HAL_GPIO_ReadPin(sw2_EXIT_GPIO_Port,sw2_EXIT_Pin) || HAL_GPIO_ReadPin(sw1_EXIT_GPIO_Port,sw1_EXIT_Pin)) && (counter<2000002))
		while((HAL_GPIO_ReadPin(sw2_EXIT_GPIO_Port,sw2_EXIT_Pin)) && (counter<2000002))
			counter++;
		if(counter>2000000)
		{	
			counter = 0;
			if(GPIO_Pin == GPIO_PIN_4)//button 0 interrupt
			{
				if(buttonsStatus[0]==1)
				{
					buttonsStatus[0]=0;		//toggle buttonsStatus[0]
					processProgramsAlarmIsSet=0;//disable process program Alarm
					processFlag=0;
					HAL_RTCEx_BKUPWrite(&hrtc, LAST_PROCESS_FLAG_STATUS, processFlag);
					DEBUG("\n\rProcess Programs Alarm is disables \n\r");

				}
				else if(buttonsStatus[0]==0&&getProcessProgramsStatus)
				{
					//button 0 pushed & get process programs is successful 
					buttonsStatus[0]=1; //toggle buttonsStatus[0]
					DEBUG("*****Button 1 Status has been changed*****\n\r");
					processFlag=1;
					HAL_RTCEx_BKUPWrite(&hrtc, LAST_PROCESS_FLAG_STATUS, processFlag);
					eventNumber=0;
					buttonId=1;
					SetProcessProgramsAlarm(&hrtc);//Button 2 Status =1 and we must aplly its first event action and set alarm for anothers.
				}
				while(HAL_GPIO_ReadPin(sw1_EXIT_GPIO_Port,sw1_EXIT_Pin));
			}
			
			if(GPIO_Pin == GPIO_PIN_5)//button 1 interrupt
			{
				if(buttonsStatus[1]==1)
				{	
					buttonsStatus[1]=0;//toggle buttonsStatus[1]
					processFlag=0;
					HAL_RTCEx_BKUPWrite(&hrtc, LAST_PROCESS_FLAG_STATUS, processFlag);
					processProgramsAlarmIsSet=0;//disable process program Alarm
					DEBUG("\n\rProcess Programs Alarm is disables \n\r");

				}
				else if(buttonsStatus[1]==0&&getProcessProgramsStatus)
				{
					//button 1 pushed & get process programs is successful 
					buttonsStatus[1]=1;//toggle buttonsStatus[1]
					DEBUG("*****Button 2 Status has been changed*****\n\r");
					processFlag=1;
					HAL_RTCEx_BKUPWrite(&hrtc, LAST_PROCESS_FLAG_STATUS, processFlag);
					eventNumber=0;
					buttonId=2;
					SetProcessProgramsAlarm(&hrtc);//Button 2 Status =1 and we must aplly its first event action and set alarm for anothers.
				}
				while(HAL_GPIO_ReadPin(sw2_EXIT_GPIO_Port, sw2_EXIT_Pin));
			}
		  memset(str,NULL,size);
			sprintf(str,"\n\r buttonsStatus: %d%d \n\r",buttonsStatus[1],buttonsStatus[0]);
			DEBUG(str);
		}
		counter = 0;


	#endif
}
/**
  * @brief  Communicate with sensors and collect their values. Then log them in debug port.
  */
void measureAndLog(void){
	
		//Tempereture and humidity
		//*
		TH = AM2305_GetData();
		if(TH[2]==0xFFFF)
		{
			TH[0]=0.0;
			TH[1]=0.0;
		}
		//*/
				
		//Moisture
		//*
		HAL_ADC_Start_DMA(&hadc1, AnalogSensors, 3);
		moisture[averageItterator]=AnalogSensors[2];
		averageItterator++;
		if (averageItterator==10)	averageItterator=0;
		
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
		
		memset(str,NULL,size);
		snprintf(str,sizeof(str),"Time: %02d:%02d:%02d *** moisture: %2.2f *** Humidity: %2.1f *** Temperature: %2.1f *** Flow1: %d *** Flow2: %d\r\n", Time.Hours, Time.Minutes, Time.Seconds,(AnalogSensors[2]*3.3)/4095, TH[0], TH[1], Flow1, Flow2);
		DEBUG(str);
		HAL_Delay(1000);
		//*/
}
/**
  * @brief  Calculate the average of sensor values (if neesed) then POST them to server
  */
void postSensors(void){
	// calculating moisture average
	for(uint8_t index=0;index<10;index++)
		AnalogSensors[2]+=moisture[index];
	AnalogSensors[2]=AnalogSensors[2]/11;
			
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"{\"land\":\"%d\",\"temp_value\":%2.2f,\"soil_moisture_value\":%2.2f,\"humidity_value\":%2.2f,\"water_value\":%d,\"trial_water_value\":%d}\r\n", myID, TH[1], (AnalogSensors[2]*3.3)/4095, TH[0], Flow1, Flow2);
	DEBUG(ContentStr);
			
	sim80x_HTTP_Post(ContentStr, SERVER_IP, "panel/api/multiSensor/", ContentStr);	
}

/**
  * @brief  Gets the "output" and "program" status from server.
	* @retval An uint8_t value according to the following:
							-- 0: Means the connection is lost
							-- 1: Means connection is stable and there is no change in programs and outputs 
							-- 2: Means connection is stable and programs are changed
							-- 3: Means connection is stable and outputs have to be changed
							-- 4: Means connection is stable and both of programs and outputs have to be changed
						The output status (in case of change) is stored in "str" variable.
  */
uint8_t GetOutput(void){
	char *startAnswer;
	char *endAnswer;
	char  value_str[30];
	uint8_t OutputIsChanged=0;
	uint8_t ProgramIsChanged=0;
	uint8_t ReturnValue = 0;
	
//	sim80x_HTTP_Start();
	
	snprintf(str,sizeof(str),"AT+HTTPPARA=\"URL\",\"http://%s/panel/api/d/%d/eo/\"\r\n",SERVER_IP, myID);
	sim80x_ATC(str,1000);
		
			if(sim80x_ATC("AT+HTTPACTION=0\r\n",10000) == HAL_OK)
			{
				DEBUG(RxBuffer);
				if(sim80x_ATC("AT+HTTPREAD\r\n",2000) == HAL_OK)
				{									
					DEBUG(RxBuffer);					
						
					//remove SIM800 commands and OK in response:		
					startAnswer	= strstr(RxBuffer, "{");
					endAnswer		= strstr(RxBuffer, "]}")+2;
					
					memset(str, NULL, size);
					for(int i=(startAnswer-RxBuffer); i<(endAnswer-RxBuffer); i++)
						str[i-(startAnswer-RxBuffer)] = RxBuffer[i];
					
					memset(RxBuffer, NULL, size);
					memcpy(RxBuffer, str, size);									// Fetching pure JSON from SIM800 response
					
					if(JSON2int(value_str, RxBuffer, "p")){
						DEBUG("\n\r  Extracted Program Status is ");
						DEBUG(value_str);
						DEBUG("\n\r");
						ProgramIsChanged = atoi(value_str);
					}
					
					if(JSON2Str_nested(value_str, RxBuffer, "d", "o")){
						DEBUG("\n\r  Extracted Output Status is ");
						DEBUG(value_str);
						DEBUG("\n\r");
						
						memset(str, NULL, size);
						memcpy(str,value_str, 30);						
						
						uint8_t string2bin=0x00;						
						string2bin = str2bin(value_str);
						LastStatus = (uint8_t) HAL_RTCEx_BKUPRead(&hrtc, LAST_STATUS_ADDRESS);
						if(LastStatus!=string2bin)
							OutputIsChanged=1;						

					}
											
					if(ProgramIsChanged==0 && OutputIsChanged==0)
						ReturnValue = 1;
					else if(ProgramIsChanged==1 && OutputIsChanged==0)
						ReturnValue = 2;
					else if(ProgramIsChanged==0 && OutputIsChanged==1)
						ReturnValue = 3;
					else if(ProgramIsChanged==1 && OutputIsChanged==1)
						ReturnValue = 4;
					else
						ReturnValue = 0;
						
				}
				else{
					ReturnValue = 0;
				}
		  }
		  else{
			  ReturnValue = 0;
		  }
	//sim80x_HTTP_Stop();
	return ReturnValue;
}

/**
  * @brief  Gets the land ID according to serial number from the server
  * @retval The land ID. If the returned value was 0 it means the request was failed.
  */
uint16_t GetID(void){
	char *startAnswer;
	char *endAnswer;
	uint16_t output = 0;
	
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"{\"serial\":\"%s\"}\r\n", __SERIAL_NUMBER);
	DEBUG(ContentStr);
	sim80x_HTTP_Post(ContentStr, SERVER_IP,"panel/api/d/get_land_by_device_serial/",ContentStr);
	
	if(strstr((char *)ContentStr,"land") != NULL){
		startAnswer = strstr(ContentStr, "land\":")+6;
		endAnswer  = strstr(ContentStr, "}");
		
	char out[30];
	memset(out, NULL, 30);
	for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)
		out[i-(startAnswer-ContentStr)] = ContentStr[i];
	output = (uint16_t)atoi(out);
	}
	
	return output;
}/* 0 */uint8_t  ID;
/**
  * @brief  Writes a program in EEPROM
  * @param  prog is a pointer to your desired processProgram struct
  */
void writeProcessProg(ProcessProgram* processProg){
	if(ee24_isConnected(&hi2c1))
	{
		//fill 	tempData with processprog data 
		uint8_t tempData[50];
		tempData[0] = processProg->ID;
		tempData[41] = processProg->events_count;
		for(int i=1;i<=processProg->events_count;i++)
		{
			tempData[i]    = processProg->Hour[i-1];
			tempData[10+i] = processProg->Minutes[i-1];
			tempData[20+i] = processProg->Second[i-1];
			tempData[30+i] = processProg->spouts[i-1];	
		}

		ee24_write(&hi2c1, (processProg->ID-1)*100+1000, tempData, 42, 2000);//write tempdata into the eeprom

	}
	else
		DEBUG("EEPROM ERROR");
}


/**
  * @brief  Writes a program in EEPROM
  * @param  prog is a pointer to your desired Program struct
	* @param  is_interrupt: if you want to call this function in an interrupt routin you have to set this value as 1. Otherwise, set this 0.
  */
void writeProg(Program* prog){
	if(ee24_isConnected(&hi2c1)){
		uint8_t tempData[10];
		
		tempData[0] = prog->ID;
		tempData[1] = prog->Month;
		tempData[2] = prog->Day;
		tempData[3] = prog->Hour;
		tempData[4] = prog->Minutes;
		tempData[5] = prog->PeriodDay;
		tempData[6] = prog->PeriodHour;
		tempData[7] = prog->PeriodMinute;
		tempData[8] = prog->Action;
		tempData[9] = prog->has_interval;
		
		ee24_write(&hi2c1, prog->ID*10, tempData, 10, 2000);
	}
	else
		DEBUG("EEPROM ERROR");
}
/**
  * @brief  Reads a Processprogram from EEPROM.
  * @param  progID is your desired program ID( the number of button).
  * @retval processProgram struct.
  */
ProcessProgram readProcessProg(uint8_t progID){
	ProcessProgram processProg;
	
	processProg.ID 					= NULL;
	processProg.events_count	= NULL;
	memset(processProg.Hour,NULL,10);
	memset(processProg.Minutes,NULL,10);
	memset(processProg.Second	,NULL,10);
	memset(processProg.spouts,NULL,10);
	
	if(ee24_isConnected(&hi2c1))
	{
		uint8_t tempData[42];
		ee24_read(&hi2c1, (progID-1)*100+1000, tempData, 42, 2000);//read process program from eeprom and pour it into the tempData
		
		//fill processProg struct with tempdata arrays data:
		processProg.ID 					= tempData[0];
		processProg.events_count = tempData[41]; 
		for(int i=1;i<=processProg.events_count;i++)
			{
				processProg.Hour[i-1]=tempData[i] ;
				processProg.Minutes[i-1]=	tempData[10+i];
				processProg.Second[i-1]=tempData[20+i];
				processProg.spouts[i-1]=tempData[30+i];	
			}
	}
	else
		DEBUG("EEPROM ERROR");
	return processProg;
}



/**
  * @brief  Reads a program from EEPROM.
  * @param  progID is your desired program ID.
  * @retval Program struct.
  */
Program readProg(uint8_t progID){
	Program output;
	output.ID 					= NULL;
	output.Month				= NULL;
	output.Day					= NULL;
	output.Hour					= NULL;
	output.Minutes			= NULL;
	output.PeriodDay		= NULL;
	output.PeriodHour		= NULL;
	output.PeriodMinute	= NULL;
	output.Action				= NULL;
	output.has_interval	= NULL;
	
	if(ee24_isConnected(&hi2c1)){
		
		uint8_t tempData[10];
		
		ee24_read(&hi2c1, progID*10, tempData, 10, 2000);
		
		output.ID 					= tempData[0];
		output.Month 				= tempData[1]; 
		output.Day 					= tempData[2];
		output.Hour					= tempData[3];
		output.Minutes			= tempData[4];
		output.PeriodDay		= tempData[5];
		output.PeriodHour		= tempData[6];
		output.PeriodMinute	= tempData[7];
		output.Action				= tempData[8];
		output.has_interval	= tempData[9];
	}
	else
		DEBUG("EEPROM ERROR");
	return output;
}
 /**
	* @brief  Delete All Previous process programs stored in EEPROM.
  * @param  prog is a pointer to your desired ProcessProgram struct
*/
void deleteAllProcessPrograms(void){
	if(ee24_isConnected(&hi2c1))
	{
	
//		for(int i=0;i<1000;i++)
//			ee24_write(&hi2c1, PROCESS_PROG_START_ADD +i , (uint8_t *)0xff, 1, 1);

	uint8_t values[1000];
	memset(values, 255, 1000);
	ee24_write(&hi2c1, PROCESS_PROG_START_ADD, values, 1000, 1000);//Delete All Previous process programs. Address: from PROCESS_PROG_START_ADD to PROCESS_PROG_START_ADD+1000
		
	}
	else
		DEBUG("EEPROM ERROR");	

}
/**
  * @brief  Delete a programs stored in EEPROM.
  * @param  prog is a pointer to your desired Program struct
	* @param  is_interrupt: if you want to call this function in an interrupt routin you have to set this value as 1. Otherwise, set this 0.
  */
void deleteProg(Program* prog){
	if(ee24_isConnected(&hi2c1)){
		uint8_t tempData[10];
		
		tempData[0] = 0xFF;
		tempData[1] = 0xFF;
		tempData[2] = 0xFF;
		tempData[3] = 0xFF;
		tempData[4] = 0xFF;
		tempData[5] = 0xFF;
		tempData[6] = 0xFF;
		tempData[7] = 0xFF;
		tempData[8] = 0xFF;
		tempData[9] = 0xFF;

		ee24_write(&hi2c1, prog->ID*10, tempData, 10, 2000);
	}
	else
		DEBUG("EEPROM ERROR");	
}

/**
  * @brief  Sends a request to get all process programs from sever, parse the programs and store them in EEPROM.
  */
uint8_t GetAllProcessPrograms(void){

	char*		startAnswer;
	char*		endAnswer;
	char    processProgram_raw[size];//used in separating processProgram of evrery buttons
	int 		buttonsNum; 
	getProcessProgramsStatus=0;//Get process program  successful /Unsuccessful flag
	ProcessProgram processProg;
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"{\"land\":%d}\r\n", myID);									// INPUT FORMAT --> { "land" : xx }
	DEBUG(ContentStr);
	
	// start GET processPrograms from server:
	//sim80x_HTTP_Start();
	snprintf(str,sizeof(str),"AT+HTTPPARA=\"URL\",\"http://%s/panel/api/d/%d/external_button_programs/\"\r\n",SERVER_IP, myID);//process prpgram api
	sim80x_ATC(str,1000);
	if(	sim80x_ATC(str,1000)== HAL_OK)
	{
			
		if(sim80x_ATC("AT+HTTPACTION=0\r\n",10000) == HAL_OK)
		{
			DEBUG(RxBuffer);
			if(sim80x_ATC("AT+HTTPREAD\r\n",2000) == HAL_OK)
			{	
				memset(ContentStr, NULL, size);
				memcpy(ContentStr, RxBuffer, size);					
				DEBUG(ContentStr);
			//sim80x_HTTP_Stop();
				
			// end GET processPrograms from server
				if(strstr((char *)ContentStr,"events_count") != NULL)
				{
						deleteAllProcessPrograms(); //delete All previous process programs from eeprom
						buttonsNum=0;
						while(	buttonsNum<BUTTONS_NUM)// continue until processProgram of evrery buttons is parsed.
						{
							//remove SIM800 commands and OK in response:
							startAnswer	= strstr(ContentStr, "{");
							endAnswer		= strstr(ContentStr, "\"events_count\"")+18;	
							memset(processProgram_raw, NULL, size);
							for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)
								processProgram_raw[i-(startAnswer-ContentStr)] = ContentStr[i]; // separating processProgram of evrery buttons
							DEBUG("\n\r*************\n\r");
							DEBUG(processProgram_raw);
							processProg=processProgramParser( processProgram_raw);//parsing the parameter of every processProgram including : number(ID), events, events_count
							writeProcessProg(&processProg);// write processPrograms parametter in eeprom
							DEBUG("\n\r*************\n\r");
							memcpy(ContentStr, ContentStr+strlen(processProgram_raw)+31, size);
							buttonsNum++;
						}

					for(buttonsNum=1;buttonsNum<=BUTTONS_NUM;buttonsNum++)
						PrintAllProcessProgram(buttonsNum);// Prints all valid Process programs stored in EEPROM.	
						
						getProcessProgramsStatus=1;//Get process program  successful 
						return  getProcessProgramsStatus;	
				}
				
				else if(strstr((char *)ContentStr,"[]") != NULL)
				{
					DEBUG("\n\r --- There is no process program in server --- \n\r");
					getProcessProgramsStatus=0;//Get process program  unsuccessful 
					return  getProcessProgramsStatus;
				}
				
				else
				{
					DEBUG("\n\r --- Erorr in getAllprocessPrograms --- \n\r");
					getProcessProgramsStatus=0;//Get process program  unsuccessful 
					return  getProcessProgramsStatus;
				}
				
			}
			else getProcessProgramsStatus=0;//Get process program  unsuccessful 
		}	
		else
			getProcessProgramsStatus=0;//Get process program  unsuccessful 
	}
	else
		getProcessProgramsStatus=0;//Get process program  unsuccessful 
	
return  getProcessProgramsStatus;	

	
}	

/**
  * @brief  Sends a request to get all programs from sever, parse the programs and store them in EEPROM.
  */
void GetAllPrograms(void){
	char*		startAnswer;
	char*		endAnswer;
	Program temp;
	
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"{\"land\":%d}\r\n", myID);									// INPUT FORMAT --> { "land" : xx }
	DEBUG(ContentStr);
	sim80x_HTTP_Post(ContentStr, SERVER_IP,"panel/api/d/p_list/",ContentStr);
	
	if(strstr((char *)ContentStr,"has_interval") != NULL){
		
		//remove SIM800 commands and OK in response:		
		startAnswer	= strstr(ContentStr, "[")+1;
		endAnswer		= strstr(ContentStr, "]")+1;
		
		memset(str, NULL, size);
		for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)
			str[i-(startAnswer-ContentStr)] = ContentStr[i];
		
		memset(ContentStr, NULL, size);
		memcpy(ContentStr, str, size);									// Fetching pure JSON from SIM800 response
		
		// Deleting previous programs:
		deleteAllPrograms();
		
		// Extract programs:
		DEBUG("All Programs:\n\r");
		
		char program_raw[110];		
		while(strstr((char *)ContentStr,"{") != NULL){			
			startAnswer = strstr(ContentStr, "{");
			endAnswer	  = strstr(ContentStr, "}")+1;
			
			memset(program_raw, NULL, 110);
			for(int i=(startAnswer-ContentStr); i<(endAnswer-ContentStr); i++)
				program_raw[i-(startAnswer-ContentStr)] = ContentStr[i];
			
			DEBUG("\n\r");
			DEBUG(program_raw);
			
			temp = ProgramParser(program_raw);
			writeProg(&temp);
			PrintProgram(temp);
			memcpy(ContentStr, ContentStr+strlen(program_raw)+1, size);	
		}
	}
	else if(strstr((char *)ContentStr,"[]") != NULL){
		DEBUG("\n\r --- There is no program in server --- \n\r");
		deleteAllPrograms();
		AlarmIsSet = 0;
	}
	else{
		DEBUG("\n\r --- Erorr in getAllPrograms --- \n\r");
	}
}


/**
  * @brief  Parse a string received from server to a processProgram struct.
	* @param  input_str is a JSON icluding one button process program like : "{"number":2,"events":[{"duration":"33.0","spouts":"010000"},{"duration":"12.0","spouts":"101100"},{"duration":"20.0","spouts":"111100"},{"duration":"15.0","spouts":"010011"}],"events_count":4}]
	* @retval processProgram struct
  */
ProcessProgram processProgramParser(char* input_str){
	char*	startAnswer;
	char*	endAnswer;
	char  value_str[200];// used in debuging Extracted JSON results
	char	json_result[20];	
	
	//Creates an empty instance from processProgram struct by NULL values
	ProcessProgram processProg;
	processProg.ID     				= NULL;
	memset(processProg.duration,NULL,10);
	memset(processProg.Hour,NULL,10);
	memset(processProg.Minutes,NULL,10);
	memset(processProg.Second,NULL,10);
	memset(processProg.spouts	,NULL,10);
	processProg.events_count		= NULL;
// Extracting number  ==> "number ":xxx, ...
	if(JSON2int(json_result, input_str, "number"))
	{
		processProg.ID= (uint8_t)atoi(json_result);
		sprintf(value_str, "\n\r   ID : \"%d\"", processProg.ID);
		DEBUG(value_str);
	}
	// Extracting events_count  ==> "events_count ":xxx, ...

	if(JSON2int(json_result, input_str, "events_count"))
	{
		processProg.events_count  = (uint8_t)atoi(json_result);
		sprintf(value_str, "\n\r   events_count : \"%d\"", processProg.events_count);
		DEBUG(value_str);
	}
	DEBUG("\n\r");

	// Extracting events:
	memset(str,NULL,size);
	startAnswer = strstr(input_str, "events")+9;
	endAnswer	  = strstr(input_str, "]")+1;
	for(int i=(startAnswer-input_str); i<(endAnswer-input_str); i++)
		str[i-(startAnswer-input_str)] = input_str[i];
	
// Extracting duration & spouts in every events  ==> "duration ":xxx, ...

	for(int i=0;i<processProg.events_count;i++)
	{
		memset(processProgram,NULL,size);
		startAnswer = strstr(str, "{");
		endAnswer	  = strstr(str, "}")+1;
		for(int j=(startAnswer-str); j<(endAnswer-str); j++)
				processProgram[j-(startAnswer-str)] = str[j];	
		if(JSON2Str(json_result, processProgram, "duration"))
			{
				processProg.duration[i]	= (uint32_t)atoi(json_result);
				//sprintf(value_str, "\n\r    event:%d duration (sec): \"%d\"",i, processProg.duration[i]);
				//DEBUG(value_str);
				//calculate Hour,Minutes,Second of every events from the received "duration(s)"
				processProg.Hour[i]=processProg.duration[i]/3600;
				processProg.Minutes[i]=processProg.duration[i]/60-processProg.Hour[i]*60;
				processProg.Second[i]=processProg.duration[i]-processProg.Minutes[i]*60-processProg.Hour[i]*3600;
				sprintf(value_str, "\n\r  event:%d Time duration   :   %02d:%02d:%02d ",i,processProg.Hour[i], processProg.Minutes[i],processProg.Second[i]);
				DEBUG(value_str);
			}			
		if(JSON2Str(json_result, processProgram, "spouts"))
			{
				processProg.spouts[i]	= (uint8_t)str2bin(json_result);
				sprintf(value_str, "\n\r   event:%d  spouts  : \"%s\"",i,json_result);
				DEBUG(value_str);
			}
		memcpy(str, str+strlen(processProgram)+1, size);
		DEBUG("\n\r");
	}
		

	return processProg;

}
/**
  * @brief  Parse a string received from server to a Program struct.
  * @param  input_str is a string like " ... "
	* @retval Program struct
  */

Program ProgramParser(char* input_str){
	char*	startAnswer;
	char*	endAnswer;
	char  value_str[50];
	char	json_result[20];
		
	//Creates an empty instance from Program struct by NULL values
	Program output;
	output.ID 					= NULL;
	output.Month				= NULL;
	output.Day					= NULL;
	output.Hour					= NULL;
	output.Minutes			= NULL;
	output.PeriodDay		= NULL;
	output.PeriodHour		= NULL;
	output.PeriodMinute	= NULL;
	output.Action				= NULL;
	output.has_interval	= NULL;
	
	// Extracting ID ==> "id":xxx, ...		
		if(JSON2Str(json_result, input_str, "pk")){
			sprintf(value_str, "\n\r  Extracted ID is \"%d\"", atoi(json_result));
			DEBUG(value_str);
			output.ID = atoi(json_result);
		}
	
	//Extract Time ==> ... ,"start":"yyyy/MM/dd,hh:mm:ss", ...	
		if(JSON2Str(json_result, input_str, "start")){
			sprintf(value_str, "\n\r  Extracted Time is \"%s\"", json_result);
			DEBUG(value_str);
			
			//	| y | y | y | y | / | M | M | / | d | d | ,  | h  | h  | :  | m  | m  | :  | s  | s  |
			//  | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 		
			output.Month		= (uint8_t)( atoi(&json_result[5]  ));
			output.Day			= (uint8_t)( atoi(&json_result[8]  ));
			output.Hour			= (uint8_t)( atoi(&json_result[11] ));
			output.Minutes	= (uint8_t)( atoi(&json_result[14] ));
		}
		
	//Extract Raw Interval ==> ... ,"interval":"d hh:mm:ss:", ...
		if(JSON2Str(json_result, input_str, "interval")){
			sprintf(value_str, "\n\r  Extracted interval is \"%s\"\n\r", json_result);
			DEBUG(value_str);
			
			sprintf(value_str, "\"%s\"", json_result);
			
				//Extract Period Day ==> "ddd ...
					char  interval_str[10];
					
					startAnswer = strstr(value_str, "\"")+1;
					endAnswer  	= strstr(value_str, " ");
					
					memset(interval_str, NULL, 10);
					for(int i=(startAnswer-value_str); i<(endAnswer-value_str); i++)
						interval_str[i-(startAnswer-value_str)] = value_str[i];
					
					DEBUG("   -- Extracted Period Day is ");
					DEBUG(interval_str);
					DEBUG("\n\r");
					
					output.PeriodDay = (uint8_t)(atoi(interval_str));
				
				//Extract Period Hour and Minutes ==> ... hh:mm:ss"
					startAnswer = strstr(value_str, " ")+1;
					endAnswer  	= strstr(value_str+1, "\""); // +1 for ignoring first "
					
					memset(interval_str, NULL, 10);
					for(int i=(startAnswer-value_str); i<(endAnswer-value_str); i++)
						interval_str[i-(startAnswer-value_str)] = value_str[i];
					
					DEBUG("   -- Extracted Period Time is ");
					DEBUG(interval_str);
					DEBUG("\n\r");
					
					output.PeriodHour		= (uint8_t)(atoi(&interval_str[0]));
					output.PeriodMinute	= (uint8_t)(atoi(&interval_str[3]));
			}
		
	//Extract Action ==> ... ,"action":"xx", ...
		if(JSON2Str(json_result, input_str, "action")){
			sprintf(value_str, "\n\r  Extracted action is \"%s\"\n\r", json_result);
			DEBUG(value_str);
			output.Action = str2bin(json_result);
		}
	
	//Extract has_interval ==> ... ,"has_interval":true/false}		
		if(JSON2Str(json_result, input_str, "has_interval")){
			sprintf(value_str, "\n\r  Extracted has_interval is \"%s\"\n\r", json_result);
			DEBUG(value_str);
		}
		
		if(!strcmp(json_result, "false")){
			output.PeriodDay		= 0x00;
			output.PeriodHour		= 0x00;
			output.PeriodMinute	= 0x00;
			output.has_interval = 0x00;
		}
		else{
			output.has_interval = 1;
		}
		
	return output;
}

/**
  * @brief  Apply output on GPIOs
	* @param  status is a number between 0 and 15 representing the outputs. For example 13 means: | 1 | 1 | 0 | 1 |
  */
void ApplyAction(uint8_t status){
	/*
		* Tozih:
			voroodi in tabe, yek adad beyne 0 ta 15 ast. bayad tabdil be binarry beshe. har bitesh male yek khorooji ast.
			masalan vorrodi hast 12,  Outputs ha injoori mishan: Output[0] = 1, Output[1] = 1, Output[2] = 0, Output[3] = 0
			nahveye map shodane har bit be har kanale relay tooye oonja ke WritePin kardam ghabele thagyeere.
	*/
	
	char 		action_str[35];
	uint8_t Outputs[4];
	
	Outputs[0] = (status>>5)&0x01;
	Outputs[1] = (status>>4)&0x01;
	Outputs[2] = (status>>3)&0x01;
	Outputs[3] = (status>>2)&0x01;
	
	DEBUG("\n\rAction applied --> ");
	memset(action_str, NULL, 20);
	snprintf(action_str, 20, "| %d | %d | %d | %d | \n\r", Outputs[0], Outputs[1], Outputs[2], Outputs[3]);
	DEBUG(action_str);
	
	HAL_RTCEx_BKUPWrite(&hrtc, LAST_STATUS_ADDRESS, status);
	if(processFlag==0)
	HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, Outputs[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, Outputs[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, Outputs[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, Outputs[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);	
}
/**
  * @brief  Print a string in debug port
  * @param  input is the string that you want to print in UART
  */
void DEBUG(char* input){
	HAL_UART_Transmit(&debugUART, (uint8_t*)input, strlen(input), 1000);
}


/**
* 	@brief  This toutin first apply  the process programs event  that it is time to   and then set alarm for next event actions.
  * @param  hrtc handler
 *  @retval return 0 if The Botton "x"  process has been done
  */
uint8_t SetProcessProgramsAlarm(RTC_HandleTypeDef* rtc){
	
	ProcessProgram 	processProg;
	uint8_t		WhatToDo;//The outputs status that must be applyed
	//CurrentProgramID	= HAL_RTCEx_BKUPRead(&hrtc, PROGRAM_ID_ADDRESS);
	processProgramsAlarmIsSet=0;//Alarm disable
	processProg		= readProcessProg(buttonId);//read desired button process program from eeprom
	
	if(eventNumber==processProg.events_count)
	{
		//means we do not have any event to set alarm for it
		processFlag=0;
		HAL_RTCEx_BKUPWrite(&hrtc, LAST_PROCESS_FLAG_STATUS, processFlag);
		buttonsStatus[buttonId-1]=0;//
		memset(str,NULL,size);
		sprintf(str,"\n\r The Botton %d process has been done\n\r",buttonId);
		DEBUG("\n\r*********************\n\r");
		DEBUG(str);	
		DEBUG("\n\r*********************\n\r");

		return 0;
	}
	
	HAL_RTCEx_BKUPWrite(&hrtc, PROCESS_PROGRAM_ID_ADDRESS, buttonId);// store the last button id in rtc backup registers
	WhatToDo = processProg.spouts[eventNumber];//read outputs status of the event that it is time to.
	ApplyAction(WhatToDo); //apply the outputs status
	memset(str,NULL,size); 
	sprintf(str,"\n\r Button %d , event %d ,Total events %d \n\r ACTION APPLIED\n\r ",buttonId, eventNumber+1,processProg.events_count);
	DEBUG(str);
	HAL_RTCEx_BKUPWrite(&hrtc, LAST_STATUS_ADDRESS, WhatToDo); // store the last outputs status in rtc backup registers
	DEBUG("\n\r VALUE SAVED \n\r");
	
	if(eventNumber<processProg.events_count)
	{
		//means we have another events that Alarms must be set for them 
		SetNextAlarm_Processprograms(&hrtc);//set alarm for next event
		HAL_RTCEx_BKUPWrite(&hrtc, LAST_EVENT_NUMBER_ADDRESS, eventNumber);// store the last event number in rtc backup registers
		eventNumber++;
		if(eventNumber==processProg.events_count)
			DEBUG("\n\r This is last event\n\r");//means we set alarm for the last event and we do not have any events to set alarm after this
	}


}
/**
  * @brief  This is a toutin that run a program in its time.
  * @param  hrtc handler
  */
void SetAlarm(RTC_HandleTypeDef* rtc){
	
	//struct tm CurrentTimeToCompare;
	//HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	//HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	//	CurrentTimeToCompare.tm_year		= Date.Year + 2000 - 1900;
	//	CurrentTimeToCompare.tm_mon		= Date.Month;
	//	CurrentTimeToCompare.tm_mday	= Date.Date;
	//	CurrentTimeToCompare.tm_hour		= Time.Hours;
	//	CurrentTimeToCompare.tm_min		= Time.Minutes;
	//	CurrentTimeToCompare.tm_sec		= 0;
	//	CurrentTimeToCompare.tm_isdst	= -1;
	
	struct tm CurrentProgramTime;
	Program 	CurrentProgram;
	uint8_t 	CurrentProgramID = 0;
	uint32_t 	epoch=0;
	uint8_t		WhatToDo;
	char 			ConvertedTimeStamp[30];
	
	// 1- Ejraye hokm:
	CurrentProgramID	= HAL_RTCEx_BKUPRead(&hrtc, PROGRAM_ID_ADDRESS);
	CurrentProgram		= readProg(CurrentProgramID);
	WhatToDo					= CurrentProgram.Action;
	
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	memset(SMStext, NULL, 120);
	snprintf(SMStext, 120, "ProgramID:%d\nTime: %d:%d\nOutput:\n| %d | %d | %d | %d |", CurrentProgramID, Time.Hours, Time.Minutes, ((WhatToDo>>3)&0x01), ((WhatToDo>>2)&0x01), ((WhatToDo>>1)&0x01), ((WhatToDo>>0)&0x01));
	DEBUG(SMStext);
	SMSisPending = 1;
	
	ApplyAction(WhatToDo);
	DEBUG("\n\r ACTION APPLIED \n\r");

	// 2- Save sheers status in backup registers:
	HAL_RTCEx_BKUPWrite(&hrtc, LAST_STATUS_ADDRESS, WhatToDo);
	DEBUG("\n\r VALUE SAVED \n\r");
	
	// 3- Jam kardane period ba zamane fe'li:
	CurrentProgramTime.tm_year	= Date.Year + 2000 - 1900;
	CurrentProgramTime.tm_mon		= CurrentProgram.Month -1;
	CurrentProgramTime.tm_mday	= CurrentProgram.Day;
	CurrentProgramTime.tm_hour	= CurrentProgram.Hour;
	CurrentProgramTime.tm_min		= CurrentProgram.Minutes;
	CurrentProgramTime.tm_sec		= 0;
	CurrentProgramTime.tm_isdst	= -1;
	
	epoch = mktime(&CurrentProgramTime);
	epoch = epoch + CurrentProgram.PeriodMinute*60 + CurrentProgram.PeriodHour*3600 + CurrentProgram.PeriodDay*(24*3600);	
	memset(ConvertedTimeStamp, NULL, 30);
	snprintf(ConvertedTimeStamp, 30, "%s", ctime(&epoch));
	
	CurrentProgram.Day			= (uint8_t)(atoi(&ConvertedTimeStamp[8]));
	CurrentProgram.Hour			= (uint8_t)(atoi(&ConvertedTimeStamp[11]));
	CurrentProgram.Minutes	= (uint8_t)(atoi(&ConvertedTimeStamp[14]));
	
	DEBUG("\n\r SUMATION DONE \n\r");
	
	writeProg(&CurrentProgram);
	DEBUG("\n\r PROGRAM SAVED IN MEMORY \n\r");
	
	// 4- neveshtane barname ye update shode dar hafeze ya pak kardane on agar tekrarshavande nabashad:
	if(CurrentProgram.has_interval==0){					// In khat check mikone bebine barname tekrar shavande hast ya na. Age nabood, hazfesh mikone
		//delete that program
		deleteProg(&CurrentProgram);
		DEBUG("\n\r PROGRAM DELETED \n\r");
	}
	
	SetNextAlarm(rtc);
}
/**
  * @brief  This routin set alarm for next process programs event.
  * @param  hrtc handler
  */
void SetNextAlarm_Processprograms(RTC_HandleTypeDef* rtc){
	 uint32_t 				eventTimeStamp=0;
	 char 						eventTime[30];					//used when convert eventTimeStamp to time
	 uint32_t 				currentTimeStamp = 0;		//used when convert current time to time stamp
	 struct tm  			currentTime;
	 ProcessProgram 	processProg;
	 processProg = readProcessProg(buttonId);//read desired button procees program from eeprom 
	 //HAL_RTCEx_BKUPWrite(rtc, PROGRAM_ID_ADDRESS, processProg.ID);
	 HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	 currentTime.tm_year	= Date.Year + 2000  - 1900;
	 currentTime.tm_mon		= Date.Month;
	 currentTime.tm_mday	= Date.Date;
	 currentTime.tm_hour	= Time.Hours;
	 currentTime.tm_min		= Time.Minutes;
	 currentTime.tm_sec		= Time.Seconds;
	 currentTime.tm_isdst	= -1;	
	 currentTimeStamp = mktime(&currentTime);//convert current time to time stamp
	 HAL_RTCEx_BKUPWrite(&hrtc, LAST_CURRENT_TIME_STAMP, currentTimeStamp);//stroe currentTimeStamp into eeprom
	 if(unfinishedEventFlag==0)//if we do not have any unfinished event
		eventTimeStamp = currentTimeStamp + processProg.Minutes[eventNumber]*60 + processProg.Hour[eventNumber]*3600 + processProg.Second[eventNumber];// Add current time stamp to event duration in sec to make event Time Stamp
	 else if(unfinishedEventFlag==1)//if we  have  unfinished event
	 {
	  eventTimeStamp =  currentTimeStamp +(lastEventTimeStamp-lastTimeStamp); //Add current time stamp to Remaining event time to make new event Time Stamp
		unfinishedEventFlag=0;
	 } 
 	 memset(eventTime, NULL, 30);
	 snprintf(eventTime, 30, "%s", ctime(&eventTimeStamp));//convert eventTimeStamp to time in this format:  Fri Apr  2 20:02:22 2021
	 HAL_RTCEx_BKUPWrite(&hrtc, LAST_EVENT_TIME_STAMP, eventTimeStamp);//stroe eventTimeStamp into eeprom
	 
	//setting next event alarm:
	 RTC_AlarmTypeDef sAlarm = {0}; 
	 sAlarm.AlarmTime.Hours =  (uint8_t)(atoi(&eventTime[11]));
	 sAlarm.AlarmTime.Minutes = (uint8_t)(atoi(&eventTime[14]));
	 sAlarm.AlarmTime.Seconds =(uint8_t)(atoi(&eventTime[17]));
	 sAlarm.AlarmTime.SubSeconds = 0;
	 sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	 sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	 sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	 sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	 sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	 sAlarm.AlarmDateWeekDay = (uint8_t)(atoi(&eventTime[8]));
	 sAlarm.Alarm = RTC_ALARM_A;
	 if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  	 Error_Handler();
	 
	processProgramsAlarmIsSet = 1; //Enable Alarm
	memset(ContentStr,NULL,size);
	snprintf(ContentStr,size,"\n\r * Time: %d/%02d/%02d %02d:%02d:%02d -- ALARM SET FOR Button %d: Day:%d  %02d:%02d:%02d   *\n\r",2000+Date.Year, Date.Month, Date.Date, Time.Hours, Time.Minutes, Time.Seconds,buttonId,sAlarm.AlarmDateWeekDay,  sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes,sAlarm.AlarmTime.Seconds);
	DEBUG(ContentStr);

	}
/**
  * @brief  This routin find the nearst program and set an alarm to run on the time.
  * @param  hrtc handler
  */
void SetNextAlarm(RTC_HandleTypeDef* rtc){
	struct tm ProgramTimeToCompare;
	Program 	program_index;
	uint8_t 	EarlierProgramID = 0;
	uint32_t 	MiniDiffSeconds = 0xFFFFFFFF;
	
	// 4.5- Update kardane barnme ha baraye jologiri az conflict dar barname ha va hang
	updatePrograms();
	
	// 5- Finding the nearest program:
	for(int i=10; i<30; i++)
	{

		program_index = readProg(i);
		ProgramTimeToCompare.tm_year	= Date.Year +2000 - 1900;
		ProgramTimeToCompare.tm_mon	= program_index.Month;
		ProgramTimeToCompare.tm_mday	= program_index.Day;
		ProgramTimeToCompare.tm_hour	= program_index.Hour;
		ProgramTimeToCompare.tm_min	= program_index.Minutes;
		ProgramTimeToCompare.tm_sec	= 0;
		ProgramTimeToCompare.tm_isdst	= -1;
		
		if(program_index.ID==i){
			if( mktime(&ProgramTimeToCompare)<MiniDiffSeconds ){
				MiniDiffSeconds = mktime(&ProgramTimeToCompare);
				EarlierProgramID = program_index.ID;
			}
		}
	}
	
	// 6- Save the nearest program ID in 19th backup registers
	program_index = readProg(EarlierProgramID);
	
	if(program_index.ID == EarlierProgramID)
	{
	
		DEBUG("\n\rNearest is: \n\r");
		PrintProgram(program_index);
		DEBUG("\n\r");
		
		HAL_RTCEx_BKUPWrite(rtc, PROGRAM_ID_ADDRESS, program_index.ID);
		
		// 7- Set alarm:
		 RTC_AlarmTypeDef sAlarm = {0};
		 sAlarm.AlarmTime.Hours = program_index.Hour;
		 sAlarm.AlarmTime.Minutes = program_index.Minutes;
		 sAlarm.AlarmTime.Seconds = 0;
		 sAlarm.AlarmTime.SubSeconds = 0;
		 sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		 sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
		 sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
		 sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
		 sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		 sAlarm.AlarmDateWeekDay = program_index.Day;
		 sAlarm.Alarm = RTC_ALARM_A;
		 if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
		 {
		   Error_Handler();
		 }
		AlarmIsSet = 1;
		 
		memset(ContentStr,NULL,size);
		snprintf(ContentStr,size,"\n\r * ALARM SET FOR: %d %d:%d:00 *\n\r", program_index.Day, program_index.Hour, program_index.Minutes);
		DEBUG(ContentStr);
	 }
	else{
		/*
			THERE IS NO SCHEDULE:
				Disable alarm.
		*/
		AlarmIsSet = 0;
		DEBUG("\n\r <no schedule found>");
	}
	
}
/**
  * @brief  Itterate on all programs, then finds the programs in the past, deletes the unrepeatable ones and moves repeatable programss to the feuture.
  */
void updatePrograms(void){
	struct tm	ProgramTimeToCompare;
	Program		program_index;
	char 			ConvertedTimeStamp[30];
	uint32_t 	epoch=0;
	uint8_t 	flag=0;
	uint8_t 	changed=0;

	struct tm CurrentTimeToCompare;
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
		CurrentTimeToCompare.tm_year	= Date.Year + 2000  - 1900;
		CurrentTimeToCompare.tm_mon		= Date.Month;
		CurrentTimeToCompare.tm_mday	= Date.Date;
		CurrentTimeToCompare.tm_hour	= Time.Hours;
		CurrentTimeToCompare.tm_min		= Time.Minutes;
		CurrentTimeToCompare.tm_sec		= Time.Seconds;
		CurrentTimeToCompare.tm_isdst	= -1;	
		uint32_t CurrentTime = 0;
		CurrentTime = mktime(&CurrentTimeToCompare);
	
	for(int i=10; i<30; i++){

		program_index = readProg(i);
		ProgramTimeToCompare.tm_year	= Date.Year + 2000 - 1900;
		ProgramTimeToCompare.tm_mon		= program_index.Month;
		ProgramTimeToCompare.tm_mday	= program_index.Day;
		ProgramTimeToCompare.tm_hour	= program_index.Hour;
		ProgramTimeToCompare.tm_min		= program_index.Minutes;
		ProgramTimeToCompare.tm_sec		= 0;
		ProgramTimeToCompare.tm_isdst	= -1;
		
		if(program_index.ID==i){
			changed = 0;
			while( CurrentTime > mktime(&ProgramTimeToCompare) ){    // yani ta vaghti in barname male gozashte ast edame bede
				
				if( program_index.has_interval == 0 ){
					deleteProg(&program_index);
					flag = 1;
					break;
				}
				
				epoch = mktime(&ProgramTimeToCompare);
				epoch = epoch + program_index.PeriodMinute*60 + program_index.PeriodHour*3600 + program_index.PeriodDay*(24*3600);	
				memset(ConvertedTimeStamp, NULL, 30);
				snprintf(ConvertedTimeStamp, 30, "%s", ctime(&epoch));
				
				program_index.Day			= (uint8_t)(atoi(&ConvertedTimeStamp[8]));
				program_index.Hour		= (uint8_t)(atoi(&ConvertedTimeStamp[11]));
				program_index.Minutes	= (uint8_t)(atoi(&ConvertedTimeStamp[14]));
				
				changed = 1;
				
				ProgramTimeToCompare.tm_year	= Date.Year + 2000 - 1900;
				ProgramTimeToCompare.tm_mon		= program_index.Month;
				ProgramTimeToCompare.tm_mday	= program_index.Day;
				ProgramTimeToCompare.tm_hour	= program_index.Hour;
				ProgramTimeToCompare.tm_min		= program_index.Minutes;
				ProgramTimeToCompare.tm_sec		= 0;
				ProgramTimeToCompare.tm_isdst	= -1;

			}
			
			if(flag==0){
				if(changed==1){
					writeProg(&program_index);
					DEBUG("\n\rUpdated program is:\n\r");
					PrintProgram(program_index);
				}
			}
			else{
				DEBUG("\n\r A NONE REPEATABLE PROGRAM FROM PAST DELETED\n\r");
				flag = 0;
			}
		}
	}
}

/**
  * @brief  Print a program in debug port
  * @param  input is the program you want to be printed
  */
void PrintProgram(Program input){
	memset(str,NULL,size);
	snprintf(str,sizeof(str)," -- Program ID: %d\n\r -- Program Month: %d\n\r -- Program Day: %d\n\r -- Program Hour: %d\n\r -- Program Minutes: %d\n\r -- Program Period Day: %d\n\r -- Program Period Hour: %d\n\r -- Program Period Minutes: %d\n\r -- Program Action: %d\n\r -- Program has_interval: %d\n\r",
		input.ID, input.Month, input.Day, input.Hour, input.Minutes, input.PeriodDay, input.PeriodHour, input.PeriodMinute, input.Action, input.has_interval);
	DEBUG(str);
}
/**
	* @brief  Converts a string like "1101" to its binarry equevalent like: 0b1101
  * @param  input is the string you want to convert to binarry
	* @param  uint8_t
  */
uint8_t str2bin(char* input){
	uint8_t string2bin=0x00;
	for(int i=0; i<strlen(input); i++){
		string2bin = string2bin * 2;
		if(input[i]=='1')
			string2bin = string2bin + 1;
	}
	return string2bin;
}
/**
  * @brief  Delete all programs stord in EEPROM
  */
void deleteAllPrograms(void){
	uint8_t values[20*10];
	memset(values, 255, 20*10);
	ee24_write(&hi2c1, 100, values, 20*10, 1000);
}
/**
  * @brief  Prints all valid programs stored in EEPROM.
  */
void PrintAllPrograms(void){
	Program programToPrint;
	char printedStr[30];
	
	HAL_Delay(500);
	for(int i=10; i<30; i++){
		programToPrint = readProg(i);
		if(programToPrint.ID == i){
			memset(printedStr, NULL, 30);
			snprintf(printedStr, 30, "\n\rProgram number %d:\n\r", i);
			DEBUG(printedStr);
			PrintProgram(programToPrint);
		}
	}
}
/**
  * @brief  Prints all valid Process programs stored in EEPROM.
  */
void PrintAllProcessProgram(int i){
	DEBUG("\n\r**************\n\r");
	DEBUG("\n\r debug read process program from eeprom: \n\r");
	ProcessProgram 		pprogram;
	pprogram=readProcessProg(i);//read Process programs stored in EEPROM.
	memset(str,NULL,size);
	sprintf(str, "\n\r ID is \"%d\"", pprogram.ID);
	DEBUG(str);
	memset(str,NULL,size);
	sprintf(str,"\n\r events_count is \"%d\"", pprogram.events_count);
	DEBUG(str);
	DEBUG("\n\r-------\n\r");
	for(int i=0;i<pprogram.events_count;i++)
	{
		memset(str,NULL,size);
		sprintf(str, "\n\r   event:   %d ",i);
		DEBUG(str);		
		memset(str,NULL,size);
		sprintf(str, "\n\r   Time duration  is :   %02d:%02d:%02d ",pprogram.Hour[i], pprogram.Minutes[i],pprogram.Second[i]);
		DEBUG(str);
		memset(str,NULL,size);
		sprintf(str, "\n\r   spouts  is :   %d ", pprogram.spouts[i]);
		DEBUG(str);
		DEBUG("\n\r-------\n\r");
	}	

	DEBUG("\n\r**************\n\r");
}
/*
void Hal_Delay(uint32_t interval){
	uint16_t times=0;
	uint16_t residue=0;
	
	times	= (interval * 1000) / 0xffff;
	residue	= (interval * 1000) % 0xffff;
	
	TIM2->CNT=0;
	HAL_TIM_Base_Start(&htim2);
	while(TIM2->CNT<residue*1000);
	HAL_TIM_Base_Stop(&htim2);
	
	for(uint16_t i=0; i< times; i++){
		TIM2->CNT=0;
		HAL_TIM_Base_Start(&htim2);
		while(TIM2->CNT<0xffff*1000);
		HAL_TIM_Base_Stop(&htim2);
	}
}
*/

uint8_t JSON2Str(char* result, char* raw_input, char* key){
	char value_str[60];
	cJSON *server_response = cJSON_Parse(raw_input);		
	const cJSON *value	= NULL; //string
	value = cJSON_GetObjectItemCaseSensitive(server_response, key);
	if (cJSON_IsString(value) && (value->valuestring != NULL))
	{
		//sprintf(value_str, "\n\r  Extracted value is \"%s\"", value->valuestring);
		//DEBUG(value_str);
		sprintf(result, "%s", value->valuestring);
		cJSON_Delete(server_response);
		return 1;
	}
	cJSON_Delete(server_response);
	return 0;
}

uint8_t JSON2Str_nested(char* result, char* raw_input, char* key_parent, char* key_child){
	char value_str[60];
	cJSON *server_response = cJSON_Parse(raw_input);		
	const cJSON *value_p	= NULL;
	const cJSON *value_c	= NULL;
	value_p = cJSON_GetObjectItemCaseSensitive(server_response, key_parent);
	value_c = cJSON_GetObjectItemCaseSensitive(value_p, key_child);
	if (cJSON_IsString(value_c) && (value_c->valuestring != NULL))
	{
		//sprintf(value_str, "\n\r  Extracted value is \"%s\"", value_c->valuestring);
		//DEBUG(value_str);
		sprintf(result, "%s", value_c->valuestring);
		cJSON_Delete(server_response);
		return 1;
	}
	cJSON_Delete(server_response);
	return 0;
}

uint8_t JSON2int(char* result, char* raw_input, char* key){
	char value_str[60];
	cJSON *server_response = cJSON_Parse(raw_input);		
	const cJSON *value	= NULL; //int
	value = cJSON_GetObjectItemCaseSensitive(server_response, key);
	//if (cJSON_IsNumber(value) && (value->valueint != NULL))
	{
		//sprintf(value_str, "\n\r  Extracted value is \"%d\"", value->valueint);
		//DEBUG(value_str);
		sprintf(result, "%d", value->valueint);
		cJSON_Delete(server_response);
		return 1;
	}
	cJSON_Delete(server_response);
	return 0;
}

/* USER CODE END 4 */

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
	if(htim->Instance==TIM5)
	{
		
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
		if(initializingFlag)
	 {
			tim5CallbackCounter++;
			if(2<tim5CallbackCounter&&tim5CallbackCounter<10)
			 {
				 ssd1306_draw_bitmap(20, 25, ldm, 104, 40);//ldm logo
			 }
			 else
				ssd1306_clear_screen(0,128,20,64);	//clear logo on logo
			 
			if(tim5CallbackCounter>10)
			{
				ssd1306_SetCursor(2,30);
				ssd1306_WriteString("Initializing", Font_7x10, White);
				switch(dotPointCounter) 
				{
					case 0:
						ssd1306_DrawPixel(93,38, White);
				  	dotPointCounter++;
					break;
					case 1:
						ssd1306_DrawPixel(93,38, White);
					  ssd1306_DrawPixel(98,38, White);
				  	dotPointCounter++;
					break;
					case 2:
						ssd1306_DrawPixel(93,38, White);
						ssd1306_DrawPixel(98,38, White);
				    ssd1306_DrawPixel(103,38, White);
				  	dotPointCounter++;
					break;
					case 3:
						dotPointCounter=0;
						ssd1306_DrawPixel(96,38,  Black);
						ssd1306_DrawPixel(98,38, Black);
				    ssd1306_DrawPixel(103,38, Black);
				}
			}
	 }
	 else if(tim5CallbackCounter>20)
	 {
			initializingDoneFlag=1;
		 	tim5CallbackCounter=0;

	 }
	 if(initializingDoneFlag&&tim5CallbackCounter<8)
	 {
		 ssd1306_clear_screen(0,128,20,64);//clear main section of screen
		 ssd1306_SetCursor(2,30);
		 ssd1306_WriteString("Initializing Done", Font_7x10, White);
		 ssd1306_draw_bitmap(45, 42, checkRight, 16, 21);
		 tim5CallbackCounter++;
		 if(tim5CallbackCounter==4)
		 {
			 	ssd1306_clear_screen(0,128,20,64);//clear main section of screen after printing "Initializing Done"
			  tim5CallbackCounter=0;
			  initializingDoneFlag=0;
		 }
	 }

	
	  	
		

	 
//		memset(oledStr,NULL, size);
//		snprintf(oledStr,sizeof(oledStr)," %d/%02d/%02d ", 2000+Date.Year, Date.Month, Date.Date);
//		ssd1306_SetCursor(50, 0);
//    ssd1306_WriteString(oledStr, Font_7x10, White);
		
		memset(oledStr,NULL, size);
	  snprintf(oledStr,sizeof(oledStr)," %02d:%02d:%02d",  Time.Hours, Time.Minutes, Time.Seconds);
		ssd1306_SetCursor(60, 3);
		ssd1306_WriteString(oledStr, Font_7x10, White);//show time on oled
		ssd1306_SetCursor(0,0);
		ssd1306_draw_bitmap(1, 2, rssiSingal_5, 16, 11);//anten
		ssd1306_draw_bitmap(25, 0, noSignal, 16, 15);//no anten
		ssd1306_draw_bitmap(1, 17, line, 128, 2);//line
	//	ssd1306_draw_bitmap(80, 35, tapOn , 20, 30);//tapOn
	//	ssd1306_draw_bitmap(1, 20, ldm, 104, 40);//ldm logo
	//	ssd1306_draw_bitmap(100, 35, tapOff , 20, 30);//tapOff
		ssd1306_UpdateScreen();
	
		
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
	DEBUG("\n\rERROR HANDLER\n\r");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
