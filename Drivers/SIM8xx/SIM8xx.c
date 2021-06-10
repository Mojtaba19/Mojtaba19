#include "SIM8xx.h"

extern TIM_HandleTypeDef htim5;

void sim80x_PWR(uint8_t state){
	if(state == ON)
	{
		if(HAL_GPIO_ReadPin(_STATUS_PORT,_STATUS_PIN) == 0)
		{
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_RESET);
			HAL_Delay(1200);
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_SET);
			HAL_Delay(3000);
			HAL_UART_Transmit(&debugUART,(uint8_t *)"*************************************SIM POWER IS ON******************************************\r\n",strlen("*************************************SIM POWER IS ON******************************************\r\n"),100);
			HAL_Delay(7000);
		}
	}
	
	if(state == OFF)
	{
		if(HAL_GPIO_ReadPin(_STATUS_PORT,_STATUS_PIN) == 1)
		{
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_RESET);
			HAL_Delay(1200);
			HAL_GPIO_WritePin(_POWER_PORT,_POWER_PIN,GPIO_PIN_SET);
			HAL_Delay(3000);
			HAL_UART_Transmit(&debugUART,(uint8_t *)"*************************************SIM POWER IS OFF******************************************\r\n",strlen("*************************************SIM POWER IS OFF******************************************\r\n"),100);
		}
	}
}
/**
  * @brief  Sending commands to SIM800
	* @param  ATCommand - e.g: "AT"
  * @param  Timeout in milliseconds
  * @retval HAL status
	* @result	SIM800 reponse stored in RxBuffer
  */
HAL_StatusTypeDef sim80x_ATC(char * ATCommand , uint32_t Timeout){
	memset(RxBuffer,NULL,size);	
	
	HAL_UART_Transmit(&simUART,(uint8_t *) ATCommand,strlen(ATCommand),Timeout);
	HAL_UART_Receive(&simUART,(uint8_t *)RxBuffer,size,Timeout);

	HAL_UART_Transmit(&debugUART,(uint8_t *) "***\r\n",strlen("***\r\n"),Timeout);
	HAL_UART_Transmit(&debugUART,(uint8_t *) RxBuffer,strlen(RxBuffer),Timeout);
	HAL_UART_Transmit(&debugUART,(uint8_t *) "***\r\n",strlen("***\r\n"),Timeout);
	
	if(HAL_UART_STATE_ERROR != HAL_UART_GetState(&simUART) && RxBuffer[1] != NULL )
	{
		if( strstr( (char *) RxBuffer, "OK" )!= NULL)				
		{
			ErrorCounter=0;
			return HAL_OK;
		}
		if( strstr( (char *) RxBuffer, "ERROR" ) != NULL)		
		{
			ErrorCounter++;
			return HAL_ERROR;
		}
	}
	else 
	{
		return HAL_TIMEOUT;
	}
}
/**
  * @brief  Checks the SIM800 state.
  * @param  Timeout in milliseconds.
  * @retval HAL status
  */
HAL_StatusTypeDef sim80x_ACK(uint32_t Timeout){	
	Sim80x_StatusTypeDef = sim80x_ATC("AT\r\n",Timeout);
	return Sim80x_StatusTypeDef;
}
/**
  * @brief  Sends SMS to a specific phone number
  * @param  phoneNumber is a string containing the phone number e.g "+98912xxxxxxx"
	* @param  msg is the message string e.g "Hello :)"
  * @param  Timeout in milliseconds
  * @retval HAL status
  */
/*
HAL_StatusTypeDef sim80x_SendSMS(char * phoneNumber , char * msg,uint32_t Timeout){
	Sim80x_StatusTypeDef = sim80x_ATC("AT+CMGF=1\r\n",Timeout);
	if (Sim80x_StatusTypeDef == HAL_OK)
	{
		
		Sim80x_StatusTypeDef = sim80x_ATC("AT+CSMP=17,167,0,0\r\n",Timeout);
		
		memset(str,NULL,size);
		snprintf(str,sizeof(str),"AT+CMGS=\"%s\"\r\n",phoneNumber);
		Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
		if (Sim80x_StatusTypeDef == HAL_OK)
		{
			memset(str,NULL,size);
			//snprintf(str,sizeof(str),"%s%c\n\r",msg,(char)26);
			snprintf(str,sizeof(str),"%s",msg);
			Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
			
			memset(str,NULL,size);
			//snprintf(str,sizeof(str),"%s%c\n\r",msg,(char)26);
			snprintf(str,sizeof(str),"%c",26);
			Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
					
			return Sim80x_StatusTypeDef;
		}
		else 	
		{
			return Sim80x_StatusTypeDef;
		}
	}
	else	
	{
		return Sim80x_StatusTypeDef;
	}
}
*/

/**
  * @brief  Sends SMS to a specific phone number
  * @param  phoneNumber is a string containing the phone number e.g "+98912xxxxxxx"
	* @param  msg is the message string e.g "Hello :)"
  * @param  Timeout in milliseconds
  * @retval HAL status
  */
HAL_StatusTypeDef sim80x_SendSMS(char * phoneNumber , char * msg, uint32_t Timeout){
	memset(str,NULL,size);
	snprintf(str,size,"AT+CMGS=\"%s\"\r\n",phoneNumber);
	Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
	if (Sim80x_StatusTypeDef == HAL_OK)
	{
		memset(str,NULL,size);
		snprintf(str,size,"%s",msg);
		Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
		
		memset(str,NULL,size);
		snprintf(str,size,"%c",26);
		Sim80x_StatusTypeDef = sim80x_ATC(str,Timeout);
				
		return Sim80x_StatusTypeDef;
	}
	else       
	{
		return Sim80x_StatusTypeDef;
	}
}

/**
  * @brief  Starts an HTTP protocol
  * @retval HAL status
  */
HAL_StatusTypeDef sim80x_HTTP_Start(void){
	if (sim80x_ATC("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n",1000)==HAL_OK)
		simCardGprsOk=1;
	else
		simCardGprsOk=0;
	sim80x_ATC("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n",1000);
	sim80x_ATC("AT+SAPBR=1,1\r\n",3000);
	sim80x_ATC("AT+SAPBR=2,1\r\n",300);
	sim80x_ATC("AT+HTTPINIT\r\n",500);
	sim80x_ATC("AT+HTTPPARA=\"CID\",1\r\n",500);
	return Sim80x_StatusTypeDef;
}

/**
  * @brief  Ends the HTTP connection
  * @retval HAL status
  */
HAL_StatusTypeDef	sim80x_HTTP_Stop(void){
	sim80x_ATC("AT+HTTPTERM\r\n",200);
	sim80x_ATC("AT+SAPBR=0,1\r\n",1000);
	return Sim80x_StatusTypeDef;
}

/**
  * @brief  Sending a POST request
  * @param  postResult is a string containing the server respone of this request
	* @param  IP is a string containing the server IP e.g "192.168.1.1"
  * @param  URL is the api string e.g "api/panel/get_clock/"
	* @param  Content is the JSON that you want to post in server e.g "{"id":5,"value":45}" 
  */
void sim80x_HTTP_Post(char* postResult, char * IP, char * URL , char * Content){
		//sim80x_HTTP_Start();
		
		memset(str,NULL,size);
		snprintf(str,size,"AT+HTTPPARA=\"URL\",\"%s/%s\"\r\n",IP, URL);  //api/multiSensor/
		sim80x_ATC(str,5000);
		
		sim80x_ATC("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n",2000);
		sim80x_ATC("AT+HTTPDATA=200,10000\r\n",2000);
		
		sim80x_ATC(Content,200);
		HAL_Delay(12000);
		sim80x_ATC("AT+HTTPACTION=1\r\n",5000);
		sim80x_ATC("AT+HTTPREAD\r\n",10000);
		
		memset(postResult, NULL, size);
	  memcpy(postResult, RxBuffer, size);
		
		//sim80x_HTTP_Stop();
	}

/**
  * @brief  ?
  */
void SIM800_handler(void){
		
	// unplug simcard handler
	if(ErrorCounter > 10 )
	{
		ErrorCounter=0;
		sim80x_PWR(OFF);
		sim80x_PWR(ON);
		action=0;
	}
	
	// unplug sim800 handler
	if(HAL_GPIO_ReadPin(SIM_STATUS_GPIO_Port,SIM_STATUS_Pin) == 0)
	{
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port,SIM_PWR_Pin,GPIO_PIN_RESET);
		HAL_Delay(1200);
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port,SIM_PWR_Pin,GPIO_PIN_SET);
		HAL_Delay(3000);
		HAL_UART_Transmit(&debugUART,(uint8_t *)"*************************************SIM POWER IS ON******************************************\r\n",strlen("*************************************SIM POWER IS ON******************************************\r\n"),100);
		HAL_Delay(7000);
	}
}

/**
  * @brief  This function makes sure that SIM800 is ready
	* @retval Returns 1 in case of success, returns 0 in case of failure.
  */
uint8_t ACKHandler(void){
	uint8_t ACKCounter=0;
	uint8_t CycleCounter=0;
	for(CycleCounter=0; CycleCounter<3; CycleCounter++){
		sim80x_PWR(ON);
		for(ACKCounter=0;ACKCounter<10;ACKCounter++){
			sim80x_ACK(500);
			if(Sim80x_StatusTypeDef == HAL_OK)
				return 1;
		}
		sim80x_PWR(OFF);
		sim80x_PWR(ON);
	}
	return 0;
}


/**
  * @brief  Set the SMS settings
  */
void SMSSetting(void){
	sim80x_ATC("AT+CMGF=1\r\n",6000);
	sim80x_ATC("AT+CSMP=17,167,0,0\r\n",4000);
}
/**
  * @brief  Sending the Status to server
*   @param  IP is a string that defined : "ldmpanel.ir" in main code
  */

void sim80x_Send_Status(char*IP){
	
	char *startAnswer; 
	char *endAnswer;
	char sendStatus[100];// a JSON including output status RSSI value and external bottons status that post to server
	HAL_TIM_Base_Stop_IT(&htim5);
	memset(sendStatus,NULL,100);
	memset(str,NULL,30);
	memset(rssiStrValue,NULL,5);
	memset(ContentStr, NULL,size);
	memset(outputsStatus,NULL,size);
	outputsStatus[0]='0';
	outputsStatus[1]='0';
	outputsStatus[2]='0';
	outputsStatus[3]='0';
	outputsStatus[4]='0';
	outputsStatus[5]='0';
	
	//Reading outputs status :
	if(HAL_GPIO_ReadPin(relay1_GPIO_Port, relay1_Pin))
		outputsStatus[0]='1';
	if(HAL_GPIO_ReadPin(relay2_GPIO_Port, relay2_Pin))
		outputsStatus[1]='1';
	if(HAL_GPIO_ReadPin(relay3_GPIO_Port, relay3_Pin))
		outputsStatus[2]='1';		
	if(HAL_GPIO_ReadPin(relay4_GPIO_Port, relay4_Pin))
		outputsStatus[3]='1';
	
	Sim80x_StatusTypeDef=sim80x_ATC("AT+CSQ\r\n",50);// SIM800 RSSI AT Command
	if(Sim80x_StatusTypeDef==HAL_OK)
		{
		//parsing sim800 response:	
		startAnswer	= strstr(RxBuffer, "+CSQ:")+5;
		endAnswer		= strstr(RxBuffer, ",");
		for(int i=(startAnswer-RxBuffer); i<(endAnswer-RxBuffer); i++)
			rssiStrValue[i-(startAnswer-RxBuffer)] = RxBuffer[i];	//ContentStr=RSSI value of sim800
		}
		if(simCardGprsOk==0)// if gprs is not enable then rssi value =0
		{
			memset(rssiStrValue,NULL,5);
			memcpy(rssiStrValue,"00",5);
		}
		
		snprintf(sendStatus,100,"{\"sc\":\"%s\",\"nc\":\"%s\",\"ebc\": \"%d%d\"}\r\n",outputsStatus,rssiStrValue,buttonsStatus[1],buttonsStatus[0]); 			// Creating pure JSON from SIM800 response. sc: output status , nc:RSSI value , ebc: external bottons status
																																																													// ContentStr is the JSON contain RSSI value , OUTPUT Status and external Buttons Status that  post in server								
	//sending status to server:
	 sim80x_HTTP_Post(str,IP,"panel/api/d/1/2/eo5/",sendStatus);
	 DEBUG( "***\r\n");
	 DEBUG("outputs_rssi_buttonsStatus: ");
	 DEBUG(sendStatus);
	 DEBUG( "***\r\n");	
		HAL_TIM_Base_Start_IT(&htim5);
		
		
}
