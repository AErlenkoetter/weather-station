/**************************************************************************//**
  \file app.c

  \brief Anwendung SPS30 mit Funknetzwerk

  \author Ana Erlenkötter, Code-Skeleton provided by Prof. Markus Krauße 

******************************************************************************/


#include <appTimer.h>
#include <zdo.h>
#include <app.h>
#include <sysTaskManager.h>
#include <usartManager.h>
#include <usart.h>
#include <i2cPacket.h>
#include <bspLeds.h>

// uncomment for wished for output behaviour
//#define DEBUG_UART 
#define SERIAL_RASP 

//timer des SPS30 und appstate intialisierung
static AppState_t appstate = APP_INIT_STATE;
static HAL_AppTimer_t sps30LoopTimer;
static HAL_AppTimer_t sps30WaitTimer;

//i2c commands
uint8_t sps30Data[SPS30_DATA_SIZE];
uint8_t sps30Req[] = {SPS30_START_MES};
uint8_t sps30mValues[] = {SPS30_READ_MES};
uint8_t sps30Stop[] = {SPS30_STOP_MES};
//callback functions
static void readSps30Done(bool success);
static void writeSps30Done(bool success);
static void reqMeasureSps30Done(bool success);
static void reqStopMeasureSps30Done(bool success);

//functions to fill the i2c-descriptor variables
static void fillWriteDescriptor(void);
static void fillReadDescriptor(void);

//saving the values to an array for further transport
uint8_t address = 0;

static void initSps30LoopTimer();
static void initSps30WaitTimer();
static void sps30requestMeasurement();
static void sps30waitDone();

//funk
static AppMessage_t transmitData;
static uint8_t deviceType;
static void ZDO_StartNetworkConf(ZDO_StartNetworkConf_t *confirmInfo);
static ZDO_StartNetworkReq_t networkParams;

APS_DataReq_t dataReq;
static void APS_DataConf(APS_DataConf_t *confInfo);
static void initTransmitData(void);

static SimpleDescriptor_t simpleDescriptor;
static APS_RegisterEndpointReq_t endPoint;
static void initEndpoint(void);
void APS_DataInd(APS_DataInd_t *indData);

HAL_AppTimer_t receiveTimerLed;
HAL_AppTimer_t transmitTimerLed;
HAL_AppTimer_t transmitTimer;
static void receiveTimerLedFired(void);
static void transmitTimerLedFired(void);
static void transmitTimerFired(void);
static void initLEDTimer(void);

//pausetimer transmit
static AppState_t nextState; 
static void initPauseTimer();
static void pausingTimer(AppState_t);
static void pauseTimerDone();
HAL_AppTimer_t pauseTimer;


static HAL_I2cDescriptor_t sps30readDescriptor={ 
	.tty = TWI_CHANNEL_0,
	.clockRate = I2C_CLOCK_RATE_62,
	.id = SPS30_DEVICE_ADDRESS,
	.lengthAddr = HAL_NO_INTERNAL_ADDRESS
};static HAL_I2cDescriptor_t sps30writeDescriptor={ 
	.tty = TWI_CHANNEL_0,
	.clockRate = I2C_CLOCK_RATE_62,
	.id = SPS30_DEVICE_ADDRESS,
	.lengthAddr = HAL_NO_INTERNAL_ADDRESS
};
void APL_TaskHandler(void){
	switch(appstate){ 
		case APP_INIT_STATE:
			appInitUsartManager();
			appWriteDataToUsart((uint8_t*)"INIT\r\n", sizeof("INIT\r\n")-1);
			initLEDTimer();
			initPauseTimer();
			BSP_OpenLeds();
			appstate=APP_STARTJOIN_NETWORK;
			SYS_PostTask(APL_TASK_ID);
			break;
		case APP_STARTJOIN_NETWORK:
			appWriteDataToUsart((uint8_t*)"start\r\n", sizeof("start\r\n")-1);
			networkParams.ZDO_StartNetworkConf = ZDO_StartNetworkConf;
			ZDO_StartNetworkReq(&networkParams);
			appstate=APP_INIT_ENDPOINT;
			SYS_PostTask(APL_TASK_ID);
			break;
		case APP_INIT_ENDPOINT:
			appWriteDataToUsart((uint8_t*)"endpoint\r\n", sizeof("endpoint\r\n")-1);
			initEndpoint();
			appstate=APP_INIT_TRANSMITDATA;
			SYS_PostTask(APL_TASK_ID);
			break;
		case APP_INIT_TRANSMITDATA:
			appWriteDataToUsart((uint8_t*)"transmit data\r\n", sizeof("transmit data\r\n")-1);
			initTransmitData();
			appstate=SPS30_INIT_STATE;
			HAL_StartAppTimer(&transmitTimer);
			SYS_PostTask(APL_TASK_ID);
			break;
			
		case SPS30_INIT_STATE:
			appWriteDataToUsart((uint8_t*)"INIT sps30\r\n", sizeof("INIT sps30\r\n")-1);
			initSps30LoopTimer();
			initSps30WaitTimer();
			break;
			
		case SPS30_REQ_MES_STATE:
			appWriteDataToUsart((uint8_t*)"REQ_MES\r\n", sizeof("REQ_MES\r\n")-1);
			fillWriteDescriptor();							
			if (-1 == HAL_OpenI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"wropen fail\r\n", sizeof("wropen fail\r\n")-1);
			if (-1 == HAL_WriteI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"write fail\r\n", sizeof("write fail\r\n")-1);
			break;
			
		case SPS30_STOP_MES_STATE:
			appWriteDataToUsart((uint8_t*)"STOP_MES\r\n", sizeof("STOP_MES\r\n")-1);
			fillWriteDescriptor();			
			if (-1 == HAL_OpenI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"stop open fail\r\n", sizeof("read open fail\r\n")-1);
			if (-1 == HAL_WriteI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"stop fail\r\n", sizeof("read fail\r\n")-1);
			break;
						
		case SPS30_WRITE_MES_STATE:
			appWriteDataToUsart((uint8_t*)"WRITE_MES\r\n", sizeof("WRITE_MES\r\n")-1);
			fillWriteDescriptor();			
			if (-1 == HAL_OpenI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"write open fail\r\n", sizeof("write open fail\r\n")-1);
			if (-1 == HAL_WriteI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"write fail\r\n", sizeof("write fail\r\n")-1);
			break;
			
		case SPS30_READ_MES_STATE:
			appWriteDataToUsart((uint8_t*)"READ_MES\r\n", sizeof("READ_MES\r\n")-1);
			fillReadDescriptor();			
			if (-1 == HAL_OpenI2cPacket(&sps30readDescriptor)) appWriteDataToUsart((uint8_t*)"data read open fail\r\n", sizeof("data read open fail\r\n")-1);
			if (-1 == HAL_ReadI2cPacket(&sps30readDescriptor)) appWriteDataToUsart((uint8_t*)"data read fail\r\n", sizeof("data read fail\r\n")-1);
			break;
			
		case APP_TRANSMIT_1:
#ifdef DEBUG_UART

			appWriteDataToUsart((uint8_t*)"UART\r\n0x", sizeof("UART\r\n0x")-1);
			for(int i=0; i<SPS30_DATA_SIZE; i++){
				uint8_t temp[]="   ";
				uint8_to_hexstr(temp, sizeof(temp), sps30Data[i], 0);
				appWriteDataToUsart(temp, sizeof(temp));
			}
			appWriteDataToUsart((uint8_t*)"\r\n", sizeof("\r\n")-1);
						
			printArray(3);
			printArray(9);
			printArray(18);
			printArray(24);			
			
#endif // DEBUG_UART
#ifdef SERIAL_RASP
;
			transmitData.data[0] = ZIGBEE_MODEL_ID;
			transmitData.data[1] = PM25_MASS_CON;
			transmitData.data[2] = sps30Data[3];
			transmitData.data[3] = sps30Data[4];
			transmitData.data[4] = sps30Data[5];
			transmitData.data[5] = 0;
			APS_DataReq(&dataReq);
			pausingTimer(APP_TRANSMIT_2);
			SYS_PostTask(APL_TASK_ID);
			break;
		
		case APP_TRANSMIT_2:
			transmitData.data[0] = ZIGBEE_MODEL_ID;
			transmitData.data[1] = PM10_MASS_CON;
			transmitData.data[2] = sps30Data[9];
			transmitData.data[3] = sps30Data[10];
			transmitData.data[4] = sps30Data[11];
			transmitData.data[5] = 0;
			APS_DataReq(&dataReq);
			pausingTimer(APP_TRANSMIT_3);
			SYS_PostTask(APL_TASK_ID);
			break;
		
		case APP_TRANSMIT_3:
			transmitData.data[0] = ZIGBEE_MODEL_ID;
			transmitData.data[1] = PM25_NUM_CON;
			transmitData.data[2] = sps30Data[18];
			transmitData.data[3] = sps30Data[19];
			transmitData.data[4] = sps30Data[20];
			transmitData.data[5] = 0;
			APS_DataReq(&dataReq);			
			pausingTimer(APP_TRANSMIT_4);			
			SYS_PostTask(APL_TASK_ID);
			break;
		
		case APP_TRANSMIT_4:
			transmitData.data[0] = ZIGBEE_MODEL_ID;
			transmitData.data[1] = PM10_NUM_CON;
			transmitData.data[2] = sps30Data[24];
			transmitData.data[3] = sps30Data[25];
			transmitData.data[4] = sps30Data[26];
			transmitData.data[5] = 0;
			APS_DataReq(&dataReq);			
			pausingTimer(APP_NOTHING_STATE);
			SYS_PostTask(APL_TASK_ID);
			break;
#endif // SERIAL_RASP	
			
		case APP_NOTHING_STATE:
			break;
	}
}

static void initSps30LoopTimer(){
	sps30LoopTimer.interval = SPS30_REQUEST_LOOP;      // Timer interval
	sps30LoopTimer.mode     = TIMER_REPEAT_MODE;       // Timer-Mode
	sps30LoopTimer.callback = sps30requestMeasurement;         // Callback function
	HAL_StartAppTimer(&sps30LoopTimer);                // Start readTimer
}

static void initSps30WaitTimer(){
	sps30WaitTimer.interval = SPS30_WAIT_MES;		// Timer interval
	sps30WaitTimer.mode     = TIMER_ONE_SHOT_MODE;       // Timer-Mode
	sps30WaitTimer.callback = sps30waitDone;		// Callback function
}

static void fillWriteDescriptor(){
	if (appstate == SPS30_REQ_MES_STATE){
		sps30writeDescriptor.data = sps30Req;
		sps30writeDescriptor.f = writeSps30Done;
		sps30writeDescriptor.length = 5;
	} else if(appstate == SPS30_WRITE_MES_STATE) {
		sps30writeDescriptor.data = sps30mValues;
		sps30writeDescriptor.f = reqMeasureSps30Done;
		sps30writeDescriptor.length = 2;
	} else if(appstate == SPS30_STOP_MES_STATE){
		sps30writeDescriptor.data = sps30Stop;
		sps30writeDescriptor.f = reqStopMeasureSps30Done;
		sps30writeDescriptor.length = 2;
	}
}

static void fillReadDescriptor() {
	if (appstate == SPS30_READ_MES_STATE){
		sps30readDescriptor.data = sps30Data;
		sps30readDescriptor.f = readSps30Done;
		sps30readDescriptor.length = SPS30_DATA_SIZE;
	}
}

static void sps30requestMeasurement(){
	appstate = SPS30_REQ_MES_STATE;
	SYS_PostTask(APL_TASK_ID);
}

static void sps30waitDone(){
	appstate = SPS30_WRITE_MES_STATE;
	SYS_PostTask(APL_TASK_ID);
}

static void readSps30Done(bool success){
	if(!success) appWriteDataToUsart((uint8_t*)"read callback fail\r\n", sizeof("read callback fail\r\n")-1);
	if(success){
		appstate = APP_TRANSMIT_1;
		appWriteDataToUsart((uint8_t*)"read callback success\r\n", sizeof("read callback success\r\n")-1);
	}
	if (-1 == HAL_CloseI2cPacket(&sps30readDescriptor)) appWriteDataToUsart((uint8_t*)"reclose fail\r\n", sizeof("reclose fail\r\n")-1);
	SYS_PostTask(APL_TASK_ID);
}

static void writeSps30Done(bool success){
	if(!success) appWriteDataToUsart((uint8_t*)"write callback fail\r\n", sizeof("write callback fail\r\n")-1);
	if(success){
		HAL_StartAppTimer(&sps30WaitTimer);
		appWriteDataToUsart((uint8_t*)"write callback success\r\n", sizeof("write callback success\r\n")-1);
	}
	if (-1 == HAL_CloseI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"wrclose fail\r\n", sizeof("wrclose fail\r\n")-1);
	SYS_PostTask(APL_TASK_ID);
}

static void reqMeasureSps30Done(bool success) {
	if(!success) appWriteDataToUsart((uint8_t*)"request measure callback fail\r\n", sizeof("request measure callback fail\r\n")-1);
	if(success){
		appstate = SPS30_READ_MES_STATE;
		appWriteDataToUsart((uint8_t*)"request measure callback success\r\n", sizeof("request measure callback success\r\n")-1);
	}
	if (-1 == HAL_CloseI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"request wrclose fail\r\n", sizeof("request wrclose fail\r\n")-1);
	SYS_PostTask(APL_TASK_ID);
}

static void reqStopMeasureSps30Done(bool success) {
	if(!success) appWriteDataToUsart((uint8_t*)"measure callback fail\r\n", sizeof("measure callback fail\r\n")-1);
	if(success){
		appWriteDataToUsart((uint8_t*)"measure callback success\r\n", sizeof("measure callback success\r\n")-1);
	}
	if (-1 == HAL_CloseI2cPacket(&sps30writeDescriptor)) appWriteDataToUsart((uint8_t*)"stop wrclose fail\r\n", sizeof("stop wrclose fail\r\n")-1);
	SYS_PostTask(APL_TASK_ID);
}


///////////////////////////

static void initEndpoint(void){
	simpleDescriptor.AppDeviceId = 1;
	simpleDescriptor.AppProfileId = 1;
	simpleDescriptor.endpoint = 1;
	simpleDescriptor.AppDeviceVersion = 1;
	endPoint.simpleDescriptor= &simpleDescriptor;
	endPoint.APS_DataInd = APS_DataInd;
	APS_RegisterEndpointReq(&endPoint);
}

void APS_DataInd(APS_DataInd_t *indData){
	BSP_OnLed(LED_RED);
	HAL_StartAppTimer(&receiveTimerLed);
	appWriteDataToUsart(indData->asdu,indData->asduLength);
	appWriteDataToUsart((uint8_t*)"\r\n",2);
}

static void initLEDTimer(void){
	transmitTimerLed.interval= 500;
	transmitTimerLed.mode= TIMER_ONE_SHOT_MODE;
	transmitTimerLed.callback=transmitTimerLedFired;
	receiveTimerLed.interval= 500;
	receiveTimerLed.mode= TIMER_ONE_SHOT_MODE;
	receiveTimerLed.callback=receiveTimerLedFired;
	transmitTimer.interval= 3000;
	transmitTimer.mode= TIMER_REPEAT_MODE;
	transmitTimer.callback=transmitTimerFired;
}

static void initTransmitData(void){
	dataReq.profileId=1;
	dataReq.dstAddrMode =APS_SHORT_ADDRESS;
	dataReq.dstAddress.shortAddress= CPU_TO_LE16(0);
	dataReq.dstEndpoint =1;
	dataReq.asdu=transmitData.data;
	dataReq.asduLength=sizeof(transmitData.data);
	dataReq.srcEndpoint = 1;
	dataReq.APS_DataConf=APS_DataConf;
}
static void APS_DataConf(APS_DataConf_t *confInfo){
	if (confInfo->status == APS_SUCCESS_STATUS){
		BSP_OnLed(LED_YELLOW);
		HAL_StartAppTimer(&transmitTimerLed);
		appstate=APP_NOTHING_STATE;
		SYS_PostTask(APL_TASK_ID);
	}
}

static void transmitTimerLedFired(void){
	BSP_OffLed(LED_YELLOW);
}
static void receiveTimerLedFired(void){
	BSP_OffLed(LED_RED);
}
static void transmitTimerFired(void){
	appstate = nextState;
	SYS_PostTask(APL_TASK_ID);
}

void ZDO_StartNetworkConf(ZDO_StartNetworkConf_t *confirmInfo){
	if (ZDO_SUCCESS_STATUS == confirmInfo->status){
		CS_ReadParameter(CS_DEVICE_TYPE_ID,&deviceType);
		if(deviceType==DEV_TYPE_COORDINATOR){
			appWriteDataToUsart((uint8_t*)"Coordinator\r\n", sizeof("Coordinator\r\n")-1);
		}
		BSP_OnLed(LED_YELLOW); //LED_GREEN für den Koordinator und LED_YELLOW für den Router und das Endgerät
		}else{
		appWriteDataToUsart((uint8_t*)"Error\r\n",sizeof("Error\r\n")-1);
	}
	SYS_PostTask(APL_TASK_ID);
}

/////pause timer
static void initPauseTimer(){
	pauseTimer.interval = SPS30_WAIT_MES;
	pauseTimer.mode		= TIMER_ONE_SHOT_MODE;
	pauseTimer.callback = pauseTimerDone;
}

static void pausingTimer(AppState_t _next_appstate){
	nextState = _next_appstate;
	HAL_StartAppTimer(&pauseTimer);
}

static void pauseTimerDone(){
	appstate = nextState;
	SYS_PostTask(APL_TASK_ID);
}

/////////////////////////


/*******************************************************************************
  \brief The function is called by the stack to notify the application about 
  various network-related events. See detailed description in API Reference.
  
  Mandatory function: must be present in any application.

  \param[in] nwkParams - contains notification type and additional data varying
             an event
  \return none
*******************************************************************************/
void ZDO_MgmtNwkUpdateNotf(ZDO_MgmtNwkUpdateNotf_t *nwkParams)
{
  nwkParams = nwkParams;  // Unused parameter warning prevention
}

/*******************************************************************************
  \brief The function is called by the stack when the node wakes up by timer.
  
  When the device starts after hardware reset the stack posts an application
  task (via SYS_PostTask()) once, giving control to the application, while
  upon wake up the stack only calls this indication function. So, to provide 
  control to the application on wake up, change the application state and post
  an application task via SYS_PostTask(APL_TASK_ID) from this function.

  Mandatory function: must be present in any application.
  
  \return none
*******************************************************************************/
void ZDO_WakeUpInd(void)
{
}

#ifdef _BINDING_
/***********************************************************************************
  \brief The function is called by the stack to notify the application that a 
  binding request has been received from a remote node.
  
  Mandatory function: must be present in any application.

  \param[in] bindInd - information about the bound device
  \return none
 ***********************************************************************************/
void ZDO_BindIndication(ZDO_BindInd_t *bindInd)
{
  (void)bindInd;
}

/***********************************************************************************
  \brief The function is called by the stack to notify the application that a 
  binding request has been received from a remote node.

  Mandatory function: must be present in any application.
  
  \param[in] unbindInd - information about the unbound device
  \return none
 ***********************************************************************************/
void ZDO_UnbindIndication(ZDO_UnbindInd_t *unbindInd)
{
  (void)unbindInd;
}
#endif //_BINDING_

/**********************************************************************//**
  \brief The entry point of the program. This function should not be
  changed by the user without necessity and must always include an
  invocation of the SYS_SysInit() function and an infinite loop with
  SYS_RunTask() function called on each step.

  \return none
**************************************************************************/
int main(void)
{
  //Initialization of the System Environment
  SYS_SysInit();

  //The infinite loop maintaing task management
  for(;;)
  {
    //Each time this function is called, the task
    //scheduler processes the next task posted by one
    //of the BitCloud components or the application
    SYS_RunTask();
  }
}

//eof app.c
