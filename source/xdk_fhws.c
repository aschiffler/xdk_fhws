
#include "XdkAppInfo.h"
#undef BCDS_MODULE_ID
#define BCDS_MODULE_ID  XDK_APP_MODULE_ID_XDK_FHWS

/* system header files */
#include "xdk_fhws.h"

#include <stdio.h>
/* additional interface header files */
#include "BCDS_Basics.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "BCDS_NetworkConfig.h"
#include "BCDS_WlanConnect.h"
#include "BCDS_ServalPal.h"
#include "BCDS_ServalPalWiFi.h"

#include "XDK_WLAN.h"
#include "XDK_Utils.h"
#include "XDK_LED.h"
#include "XDK_Button.h"


/* own header files */
#include "BCDS_CmdProcessor.h"
#include "BCDS_Assert.h"
#include "Serval_Mqtt.h"
#include <Serval_Clock.h>
#include <Serval_Msg.h>
#include <Serval_XUdp.h>
#include <Serval_Log.h>
#include "EnvironmentalSensor.h"
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "InertialSensor.h"
#include "LightSensor.h"
#include "Magnetometer.h"
#include "Acoustic.h"
#include "BCDS_SDCard_Driver.h"
#include "ff.h"
#include "fs.h"

#include "XdkSensorHandle.h"

// Global array of all sensors => true : enable -- false : disable
bool typesSensors[8] = {
		true, // 0 ENVIROMENTAL
		true, // 1 ACCELEROMETER
		true, // 2 GYROSCOPE
		true, // 3 INERTIAL
		true, // 4 LIGHT
		true, // 5 MAGNETOMETER
		true, // 6 ACOUSTIC
		true  // 7 WLAN_ENTERPRISE
};

char* DEVICE_NAME = NULL;
char* WLAN_SSID = NULL;
char* WLAN_USER = NULL;
char* WLAN_PSK = NULL;
char* NTP_SERVER_HOST = NULL;
char* NTP_SERVER_PORT = NULL;
char* MQTT_BROKER_HOST = NULL;
char* MQTT_BROKER_PORT = NULL;
char* PUBLISHTIMER_PERIOD_IN_MS = NULL;
char* MQTT_USERNAME = NULL;
char* MQTT_PASSWORD = NULL;
char* TOPIC = NULL;

static CmdProcessor_T *AppCmdProcessor;
static uint32_t SysTime = UINT32_C(0);

static MqttSession_T Session;
static MqttSession_T *SessionPtr;

static FIL fileObject;

static uint8_t PublishInProgress = 0;

static TimerHandle_t PublishTimerHandle;
static StringDescr_T PublishTopicDescription;

static StringDescr_T Topics[1];
static Mqtt_qos_t Qos[1];

static char MqttBroker[50];
static const char MqttBrokerAddressFormat[50] = "mqtt://%s:%d";

static void Button1Callback(ButtonEvent_T buttonEvent);
static void Button2Callback(ButtonEvent_T buttonEvent);

static Button_Setup_T ButtonSetup =
        {
                .CmdProcessorHandle = NULL,
                .InternalButton1isEnabled = true,
                .InternalButton2isEnabled = true,
                .InternalButton1Callback = Button1Callback,
                .InternalButton2Callback = Button2Callback,
        };/**< Button setup parameters */

/**
 * @brief Callback for Button 1.
 *
 * @param[in]    buttonEvent
 * If it is BUTTON_EVENT_PRESSED, then Red and Yellow LED's are turned ON
 * If it is BUTTON_EVENT_RELEASED, then Orange LED is turned ON
 *
 */
static void Button1Callback(ButtonEvent_T buttonEvent)
{
    Retcode_T retcode = RETCODE_OK;
    switch (buttonEvent)
    {
    case BUTTON_EVENT_PRESSED:
        {
        retcode = LED_On(LED_INBUILT_RED);
        if (RETCODE_OK == retcode)
        {
            retcode = LED_Off(LED_INBUILT_ORANGE);
        }
        if (RETCODE_OK == retcode)
        {
            retcode = LED_On(LED_INBUILT_YELLOW);
        }
        if (RETCODE_OK == retcode)
        {
            printf("Button1Callback : PB1 Pressed \r\n");
        }
        else
        {
            printf("Button1Callback : PB1 Pressed but setting LED state failed \r\n");
        }
    }
        break;

    case BUTTON_EVENT_RELEASED:
        {
        retcode = LED_Off(LED_INBUILT_RED);
        if (RETCODE_OK == retcode)
        {
            retcode = LED_On(LED_INBUILT_ORANGE);
        }
        if (RETCODE_OK == retcode)
        {
            retcode = LED_Off(LED_INBUILT_YELLOW);
        }
        if (RETCODE_OK == retcode)
        {
            printf("Button1Callback : PB1 Released \r\n");
        }
        else
        {
            printf("Button1Callback : PB1 Released but setting LED state failed \r\n");
        }
    }
        break;

    default:
        printf("Button1Callback : Unsolicited button event occurred for PB1 \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
        break;
    }
    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
    }
}

/**
 * @brief Callback for Button 2.
 *
 * @param[in]    buttonEvent
 * If it is BUTTON_EVENT_PRESSED, then Red and Orange LED's are turned ON
 * If it is BUTTON_EVENT_RELEASED, then Yellow LED is turned ON
 *
 */
static void Button2Callback(ButtonEvent_T buttonEvent)
{
    Retcode_T retcode = RETCODE_OK;
    switch (buttonEvent)
    {
    case BUTTON_EVENT_PRESSED:
    {
    	InitSntpTime();
    	retcode = LED_On(LED_INBUILT_RED);
    	if (RETCODE_OK == retcode)
    	{
    		retcode = LED_On(LED_INBUILT_ORANGE);
    	}
    	if (RETCODE_OK == retcode)
    	{
    		retcode = LED_Off(LED_INBUILT_YELLOW);
    	}
    	if (RETCODE_OK == retcode)
    	{
    		printf("Button2Callback : PB2 pressed \r\n");
    	}
    	else
    	{
    		printf("Button2Callback : PB2 Pressed but setting LED state failed \r\n");
    	}
    }
    break;

    case BUTTON_EVENT_RELEASED:
        {
        retcode = LED_Off(LED_INBUILT_RED);
        if (RETCODE_OK == retcode)
        {
            retcode = LED_Off(LED_INBUILT_ORANGE);
        }
        if (RETCODE_OK == retcode)
        {
            retcode = LED_On(LED_INBUILT_YELLOW);
        }

        if (RETCODE_OK == retcode)
        {
            printf("Button2Callback : PB2 Released\n\r");
        }
        else
        {
            printf("Button2Callback : PB2 Released but setting LED state failed \r\n");
        }
    }
        break;

    default:
        printf("Button2Callback : Unsolicited button event occurred for PB2 \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
        break;
    }
    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
    }
}

Retcode_T InitSdCard(void){

	Retcode_T retVal =RETCODE_FAILURE;
	FRESULT FileSystemResult = 0;
	static FATFS FatFileSystemObject;
	SDCardDriver_Initialize();
	if(SDCARD_INSERTED== SDCardDriver_GetDetectStatus()){
		retVal = SDCardDriver_DiskInitialize(DRIVE_ZERO);
		if(RETCODE_OK == retVal){
			printf("SD Card Disk initialize succeeded \n\r");
			FileSystemResult = f_mount(&FatFileSystemObject, DEFAULT_LOGICAL_DRIVE, FORCE_MOUNT);
			if(0 != FileSystemResult){
				printf("Mounting SD card failed \n\r");
			}
		}
	}else{
		printf("SD card failed \n\r");
	}
	return retVal;

}

Retcode_T searchForFileOnSdCard(const char * filename, FILINFO * fileData){

	if(0  == f_stat(filename, fileData)){
		printf("File %s found on SD card. \n\r"	,filename);
		return RETCODE_OK;
	}
	else{
		printf(	"File %s does not exist. \n\r"	,filename);
		return	RETCODE_FAILURE;
	}
}

void readDataFromFileOnSdCard(const char* filename){
	FRESULT fileSystemResult;
	FILINFO fileInfo;
	char bufferRead[UINT16_C(512)];
	UINT bytesRead;
	int i=0;
	DEVICE_NAME = calloc(96, sizeof(char));
	WLAN_SSID = calloc(128, sizeof(char));
	WLAN_USER = calloc(128, sizeof(char));
	WLAN_PSK = calloc(128, sizeof(char));
	NTP_SERVER_HOST = calloc(128, sizeof(char));
	NTP_SERVER_PORT = calloc(128, sizeof(char));
	MQTT_BROKER_HOST = calloc(20, sizeof(char));
	MQTT_BROKER_PORT = calloc(10, sizeof(char));
	PUBLISHTIMER_PERIOD_IN_MS = calloc(20, sizeof(char));
	MQTT_USERNAME = calloc(64, sizeof(char));
	MQTT_PASSWORD = calloc(64, sizeof(char));
	TOPIC = calloc(64, sizeof(char));


	if(RETCODE_OK == searchForFileOnSdCard(filename,&fileInfo)){
		f_open(&fileObject, filename, FA_OPEN_EXISTING | FA_READ);
		f_lseek(&fileObject, FIRST_LOCATION);
		fileSystemResult = f_read(&fileObject, bufferRead, fileInfo.fsize,&bytesRead);
		if((fileSystemResult !=0) || (fileInfo.fsize != bytesRead)){
			printf("Error: Cannot read file %s \n\r",filename);
		}
		else{
			bufferRead[bytesRead] ='\0';
			printf("Read data from file %s \n\r",filename);
			int j=0,tipo=1;
			for(i=0;i<bytesRead;i++){
				if(bufferRead[i]=='='){
					i++;
					j=0;
					while(bufferRead[i+1]!='\n'){
						switch(tipo){
						case t_DEVICE_NAME:
							DEVICE_NAME[j] = bufferRead[i];
							break;
						case t_WLAN_SSID:
							WLAN_SSID[j] = bufferRead[i];
							break;
						case t_WLAN_USER:
							WLAN_USER[j] = bufferRead[i];
							break;
						case t_WLAN_PSK:
							WLAN_PSK[j] = bufferRead[i];
							break;
						case t_NTP_SERVER_HOST:
							NTP_SERVER_HOST[j] = bufferRead[i];
							break;
						case t_NTP_SERVER_PORT:
							NTP_SERVER_PORT[j] = bufferRead[i];
							break;
						case t_MQTT_BROKER_HOST:
							MQTT_BROKER_HOST[j] = bufferRead[i];
							break;
						case t_MQTT_BROKER_PORT:
							MQTT_BROKER_PORT[j] = bufferRead[i];
							break;
						case t_PUBLISHTIMER_PERIOD_IN_MS:
							PUBLISHTIMER_PERIOD_IN_MS[j] = bufferRead[i];
							break;
						case t_MQTT_USERNAME:
							MQTT_USERNAME[j] = bufferRead[i];
							break;
						case t_MQTT_PASSWORD:
							MQTT_PASSWORD[j] = bufferRead[i];
							break;
						case t_TOPIC:
							TOPIC[j] = bufferRead[i];
							break;
						default:
							if(bufferRead[i]=='Y' || bufferRead[i]=='E' || bufferRead[i]=='S')
								typesSensors[tipo-MAX_PARAMETERS_ARRAY-1]=true;
							else
								typesSensors[tipo-MAX_PARAMETERS_ARRAY-1]=false;
							break;
						}
						j++;i++;
					}
					tipo++;

				}
			}
			for (i=0;i<MAX_SENSORS_ARRAY;i++){
				printf("P %d ist %d\r\n",i,typesSensors[i]);
			}
			printf("%s\r\n",DEVICE_NAME);
			printf("%s\r\n",WLAN_SSID);
			printf("%s\r\n",WLAN_USER);
			printf("%s\r\n",WLAN_PSK);
			printf("%s\r\n",NTP_SERVER_HOST);
			printf("%s\r\n",NTP_SERVER_PORT);
			printf("%s\r\n",MQTT_BROKER_HOST);
			printf("%s\r\n",MQTT_BROKER_PORT);
			printf("%s\r\n",PUBLISHTIMER_PERIOD_IN_MS);
			printf("%s\r\n",MQTT_USERNAME);
			printf("%s\r\n",MQTT_PASSWORD);
			printf("%s\r\n",TOPIC);
		}
		f_close(&fileObject);
	}else{
		printf("No file with name %s exists on the SD card \n\r",filename);
	}
}


static Retcode_T NetworkSetup(void)
{
	printf("WLAN_ENTERPRISE: = %d\r\n", typesSensors[7]);
	WLAN_Setup_T WLANSetupInfo =
	{
			.IsEnterprise = typesSensors[7],
			.IsHostPgmEnabled = false,
			.SSID = WLAN_SSID,
			.Username = WLAN_USER,
			.Password = WLAN_PSK,
			.IsStatic = 0,
			.IpAddr = XDK_NETWORK_IPV4(0, 0, 0, 0),
			.GwAddr = XDK_NETWORK_IPV4(0, 0, 0, 0),
			.DnsAddr = XDK_NETWORK_IPV4(0, 0, 0, 0),
			.Mask = XDK_NETWORK_IPV4(0, 0, 0, 0),
	};

	retcode_t rc = RC_OK;
	rc = WLAN_Setup(&WLANSetupInfo);

	if (RC_OK != rc)
	{
		printf("appInitSystem: network init/connection failed. error=%d\r\n", rc);
		return rc;
	}
	printf("ServalPal Setup\r\n");
	rc = ServalPAL_Setup(AppCmdProcessor);

	Retcode_T retcode = WLAN_Enable();
	if (RETCODE_OK == retcode)
	{
		retcode = ServalPAL_Enable();
	}

	return retcode;
}


static retcode_t SubscribeToOwnPublishTopic(void)
{
	int8_t topic_buffer[40];
	strncpy((char *)topic_buffer, Topics[0].start, sizeof(topic_buffer));
	printf("Subscribing to topic: %s, Qos: %d\n\r", topic_buffer, Qos[0]);
	retcode_t rc_subscribe = Mqtt_subscribe(SessionPtr, 1, Topics, Qos);
	return rc_subscribe;
}

uint32_t GetUtcTime(void) {
	retcode_t rc = RC_CLOCK_ERROR_FATAL;
	uint32_t sysUpTime;
	rc = Clock_getTime(&sysUpTime);
	if (rc != RC_OK) {
		printf("Failed to get the Clock Time \r\n");
	}
	return sysUpTime + SysTime;
}



static char* receiveBufferFromSensors(void){
	int i=0;
	bool typeSensor;

	char *buffer = calloc(1024, sizeof(char));
	char *aux = calloc(128, sizeof(char));;


	strcat(buffer,"{\"XDK_FHWS\":[");

	for(i=0;i<MAX_SENSORS_ARRAY-ADD_BOOL_PARA;i++){

		typeSensor = typesSensors[i];

		if(typeSensor){

			switch(i)
			{
			case ENVIROMENTAL:
				aux = processEnvSensorData(null,0);
				break;
			case ACCELEROMETER:
				aux = processAccelData(null,0);
				break;
			case GYROSCOPE:
				aux = processGyroData(null,0);
				break;
			case INERTIAL:
				aux = processInertiaSensor(null,0);
				break;
			case LIGHT:
				aux = processLightSensorData(null,0);
				break;
			case MAGNETOMETER:
				aux = processMagnetometerData(null,0);
				break;
			case ACOUSTIC:
				aux = processAcousticData(null,0);
				break;

			}
			strcat(buffer,aux);
			strcat(buffer,",");
			free(aux);
		}
	}

	if(buffer[strlen(buffer)-1]==',')
		buffer[strlen(buffer)-1]=' ';

	char * deviceName = calloc(255, sizeof(char));
	char * timestamp = calloc(255, sizeof(char));
	char * terminate = '\0';
	sprintf(deviceName,"\"device\": \"%s\",",DEVICE_NAME);
	sprintf(timestamp,"\"timestamp\": \"%ld\"}",GetUtcTime());
	strcat(buffer,"],");
	strcat(buffer,deviceName);
	strcat(buffer,timestamp);
	strcat(buffer,terminate);
	free(deviceName);
	free(timestamp);

	return (char*)buffer;

}



static void PublishData(void *param1, uint32_t param2)
{
	BCDS_UNUSED(param1);
	BCDS_UNUSED(param2);
	retcode_t rc_publish;


	char* httpBodyBuffer =  receiveBufferFromSensors();
	rc_publish = Mqtt_publish(SessionPtr, PublishTopicDescription,httpBodyBuffer, strlen(httpBodyBuffer), (uint8_t) MQTT_QOS_AT_MOST_ONE, false);
	if (rc_publish == RC_OK)
	{
		PublishInProgress = 1;
	}
	else
	{
		PublishInProgress = 0;
		printf("Mqtt_publish is failed:Stack erro code :%u \n\r",(unsigned int)rc_publish);
		Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_PUBLISH_FAIL));
	}
	free(httpBodyBuffer);
}

static void EnqueueMessagePublish(TimerHandle_t pxTimer)
{
	BCDS_UNUSED(pxTimer);

	if (!PublishInProgress)
	{
		Retcode_T retcode = CmdProcessor_Enqueue(AppCmdProcessor, PublishData, NULL, 0);
		if (RETCODE_OK != retcode)
		{
			printf("CmdProcessor_Enqueue is failed \n\r");
			Retcode_RaiseError(retcode);
		}
	}
}

/**
 * @brief Create and start software timer for MQTT publishing the sensor data
 *
 */
static void CreateAndStartPublishingTimer(void)
{
	PublishTimerHandle = xTimerCreate(
			(const char * const ) "Publish Timer",
			(atoi(PUBLISHTIMER_PERIOD_IN_MS)/portTICK_RATE_MS),
			pdTRUE,
			NULL,
			EnqueueMessagePublish);
	if(NULL == PublishTimerHandle)
	{
		printf("xTimerCreate is failed \n\r");
		Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_FATAL, RETCODE_OUT_OF_RESOURCES));
	}
	else if ( pdFAIL == xTimerStart(PublishTimerHandle, 10000))
	{
		printf("xTimerStart is failed \n\r");
	}
}

static void HandleEventConnection(MqttConnectionEstablishedEvent_T connectionData)
{
	printf("Connection Event:\n\r"
			"\tServer Return Code: %d (0 for success)\n\r"
			"\tReused Session Flag: %d\n\r",
			(int) connectionData.connectReturnCode,
			(int) connectionData.sessionPresentFlag);

	if (connectionData.connectReturnCode == 0)
	{
		retcode_t rc = SubscribeToOwnPublishTopic();
		if(RC_OK != rc)
		{
			printf("SubscribeToOwnPublishTopic is failed\n\r");
			printf("Stack error code :%u \n\r",(unsigned int)rc);
			Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_SUBSCRIBE_FAIL));
		}
	}

}

static void HandleEventIncomingPublish(
		MqttPublishData_T publishData)
{
	//char published_topic_buffer[COMMON_BUFFER_SIZE];
	//char published_data_buffer[COMMON_BUFFER_SIZE];
	static int incoming_message_count = 0;

	//strncpy(published_data_buffer, (const char *)publishData.payload, sizeof(published_data_buffer));
	//strncpy(published_topic_buffer, publishData.topic.start, sizeof(published_topic_buffer));
/*
	printf("#%d, Incoming Published Message:\n\r"
			"\tTopic: %s\n\r"
			"\tPayload: \n\r\"\"\"\n\r%s\n\r\"\"\"\n\r", incoming_message_count,
			published_topic_buffer, published_data_buffer);
			*/
	printf("%d",incoming_message_count++);
}


static void HandleEventSuccessfulPublish(void *param1, uint32_t param2)
{
	BCDS_UNUSED(param1);
	BCDS_UNUSED(param2);
	PublishInProgress = 0;

}


static retcode_t EventHandler(MqttSession_T* session, MqttEvent_t event,
		const MqttEventData_t* eventData)
{
	BCDS_UNUSED(session);
	Retcode_T retcode = RETCODE_OK;
	printf("EventHandler Event : %d\n\r", (int)event);
	switch (event)
	{
	case MQTT_CONNECTION_ESTABLISHED:
		HandleEventConnection(eventData->connect);
		break;
	case MQTT_CONNECTION_ERROR:
		HandleEventConnection(eventData->connect);
		break;
	case MQTT_INCOMING_PUBLISH:
		HandleEventIncomingPublish(eventData->publish);
		break;
	case MQTT_PUBLISHED_DATA:

		retcode = CmdProcessor_Enqueue(AppCmdProcessor, HandleEventSuccessfulPublish, NULL, 0);
		if (RETCODE_OK != retcode)
		{
			printf("CmdProcessor_Enqueue is failed \n\r");
			Retcode_RaiseError(retcode);
		}
		break;
	case MQTT_SUBSCRIBE_SEND_FAILED:
		printf("MQTT_SUBSCRIBE_SEND_FAILED\n\r");
		break;
	case MQTT_SUBSCRIPTION_ACKNOWLEDGED:
		if (USE_PUBLISH_TIMER)
		{
			CreateAndStartPublishingTimer();
		}
		else
		{

			PublishData(NULL, 0);
		}
		printf("MQTT_SUBSCRIPTION_ACKNOWLEDGED\n\r");
		break;
	case MQTT_CONNECT_TIMEOUT:
		printf("MQTT_CONNECT_TIMEOUT\n\r");
		break;
	default:
		printf("Unhandled MQTT Event Number: %d\n\r", event);
		break;
	}
	return RC_OK;
}


static void Start(void)
{
	retcode_t rc = RC_INVALID_STATUS;
	rc = Mqtt_connect(SessionPtr);
	if (rc != RC_OK)
	{
		printf("Could not Connect, error 0x%04x\n", rc);
		Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_CONNECT_FAIL));
	}
}


static void ConfigureSession(void)
{
	// set target
	int8_t server_ip_buffer[13];
	Ip_Address_T ip;

	Retcode_T rc;
	rc = NetworkConfig_GetIpAddress((uint8_t *) MQTT_BROKER_HOST, &ip);
	if(RETCODE_OK != rc)
	{
		printf("Getting Broker address failure\n\r");
		Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_IPCONIG_FAIL));
	}
	else
	{
		int32_t retval = Ip_convertAddrToString(&ip, (char *)server_ip_buffer);
		if(0 == retval)
		{
			printf("Ip_convertAddrToString return value is zero\n\r");
			Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_IPCONIG_FAIL));
		}
		else
		{
			sprintf(MqttBroker, MqttBrokerAddressFormat, server_ip_buffer,
					(uint16_t)atoi(MQTT_BROKER_PORT));

			rc = SupportedUrl_fromString((const char *)MqttBroker, (uint16_t) strlen((const char *)MqttBroker),
					&SessionPtr->target);
			if(RC_OK != rc)
			{
				printf("SupportedUrl_fromString failure\n\r");
				Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_IPCONIG_FAIL));
			}
			else
			{
				SessionPtr->target.scheme = SERVAL_SCHEME_MQTT;

				printf("Broker address: %s\n\r", MqttBroker);

				SessionPtr->onMqttEvent = EventHandler;


				SessionPtr->MQTTVersion = 3;
				SessionPtr->keepAliveInterval = 100;
				SessionPtr->cleanSession = true;
				SessionPtr->will.haveWill = false;

				StringDescr_T username;
				StringDescr_wrap(&username, MQTT_USERNAME);
				SessionPtr->username = username;
				StringDescr_T password;
				StringDescr_wrap(&password, MQTT_PASSWORD);
				SessionPtr->password = password;

				StringDescr_T device_name_descr;
				StringDescr_wrap(&device_name_descr, DEVICE_NAME);
				SessionPtr->clientID = device_name_descr;

				StringDescr_wrap(&PublishTopicDescription, (const char *)TOPIC);
				StringDescr_wrap(&(Topics[0]), TOPIC);
				Qos[0] = MQTT_QOS_AT_MOST_ONE;
			}
		}
	}
}
void SetUtcTime(uint32_t utcTime) {
	retcode_t rc = RC_CLOCK_ERROR_FATAL;
	uint32_t sysUpTime;
	rc = Clock_getTime(&sysUpTime);
	if (rc != RC_OK) {
		printf("Failed to get the Clock Time \r\n");
	}
	SysTime = utcTime - sysUpTime;
}


static void ReceiveCallback(Msg_T *msg_ptr, retcode_t status) {

	unsigned int payloadLen;
	uint8_t *payload_ptr;

	if (status != RC_OK) {

	}
	XUdp_getXUdpPayload(msg_ptr, &payload_ptr, &payloadLen);

	if (payloadLen >= NTP_PACKET_SIZE) {
		uint64_t secsSince1900;
		secsSince1900 = (unsigned long)payload_ptr[40] << 24;
		secsSince1900 |= (unsigned long)payload_ptr[41] << 16;
		secsSince1900 |= (unsigned long)payload_ptr[42] << 8;
		secsSince1900 |= (unsigned long)payload_ptr[43];
		uint64_t secsSince1970 = secsSince1900 - 2208988800UL;
		printf("NTP got UTC secSince1970: %llu\n\r", secsSince1970);
		SetUtcTime(secsSince1970);
	} else {
		printf("NTP response not valid!\n\r");
	}
}


static void SendCallback(Msg_T *msg_ptr, retcode_t status) {
	BCDS_UNUSED(msg_ptr);

	printf("NTP request Sending Complete\n\r");

	if (status != RC_OK) {
		printf("Sending status not RC_OK; status=" RC_RESOLVE_FORMAT_STR "\n\r",
				RC_RESOLVE((int)status));
	}
}


void InitSntpTime(void) {
	retcode_t rc = RC_OK;
	Msg_T *MsgHandlePtr = NULL;
	uint32_t sntpIpAddress = 0x00;
	uint8_t buffer[NTP_PACKET_SIZE];
	unsigned int bufferLen = NTP_PACKET_SIZE;

	/* init request: */
	memset(buffer, 0, NTP_PACKET_SIZE);
	buffer[0] = 0b11100011; /* LI, Version, Mode */ /*lint !e146 */
	buffer[1] = 0;    /* Stratum, or type of clock */
	buffer[2] = 6;    /* Polling Interval */
	buffer[3] = 0xEC; /* Peer Clock Precision */
	/* 8 bytes of zero for Root Delay & Root Dispersion */
	buffer[12] = 49;
	buffer[13] = 0x4E;
	buffer[14] = 49;
	buffer[15] = 52;

	rc = NetworkConfig_GetIpAddress((uint8_t *) NTP_SERVER_HOST, &sntpIpAddress);
	if(RETCODE_OK != rc)
	{
		printf("Getting NTP Server address failure\n\r");
		Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_MQTT_IPCONIG_FAIL));
	}

	rc = XUdp_initialize();
	if (rc != RC_OK) {
		printf("FAILED TO INIT\n");
		LOG_ERROR("Failed to init XUDP; rc=" RC_RESOLVE_FORMAT_STR "\n",RC_RESOLVE((int)rc));
		return;
	}

	rc = XUdp_start(Ip_convertIntToPort((uint16_t)atoi(NTP_SERVER_PORT)), ReceiveCallback);
	if (rc != RC_OK) {
		printf("FAILED TO START\n");
		LOG_ERROR("Failed to start XUDP; rc=" RC_RESOLVE_FORMAT_STR "\n",RC_RESOLVE((int)rc));
		return;
	}

	/* now send request: */
	rc = XUdp_push(&sntpIpAddress, (uint16_t)atoi(NTP_SERVER_PORT), buffer, bufferLen, SendCallback, &MsgHandlePtr);
	if (rc != RC_OK) {
		LOG_ERROR("Sending failure; rc=" RC_RESOLVE_FORMAT_STR "\n",
				RC_RESOLVE((int)rc));
		return;
	}
	LOG_INFO("Pushed Echo\n");
	return;
}


static void Init(void)
{
	retcode_t rc_initialize = Mqtt_initialize();

	if (rc_initialize != RC_OK)
	{
		printf("Could not initialize mqtt, error 0x%04x\n", rc_initialize);
	}
	else
	{
		SessionPtr = &Session;
		memset(SessionPtr, 0, sizeof(*SessionPtr));
		rc_initialize = Mqtt_initializeInternalSession(SessionPtr);

		if (rc_initialize != RC_OK)
		{
			printf("Mqtt_initializeInternalSession is failed\n\r");
		}
		else
		{
			int i=0;
			for(i=0;i<MAX_SENSORS_ARRAY;i++){
				if(typesSensors[i]){
					switch(i)
					{
					case ENVIROMENTAL:
						environmentalSensorInit();
						break;
					case ACCELEROMETER:
						accelerometerSensorInit();
						break;
					case GYROSCOPE:
						gyroscopeSensorInit();
						break;
					case INERTIAL:
						inertialSensorInit();
						break;
					case LIGHT:
						lightsensorInit();
						break;
					case MAGNETOMETER:
						magnetometerSensorInit();
						break;
					case ACOUSTIC:
						acousticSensorInit();
						break;
					}
				}
			}
			ConfigureSession();
			Start();
		}
	}
}

void AppInitSystem(void * cmdProcessorHandle, uint32_t param2)
{
	BCDS_UNUSED(param2);
	Retcode_T rt = RETCODE_FAILURE;

	vTaskDelay(3000 / portTICK_RATE_MS);
	if (cmdProcessorHandle == NULL)
	{
		printf("Command processor handle is null \n\r");
		Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_FATAL,RETCODE_NULL_POINTER));
	}

	AppCmdProcessor = (CmdProcessor_T *) cmdProcessorHandle;

	if(InitSdCard() != RETCODE_FAILURE){
		readDataFromFileOnSdCard(FILE_NAME);
	}

	if (NetworkSetup() == RETCODE_OK){
		Button_Setup(&ButtonSetup);
		Button_Enable();
		LED_Enable();
		InitSntpTime();
		Init();
	}
	else{
		printf("Connect Failed\n\r");
		Retcode_RaiseError(rt);
	}
}

/** ************************************************************************* */
