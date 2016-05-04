/*
 Name:		IoTMega2560.ino
 Created:	1/7/2016 8:52:08 PM
 Author:	Motionlife

 Schematic:	D0	(Rx1)-->ESP8266(Tx)
			D1	(Tx2)-->ESP8266(Rx)
			D2	(EINT0)-->FPC1020+ IRQ
			D3	(EINT1)-->门开锁闭锁信号线
			16	(Tx3)-->Debug USB_Rx
			17	(Rx3)-->Debug USB_Tx
			18	(Tx2)FPC1020 UART_Rx
			19	(Rx2)FPC1020 UART_Tx
			A0	Driver signal of Power SYSTEM of ESP8266. High means power on
			A1  VCC of RTC_DS3231
			A2	Buzzer
				SDA Connected with RTC_DS3231
				SCL connected with RTC_DS3231
*/
#include <TimerThree.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
//----------------------------------------DECLARATION OF FUCTIONS GOT FROM ESP8266 Default Parameter----------------------------------
typedef struct Payload
{
	uint8_t fingerId;
	uint32_t utime;
	uint32_t keyStates[8];
};
typedef uint8_t(*GeneralIfOKFunction)();
uint8_t ATCommand(const char *command, const char *target, uint32_t timeout = 1588);
uint8_t successWithin(GeneralIfOKFunction jobToFinish, unsigned long time);
// watchdog intervals
// sleep bit patterns for WDTCSR
enum
{
	WDT_16_MS = 0b000000,
	WDT_32_MS = 0b000001,
	WDT_64_MS = 0b000010,
	WDT_128_MS = 0b000011,
	WDT_256_MS = 0b000100,
	WDT_512_MS = 0b000101,
	WDT_1_SEC = 0b000110,
	WDT_2_SEC = 0b000111,
	WDT_4_SEC = 0b100000,
	WDT_8_SEC = 0b100001,
};  // end of WDT intervals enum
//------------------------------------------------------------------------------------------------------------------------------------
#pragma region SYSTEM CONSTANTS
#define CMD_SLEEP			0x2C	//Sleep Module
#define CMD_ENROLL1  		0x01	//添加指纹步骤一
#define CMD_ENROLL2  		0x02	//添加指纹步骤二
#define CMD_ENROLL3  		0x03	//添加指纹步骤三
#define CMD_DELETE  		0x04	//删除指定编号指纹
#define CMD_CLEAR  			0x05	//清空所有指纹
#define CMD_USERNUMB  		0x09	//取用户总数
#define CMD_GETPRIORITY  	0x0A	//取用户权限
#define CMD_IDENTIFY  		0x0B	//1:1比对
#define CMD_SEARCH  		0x0C	//1:N比对
#define CMD_SETPAIRLEVEL  	0x28	//设置对比等级
#define CMD_SETTIMEOUT  	0x2E	//设置超时时间
#define ACK_SUCCESS  		0x00	//操作成功
#define ACK_FAIL	  		0x01	//操作失败
#define ACK_FULL	  		0x04	//指纹数据库已满
#define ACK_NOUSER   		0x05	//无此用户
#define ACK_USER_OCCUPIED	0x06	//此ID用户已存在
#define ACK_USER_EXIST		0x07	//用户已存在
#define ACK_TIMEOUT  		0x08	//采集超时
#define BUZZER_PIN			A0
#define MOTORPLUS			A2
#define MOTORMINUS			12
#define ESP_POWER_SIGNAL	A5
#define FPC1020_LED			A8
#define AIRKISS_LED			A9
#define DOORINFO_LED		A10
#define RTC_POWER_PING		7
#define FPCSOFTUARTRX		10
#define FPCSOFTUARTTX		11
#define FPCWAKEUP_PIN		18
#define DOORPST_PIN			19
#define SDA_PIN				20
#define SCL_PIN				21
#define INFO_DOOR_SUCCESS		(0)
#define INFO_DOOR_ALERT			(1)
#define INFO_DEVICE_START		(2)
#define INFO_SERVER_GOTIT		(3)
#define INFO_PAD_TOUCH1ST		(4)
#define INFO_PAD_TOUCH2ND		(5)
#define INFO_PAD_TOUCH3RD		(6)
#define INFO_AIRKISS_BEGIN		(7)
#define INFO_AIRKISS_SUCCESS	(8)
#define INFO_AIRKISS_END		(9)
#define INFO_UDP_OK				(10)
#define INFO_READ_FINGER_OVER	(11)
#define DEVICE_STATE_DOOROPENING		(1)
#define DEVICE_STATE_DATASENDING		(2)
#define DEVICE_STATE_WIFIFULLCONFIG		(3)
#define DEVICE_STATE_FPCCONFIG			(4)
#define DEVICE_STATE_WIFIRESTORECONFIG	(5)
#define WIFI_NO_RESPONSE	(0)		//not response
#define WIFI_RESPOND_NOIP	(1)		//can reply any AT command as expectly
#define WIFI_GOT_IP			(2)		//got an IP which wich can access Internet
#define WIFI_WORKMODE_OK	(3)		//set up working mode
#define WIFI_STATIC_OK		(4)		//config esp8266 with the static ip address.
#define DOOR_STATE_CLOSED	(0)
#define DOOR_STATE_OPENED	(1)
#define SMODE_CONFIG_TOUCH	(88)
#define SMODE_FPC_WIFI_TO		(600000)
#define SMODE_FPC_ENROLL_TO		(3888)
#define SMODE_FPC_AUTH_TO		(2888)
//#define SMODE_WIFI_AIRKISS_TO	(58888)
//#define SMODE_WIFI_GOIP_TO		(5888)
//#define SMODE_WIFI_MOTIONLIFE	(1888)
#define WAKER_PCI			(0)
#define WAKER_DOOR_OPEN		(1)
#define WAKER_FPC1020_IRQ   (2)
#define WAKER_DOOR_CLOSE	(3)
#define WAKER_WDT			(4)
#define TIMER3_RESOLUTION	(20000)	//20000 micro seconds
#pragma endregion

#define Esp8266Uart Serial3
//#define FPC1020Uart Serial2

#pragma region Arduino
SoftwareSerial FPC1020Uart(FPCSOFTUARTRX, FPCSOFTUARTTX);
volatile uint8_t waker;				//could be changed in the ISR,0:by PCI;1:by EXT0 or EXT1;2:by WDT;
volatile uint8_t wdtCounter;
volatile uint8_t fpcasleep;
volatile uint32_t timer3counter;
uint8_t deviceState;				//current device state,
uint8_t totalUsers;					//user finger_id or total number
Payload payload = { 0 };
uint8_t columnPinMap[] = { 22,24,26,28,30,32,34,37,36,39,38,41,40,43,42 };
uint8_t rowPinMap[] = { 23,25,27,29,31,33,35 };
RTC_DS1307 RTC;
// the setup function runs once when you press reset or power the board
void setup() {
	Esp8266Uart.begin(115200);
	FPC1020Uart.begin(19200);
	pinMode(13, OUTPUT); digitalWrite(13, LOW);		//DISABLE THIS LED TO SAVE MORE POWER
	NotifyUser(INFO_DEVICE_START);
	choseState();
}
// the loop function runs over and over again until power down or reset
void loop() {
	switch (deviceState) {
	case DEVICE_STATE_DOOROPENING://FPC1020+sending signal triger INT0, wake up me, about to open the door or not.
		DoorOpening();
		break;
	case DEVICE_STATE_DATASENDING://someone closed the door (wake up by INT1), about to sample and send data
		DataSending();
		break;
	case DEVICE_STATE_WIFIFULLCONFIG://WHICH MEANS user wanna config the device, eg config ESP8266 OR FPC1020+
	case DEVICE_STATE_WIFIRESTORECONFIG:
		WifiConfig();
		break;
	case DEVICE_STATE_FPCCONFIG:
		FingerPrintConfig();
		break;
	}
	SleepDeviceOn(currentDoorState());
}
#pragma endregion

#pragma region LIFECYCLE
/*
 determine whether or not user want to enter the config mode
*/
void choseState()
{
	uint8_t touchCount = 0;
	do
	{
		sleepFpc1020();//Make sure FPC1020+ is in sleep mode, since when wake up FPC1020 would spit out 'F5 09 02 D0 FF 00 24 F5' in some milliseconds
		sleepNow(SMODE_CONFIG_TOUCH);
		if (FPC1020Uart.available())
		{
			fpcasleep = 0;
			NotifyUser(++touchCount + 3);
			if (touchCount == 3) break;
		}
	} while (waker != WAKER_WDT);
	deviceState = DEVICE_STATE_DATASENDING + touchCount;
	if (touchCount) ConfigBegin();
}
void WifiConfig()
{
	ConifgEnd(AIRKISS_LED, Esp8266Config());
}
void FingerPrintConfig() {
	if (Esp8266Config() > WIFI_RESPOND_NOIP && successWithin(UdpConn, 3188))
	{
		NotifyUser(INFO_UDP_OK);
		FPC1020Config();
		sleepFpc1020();
	}
	ConifgEnd(FPC1020_LED, 2);
}
/*
This function was used to let user set wifi ssid and password through wechat AIR-KISS smartlink technology.
return 1 means that the Air Kiss works successfully, that is:
#1: The wifi module got its ip from a router with which it can use to access the Internet;
#2: The wifi module has been configured with a default working mode.
#3: The wifi has set up an UPD connection to listen to certern port with which can communicate with user
option: 0 MEANS NORMAL FULL WIFI CONFIG; 1 MEANS JUST FOR UDP CONNECTION; 2 MEANS CONFIG WITH RESTORE THE MODULE
*/
uint8_t Esp8266Config() {
	//1.ESPE266开机
	powerOnEsp8266();
	//2.ESPE266解冻
	if (!breakFrozen()) { return WIFI_NO_RESPONSE; }
	//what if user intend to clear wifi setting???
	if (deviceState == DEVICE_STATE_WIFIRESTORECONFIG)
	{
		ATCommand("AT+RESTORE", "OK"); //ESP8266 will restart after this command
		sleepNow(WDT_2_SEC);//wait to stable
	}
	//3.ESPE266智能配置
	if (!smartConfig()) { return WIFI_RESPOND_NOIP; }
	//4.Setting for wifi normal working mode
	if (!setWifiWorkMode()) { return WIFI_GOT_IP; }
	//5.Set static ip for fast speed
	if (!setStaticIp()) { return WIFI_WORKMODE_OK; }
	return WIFI_STATIC_OK;
}
/*
Comunicating with smart phone through the UDP connection to config PFC1020+
*/
void FPC1020Config()
{
	//User can enter this mode we pretty sure fpc was awake
	//Since now we use 2560 there is no need to save sram we could use an entire string line to communicate for convienence
	String cml;
	char ch;
	uint32_t start = millis();
	while (millis() - start < SMODE_FPC_WIFI_TO)
	{
		while (Esp8266Uart.available())
		{
			ch = Esp8266Uart.read();
			// Process message when new line character is recieved
			if (ch == '\n')
			{
				if (cml.indexOf("auth") != -1) {
					speakThroughWifi("TEST AUTH BEGIN");
					Auth();
					Esp8266Uart.println("TEST AUTH END");
				}
				else if (cml.indexOf("record") != -1) {
					// Record or delete a user's fingerprint under a specified Id
					RecordById(cml);
				}
				else if (cml.indexOf("delete") != -1) {
					DeleteById(cml);
				}
				else if (cml.indexOf("clearall") != -1) {
					//Clear all users' fingerprints
					speakThroughWifi(Fpc1020(CMD_CLEAR, 0, 0, 188) != ACK_FAIL ? "OK, Cleared" : "FAILED");
				}
				else if (cml.indexOf("total") != -1) {
					totalUsers = 0;
					//Query all the total number of the enrolled user
					if (Fpc1020(CMD_USERNUMB, 0, 0, 188) != ACK_FAIL)
					{
						speakThroughWifi("TOTAL USER NUMBER IS: " + String(totalUsers));
					}
					else
					{
						speakThroughWifi("Operation fail, make sure that fpc1020 is wake up");
					}
				}
				else if (cml.indexOf("quit") != -1) {
					speakThroughWifi("CONFIG FINISH");
					return;
				}
				else {
					speakThroughWifi("WRONG COMMAND!");
				}

				cml = ""; // Clear recieved buffer
			}
			else {
				cml += ch;
			}
			start = millis();
		}
	}
	speakThroughWifi("TIMEOUT CONFIG END");
}

/*
This state will begin by the user touching the fingerprint module which then wake up Arduino. Arduino
then will communicate with FPC1020+ to decide whether or not to open the door
return value means sleep after mode:
1: open success back to sleep and wait for lock;
0: open failed back to sleep and wait for open again;
*/
void DoorOpening() {
	//whether it was mechanically opened (INT1 RISING)
	if (waker == WAKER_DOOR_OPEN) {
		payload.fingerId = 0;
		//since it may caused by button bouncing or shaking, temporarilly sleep here for 1s
		sleepNow(WDT_2_SEC);
		if (currentDoorState() == DOOR_STATE_OPENED)
		{
			scanKeyPad(0);
		}
	}
	else if (waker == WAKER_FPC1020_IRQ)//begin ask 1020 to do 1:N comparison of the fingerprint
	{
		scanKeyPad(0);
		Auth();
	}
}
void Auth()
{
	uint8_t result = Fpc1020(CMD_SEARCH, 0, 0, SMODE_FPC_AUTH_TO);
	sleepFpc1020();//sleep fpc right after finishing the auth part
	if (result != ACK_FAIL && result != ACK_NOUSER && !result != ACK_TIMEOUT > 0) {
		openLock();
		NotifyUser(INFO_DOOR_SUCCESS);
		sleepNow(WDT_2_SEC);
		sleepNow(WDT_256_MS);
		closeLock();
	}
	else {
		NotifyUser(INFO_DOOR_ALERT);
	}

}
/*
what should do after waking up by door closed (locked) signal.
return value means sleep mode:
1: lock failed back to sleep wait for lock again;
0: lock success back to sleep and wait for open;
*/
void DataSending() {
	if (waker != WAKER_DOOR_CLOSE) { return; }
	NotifyUser(INFO_DOOR_SUCCESS);
	getTime();
	scanKeyPad(1);
	int failures = EEPROM.read(0);
	powerOnEsp8266();
	WaitTillEmpty(WDT_1_SEC);//WAIT ESP GOT IP
	if (sendPayloadViaHttp())
	{	//fetch all the previous payloads had not yet been sent
		for (; failures > 0; failures--)
		{
			EEPROM.get((failures - 1)*sizeof(Payload) + 1, payload);
			if (!sendPayloadViaHttp()) break;
		}
	}
	else
	{	//PUT the PAYLOAD of this time INTO EEPROM FOR NEXT TIME SENDING
		EEPROM.put(failures*sizeof(Payload) + 1, payload);
		if (failures < EEPROM.length() / sizeof(Payload)) failures++;
	}
	EEPROM.update(0, failures);
}
#pragma endregion

#pragma region Power Saving Plan
//=====================================================Sleep Fuctions=================================================================
/*
The interruption ISR funtion
*/
//handling pin int rising: Flag of system awakened by door mechanically opened without fingerprint authentication.
void DoorOpened() {
	waker = WAKER_DOOR_OPEN;
	detachInterrupt(digitalPinToInterrupt(DOORPST_PIN));
}
//handling int falling:	Flag of system awakened by FPC1020+ IRQ impulse
void FingerTouched()
{
	waker = WAKER_FPC1020_IRQ;
	fpcasleep = 0;
	detachInterrupt(digitalPinToInterrupt(FPCWAKEUP_PIN));
}
//handling int falling:	Flag of Door mechanically Locked
void DoorClosed()
{
	waker = WAKER_DOOR_CLOSE;
	detachInterrupt(digitalPinToInterrupt(DOORPST_PIN));
	wdt_disable();
}
//handling watchdog alarm:	Flag of door opened timeout
ISR(WDT_vect)
{
	waker = WAKER_WDT;
	wdtCounter++;
	wdt_disable();
}
/*The sleep function*/
void sleepNow(uint32_t sleepValue) {
	uint32_t startCounter = timer3counter;
	cli();//diable all the interrupts
	waker = WAKER_PCI;//reset waker
	//for an unpredictable long state of door closed
	if (!timer3counter)
	{
		if (sleepValue == DOOR_STATE_CLOSED)
		{
			attachInterrupt(digitalPinToInterrupt(FPCWAKEUP_PIN), FingerTouched, FALLING);
			attachInterrupt(digitalPinToInterrupt(DOORPST_PIN), DoorOpened, RISING);
		}
		//for an unpredictable long state of door opened
		else if (sleepValue == DOOR_STATE_OPENED)
		{
			setWatchDogTimer(WDT_8_SEC);//开门放狗, as a reminder for those who forget to close the door
			attachInterrupt(digitalPinToInterrupt(DOORPST_PIN), DoorClosed, FALLING);
		}
		//for choosing device configuration
		else if (sleepValue == SMODE_CONFIG_TOUCH) {
			setWatchDogTimer(WDT_4_SEC);
		}
		else //just taking a nap during certain mission
		{
			PCICR &= ~(1 << PCIE0);//Disable pin change interrupt used by fpc softserial
			setWatchDogTimer(sleepValue);//SLEEP FOR DEBOUNCING CAUSED BY DOOR SHAKING
		}
		EIFR |= 1 << (digitalPinToInterrupt(sleepValue == DOOR_STATE_OPENED ? DOORPST_PIN : FPCWAKEUP_PIN)); // clear flag for interrupts!!!!

		ADCSRA = 0; //disable ADC
		ADCSRB = 0;
		power_all_disable();// turn off various modules
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);//chose the most efficient power saving mode
	}
	else
	{
		set_sleep_mode(SLEEP_MODE_IDLE);
		power_adc_disable();
		power_spi_disable();
		power_twi_disable();
		power_usart0_disable();
		power_usart1_disable();
		power_usart2_disable();
		//power_usart3_disable();//Don't disable ESP8266 USART when communicating
		power_timer0_disable();
		power_timer1_disable();
		power_timer2_disable();
		//power_timer3_disable();//My led blink timer
		power_timer4_disable();
		power_timer5_disable();
	}
	sleep_enable();
	sei();//enable interrupts
	do
	{
		sleep_cpu();
		/* ++++++++The program will continue from here.+++++++++ */
		//the first thing to do from waking up is to cancel sleep as a precaution
		sleep_disable();
	} while (timer3counter && (getMillis(sleepValue) > (timer3counter - startCounter) * 20));

	power_all_enable();   // enable modules again
	PCICR = bit(PCIE0);  // Enable Interrupt D10 11 fpc software serial to detect fpc irq very time
	PCIFR = bit(PCIF0);   // clear any outstanding interrupts
}
//Sleep the device based on door state
void SleepDeviceOn(uint8_t doorState) {
	sleepFpc1020();
	powerOffEsp8266();
	wdtCounter = 0;
	do {
		if (wdtCounter > 7) NotifyUser(INFO_DOOR_ALERT);
		sleepNow(doorState);
		//dbgSerial.println(waker); dbgSerial.flush();
	} while (forgetLock());
	if (waker == WAKER_PCI) fpcasleep = 0;
	deviceState = doorState + 1;//prepare for next wake up loop
}
#pragma endregion

#pragma region Utility Funtions
//===============================================WHAT BELOW ARE HLEPER FUNCTIONS========================================================
uint32_t getMillis(uint32_t smode)
{
	switch (smode)
	{
	case WDT_16_MS:
		return 16;
	case WDT_32_MS:
		return 32;
	case WDT_64_MS:
		return 64;
	case WDT_128_MS:
		return 128;
	case WDT_256_MS:
		return 256;
	case WDT_512_MS:
		return 512;
	case WDT_1_SEC:
		return 1000;
	case WDT_2_SEC:
		return 2000;
	case WDT_4_SEC:
		return 4000;
	case WDT_8_SEC:
		return 8000;
	default:
		break;
	}
	return smode;
}
void setWatchDogTimer(uint8_t interval) {
	// clear various "reset" flags
	MCUSR = 0;
	// allow changes, disable reset
	WDTCSR = bit(WDCE) | bit(WDE);
	// set interrupt mode and an interval 
	WDTCSR = bit(WDIE) | interval;
	wdt_reset();  // pat the dog
}
uint8_t currentDoorState() { return  digitalRead(DOORPST_PIN) ? DOOR_STATE_OPENED : DOOR_STATE_CLOSED; }
uint8_t forgetLock() { return waker == WAKER_WDT && currentDoorState() == DOOR_STATE_OPENED; }
void WaitTillEmpty(uint8_t time)
{
	sleepNow(time);
	while (Esp8266Uart.available()) { Esp8266Uart.read(); }
}
void EmptyFpcUartBuffer() { while (FPC1020Uart.available()) FPC1020Uart.read(); }
uint8_t successWithin(GeneralIfOKFunction jobToFinish, unsigned long time)
{
	unsigned long start = millis();
	while (millis() - start < time)
	{
		if (jobToFinish())
		{
			return 1;
		}
	}
	return 0;
}
#pragma endregion

#pragma region ESP8266
//=======================================================ESP8266 Manipulation=============================================================
uint8_t sendPayloadViaHttp()
{
	uint8_t counter = 0;
	uint8_t length;
	char data[168];
	sprintf(data, "a0=%lu&b0=%lu&c0=%lu&d0=%lu&a1=%lu&b1=%lu&c1=%lu&d1=%lu&finger_id=%u&acted_at=%lu",
		payload.keyStates[0], payload.keyStates[1], payload.keyStates[2], payload.keyStates[3], payload.keyStates[4], payload.keyStates[5], payload.keyStates[6], payload.keyStates[7], payload.fingerId, payload.utime);
	length = strchr(data, '\0') - data;
	char recvbuffer[256] = { 0 };
	while (counter++ < 5)
	{
		Esp8266Uart.println(String("POST /record HTTP/1.1\r\nContent-Length: " + String(length)
			+ "\r\nContent-Type: application/x-www-form-urlencoded; charset=GBK\r\nHost: iot.sg-z.com\r\nConnection: close\r\n\r\n")
			+ String(data));
		if (recvStore("Motionlife-OK", recvbuffer, sizeof(recvbuffer), 1888))
		{
			NotifyUser(INFO_SERVER_GOTIT);
			return 1;
		}
	}
	return 0;
}
void powerOffEsp8266() {
	digitalWrite(ESP_POWER_SIGNAL, LOW);
	pinMode(ESP_POWER_SIGNAL, INPUT);
}
void powerOnEsp8266()
{
	if (digitalRead(ESP_POWER_SIGNAL) == LOW)
	{
		pinMode(ESP_POWER_SIGNAL, OUTPUT);
		digitalWrite(ESP_POWER_SIGNAL, HIGH);
		sleepNow(WDT_2_SEC);//wait for stable
	}
}
void restartEsp8266()
{
	digitalWrite(ESP_POWER_SIGNAL, LOW);
	sleepNow(WDT_128_MS);
	digitalWrite(ESP_POWER_SIGNAL, HIGH);
	sleepNow(WDT_2_SEC);//wait for stable
}
uint8_t exitFromTransMode()
{
	if (ATKick()) { return 1; }	//Check if already live
	Esp8266Uart.print("+++");
	Esp8266Uart.flush();
	sleepNow(WDT_256_MS);
	return ATKick();
}
uint8_t breakFrozen()
{
	if (exitFromTransMode()) { return 1; }
	if (successWithin(ATKick, 2888)) { return 1; }
	restartEsp8266();
	return exitFromTransMode();
}
uint8_t smartConfig()
{
	//Enable station dhcp now!
	if (deviceState != DEVICE_STATE_WIFIRESTORECONFIG)
	{
		ATCommand("AT+CWDHCP_CUR=1,1", "OK");
		if (successWithin(gotIpAddr, 5888)) return 1;
	}
	NotifyUser(INFO_AIRKISS_BEGIN);
	if (successWithin(AirKiss, 58888))
	{
		NotifyUser(INFO_AIRKISS_SUCCESS);
		return 1;
	}
	NotifyUser(INFO_AIRKISS_END);
	return 0;
}
uint8_t gotIpAddr()
{
	WaitTillEmpty(WDT_512_MS);
	Esp8266Uart.println("AT+CIPSTATUS");
	return recvFind(":2", ":3", ":4", 1588);
}
uint8_t setWifiWorkMode()
{
	uint8_t counter = 0;
	uint8_t success = 0;
	while (!success && counter++ < 3)
	{
		ATCommand("AT+CWMODE_DEF=1", "OK");
		ATCommand("AT+CIPMUX=0", "OK");
		ATCommand("AT+CIPMODE=1", "OK");
		success = ATCommand("AT+SAVETRANSLINK=1,\"121.41.82.159\",80,\"TCP\"", "OK");//for production
		//success = ATCommand("AT+SAVETRANSLINK=1,\"192.168.2.7\",80,\"TCP\"", "OK");//for local debug
	}
	ATCommand("AT+CIPCLOSE", "OK");
	return success;
}
uint8_t setStaticIp()
{
	//crucial GIVE HIM ENOUGH TIME TO GOT IP THROUGH
	if (!successWithin(gotIpAddr, 5888)) return 0;
	char buffer[158] = { 0 };					//MUST DO THE INITIALIZATION!!!YOU IDIOT!
	WaitTillEmpty(WDT_512_MS);
	Esp8266Uart.println("AT+CIPSTA?");
	if (recvStore("OK", buffer, sizeof(buffer), 1588))
	{
		char command[] = "AT+CIPSTA_DEF=";
		uint8_t cmd_len = strlen(command);
		char *ipend = strchr(buffer, 'g') - 10;
		char *wayend = strchr(buffer, 'n') - 10;
		char *maskend = strchr(buffer, 'O') - 4;
		WaitTillEmpty(WDT_512_MS);		//CRUCIAL:THERE MUST BE A PROPER DELAY BETWEEN ESP8266 SENDING AND PROCECING RECIEVING FROM ITS UART
		for (uint8_t i = 0; i < cmd_len; i++) Esp8266Uart.print(*(command + i));
		for (char *i = strchr(buffer, 'p') + 2; i < ipend; i++) Esp8266Uart.print(*i);
		Esp8266Uart.print(',');
		for (char *i = strchr(buffer, 'y') + 2; i < wayend; i++) Esp8266Uart.print(*i);
		Esp8266Uart.print(',');
		for (char *i = strchr(buffer, 'k') + 2; i < maskend; i++)  Esp8266Uart.print(*i);
		Esp8266Uart.println();
		return recvFind("OK", 1588);
	}
	return 0;
}
uint8_t UdpConn() { return ATCommand("AT+CIPSTART=\"UDP\",\"255.255.255.255\",8888,7777,0", "CONNECT") && ATCommand("AT+CIPSEND", ">"); }
uint8_t AirKiss() { return ATCommand("AT+CWSMARTSTART", "SUCCESS", 18888); }
uint8_t ATKick() { return ATCommand("AT", "\r\n"); }
uint8_t ATCommand(const char *command, const char *target, uint32_t timeout)
{	//CRUCIAL:THERE MUST BE A PROPER DELAY BETWEEN ESP8266 SENDING AND RECIEVING DATA THROUGH ITS UART
	WaitTillEmpty(WDT_512_MS);		//OTHERWIZE ESP8266 WILL DISPOSE THE RECIEVING DATA.
	Esp8266Uart.println(command);
	return recvFind(target, timeout);
}
/*
*	keep reading Rx untill find the target or time out; return whether got what expected or not
*/
uint8_t recvFind(const char *target, uint32_t timeout)
{
	uint8_t position = 0;
	char str[128] = { 0 };//must initialize otherwize you will get garbage;
	unsigned long start = millis();
	while (millis() - start < timeout) {
		while (Esp8266Uart.available() && position < 128) { *(str + position++) = Esp8266Uart.read(); }
		if (strstr(str, target) != NULL) { return 1; }
	}
	return 0;
}
uint8_t recvFind(const char *target1, const char *target2, const char *target3, uint32_t timeout)
{
	uint8_t position = 0;
	char str[128] = { 0 };
	unsigned long start = millis();
	while (millis() - start < timeout) {
		while (Esp8266Uart.available() && position < 128) { *(str + position++) = Esp8266Uart.read(); }
		if ((strstr(str, target1) != NULL) || (strstr(str, target2) != NULL) || (strstr(str, target3) != NULL)) { return 1; }
	}
	return 0;
}
uint8_t recvStore(const char *target, char *buffer, uint16_t len, uint32_t timeout)
{
	uint16_t position = 0;
	unsigned long start = millis();
	while (millis() - start < timeout)
	{
		while (Esp8266Uart.available() && position < len) { *(buffer + position++) = Esp8266Uart.read(); }
		if (strstr(buffer, target) != NULL) { return 1; }
	}
	return 0;
}
#pragma endregion

#pragma region FPC1020+
//=======================================================FPC1020+ Manipulation============================================================
/*******************************************************************
**Fucntion：Construct command and send then decode the response
**Parameters：cmd_type:  type of the command;
setting_userId:settings or user id.
mode_priority:set or read mode/the little end of user's priority
**Return: result of excution of the previous command Q3
*******************************************************************/
uint8_t Fpc1020(uint8_t cmd_type, uint8_t setting_userId, uint8_t mode_priority, uint16_t timeout)
{
	//First we construct the 8 bytes command.
	uint8_t statusCheck = 0;
	uint8_t cmd_buffer[8] = { 0xF5,cmd_type,0, setting_userId,mode_priority,0,0,0xF5 };
	uint8_t rx_buffer[8] = { 0 };
	for (uint8_t i = 1; i < 6; i++) cmd_buffer[6] ^= cmd_buffer[i];
	EmptyFpcUartBuffer();	//empty previous buffer to make sure the rx buffer is fresh and ready for the new coming
	FPC1020Uart.write(cmd_buffer, 8);
	//We can proccess the 8 bytes response from now on.
	uint8_t position = 0;
	unsigned long start = millis();
	while (millis() - start < timeout)
	{
		while (FPC1020Uart.available())
		{
			rx_buffer[position++] = FPC1020Uart.read();
			if (position == 8) {
				if (!statusCheck)
				{
					statusCheck++;
					uint8_t wakeupSpit[8] = { 0xF5,0x09, 0x02, 0xD0, 0xFF, 0x00, 0x24, 0xF5 };
					if (memcmp(rx_buffer, wakeupSpit, 8) == 0)
					{	//when wake up FPC1020 would spit out 'F5 09 02 D0 FF 00 24 F5' in some milliseconds before he can process any recieving command
						position = 0;
						FPC1020Uart.write(cmd_buffer, 8);
						break;
					}
				}
				if (rx_buffer[0] != 0xF5 || rx_buffer[1] != cmd_type || rx_buffer[7] != 0xF5) { return ACK_FAIL; }
				//(cmd_type == CMD_USERNUMB ||cmd_type == CMD_SETTIMEOUT || cmd_type == CMD_SETPAIRLEVEL) ? buffer[3] : 0;
				switch (cmd_type)
				{
				case CMD_USERNUMB:
					totalUsers = rx_buffer[3];
					break;
				case CMD_SEARCH:
					payload.fingerId = rx_buffer[3];
					break;
					//case CMD_SLEEP:
					//	//fpcasleep = 1;
					//	break;
					//case CMD_SETPAIRLEVEL:
					//	break;
					//case CMD_SETTIMEOUT:
					//	break;
					//default:
					//	break;
				}
				return rx_buffer[4];
			}
		}
	}
	return ACK_FAIL;
}
uint8_t sleepFpc1020()
{
	//since fpc1020 is quite stable there is no need to check result just send this command
	if (!fpcasleep)
	{
		if (successWithin(fpcSleepOK, 388)) fpcasleep = 1;
	}
	else
	{
		//this redundant to make sure fpc is asleep
		fpcasleep = ACK_SUCCESS == Fpc1020(CMD_SLEEP, 0, 0, 88);
	}
}
uint8_t fpcSleepOK() { return ACK_SUCCESS == Fpc1020(CMD_SLEEP, 0, 0, 188); }
void RecordById(String cmd)
{
	uint8_t usr_id = 0;
	uint8_t Q3 = ACK_FAIL;
	usr_id = cmd.substring(7).toInt();
	if (usr_id)
	{
		speakThroughWifi("RECORD BEGIN! STEP 1: PUT YOUR FINGER ON THE PAD!");
		BuzzeNumber(DOORINFO_LED,2);
		if ((Q3 = Fpc1020(CMD_ENROLL1, usr_id, 2, SMODE_FPC_ENROLL_TO)) == ACK_SUCCESS)
		{
			Esp8266Uart.println("STEP 2: OK, ONE MORE TIME!");
			NotifyUser(INFO_READ_FINGER_OVER);
			sleepNow(WDT_256_MS);
			if ((Q3 = Fpc1020(CMD_ENROLL2, usr_id, 2, SMODE_FPC_ENROLL_TO)) == ACK_SUCCESS)
			{
				Esp8266Uart.println("STEP 3: GOOD, LAST TIME");
				NotifyUser(INFO_READ_FINGER_OVER);
				sleepNow(WDT_256_MS);
				if ((Q3 = Fpc1020(CMD_ENROLL3, usr_id, 2, SMODE_FPC_ENROLL_TO)) == ACK_SUCCESS)
				{
					NotifyUser(INFO_READ_FINGER_OVER);
					Esp8266Uart.println("SUCCESS! THE NEW FINGER ID IS: " + String(usr_id));
					return;
				}
			}
		}
	}
	else
	{
		speakThroughWifi("FAIL! INVALID ID"); return;
	}
	NotifyUser(INFO_DOOR_ALERT);
	speakThroughWifi("FAIL, ERROR CODE:" + String(Q3) + "=>" + getErrorMessage(Q3));
}
void DeleteById(String cmd)
{
	uint8_t usr_id = 0;
	uint8_t Q3 = ACK_FAIL;
	usr_id = cmd.substring(7).toInt();
	if (usr_id)
	{
		if ((Q3 = Fpc1020(CMD_DELETE, usr_id, 0, 188)) == ACK_SUCCESS)
		{
			speakThroughWifi("DELETE SUCCESS, REMOVED ID: " + String(usr_id));
			return;
		}

	}
	else
	{
		speakThroughWifi("FAIL! INVALID ID"); return;
	}
	speakThroughWifi("FAIL, ERROR CODE:" + String(Q3) + "=>" + getErrorMessage(Q3));
}
String getErrorMessage(uint8_t q3)
{
	switch (q3)
	{
	case ACK_FAIL: return "EXECUTE FAIL";	//操作失败
	case ACK_FULL:return "DATABASE FULL";	//指纹数据库已满
	case ACK_NOUSER:return "USER NOT RECOGNIZED";//无此用户
	case ACK_USER_OCCUPIED:return "ID OCCUPIED";	//此ID用户已存在
	case ACK_USER_EXIST:return "USER EXISTED";	//用户已存在
	case ACK_TIMEOUT: return "EXECUTE TIMEOUT";//采集超时
	default:
		return "UNKNOWN ERROR";
	}
}
void speakThroughWifi(String content)
{
	sleepNow(WDT_256_MS);
	Esp8266Uart.println(content);
}
#pragma endregion

#pragma region DS1307
void getTime() {
	pinMode(RTC_POWER_PING, OUTPUT);
	digitalWrite(RTC_POWER_PING, HIGH);	//power on rtc
	Wire.begin();						// Start the I2C interface
										// find the time  
	//DateTime now = RTC.now();
	//snprintf(payload.time, sizeof(payload.time), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
	payload.utime = RTC.now().unixtime() - 3600 * 8;
	//FINISH WITH RTC
	digitalWrite(RTC_POWER_PING, LOW);//power off rtc
	pinMode(RTC_POWER_PING, INPUT);
	// turn off I2C
	TWCR &= ~(bit(TWEN) | bit(TWIE) | bit(TWEA));
	// turn off I2C pull-ups
	digitalWrite(SDA, LOW);
	digitalWrite(SCL, LOW);
}
#pragma endregion

#pragma region Keypad
void setKeypadpinMode(uint8_t begin)
{
	//set column pins as output if begin or input if end
	for (uint8_t i = 0; i < 15; i++)
	{
		pinMode(columnPinMap[i], begin ? OUTPUT : INPUT);
	}
	//set row pins as input
	for (uint8_t i = 0; i < 7; i++)
	{
		pinMode(rowPinMap[i], INPUT);
	}
	//Write all the pins to LOW
	for (uint8_t i = 22; i <= 43; i++)
	{
		digitalWrite(i, LOW);
	}
}
void scanKeyPad(uint8_t after)
{
	setKeypadpinMode(1);
	uint8_t num = 0;
	uint32_t temp[4] = { 0 };
	for (uint8_t i = 0; i < 15; i++)
	{
		digitalWrite(columnPinMap[i], HIGH);
		for (uint8_t j = 0; j < 7; j++)
		{
			if (digitalRead(rowPinMap[j]))
			{
				num = j * 15 + i;
				temp[num / 32] |= bit(31 - num % 32);
			}
		}
		digitalWrite(columnPinMap[i], LOW);
	}
	for (size_t i = 0; i < 4; i++)
	{
		payload.keyStates[after * 4 + i] = temp[i];
	}
	setKeypadpinMode(0);
}
#pragma endregion

#pragma region LockMotor
void openLock()
{
	pinMode(MOTORMINUS, OUTPUT);
	digitalWrite(MOTORMINUS, HIGH);
	digitalWrite(MOTORPLUS, LOW);//redundant?
	delay(77);
	digitalWrite(MOTORMINUS, LOW);
	pinMode(MOTORMINUS, INPUT);
}
void closeLock()
{
	pinMode(MOTORPLUS, OUTPUT);
	digitalWrite(MOTORPLUS, HIGH);
	digitalWrite(MOTORMINUS, LOW);//redundant?
	delay(66);
	digitalWrite(MOTORPLUS, LOW);
	pinMode(MOTORPLUS, INPUT);
}
#pragma endregion

#pragma region Signal System
//Basically it includes 1 Buzzer and  3 leds
void NotifyUser(uint8_t state)
{
	switch (state)
	{
	case INFO_DOOR_SUCCESS:
	case INFO_DOOR_ALERT:
		doorStateSignal(state);
		break;
	case INFO_DEVICE_START:
		deviceStartSignal();
		break;
	case INFO_SERVER_GOTIT:
		flashBuzzer(AIRKISS_LED, 0, 0);
		break;
	case INFO_PAD_TOUCH1ST:
	case INFO_PAD_TOUCH2ND:
	case INFO_PAD_TOUCH3RD:
		flashBuzzer(state == INFO_PAD_TOUCH1ST ? AIRKISS_LED : (state == INFO_PAD_TOUCH2ND ? FPC1020_LED : DOORINFO_LED), 2188, 0);
		break;
	case INFO_AIRKISS_BEGIN:
	case INFO_AIRKISS_SUCCESS:
	case INFO_AIRKISS_END:
		airkissSignal(state);
		break;
	case INFO_UDP_OK:
		BuzzeNumber(FPC1020_LED, 3);
		break;
	case INFO_READ_FINGER_OVER:
		flashBuzzer(DOORINFO_LED, 2188, 0);
		break;
	default:
		break;
	}
}
void ConfigBegin()
{
	uint8_t ledPin = (deviceState == DEVICE_STATE_WIFIFULLCONFIG || deviceState == DEVICE_STATE_WIFIRESTORECONFIG) ? AIRKISS_LED : FPC1020_LED;
	pinMode(ledPin, OUTPUT);
	Timer3.initialize(TIMER3_RESOLUTION);//20ms 
	timer3counter = 1;
	Timer3.attachInterrupt(normalBlink);
}
void ConifgEnd(uint8_t ledPin, int result)
{
	BuzzeNumber(ledPin, result);//may freeze here???
	Timer3.stop();
	Timer3.detachInterrupt();
	//delay(21);//this is the last resort, if it freeze again...
	timer3counter = 0;	//make sure timer3counter is 0 before exit from config
	digitalWrite(ledPin, LOW);
	pinMode(ledPin, INPUT);
}
void BuzzeNumber(uint8_t ledPin, uint8_t count)
{
	while (count)
	{
		count--;
		flashBuzzer(ledPin, 2188, 1);
		sleepNow(WDT_64_MS);
	}
}
void normalBlink(void)
{
	if (++timer3counter % 10) return;//very 20ms*10
	uint8_t ledPin = (deviceState == DEVICE_STATE_WIFIFULLCONFIG || deviceState == DEVICE_STATE_WIFIRESTORECONFIG) ? AIRKISS_LED : FPC1020_LED;
	digitalWrite(ledPin, !digitalRead(ledPin));
}
void fastBlink(void)
{
	if (++timer3counter % 2) return;//very 20ms*2
	digitalWrite(AIRKISS_LED, !digitalRead(AIRKISS_LED));
}
void airkissSignal(uint8_t state)
{
	switch (state)
	{
	case INFO_AIRKISS_BEGIN:
		Timer3.attachInterrupt(fastBlink);
		pinMode(AIRKISS_LED, OUTPUT);
		if (deviceState == DEVICE_STATE_FPCCONFIG)
		{
			digitalWrite(FPC1020_LED, HIGH);
		}
		break;
	case INFO_AIRKISS_SUCCESS:
		flashBuzzer(AIRKISS_LED, 2188, 1);//TODO: How to notify user most effectively when airkiss succeed?
	case INFO_AIRKISS_END:
		Timer3.attachInterrupt(normalBlink);//restore to 200ms blink
		if (deviceState == DEVICE_STATE_FPCCONFIG)
		{
			digitalWrite(AIRKISS_LED, LOW);
			pinMode(AIRKISS_LED, INPUT);
		}
		break;
	default:
		break;
	}
}
void doorStateSignal(uint8_t level)
{
	//because tone() use timer here we cannot use sleepNow
	tone(BUZZER_PIN, level == INFO_DOOR_ALERT ? 2888 : 2188, 180);
	pinMode(DOORINFO_LED, OUTPUT);
	//flash the led for 5 times
	for (uint8_t i = 0; i < 3; i++)
	{
		digitalWrite(DOORINFO_LED, HIGH);
		delay(64);
		if (level == INFO_DOOR_ALERT)
		{
			if (i == 1)
			{
				tone(BUZZER_PIN, 2888, 180);
			}
			digitalWrite(DOORINFO_LED, LOW);
		}
		delay(64);
	}
	digitalWrite(DOORINFO_LED, LOW);//power off led
	pinMode(DOORINFO_LED, INPUT);
	noTone(BUZZER_PIN);
}
void deviceStartSignal()
{
	flashBuzzer(AIRKISS_LED, 2188, 0);
	sleepNow(WDT_256_MS);
	flashBuzzer(FPC1020_LED, 2188, 0);
	sleepNow(WDT_256_MS);
	flashBuzzer(DOORINFO_LED, 2188, 0);
	openLock();
	sleepNow(WDT_512_MS);
	closeLock();
}
void flashBuzzer(uint8_t ledPin, uint16_t buzzerFreq, uint8_t pinmode)
{
	if (buzzerFreq) tone(BUZZER_PIN, buzzerFreq);
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);
	delay(128);
	digitalWrite(ledPin, LOW);
	if (!pinMode) pinMode(ledPin, INPUT);
	if (buzzerFreq) noTone(BUZZER_PIN);
}
#pragma endregion
