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
#define AIRKISS_LED			A8
#define FPC1020_LED			A9
#define INFO_LED			A10
#define RTC_POWER_PING		7
#define FPCSOFTUARTRX		10
#define FPCSOFTUARTTX		11
#define FPCWAKEUP_PIN		18
#define DOORPST_PIN			19
#define DEVICE_STATE_MODULESCONFIGRING	(0)
#define DEVICE_STATE_DOOROPENING		(1)
#define DEVICE_STATE_DATASENDING		(2)
#define WIFI_NO_RESPONSE	(0)		//not response
#define WIFI_RESPOND_NOIP	(1)		//can reply any AT command as expectly
#define WIFI_GOT_IP			(2)		//got an IP which wich can access Internet
#define WIFI_WORKMODE_OK	(3)		//set up working mode
#define WIFI_STATIC_OK		(4)		//config esp8266 with the static ip address.
#define DOOR_STATE_CLOSED	(0)
#define DOOR_STATE_OPENED	(1)
#define WAKER_PCI			(0)
#define WAKER_DOOR_OPEN		(1)
#define WAKER_FPC1020_IRQ   (2)
#define WAKER_DOOR_CLOSE	(3)
#define WAKER_WDT			(4)
#pragma endregion

#define Esp8266Uart Serial3
//#define FPC1020Uart Serial2
#define dbgSerial Serial

#pragma region Arduino
SoftwareSerial FPC1020Uart(FPCSOFTUARTRX, FPCSOFTUARTTX);
volatile uint8_t waker;				//could be changed in the ISR,0:by PCI;1:by EXT0 or EXT1;2:by WDT;
volatile uint8_t wdtCounter;
volatile uint8_t fpcasleep;
uint8_t deviceState;				//current device state,
uint8_t totalUsers;			//user finger_id or total number
Payload payload = { 0 };
uint8_t columnPinMap[] = { 22,24,26,28,30,32,34,37,36,39,38,41,40,43,42 };
uint8_t rowPinMap[] = { 23,25,27,29,31,33,35 };
RTC_DS1307 RTC;
// the setup function runs once when you press reset or power the board
void setup() {
	Esp8266Uart.begin(115200);
	dbgSerial.begin(115200);
	FPC1020Uart.begin(19200);
	dbgSerial.println("~~~START~~~");
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(13, OUTPUT); digitalWrite(13, LOW);		//DISABLE THIS LED TO SAVE MORE POWER
	chooseState();//Choose machine state when start up
}
// the loop function runs over and over again until power down or reset
void loop() {
	switch (deviceState) {
	case DEVICE_STATE_MODULESCONFIGRING://WHICH MEANS user wanna config the device, eg config ESP8266 OR FPC1020+
		ConfigPeripheralModules();
		break;
	case DEVICE_STATE_DOOROPENING://FPC1020+sending signal triger INT0, wake up me, about to open the door or not.
		DoorOpening();
		break;
	case DEVICE_STATE_DATASENDING://someone closed the door (wake up by INT1), about to sample and send data
		DataSending();
		break;
	}
	SleepDeviceOn(currentDoorState());
}
#pragma endregion

#pragma region LIFECYCLE
/*
Configure ESP8266 and FPC1020+ WHEN BOOTUP IF IT'S NECCESSARY
otherwise exit to working mode.
*/
void ConfigPeripheralModules() {
	Notify_User("Config WIFI");
	dbgSerial.flush();
	dbgSerial.println(WifiConfig());
	dbgSerial.flush();
	if (successWithin(UdpConn, 3188))
	{
		Notify_User("Config FPC1020");//also can broadcast the result of wifi config to user
		FPC1020Config();
		sleepFpc1020();
	}
	else
	{
		dbgSerial.println("UDP Connect Fail");
	}
	//power off esp8266
	powerOffEsp8266();
}
/*
This function was used to let user set wifi ssid and password through wechat AIR-KISS smartlink technology.
return 1 means that the Air Kiss works successfully, that is:
#1: The wifi module got its ip from a router with which it can use to access the Internet;
#2: The wifi module has been configured with a default working mode.
#3: The wifi has set up an UPD connection to listen to certern port with which can communicate with user
*/
uint8_t WifiConfig() {
	//1.ESPE266开机
	powerOnEsp8266();
	//2.ESPE266解冻
	if (!breakFrozen()) { return WIFI_NO_RESPONSE; }
	//3.ESPE266智能配置
	if (!smartConfig()) { return WIFI_RESPOND_NOIP; }
	//3.Setting for wifi normal working mode
	if (!setWifiWorkMode()) { return WIFI_GOT_IP; }
	//4.Set static ip for fast speed
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
	while (1)
	{
		if (Esp8266Uart.available())
		{
			ch = Esp8266Uart.read();
			// Process message when new line character is recieved
			if (ch == '\n')
			{

				if (cml.indexOf("auth") != -1) {
					Auth();
				}
				else if (cml.indexOf("record") != -1 || cml.indexOf("delete") != -1) {
					// Record or delete a user's fingerprint under a specified Id
					OperateById(cml);
				}
				else if (cml.indexOf("clear") != -1) {
					//Clear all users' fingerprints
					SpeakThroughWifi(Fpc1020(CMD_CLEAR, 0, 0, 188) != ACK_FAIL ? "Cleared" : "FAIL");
				}
				else if (cml.indexOf("all") != -1) {
					totalUsers = 0;
					//Query all the total number of the enrolled user
					SpeakThroughWifi(Fpc1020(CMD_USERNUMB, 0, 0, 188) != ACK_FAIL ? String(totalUsers) : "FAIL");
				}
				else if (cml.indexOf("quit") != -1) {
					SpeakThroughWifi("CONFIG END");
					break;
				}
				else {
					SpeakThroughWifi("WRONG COMMAND");
				}

				cml = ""; // Clear recieved buffer
			}
			else {
				cml += ch;
			}
		}
	}
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
			dbgSerial.println("==>Opened without fpc auth");
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
	uint8_t result = Fpc1020(CMD_SEARCH, 0, 0, 3888);
	sleepFpc1020();//sleep fpc right after finishing the auth part
	if (result != ACK_FAIL && result != ACK_NOUSER && !result != ACK_TIMEOUT > 0) {
		openLock();
		Notify_User("Door Open! Welcome, user:");
		dbgSerial.println(payload.fingerId);
		dbgSerial.flush();
		sleepNow(WDT_2_SEC);
		sleepNow(WDT_512_MS);
		closeLock();
	}
	else {
		Notify_User("Error Code:");
		dbgSerial.println(result);
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
	dbgSerial.println("-->Door locked!");
	getTime();
	scanKeyPad(1);
	uint8_t failures = EEPROM.read(0);
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
	powerOffEsp8266();
	//if the real processing time is long and the door opened again by a key, sleep wait for lock; you could check the level of D3 here
	//may be use only one switch to set both INT1 and PCI for handling door lock and open by a key.
	if (currentDoorState() == DOOR_STATE_OPENED)
	{
		Notify_User("-->Mission Completed, but door still opened");
	}
	else {
		Notify_User("-->Mission Completed!");
	}
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
}
//handling watchdog alarm:	Flag of door opened timeout
ISR(WDT_vect)
{
	waker = WAKER_WDT;
	wdtCounter++;
	wdt_disable();
}
/*The sleep function*/
void sleepNow(uint8_t doorState) {
	cli();//diable all the interrupts
	waker = WAKER_PCI;//reset waker
	if (doorState == DOOR_STATE_CLOSED)
	{
		attachInterrupt(digitalPinToInterrupt(FPCWAKEUP_PIN), FingerTouched, FALLING);
		attachInterrupt(digitalPinToInterrupt(DOORPST_PIN), DoorOpened, RISING);
	}
	else if (doorState == DOOR_STATE_OPENED)
	{
		setWatchDogTimer(WDT_8_SEC);//开门放狗, as a reminder for those who forget to close the door
		attachInterrupt(digitalPinToInterrupt(DOORPST_PIN), DoorClosed, FALLING);
	}
	else
	{
		PCICR &= ~(1 << PCIE0);//Disable pin change interrupt used by fpc softserial
		setWatchDogTimer(doorState);//SLEEP FOR DEBOUNCING CAUSED BY DOOR SHAKING
	}
	EIFR |= 1 << (digitalPinToInterrupt(doorState == DOOR_STATE_OPENED ? DOORPST_PIN : FPCWAKEUP_PIN)); // clear flag for interrupts!!!!

	ADCSRA = 0; //disable ADC
	ADCSRB = 0;
	power_all_disable();// turn off various modules
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);//chose the most efficient power saving mode
	sleep_enable();
	sei();//enable interrupts
	sleep_cpu();
	/* ++++++++The program will continue from here.+++++++++ */
	//the first thing to do from waking up is to cancel sleep as a precaution
	sleep_disable();
	power_all_enable();   // enable modules again
	PCICR = bit(PCIE0);  // Enable Interrupt D10 11 fpc software serial to detect fpc irq very time
	PCIFR = bit(PCIF0);   // clear any outstanding interrupts
}
//Sleep the device based on door state
void SleepDeviceOn(uint8_t doorState) {
	sleepFpc1020();
	wdtCounter = 0;
	do {
		if (wdtCounter > 4)
		{
			dbgSerial.println("Warning:Beeper!Not Lock!");
		}
		dbgSerial.println("Sleep Now!"); dbgSerial.flush();
		sleepNow(doorState);
		dbgSerial.print("Awake!waker="); dbgSerial.println(waker); dbgSerial.flush();
	} while (forgetLock());
	if (waker == WAKER_PCI) fpcasleep = 0;
	deviceState = doorState + 1;//prepare for next wake up loop
}
#pragma endregion

#pragma region Utility Funtions
//===============================================WHAT BELOW ARE HLEPER FUNCTIONS========================================================
void setWatchDogTimer(uint8_t interval) {
	// clear various "reset" flags
	MCUSR = 0;
	// allow changes, disable reset
	WDTCSR = bit(WDCE) | bit(WDE);
	// set interrupt mode and an interval 
	WDTCSR = bit(WDIE) | interval;
	wdt_reset();  // pat the dog
}
void chooseState() {
	//Make sure FPC1020+ is in sleep mode after machine start up 
	sleepFpc1020();
	deviceState = successWithin(isFPCTouched, 3888) ? DEVICE_STATE_MODULESCONFIGRING : DEVICE_STATE_DOOROPENING;
}
uint8_t currentDoorState() { return  digitalRead(DOORPST_PIN) ? DOOR_STATE_OPENED : DOOR_STATE_CLOSED; }
uint8_t forgetLock() { return waker == WAKER_WDT && currentDoorState() == DOOR_STATE_OPENED; }
void Notify_User(String msg)
{
	//tone(BUZZER_PIN, 888, 8);
	dbgSerial.println(msg);
}
void SpeakThroughWifi(String response)
{
	sleepNow(WDT_512_MS);			//CRUCIAL: THERE MUST BE A PROPER DELAY BETWEEN ESP8266 SENDING AND READIND DATA THROUGH ITS UART
	Esp8266Uart.println(response);	//OTHERWIZE IT WILL DISPOSE THE RECIEVING DATA. SEE setStaticIp		
	//tone(BUZZER_PIN, 888, 8);
}
void WaitTillEmpty(unsigned long time)
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
	dbgSerial.println("Data Sending...");
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
		if (recvStore("Motionlife-OK", recvbuffer, sizeof(recvbuffer), 1888)) {
			dbgSerial.println("One payload was sent out!");
			return 1;
		}
	}
	dbgSerial.println("Sending Failure!");
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
	if (successWithin(ATKick, 3888)) { return 1; }
	restartEsp8266();
	return exitFromTransMode();
}
uint8_t smartConfig()
{
	//Enable station dhcp now!
	ATCommand("AT+CWDHCP=1,1", "OK");
	if (successWithin(gotIpAddr, 6888)) return 1;
	digitalWrite(AIRKISS_LED, HIGH);//START AIR-KISS. TODO: NOTIFY USER NAVIGATE TO CONFIG PAGE!!!
	if (successWithin(AirKiss, 58888))
	{
		digitalWrite(AIRKISS_LED, LOW);
		return 1;
	}
	digitalWrite(AIRKISS_LED, LOW);
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
		//success = ATCommand("AT+SAVETRANSLINK=1,\"121.41.82.159\",80,\"TCP\"", "OK");//for production
		success = ATCommand("AT+SAVETRANSLINK=1,\"192.168.2.7\",80,\"TCP\"", "OK");//for local debug
	}
	ATCommand("AT+CIPCLOSE", "OK");
	return success;
}
uint8_t setStaticIp()
{
	//crucial GIVE HIM ENOUGH TIME TO GOT IP THROUGH
	if (!successWithin(gotIpAddr, 6888)) return 0;
	WaitTillEmpty(WDT_512_MS);
	char buffer[158] = { 0 };					//MUST DO INITIALIZATION!!!YOU IDIOT!
	Esp8266Uart.println("AT+CIPSTA?");
	if (recvStore("OK", buffer, sizeof(buffer), 1588))
	{
		char command[] = "AT+CIPSTA_DEF=";
		uint8_t cmd_len = strlen(command);
		char *ipend = strchr(buffer, 'g') - 10;
		char *wayend = strchr(buffer, 'n') - 10;
		char *maskend = strchr(buffer, 'O') - 4;
		WaitTillEmpty(WDT_512_MS);		//CRUCIAL:THERE MUST BE A PROPER DELAY BETWEEN ESP8266 SENDING AND RECIEVING DATA THROUGH ITS UART
									//OTHERWIZE ESP8266 WILL DISPOSE THE RECIEVING DATA. SEE SpeakThroughWifi
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
{
	//WaitTillEmpty(688);	//CRUCIAL:THERE MUST BE A PROPER DELAY BETWEEN ESP8266 SENDING AND RECIEVING DATA THROUGH ITS UART
	WaitTillEmpty(WDT_512_MS);		//OTHERWIZE ESP8266 WILL DISPOSE THE RECIEVING DATA. SEE SpeakThroughWifi
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
uint8_t isFPCTouched() {
	//when wake up FPC1020 would spit out 'F592D0FF024F5'
	return FPC1020Uart.available();
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
		fpcasleep = ACK_SUCCESS == Fpc1020(CMD_SLEEP, 0, 0, 68);
	}
}
uint8_t fpcSleepOK() { return ACK_SUCCESS == Fpc1020(CMD_SLEEP, 0, 0, 188); }
void OperateById(String cmd)
{
	uint8_t usr_id = 0;
	uint8_t index = 0;
	uint8_t Q3 = ACK_FAIL;

	if ((index = cmd.indexOf("record")) != -1)												//Record New User
	{
		usr_id = cmd.substring(index + 7).toInt();
		SpeakThroughWifi("PUT YOUR FINGER ON THE PAD");
		if ((Q3 = Fpc1020(CMD_ENROLL1, usr_id, 2, 3888)) == ACK_SUCCESS)
		{
			SpeakThroughWifi("OK, ONE MORE TIME");
			if ((Q3 = Fpc1020(CMD_ENROLL2, usr_id, 2, 3888)) == ACK_SUCCESS)
			{
				SpeakThroughWifi("GOOD, LAST TIME");
				if ((Q3 = Fpc1020(CMD_ENROLL3, usr_id, 2, 3888)) == ACK_SUCCESS)
				{
					SpeakThroughWifi("RECORD SUCCESS, FINGER ID: " + usr_id);
					return;
				}
			}
		}
	}
	else if ((index = cmd.indexOf("delete")) != -1)										//Delete User
	{
		usr_id = cmd.substring(index + 7).toInt();
		if ((Q3 = Fpc1020(CMD_DELETE, usr_id, 0, 188)) == ACK_SUCCESS)
		{
			SpeakThroughWifi("DELETED OK, FINGER ID: " + usr_id);
			return;
		}
	}
	SpeakThroughWifi(Q3 == ACK_FAIL ? "JOB FAIL CODE:" + Q3 : "SUCCESS");
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
	digitalWrite(20, LOW);
	digitalWrite(21, LOW);
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
	delay(78);
	digitalWrite(MOTORMINUS, LOW);
	pinMode(MOTORMINUS, INPUT);
}
void closeLock()
{
	pinMode(MOTORPLUS, OUTPUT);
	digitalWrite(MOTORPLUS, HIGH);
	digitalWrite(MOTORMINUS, LOW);//redundant?
	delay(68);
	digitalWrite(MOTORPLUS, LOW);
	pinMode(MOTORPLUS, INPUT);
}
#pragma endregion
