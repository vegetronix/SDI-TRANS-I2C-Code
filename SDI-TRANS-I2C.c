// SdiTranslator.c

/*************************************************
Copyright 2023, Vegetronix, Inc.
Project: SDI-12 Translator FRAM Processor Version
*************************************************/

/************************************************
Hardware and Compiler:

Processor: MSP430FR2355
XTAL Speed: 32.768KHz
MCLK Speed: 24MHz.

Compiler  CCS
Level of Printf Support:  MINIMAL.

VERSIONS:
	4.0 - 1/1/2023 - new FRAM processor version.
	4.1 - 10/7/2024 - changed rx/tx buffer sizes from 64 to 128 bytes. Fixed the INFO section in the linker command file, to proper syntax to remove warning.
	4.2 - 9/5/2026 - Added Full transparent mode TRANSPARENT ON/OFF.
	5.0 - 11/1/2027 - I2C Functionality. 
*/
/************ Includes ********************/
#include "SysDep.h"
#include "Comm2X.h"
#include "FlashFR2X.h"
#include <stdio.h>
#include <string.h>



/************Defines***********************/
#define STR_HELLO "\r\n\n\n\nCopyright 2026 -  Vegetronix, Version: "

#define XTAL_32768

#define VERSION	"4.20"


#define STX		0x02
#define ETX 	0x03
#define ACK 	0x06
#define DLE 	0x10
#define ESC 	0x1B
#define XON 	0x11
#define XOFF	0x13
#define NAK		0x15


#define NO_SENSOR		0xFF	// valid sensors are 0 to 0x7F. 

#define FL_MEASURE_TYPE				0x03
#define FL_USE_CRC					0x04

#define MEASURE_TYPE_NORMAL			(0)
#define MEASURE_TYPE_CONCURRENT		(1)
#define MEASURE_TYPE_CONTINUOUS		(2)	// I haven't tested this option out yet!!!!!!!!!!!!!!!!


#define PHASE_CONCURRENT_INIT		0
#define PHASE_CONCURRENT_WAIT		1


typedef struct t_rx {
	WORD head;								// oldest data is pulled from head
	WORD tail;								// newest data is pushed to tail. 
	BYTE delimeter;							// packet delimeter
	BYTE delimeter_count;					// increment every time we receive a packet delimeter.
	BYTE pkt_size;							// 
	WORD overflow_count;
	BYTE er_reg;							// save URCTL1
	char buf[RX_BUFFER_SIZE];				// FIFO circular queue storing the incomming bytes
} Rx;

/* Transmitter */
typedef struct t_tx {
	char buf[TX_BUFFER_SIZE]; 		// circular queue. 
	WORD head;
	WORD tail;
} Tx;

//****************** Function Prototypes *******************************

DWORD gSecs;					// global time, since powered on.  

typedef struct t_capture {		// capture TAR
	WORD	sdi;				// general purpose SDI-12 timer - for break and marks, etc.
	WORD	inactive_tar;		// number of tics (.5ms) since sensor sent last byte
	WORD	retry;				// .5ms	
} Capture;

Capture gCapture;

BOOL gDoHeart;

typedef struct t_sleep_capture {		// capture TAR
	WORD	tar;		// number of tics since last non-idle operation. 
} SleepCapture;

SleepCapture gSleepCapture;

BOOL gOkayOn;
BOOL gIsTransparent;



//******** Host commands **********************************
#define CMD_PING		0x00
#define CMD_ADD_SENSOR	0x01	// address,sample period>10, 0x00 if none, 
#define CMD_DEL_SENSOR  0x02	// 
#define CMD_MEASURE		0x03	// immediately start measurement a sensor,address,



#define SDI_TX_BUF_SIZE		48
#define	SDI_RX_BUF_SIZE 	76 // the max length of a D command is 75, and we need room for a null terminator


#define SDI_CMD_NONE				0
#define SDI_CMD_ACK					1
#define SDI_CMD_ADDRESS_CHANGE		2
#define SDI_CMD_IDENT				3
#define SDI_CMD_QUERY_ADDRESS		4
#define SDI_CMD_MEASURE				5
#define SDI_CMD_DATA				6
#define SDI_CMD_VERIFY				7
#define SDI_CMD_CONCURRENT			8	// no service request issued. 
#define SDI_CMD_CONTINUOUS			9	// data immediately available
#define SDI_CMD_TRANSPARENT			10
//#define SDI_CMD_EXTENDED			11



#ifdef XTAL_32768
// Times are in 1/4096 seconds which is TB0_HZ.
#define TIME_12p5ms			(51)
#define TIME_8p3ms			(34)
#define TIME_10ms			(41)
#define TIME_MARK			(TIME_10ms)		// The marking time is 8.33 min - I was too close to 8.33 on rev V2.0
#define TIME_16p7ms			(68)
#define TIME_100ms			(410)			// timeout between received characters
#define TIME_CHAR			TIME_8p3ms		// 1200baud - 10 bits = 8.33ms
#define TIME_8ms			(33)
#define TIME_87ms			(356)
#define TIME_112p5ms		(461)

#else //96KHz
// Times are in 1/12000 seconds 
#define TIME_12p5ms			(150)
#define TIME_8p3ms			(100)
#define TIME_16p7ms			(200)
#define TIME_100ms			(1200)			// timeout between received characters
#define TIME_CHAR			TIME_8p3ms		// 1200baud - 10 bits = 8.33ms
#define TIME_8ms			(96)
#define TIME_87ms			(1044)
#define TIME_112p5ms		(1350)
#endif

#define ST_SDI_IDLE				0
#define ST_SDI_WAIT_CMD			1
#define ST_SDI_BREAK_SEND		2
#define ST_SDI_BREAK_WAIT		3
#define ST_SDI_MARK_SEND		4
#define ST_SDI_MARK_WAIT		5
#define ST_SDI_TRANSMIT			6
#define ST_SDI_RECEIVE			7
#define ST_SDI_CHECK_RETRIES	8


#define LOW			0
#define HIGH		1
#define UART_TX		2
#define UART_RX		3



typedef struct sdi_t {
	BYTE idx;				// index into the sensor/measurement array. 
	BYTE state;
	BYTE addr;				// SDI address of current sensor. (ASCII char, 0-9,a-z,A-Z)
	BYTE new_addr;			// new SDI address
	BYTE measurement;		// current measurement
	BOOL cmd_ready;
	BOOL busy;				// the sdi bus is busy 
	BOOL inactive_87ms;	// no character has been recieved for > 87ms.
	char retry_inner;		// inner retry counter
	char retry_outer;		// outer retry counter
	BYTE cur_cmd;			// current command being executed.
	BOOL use_crc;			// use CRC on current command.
	BOOL received_char;		// we've received the first character from a sensor response. 
	BOOL response_ready;	// we've received a response from the sensor - SNICK NOT USED
	
	int d_count;			// current data command
	
	BYTE rx_buf[SDI_RX_BUF_SIZE];		//incoming data
	BOOL rx_done;		//
	BOOL rx_error; 	// something wrong with received data
	
	BYTE tx_buf[SDI_TX_BUF_SIZE];		// outgoing command
	
	BOOL wait_service_request; // set by commands that need to wait for a service request. 
	int measurement_secs;   // time count for current measurement;
	int measurement_time;	// returned by the sdi measurement command, tells us the maximum time to wait for a service request. 
	int num_values;		// returned by the sdi measurement command, tells us how many entries to retrive using D command
	int values_count; 	// how many values have been retrieved from the current sequence of D commands. 
	WORD no_response; 	//never received an SDI response from the sensor
	BOOL echo;		// echo back everything on the SDI bus. 
	
	
} Sdi;

Sdi gSdi;

/************ Function Prototypes *********/

void Sleep();

//unsigned long MakeLong(BYTE b0,BYTE b1,BYTE b2,BYTE b3);
//unsigned short MakeShort(BYTE b0,BYTE b1);
//float MakeFloat(BYTE b0,BYTE b1,BYTE b2,BYTE b3);
short min(short a, short b);


void LedInit();
void LedSequence();

void TimerB0Init();
void TimerB1Init();

void SdiProc();
void SdiDataLine(BYTE val);
void SdiLoadCmd(BYTE cmd);
void SdiDirectCmd(char* str,BYTE cmd);
void SdiClearCmd();
void SdiCrc(BYTE *buf,int len,BYTE* crcbuf);
BOOL SdiCrcCheck(BYTE *buf);
BOOL SdiExpectingCRC();
BOOL SdiAddrValid(char addr);


BOOL SensorIsActive(int idx);
void SensorImmediate(int idx);
void SensorClear(int idx);

void SensorsTimeProc();
BOOL SensorsFind(int addr,int measurement,int *sensor);
BYTE SensorsGetNext();
BOOL SensorsAdd(int addr,int measurement,int period,int use_crc,int type);
BOOL SensorsDel(int addr,int measurement);
int SensorsNumActive();
void SensorsStart(BOOL on);
void SensorsProc();

BOOL CaptureExpired(WORD tar_old,WORD interval);
//BOOL Inactive87ms();

void CmdProc();

/***********************Global Variable*****************/


/**************** Interrupt ISRs ****************************/

#pragma vector= unused_interrupts
__interrupt void ISR_trap()
{
  // the following will cause an access violation which results in a PUC reset
  WDTCTL = 0;
}


#pragma vector = TIMER0_B0_VECTOR
__interrupt void  TIMER0_B0_ISR() {
	//LED_OUT^= (LED_3);
	WATCHDOG_CLEAR;
	gSecs++;
	SensorsTimeProc();
	LPM3_EXIT;
}

#pragma vector = TIMER0_B1_VECTOR
__interrupt void  TIMER0_B1_ISR() {}


/******************************************** TIMER_A Functions ******************************/
void TimerB0Init(){
	TB0CTL = TBSSEL__ACLK | ID_3 | MC0 ;	// set the clock source to ACLK, set clock divider to clk/8, enable the interrupt generator
	TB0CCTL0= CCIE;					// CC interrupt 0.
	TB0CCR0 = (TB0_HZ-1);	// interrupt once per second
	gSecs= 0;
}

void TimerB1Init(){
}

/************************** Capture Functions ****************************/

BOOL CaptureExpired(WORD tar_old,WORD interval) {
	WORD tar_now;
	WORD elapsed;
	
	tar_now= TB0R;
	if(tar_now >= tar_old) {
		elapsed= tar_now-tar_old;
	} else {	// handle wrap around. 
		elapsed= (TB0_HZ-tar_old);
		elapsed+= tar_now;
	}
	if(elapsed >= interval) return true;
	else return false;
}

/*************** System Functions **************************/

/*void OscillatorInit(BOOL clocks_to_pins) {
    FRCTL0 = FRCTLPW | NWAITS_2 ;

    __bis_SR_register(SCG0);                     // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                    // Set XT1 as FLL reference source
    CSCTL0 = 0;                                  // clear DCO and MOD registers
    CSCTL1 |= DCORSEL_7;                         // Set DCO = 24MHz
    CSCTL2 = FLLD_0 + 731;                       // DCOCLKDIV = 24MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                     // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));   // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;   // set REFOCLK (~32768Hz) as ACLK source, ACLK = 32768Hz
                                                 // default DCOCLKDIV as MCLK and SMCLK source
    if(clocks_to_pins) {
        P1DIR |= 0x03;                          // set ACLK SMCLK pin as output
        P1SEL1 |= 0x03;                         // set ACLK and  SMCLK pin as second function
    }
    PM5CTL0 &= ~LOCKLPM5;                       // Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
}*/

void OscillatorInit(BOOL clocks_to_pins) {
    FRCTL0 = FRCTLPW | NWAITS_2 ;

    P2SEL1 |= BIT6 | BIT7;                       // P2.6~P2.7: crystal pins
    do {
		CSCTL7 &= ~(XT1OFFG | DCOFFG);           // Clear XT1 and DCO fault flag
		SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);                   // Test oscillator fault flag
    __bis_SR_register(SCG0);                     // disable FLL
    CSCTL3 |= SELREF__XT1CLK;                    // Set XT1 as FLL reference source
    CSCTL0 = 0;                                  // clear DCO and MOD registers
    CSCTL1 |= DCORSEL_7;                         // Set DCO = 24MHz
    CSCTL2 = FLLD_0 + 731;                       // DCOCLKDIV = 24MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                     // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));   // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;     // set REFOCLK (~32768Hz) as ACLK source, ACLK = 32768Hz
                                                 // default DCOCLKDIV as MCLK and SMCLK source
    if(clocks_to_pins) {
        P1DIR |= 0x03;                          // set ACLK SMCLK pin as output
        P1SEL1 |= 0x03;                         // set ACLK and  SMCLK pin as second function
    }
    PM5CTL0 &= ~LOCKLPM5;                       // Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
}




void DelayMs(WORD num_ms) { // This functions delays about num_ms milliseconds.
	WORD i;

	for(i= 0;i < num_ms; i++) {
		__delay_cycles(MCLK_HZ/1000-10); // -10 is for loop overhead.
		WATCHDOG_CLEAR;
	}
}

void Save() {
	int i;
	gFlashVars.num_sensors= gSensors.num_sensors;
	gFlashVars.transparent_mode= gIsTransparent;
	for(i=0; i < gSensors.num_sensors; i++) {
		gFlashVars.sensors[i].addr=			gSensors.sensors[i].addr;
		gFlashVars.sensors[i].flags=			gSensors.sensors[i].flags;
		gFlashVars.sensors[i].measurement=	gSensors.sensors[i].measurement;
		gFlashVars.sensors[i].period=		gSensors.sensors[i].period;
	}
	FlashMspVarsSave();
}


/******************************************** LED Functions ******************************/

void LedInit() {
	LED_DIR|= LED_ALL;
	LED_SEL0&= ~LED0;
	LED_SEL1&= ~LED1;
}

void LedSequence() { //blink 3 times. 
	int i;
	
	for(i=0;i<3;i++) {
		LED_OUT|= (LED_ALL); // on
		DelayMs(200);
		LED_OUT&= ~LED_ALL;	// off
		DelayMs(200);
	}
}

/*void TriggerPulse() {
	int i;
	LED_OUT|= LED_3; // on
	for(i=0;i<5;i++) CLEAR_WATCHDOG;;
	LED_OUT&= ~LED_3; // off 
}*/


/******************************************** SDI Sensor Functions ******************************/

void SensorClear(int idx) {
	gSensors.sensors[idx].measurement= 	0;
	gSensors.sensors[idx].period= 		0;
	gSensors.sensors[idx].period_concurrent= 0;
	gSensors.sensors[idx].flags= 		(0|MEASURE_TYPE_NORMAL);
	gSensors.sensors[idx].sec= 			0;
	gSensors.sensors[idx].immediate= 	false;
	gSensors.sensors[idx].phase= 		PHASE_CONCURRENT_INIT;
}

BOOL SensorIsActive(int idx) {
	return (gSensors.sensors[idx].period > 0);
}

BOOL SensorReadyToMeasure(int idx) {
	if(!SensorIsActive(idx)) return false;
	if(gSensors.sensors[idx].phase == PHASE_CONCURRENT_WAIT) {
		if( gSensors.sensors[idx].sec >= gSensors.sensors[idx].period_concurrent ) return true;
	} else {
		if( gSensors.sensors[idx].sec >= gSensors.sensors[idx].period ) return true;
	}
	return false;
}

void SensorImmediate(int idx) { // bumps up the sensor so it is ready to be sampled. 
	gSensors.sensors[idx].immediate= true;
}

/************************** Sensors Array Functions ****************************/
void SensorsTimeProc() {
	int i;
	if(gSensors.run) {
		for(i=0;i<gSensors.num_sensors;i++) {
			if(SensorIsActive(i)) gSensors.sensors[i].sec++;
			WATCHDOG_CLEAR;
		}
	}
	if(gSdi.wait_service_request) {
		gSdi.measurement_secs++;
	}
}

BOOL SensorsAdd(int addr,int measurement,int period,int use_crc,int type) {
	// always delete and shift down, so new is at end. 
	int n= gSensors.num_sensors;
	if(gSensors.num_sensors < MAX_SENSORS) {
		gSensors.sensors[n].addr= 			addr;
		gSensors.sensors[n].measurement= 	measurement;
		gSensors.sensors[n].period= 		period;
		gSensors.sensors[n].period_concurrent= 		0;
		gSensors.sensors[n].flags= type;		// clears the FL_USE_CRC	
		if(use_crc) {
			gSensors.sensors[n].flags|= 	FL_USE_CRC;
		}
		gSensors.sensors[n].phase= 			PHASE_CONCURRENT_INIT;
		gSensors.num_sensors++;
		return true;
	} else { // full 
		return false;
	}
}

BOOL SensorsDel(int addr,int measurement) {
	int i,j;
	for(i=0;i<gSensors.num_sensors;i++) {
		WATCHDOG_CLEAR;
		if((gSensors.sensors[i].addr==addr)&&(gSensors.sensors[i].measurement==measurement)) {
			for(j=i;j<gSensors.num_sensors;j++) { // shift down, overwrite found item.
				gSensors.sensors[j].addr= 			gSensors.sensors[j+1].addr;
				gSensors.sensors[j].measurement=	gSensors.sensors[j+1].measurement;
				gSensors.sensors[j].period= 		gSensors.sensors[j+1].period;
				gSensors.sensors[j].period_concurrent= gSensors.sensors[j+1].period_concurrent;
				gSensors.sensors[j].flags= 			gSensors.sensors[j+1].flags;
				gSensors.sensors[j].phase= 			gSensors.sensors[j+1].phase;
			}
			gSensors.num_sensors--;
			SensorClear(gSensors.num_sensors);
			return true;
		}
	}
	return false;
}

BOOL SensorsFind(int addr,int measurement,int *sensor) {
	int i;
	for(i=0;i<gSensors.num_sensors;i++) {
		WATCHDOG_CLEAR;
		if((gSensors.sensors[i].addr==addr)&&(gSensors.sensors[i].measurement==measurement)) {
			*sensor= i;
			return true;
		}
	}
	*sensor= NO_SENSOR;
	return false;
}

BYTE SensorsGetNext() {
	int i;
	for(i=0;i<gSensors.num_sensors;i++) { // check the immediate first!
		WATCHDOG_CLEAR;
		if(gSensors.sensors[i].immediate) {
			gSensors.sensors[i].immediate= false;
			return i;
		}
	}
	for(i=0;i<gSensors.num_sensors;i++) {
		WATCHDOG_CLEAR;
		if(SensorReadyToMeasure(i)) return i;
	}
	return NO_SENSOR;
}

void SensorsStart(BOOL on) {
	gSensors.run= on;
	if(!on) {
		gSdi.busy= false; // good way to clear up the bus. 
	}
}

int SensorsNumActive() {
	int i;
	int count;
	for(i=0;i<gSensors.num_sensors;i++){
		WATCHDOG_CLEAR;
		if(SensorIsActive(i)) count++;
	}
	return count;
}


void SensorsProc() {
	int next;

	// make sure the sdi transaction has fully completed and is ready for new command
	if((!gSdi.busy) && (gSdi.state==ST_SDI_WAIT_CMD)) {
		next= SensorsGetNext();
		if(next!=NO_SENSOR) {
			gSdi.idx= next;
			gSdi.addr= gSensors.sensors[next].addr;
			gSdi.measurement= gSensors.sensors[next].measurement;
			gSdi.use_crc= (gSensors.sensors[next].flags&FL_USE_CRC);
			gSensors.sensors[next].sec= 0;		
			switch(gSensors.sensors[next].flags & FL_MEASURE_TYPE) {
				case MEASURE_TYPE_NORMAL:
					SdiLoadCmd(SDI_CMD_MEASURE);
				break;
				case MEASURE_TYPE_CONCURRENT:
					if(gSensors.sensors[next].phase == PHASE_CONCURRENT_INIT) {
						SdiLoadCmd(SDI_CMD_CONCURRENT);	// send concurrent command first
					} else {
						SdiLoadCmd(SDI_CMD_DATA);
						gSensors.sensors[next].phase= PHASE_CONCURRENT_INIT;
					}
				break;
				case MEASURE_TYPE_CONTINUOUS:
					SdiLoadCmd(SDI_CMD_CONTINUOUS);
				break;
			}
			gSdi.busy= true;
		}
	} else {
		if(gSdi.wait_service_request) {
			if(gSdi.measurement_secs>=gSdi.measurement_time) {
				gSdi.wait_service_request= false;
				SdiLoadCmd(SDI_CMD_DATA);
			}			
		}
	}
}

void SdiCrc(BYTE *buf,int len,BYTE* crcbuf) {
	// This function will determine the CRC from the packet that will be sent back to the SDI master.
	WORD CRC_INT = 0x0000;
	int i,j;

	for(i=0;i<len;i++){
		WATCHDOG_CLEAR;
		CRC_INT ^= buf[i];
		for(j=0;j<8;j++){
			if(CRC_INT & 0x0001){
				CRC_INT = CRC_INT>>1;		
				CRC_INT ^= 0xA001;
			}else{
				CRC_INT= CRC_INT>>1;			
			}		
		}
	}
	crcbuf[0]= 0x40 | (CRC_INT>>12);
	crcbuf[1]= 0x40 | ((CRC_INT>>6) & 0x3F);
	crcbuf[2]= 0x40 | (CRC_INT & 0x3F);
	crcbuf[3]= 0x00;
}

BOOL SdiCrcCheck(BYTE *buf) {
	int i,len;
	BYTE crcbuf[4];
	
	for(i=0;i<SDI_RX_BUF_SIZE-3;i++) {
		WATCHDOG_CLEAR;
		if((buf[i]==0) || (buf[i]=='\r')  || (buf[i]=='\n')) {
			len= i-3; // last 3 bytes are expected to be CRC. 
			break;
		}
	}
	SdiCrc(buf,len,crcbuf);

	if(buf[len]!=crcbuf[0]) return false;
	if(buf[len+1]!=crcbuf[1]) return false;
	if(buf[len+2]!=crcbuf[2]) return false;
	return true;
	
}

/************************** SDI Functions ****************************/


void SdiInit() {
	SDI_OUT&= ~SDI_OUTS;	//drop it low briefly
	SDI_DIR|= SDI_OUTS;
	SDI_SEL0&= ~DIR_OUT;
	SDI_SEL1&= ~DIR_OUT;
	SdiDataLine(UART_RX);

	gSdi.state= ST_SDI_WAIT_CMD;
	gSdi.inactive_87ms= true;         // make sure it starts with a break;
	gSdi.addr= '0';
	gSdi.new_addr= '0';
	gSdi.busy= false;
	gSdi.cmd_ready= false;

	gCapture.retry= TB0R;
	gSdi.rx_error = false;
	gSdi.rx_done= false;
	//gSdi.m_count= 0;
	gSdi.received_char= false;
	gSdi.response_ready= false;
	
	gSdi.wait_service_request= false;
	gSdi.measurement_time= 0;
	gSdi.measurement_secs=0;
	gSdi.num_values= 0;
	gSdi.values_count= 0;
	
	gSdi.no_response= 0;		// number of failed responses.
	
	gSdi.use_crc= false;
	SensorsStart(false);
}

BOOL SdiAddrValid(char addr) {
	if( (('0'<=addr)&&(addr<='9')) || (('a'<=addr)&&(addr<='z')) || (('A'<=addr)&&(addr<='Z')) ) {
		return true;
	} else return false;
}


void SdiDataLine(BYTE val) {
	// switch sdi data line between uart and break mode.
	switch(val) { // goes through an inverting buffer.
		case LOW:
			UCA1CTLW0|= UCSWRST;		// UART OFF
			SDI_OUT&= ~DIR_OUT;		// enable 
			SDI_OUT|= TX_A1;		// low
			SDI_SEL0&= ~TX_A1;		// GPIO
		break;
		case HIGH:
			UCA1CTLW0|= UCSWRST;		// UART OFF
			SDI_OUT&= ~DIR_OUT;		// enable 
			SDI_OUT&= ~TX_A1;		// high
			SDI_SEL0&= ~TX_A1;		// GPIO
			
		break;
		case UART_TX:
			LED_OUT|= 	LED0;
			SDI_OUT&= 	~DIR_OUT;		// enable
			SDI_SEL0|= 	TX_A1;			// UART TX pin
			UCA1CTLW0&= ~UCSWRST;		// UART ON
			UCA1IE|= 	UCTXIE;			// must come after UCSWRST
		break;
		case UART_RX: // RX
			LED_OUT&= ~LED0;
			SDI_OUT|= 	DIR_OUT;		// tristate.
			UCA1CTLW0|= 	UCSWRST;	// UART OFF
			UCA1CTLW0&=  ~UCSWRST;		// UART ON  // Resets all uart flags and regs
			UCA1IE|= 	UCRXIE;			// UART RX_IE ENABLED
		break;
	}
}



void SdiDirectCmd(char* str,BYTE cmd) {
	strcpy((char*)gSdi.tx_buf,str);
	gSdi.cur_cmd= cmd;
	gSdi.cmd_ready= true; // Tell SdiProc that we have a command to send. 
}

void SdiLoadCmd(BYTE cmd){
	gSdi.cur_cmd= cmd;
	switch(cmd) {
		case SDI_CMD_ACK:
			sprintf((char*)gSdi.tx_buf,"%c!",gSdi.addr);
		break;
		case SDI_CMD_ADDRESS_CHANGE:
			sprintf((char*)gSdi.tx_buf,"%dA%c!",gSdi.addr,gSdi.new_addr);
		break;
		case SDI_CMD_IDENT:
			sprintf((char*)gSdi.tx_buf,"%cI!",gSdi.addr);
		break;
		case SDI_CMD_QUERY_ADDRESS:
			sprintf((char*)gSdi.tx_buf,"?!");
		break;
		case SDI_CMD_MEASURE:
			if(gSdi.use_crc) {
				if(gSdi.measurement==0) {
					sprintf((char*)gSdi.tx_buf,"%cMC!",gSdi.addr);	
				} else {
					sprintf((char*)gSdi.tx_buf,"%cMC%d!",gSdi.addr,gSdi.measurement);
				}
			} else {
				if(gSdi.measurement==0) {
					sprintf((char*)gSdi.tx_buf,"%cM!",gSdi.addr);	
				} else {
					sprintf((char*)gSdi.tx_buf,"%cM%d!",gSdi.addr,gSdi.measurement);
				}
			}
		break;
		case SDI_CMD_DATA:
			sprintf((char*)gSdi.tx_buf,"%cD%d!",gSdi.addr,gSdi.d_count);	
		break;
		case SDI_CMD_VERIFY:
			sprintf((char*)gSdi.tx_buf,"%cV!",gSdi.addr);
		break;
		case SDI_CMD_CONCURRENT:
			if(gSdi.use_crc) {
				if(gSdi.measurement==0) {
					sprintf((char*)gSdi.tx_buf,"%cCC!",gSdi.addr);	
				} else {
					sprintf((char*)gSdi.tx_buf,"%cCC%d!",gSdi.addr,gSdi.measurement);
				}
			} else {
				if(gSdi.measurement==0) {
					sprintf((char*)gSdi.tx_buf,"%cC!",gSdi.addr);	
				} else {
					sprintf((char*)gSdi.tx_buf,"%cC%d!",gSdi.addr,gSdi.measurement);
				}
			}
		break;
		case SDI_CMD_CONTINUOUS:
			if(gSdi.use_crc) {
				sprintf((char*)gSdi.tx_buf,"%cRC%d!",gSdi.addr,gSdi.measurement);
			} else {
				sprintf((char*)gSdi.tx_buf,"%cR%d!",gSdi.addr,gSdi.measurement);
			}
		break;
	}
//	TriggerPulse();
	gSdi.cmd_ready= true; // Tell SdiProc that we have a command to send. 
}

void SdiClearCmd() {
	gSdi.cmd_ready= false;
	gSdi.tx_buf[0]= 0;
}

BOOL SdiExpectingCRC() {
	return (gSdi.use_crc && ((gSdi.cur_cmd==SDI_CMD_DATA) || (gSdi.cur_cmd==SDI_CMD_CONTINUOUS)));
}

void SdiProc() {
	switch(gSdi.state) {
		case ST_SDI_IDLE:	
			gSdi.inactive_87ms= false;
			gSdi.state= ST_SDI_WAIT_CMD;
		break;
		case ST_SDI_WAIT_CMD:
			if(!gSdi.inactive_87ms) { 
				// Latch INACTIVE_87ms. We don't want to keep testing CaptureExpired
				// because when it wraps around, you'll get false negatives. 
				if(CaptureExpired(gCapture.inactive_tar,TIME_87ms)) 
					gSdi.inactive_87ms= true;
			}
			if(gSdi.cmd_ready) { // start state machine. 
				if(CaptureExpired(gCapture.inactive_tar,TIME_8ms) ) {
					gSdi.retry_inner= 3;
					gSdi.retry_outer= 3;
					// if bus is inactive for > 87ms then a break must be sent, otherwise just a mark. 
					if(gSdi.inactive_87ms) {
						gCapture.retry= TB0R;
						gSdi.state= ST_SDI_BREAK_SEND;
					} else {
						gSdi.state= ST_SDI_MARK_SEND;
					}
				}
			}
		break;
		case ST_SDI_BREAK_SEND:
			SdiDataLine(HIGH);
			gSdi.state= ST_SDI_BREAK_WAIT;
			gCapture.sdi= TB0R;
		break;
		case ST_SDI_BREAK_WAIT:
			if(CaptureExpired(gCapture.sdi,TIME_12p5ms)) {
				gSdi.state= ST_SDI_MARK_SEND;
				SdiDataLine(LOW);
				gCapture.sdi= TB0R;
			}
		break;
		case ST_SDI_MARK_SEND:
			SdiDataLine(LOW);
			gSdi.state= ST_SDI_MARK_WAIT;
			gCapture.sdi= TB0R;
		break;
		case ST_SDI_MARK_WAIT:
			if(CaptureExpired(gCapture.sdi,TIME_MARK)) {
				gSdi.state= ST_SDI_TRANSMIT;
				SdiDataLine(UART_TX);
				CommWriteStr(COM_SDI,(char*)gSdi.tx_buf);
				gCapture.sdi= TB0R;
			}
		break;
		case ST_SDI_TRANSMIT:
			if(CommTxIsDone(COM_SDI)) { // Wait til finished transmitting.
				SdiDataLine(UART_RX);
				gSdi.rx_done= false;
				gSdi.received_char= false;
				gCapture.sdi= TB0R;
				gSdi.state= ST_SDI_RECEIVE;
			}
		break;
		case ST_SDI_RECEIVE:
			// transparent commands are treated differently.
			if(gSdi.rx_done) {
				if(SdiExpectingCRC()) {
					if(!SdiCrcCheck(gSdi.rx_buf)) {
						gSdi.rx_error= true;
					}
				}
				if(gSdi.rx_error) {
					gSdi.state= ST_SDI_CHECK_RETRIES;
				} else { // success
					gSdi.response_ready= true;
					SdiClearCmd();
					gSdi.state= ST_SDI_IDLE;
				}
			} else {
				if((!gSdi.received_char) && (CaptureExpired(gCapture.sdi,TIME_16p7ms+TIME_CHAR+TIME_CHAR))) { // time since last sent character!!
					gSdi.state= ST_SDI_CHECK_RETRIES;
				} else { // we've received a character, timeout in 100ms if we don't have a complete and valid response 
					if(CaptureExpired(gCapture.sdi,TIME_100ms)) {
						gSdi.state= ST_SDI_CHECK_RETRIES;
					}
				}
			}
		break;
		case ST_SDI_CHECK_RETRIES:
			gSdi.rx_error = false;
			if(!CaptureExpired(gCapture.retry,TIME_112p5ms)) {
				gSdi.state= ST_SDI_MARK_SEND;
			} else {
				gSdi.retry_inner--;
				if(gSdi.retry_inner<=0) {
					gSdi.retry_inner= 3;
					gSdi.retry_outer--;
					if(gSdi.retry_outer<=0) {
						SdiClearCmd();
						gSdi.state= ST_SDI_IDLE;
						// set no response flag. 
						gSdi.no_response++;
						gSdi.busy= false; // allow new sdi commands to be issued
					} else {
						gSdi.state= ST_SDI_BREAK_SEND;
					}
				} else {
					gSdi.state= ST_SDI_MARK_SEND;
				}
			}
		break;
	}
}

int SdiGetNumValues(BYTE *buf,int size) {
	int i;
	int count=0;
	for(i=0;i<size;i++) {
		WATCHDOG_CLEAR;
		if((buf[i]=='+')||(buf[i]=='-'))count++;
	}
	return count;
}

void UartA1RxIsrHook(BYTE ch) { // goes inside of UartA1RxIsr
	if((gSdi.cur_cmd == SDI_CMD_TRANSPARENT) || gSdi.echo) { // return chars in transparent mode
		// Since incoming chars are at a slower baud rate, we shouldn't have to test if the UART is full.
		CommWriteBuf(COM_USB,(char*)&ch,1);
	}
	gSdi.received_char= true;
	if(gSdi.state == ST_SDI_RECEIVE) gCapture.sdi= TB0R; // reset timer whenever we get a char.
	gCapture.inactive_tar= TB0R;
	gSleepCapture.tar= TB0R;
}

void UartA0RxIsrHook(BYTE ch) { // goes inside of UartA0RxIsr
	gSleepCapture.tar= TB0R;
}

void SdiRxProc() {
	int num_vals;
	int num_params;
	WORD slen; 
	// data is stored in gSdi.rx_buf[]
	char buf[32];
	if(RxDelimeterFound(COM_SDI)) {
		slen= RxBufGetPkt(COM_SDI,gSdi.rx_buf,SDI_RX_BUF_SIZE);
		
		switch(gSdi.cur_cmd) { // every command, but change address should start with the SDI address. 
			case SDI_CMD_ADDRESS_CHANGE:
				if(gSdi.new_addr != (gSdi.rx_buf[0])) { // make sure response was previously sent new address.
					gSdi.rx_error= true;
				} // error
			break;
			case SDI_CMD_QUERY_ADDRESS: // ignore address byte since it is unknown
			break;
			case SDI_CMD_TRANSPARENT: // ignore address byte on transparent mode.
			break;
			default: // all other commands except the address query should start with the gSdi.addr.  
				if((gSdi.rx_buf[0] != '?' ) && (gSdi.addr != (gSdi.rx_buf[0]))) {
					gSdi.rx_error= true;
				} // error
			break;
		}
		if(gSdi.rx_error) {
			// clear RX buffer. 
		} else 	{
			if(gSdi.wait_service_request) {
				gSdi.response_ready= true;
			} else {
				gSdi.rx_done= true;
			}
		}
		
		
		if(gSdi.wait_service_request) {
			gSdi.wait_service_request= false;
			SdiLoadCmd(SDI_CMD_DATA);
		} else {
			switch(gSdi.cur_cmd) {
				case SDI_CMD_NONE: // error condition
					gSdi.busy= false;
				break;
				case SDI_CMD_ACK: // Expecting a\r\n
					gSdi.busy= false;
				break;		
				case SDI_CMD_ADDRESS_CHANGE: // Expecting b\r\n  where b is new address
					gSdi.busy= false;
				break;
				case SDI_CMD_IDENT: // Expecting a string "allccccccccmmmmmmvvvxxx...xx"
					gSdi.busy= false;
				break;		
				case SDI_CMD_QUERY_ADDRESS: // Expecting a\r\n
					gSdi.busy= false;
				break;
				case SDI_CMD_VERIFY:  // Expecting atttn\r\n
				case SDI_CMD_CONCURRENT:  // Expecting atttn\r\n
				case SDI_CMD_MEASURE: // Expecting atttn\r\n
					gSdi.num_values= gSdi.rx_buf[4]-'0';
					if(gSdi.num_values<0 || gSdi.num_values>9) {// error
						gSdi.num_values= 0;
						sprintf(buf,"ER: (2) response %c:%d",gSdi.addr,gSdi.measurement);
						CommWriteStr(COM_USB,buf);
						gSdi.busy= false;
					}
					gSdi.rx_buf[4]= 0;
					num_params= sscanf((char*)&gSdi.rx_buf[1],"%d",&gSdi.measurement_time);
					gSdi.measurement_time+=2; // add 2 seconds fudge factor. 
					if(num_params!=1) { // error. 
						sprintf(buf,"ER: (3) response %c:%d",gSdi.addr,gSdi.measurement);
						CommWriteStr(COM_USB,buf);
						gSdi.busy= false;
					}
					switch(gSdi.cur_cmd) {
						case SDI_CMD_MEASURE:
						case SDI_CMD_VERIFY:
							gSdi.wait_service_request= true;
						break;
						case SDI_CMD_CONCURRENT:
							gSdi.busy= false;
							gSensors.sensors[gSdi.idx].phase= PHASE_CONCURRENT_WAIT;
							gSensors.sensors[gSdi.idx].period_concurrent= gSdi.measurement_time;
						break;
					}
					gSdi.measurement_secs=0;
					gSdi.d_count= 0;
					gSdi.values_count= 0;
				break;		
				case SDI_CMD_DATA: // Expecting a<values>\r\n
					// send to Host
					if(gSdi.d_count==0) {
						sprintf(buf,"DATA %c:%d ",gSdi.addr,gSdi.measurement);
						CommWriteStr(COM_USB, buf);
					}
					num_vals= SdiGetNumValues(&gSdi.rx_buf[1],slen-3);
					gSdi.values_count+= num_vals;
					CommWriteBuf(COM_USB,(char*)&gSdi.rx_buf[1],slen-3); // sub 3: first char, and CR.
					if(gSdi.values_count<gSdi.num_values) { // send next D command. 
						gSdi.d_count++;
						SdiLoadCmd(SDI_CMD_DATA);
						gSdi.state= ST_SDI_IDLE; // Allows data command to be sent
						if(gSdi.d_count>gSdi.num_values) {
							// Error - the number of Dx commands should always be less
							// than the number of values.  Reset state machine. 
							// It could get here if, the sensor keeps sending crap. 
							gSdi.busy= false;
							CommWriteStr(COM_USB,"\r\n");
						}
					} else { // got all measurements. 
						gSdi.busy= false;
						CommWriteStr(COM_USB,"\r\n");
					}
				break;
				case SDI_CMD_CONTINUOUS: // Expecting a<values>\r\n
					if(gSdi.d_count==0) {
						CommWriteBuf(COM_USB,"D",1);
						DelayMs(100);
						sprintf(buf,"ATA %c:%d ",gSdi.addr,gSdi.measurement);
						CommWriteStr(COM_USB,buf);
						CommWriteBuf(COM_USB,(char*)&gSdi.rx_buf[1],slen-3); // sub 3: first char, and CR.
					}
				break;
			}
		}
	}
}


void CmdProc() {
	int i;
	int num_params;
	char cmd[32];
	char resp[32];
	char str[32];
	int slen;
	BOOL command_found= false;
	
	int	sensor,measurement,period,use_crc,type;
	char addr;	// must be a char!

	SensorsStart(true); // we default to having the scheduler run at power up.
	if(RxDelimeterFound(COM_USB)) { // a command is ready
		RxBufGetPkt(COM_USB,(BYTE*)cmd,32);
		if(gIsTransparent) {
			if(strncmp(cmd,"TRANSPARENT OFF",14)==0) {
				gIsTransparent= false; 
				Save();
				gSdi.cur_cmd= SDI_CMD_NONE;  
				CommWriteStr(COMM_A1,"OK\r\n");
				return;
			} else {
				slen= strlen(cmd);
				if(slen >= 32) slen=1; // check bounds. 
				cmd[slen-1]= 0; // remove tail delimiter
				SdiDirectCmd(cmd,SDI_CMD_TRANSPARENT);
			}
		} else 
		switch(cmd[0]) {
			case 'A': // Add Sensor
				num_params= sscanf(cmd,"ADD %c:%d,%d,%d,%d",&addr,&measurement,&period,&use_crc,&type);
				if(num_params != 5) {
					CommWriteStr(COM_USB,"ER: Incorrect number of parameters.\r\n");
					return;
				}
				if(!SdiAddrValid(addr)) {
					CommWriteStr(COM_USB,"ER: Sensor address out of range.\r\n");
					return;
				} 
				if((measurement<0) || (measurement>9)) {
					CommWriteStr(COM_USB,"ER: Measurement number out of range.\r\n");
					return;
				}
				if((period<10) && (measurement!=0)) {
					CommWriteStr(COM_USB,"ER: Sample period must be zero or >= 10 secs.\r\n");
					return;
				}
				if((use_crc<0) || (use_crc>1)) {
					CommWriteStr(COM_USB,"ER: CRC must be 0:no crc or 1:use crc.\r\n");
					return;
				}
				if((type<0) || (type>2)) {
					CommWriteStr(COM_USB,"ER: Type must be a integer value 0 to 2.\r\n");
					return;
				}
				if(!SensorsAdd(addr,measurement,period,use_crc,type)) {
					CommWriteStr(COM_USB,"ER: The limit on adding sensors was reached.\r\n");
					return;
				}
				CommWriteStr(COM_USB,"OK\r\n");
			break;
			case 'M':
				if(gSdi.echo) {
					gSdi.echo= false;
					CommWriteStr(COM_USB,"Echo Off.\r\n");
				} else {
					gSdi.echo= true;
					CommWriteStr(COM_USB,"Echo On.\r\n");
				}
			break;
			case 'D':
				if(strncmp(cmd,"DEL ALL",7)==0) {
					gSensors.num_sensors= 0; // clear all.
				} else {
					num_params= sscanf(cmd,"DEL %c:%d\r\n",&addr,&measurement);
					if(num_params != 2) {
						CommWriteStr(COM_USB,"ER: Incorrect number of parameters.\r\n");
						return;
					}
					if(!SdiAddrValid(addr)) {
						CommWriteStr(COM_USB,"ER: Sensor address out of range.\r\n");
						return;
					}
					if((measurement<0) || (measurement>9)) {
						CommWriteStr(COM_USB,"ER: Measurement number out of range.\r\n");
						return;
					}
					if(!SensorsDel(addr,measurement)) {
						CommWriteStr(COM_USB,"ER: The sensor has not been defined, so can't be deleted.\r\n");
						return;
					}
				}
				CommWriteStr(COM_USB,"OK\r\n");
			break;
			case 'S':
				if(strncmp(cmd,"START",3)==0) {
					SensorsStart(true);
					if(SensorsNumActive()>0) {
						CommWriteStr(COM_USB,"OK\r\n");
					} else {
						CommWriteStr(COM_USB,"WARNING: No sensors are active. Use ADD command.\r\n");
					}
				} else if(strncmp(cmd,"STOP",3)==0) {
					SensorsStart(false);
					CommWriteStr(COM_USB,"OK\r\n");
				} else if(strncmp(cmd,"SAVE",4)==0) {
					Save();
					CommWriteStr(COM_USB,"OK\r\n");
				} else if(strncmp(cmd,"SCHEDULE",4)==0) {
					sprintf(resp,"Scheduled Measurements: %d\r\n", gSensors.num_sensors);
					CommWriteStr(COM_USB,resp);
					for(i=0;(i<gSensors.num_sensors)&&(i<MAX_SENSORS);i++) {
						use_crc=0;
						if(gSensors.sensors[i].flags&FL_USE_CRC) use_crc= 0x01;
						sprintf(resp,"%c:%d,%d,%d,%d\r\n", 
								gSensors.sensors[i].addr,
								gSensors.sensors[i].measurement,
								gSensors.sensors[i].period,
								use_crc,
								(gSensors.sensors[i].flags&FL_MEASURE_TYPE)
								);
						CommWriteStr(COM_USB,resp);
					}
				} else { // bad command
					CommWriteStr(COM_USB,"ER: Unrecognized Command.\r\n");
				}
			break;
			case 'Q': // Query
				num_params= sscanf(cmd,"QUERY %c:%d\r\n",&addr,&measurement);
				if(num_params != 2) {
					CommWriteStr(COM_USB,"ER: Incorrect number of parameters.\r\n");
					return;
				}
				if(!SdiAddrValid(addr)) {
					CommWriteStr(COM_USB,"ER: Sensor address out of range.\r\n");
					return;
				}
				if((measurement<0) || (measurement>9)) {
					CommWriteStr(COM_USB,"ER: Measurement number out of range.\r\n");
					return;
				}
				if(!SensorsFind(addr,measurement,&sensor)) {
					CommWriteStr(COM_USB,"ER: The sensor has not been defined. Use ADD command.\r\n");
					return;
				}
				use_crc=0;
				if(gSensors.sensors[sensor].flags&FL_USE_CRC) use_crc= 0x01;
				sprintf(resp,"OK, %c:%d,%d,%d,%d\r\n", 
						gSensors.sensors[sensor].addr,
						gSensors.sensors[sensor].measurement,
						gSensors.sensors[sensor].period,
						use_crc,
						(gSensors.sensors[sensor].flags&FL_MEASURE_TYPE)
						);
				CommWriteStr(COM_USB,resp);
				CommWriteStr(COM_USB,"OK\r\n");
			break;
			case 'B': // Toggle heart Beat
				if(gDoHeart) {
					gDoHeart= false;
					LED_OUT&= ~LED0;
					sprintf(resp,"OK, Heart Beat Off.\r\n");
				} else {
					gDoHeart= true;
					sprintf(resp,"OK, Heart Beat On.\r\n");
				}
				CommWriteStr(COM_USB,resp);
			break;			
			case 'V': // Send back the version
				sprintf(resp,"OK, %s\r\n",VERSION);
				CommWriteStr(COM_USB,resp);
			break;
			case 'R': // Immediate Measurement
				num_params= sscanf(cmd,"READ %c:%d",&addr,&measurement);
				if(num_params != 2) {
					CommWriteStr(COM_USB,"ER: Incorrect number of parameters.\r\n");
					return;
				}
				if(!SdiAddrValid(addr)) {
					CommWriteStr(COM_USB,"ER: Sensor address out of range.\r\n");
					return;
				} 
				if((measurement<0) || (measurement>9)) {
					CommWriteStr(COM_USB,"ER: Measurement number out of range.\r\n");
					return;
				}
				if(!SensorsFind(addr,measurement,&sensor)) {
					CommWriteStr(COM_USB,"ER: The sensor has not been defined. Use ADD command.\r\n");
					return;
				}
				SensorImmediate(sensor);
				CommWriteStr(COM_USB,"OK\r\n");
			break;
			case 'T': // Transparent
				if(strncmp(cmd,"TRANSPARENT ON",14)==0) {
					gIsTransparent= true; // Go fully transparent
					Save();
					gSdi.cur_cmd= SDI_CMD_NONE;  
					CommWriteStr(COMM_A1,"OK\r\n");
					return;
				}
 				if(strncmp(cmd,"TR OFF",6)==0) {
					gSdi.cur_cmd= SDI_CMD_NONE; // turn off trans mode echo. 
					CommWriteStr(COM_USB,"OK\r\n");
					return;
				}
				if(strncmp(cmd,"TRANS ",6)==0) {
					command_found= true;
					strncpy(str,cmd+6,32-6);
				}
				if(strncmp(cmd,"T ",2)==0) {
					command_found= true;
					strncpy(str,cmd+2,32-2);
				} 
				if(!command_found) { // no command found
					CommWriteStr(COM_USB,"ER: Unknown command.\r\n");
					return;
				}
				if(gSdi.busy) {
					CommWriteStr(COM_USB,"BUSY\r\n");
					return;
				}
				slen= strlen(str);
				if(slen>=32) slen=1; // check bounds. 
				str[slen-1]= 0; // remove tail delimiter
				if(gOkayOn) {
					CommWriteStr(COM_USB,"OK\r\n");
				}
				SdiDirectCmd(str,SDI_CMD_TRANSPARENT);
			break;
			case 'O':
				if(strncmp(cmd,"OK OFF",6)==0) {
					gOkayOn= false; // turn off trans mode OK response
					Save();
					CommWriteStr(COM_USB,"OK\r\n");
					return;
				} else if(strncmp(cmd,"OK ON",5)==0) {
					gOkayOn= true; // turn off trans mode OK response
					Save();
					CommWriteStr(COM_USB,"OK\r\n");
					return;
				}
			break;
//			case 'X': // Extended 'X' SDI compliant commands, or regular SDI commands. 
//				if(strncmp(cmd,"XTEND ",6)!=0) {
//					CommWriteStr(COM_USB,"ER: Unknown command.\r\n");
//					return;
//				}
//				if(gSdi.busy) {
//					CommWriteStr(COM_USB,"BUSY\r\n");
//					return;
//				}
//				strncpy(str,cmd+6,48);
//				CommWriteStr(COM_USB,"OK\r\n");
//				SdiDirectCmd(str,SDI_CMD_EXTENDED);
//			break;
			case 'H': // help
				CommWriteStr(COM_USB,"\r\nH - HELP:\r\n");
				CommWriteStr(COM_USB,"B - Turn on flashing heart beat LED,\r\n");
				CommWriteStr(COM_USB,"ADD [sensor number (0-9,a-z,A-Z)]:[Measurement number],\r\n");
				CommWriteStr(COM_USB,"    [sample period (in seconds, 0 for no sampling)],[use CRC (1:true/0:false)],\r\n");
				CommWriteStr(COM_USB,"    [Type (0:Normal,1:Concurrent)]<CR>\r\n");
				CommWriteStr(COM_USB,"Add up to 20 measurements to the scheduler.\r\n");
				CommWriteStr(COM_USB,"Example: ADD 0:0,120,0,0<CR>  - sensor 1 is sampled every 2 minutes without CRC.\r\n");
				CommWriteStr(COM_USB,"DEL [sensor number (0-9,a-z,A-Z)]:[Measurement number]<CR> - delete a particular measurement from scheduler\r\n");
				CommWriteStr(COM_USB,"DEL ALL<CR> - Removes all measurements from scheduler\r\n");
				CommWriteStr(COM_USB,"START<CR>   - Start scheduler.\r\n");
				CommWriteStr(COM_USB,"STOP<CR>    - Stop scheduler.\r\n");
				CommWriteStr(COM_USB,"QUERY [sensor address (0-9,a-z,A-Z)]:[Measurement number]<CR> - Prints particular measurement and SDI ID string.\r\n");
				CommWriteStr(COM_USB,"SCHEDULE<CR> Return all entries in the scheduler.\r\n");
				CommWriteStr(COM_USB,"V<CR> - Returns version of SDI Logger.\r\n");
				CommWriteStr(COM_USB,"READ [sensor address (0-9,a-z,A-Z)]:[Measurement number]<CR> - Perform immediate measurement.\r\n");
				CommWriteStr(COM_USB,"T [String]<CR> - Transparently sends a string to SDI bus.\r\n");
				CommWriteStr(COM_USB,"Remember to terminate command with a '!'\r\n");
				CommWriteStr(COM_USB,"TR OFF<CR> - Turns off transparent mode monitoring of SDI bus.\r\n");
//				CommWriteStr("XTEND [String]<CR> - Send SDI compliant X command.\r\n\r\n");
				CommWriteStr(COM_USB,"DATA [sensor number (0-9,a-z,A-Z)]:[Measurement number]<SDI values> - Response from logger to measurements.\r\n");
//				CommWriteStr(COM_USB,"M - Toggle monitoring of SDI bus. Where all bus data is echoed back.\r\n\r\n");
				CommWriteStr(COM_USB,"SAVE<CR> - Save Scheduler info to flash memory.\r\n");
				CommWriteStr(COM_USB,"OK (ON/OFF)<CR> - Turn on/off OK response on transparent commands.\r\n");
				CommWriteStr(COM_USB,"TRANSPARENT (ON/OFF)<CR> - Turn on/off transparent mode - (no 'T' prefix needed).\r\n");
			break;
			default: {
				CommWriteStr(COM_USB,"ER: Unrecognized Command\r\n");
			}
			break;
		}
	}
}



/*********************** Main *******************/

#define HEART_BEAT_COUNT		(MCLK_HZ/2) // 1 Hz.

void Sleep() {
	if(gSdi.state == ST_SDI_IDLE) {
		if(CaptureExpired(gSleepCapture.tar, TIME_100ms)) {
			LPM3;
		}
	}
}

void UnusedPins() {
    P1SEL0&= ~P1UNUSED; // inputs and pulldown
    P1SEL1&= ~P1UNUSED;
    P1DIR&= ~P1UNUSED;
    P1OUT&= ~P1UNUSED;
    P1REN|= P1UNUSED;

    P2SEL0&= ~P2UNUSED; // inputs and pulldown
    P2SEL1&= ~P2UNUSED;
    P2DIR&= ~P2UNUSED;
    P2OUT&= ~P2UNUSED;
    P2REN|= P2UNUSED;

    P3SEL0&= ~P3UNUSED; // inputs and pulldown
    P3SEL1&= ~P3UNUSED;
    P3DIR&= ~P3UNUSED;
    P3OUT&= ~P3UNUSED;
    P3REN|= P3UNUSED;

    P4SEL0&= ~P4UNUSED; // inputs and pulldown
    P4SEL1&= ~P4UNUSED;
    P4DIR&= ~P4UNUSED;
    P4OUT&= ~P4UNUSED;
    P4REN|= P4UNUSED;
}
                                 
void main(){
	char str[16];
	int heart=0;
	int i;

	WATCHDOG_HOLD;
	
	OscillatorInit(false);
	UnusedPins();
	
//	LPM3;
//	while(1);
	
	gDoHeart= false;
	SdiInit();

    LedInit();
    TimerB0Init();
    TimerB1Init();

    CommInit(COM_USB,'\r',9600,PARITY_NONE,0,false);	// RS232/USB
    CommInit(COM_SDI,'\n',1200,PARITY_EVEN,0,false);	// SDI-12

    LedSequence();

    /*//Wiggle DIR_OUT to verify connectivity
	while(1){
		SDI_OUT^= DIR_OUT;
		__delay_cycles(10000);
	}*/

	/*//Wiggle RX_A1 to verify connectivity
	SDI_SEL0&= ~RX_A1;
	SDI_SEL1&= ~RX_A1;
	SDI_DIR|= RX_A1;
	while(1){
		SDI_OUT^= RX_A1;
		__delay_cycles(10000);
	}*/
    
	_EINT();

	if(FlashMspVarsGet()) {
		gSdi.echo= gFlashVars.echo;
		gSensors.num_sensors= gFlashVars.num_sensors;
		gIsTransparent= gFlashVars.transparent_mode;
		for(i=0;i<gSensors.num_sensors;i++) {
			gSensors.sensors[i].addr=			gFlashVars.sensors[i].addr;
			gSensors.sensors[i].flags=			gFlashVars.sensors[i].flags;
			gSensors.sensors[i].measurement=	gFlashVars.sensors[i].measurement;
			gSensors.sensors[i].period=			gFlashVars.sensors[i].period;
		}
	} else {
		gSdi.echo= false;
		gSensors.num_sensors= 0;
		gOkayOn= true;
		gIsTransparent= false; 
	}
	CommWriteStr(COM_USB, STR_HELLO);
	sprintf(str,"%s\r\n", VERSION);
	CommWriteStr(COM_USB,str);
	CommWriteStr(COM_USB,"Type 'H' for Help.\r\n");
	
	/*
	// Timer Diagnostic
    while(1){
    	gCapture.inactive_tar= TB0R;
    	while(!CaptureExpired(gCapture.inactive_tar,TIME_8p3ms)){
			WATCHDOG_CLEAR;
    	}
    	LED_OUT^= (LED1);
	}*/
	
    while(1){
    	WATCHDOG_CLEAR;
    	if(gDoHeart) {
	    	heart++;
	    	if(heart > HEART_BEAT_COUNT) {
	    		heart=0;
	    		LED_OUT^= (LED0);
	    	}
	    }
		SdiProc();
		SdiRxProc();
		CmdProc();
		SensorsProc();
    	Sleep();		
	}
}
