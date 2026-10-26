// SysDep.h

#ifndef SYSDEP_H
#define SYSDEP_H
#include  <msp430.h>

//#define WATCHDOG_CLEAR		WDTCTL = (WDTPW | WDTCNTCL | WDTSSEL) // 1sec interval.
#define WATCHDOG_CLEAR		WDTCTL = (WDTPW | WDTIS2 | WDTCNTCL | WDTSSEL__VLOCLK) // ~3 sec interval, VLOCLK
#define WATCHDOG_HOLD		WDTCTL = WDTPW + WDTHOLD

/***************** Defines *****f***********/

#define MIN(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})

#define MAX(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})


#define	BYTE				unsigned char
#define	WORD				unsigned short
#define	BOOL				unsigned char
#define	DWORD   			unsigned long
#ifndef false
#define	false   			0x00
#endif
#ifndef true
#define	true				0xFF
#endif

#define MAX_SENSORS			20

#define COMM_A0				0x01
#define COMM_A1				0x02
#define RX_BUFFER_SIZE		128
#define TX_BUFFER_SIZE		128

#define ACLK_HZ				(32768)		// 32768 Hz
#define DCO_HZ				24000000	// 24 MHz
#define MCLK_HZ				(DCO_HZ)
#define SMCLK_HZ			(DCO_HZ)
#define TB0_HZ				(ACLK_HZ/8)	// 4096Hz


#define UARTA0RXISRHOOK
void UartA0RxIsrHook(BYTE ch);
#define UARTA1RXISRHOOK
void UartA1RxIsrHook(BYTE ch);

// PORT 1
#define P1_0			0x01
#define P1_1			0x02
#define SDA				0x04 
#define SCL	            0x08	
#define P1_4			0x10
#define CTS_A0			0x20
#define RX_A0			0x40
#define TX_A0			0x80


#define COM_A0_SEL0			P1SEL0
#define COM_A0_SEL1			P1SEL1
#define COM_A0_DIR			P1DIR
#define COM_A0_OUT			P1OUT


#define P1UNUSED		(P1_0|P1_1|P1_4)

// PORT 2
#define SDI_BREAK		0x01
#define P2_1			0x02 
#define P2_2_NC			0x04
#define P2_3_NC			0x08
#define LED0			0x10
#define LED1			0x20
#define XOUT			0x40
#define XIN				0x80

#define P2UNUSED			(P2_1|P2_2_NC|P2_3_NC);

#define LED_ALL				(LED0|LED1)
#define LED_SEL0			P2SEL0
#define LED_SEL1			P2SEL1
#define LED_DIR				P2DIR
#define LED_OUT				P2OUT

// PORT 3
#define P3_0				0x01
#define P3_1				0x02
#define P3_2				0x04
#define P3_3				0x08
#define P3_4				0x10
#define P3_5				0x20
#define P3_6				0x40
#define P3_7				0x80

#define P3UNUSED			(0xFF)

// PORT 4
#define P4_0				0x01
#define P4_1				0x02
#define RX_A1				0x04
#define TX_A1				0x08
#define P4_4_NC				0x10
#define P4_5_NC				0x20
#define DIR_OUT				0x40
#define P4_7				0x80

#define P4UNUSED			(P4_0|P4_1|P4_4_NC|P4_5_NC|P4_7)

#define COM_A1_SEL0			P4SEL0
#define COM_A1_SEL1			P4SEL1
#define COM_A1_DIR			P4DIR
#define COM_A1_OUT			P4OUT

#define CTS_A1				0x00

#define SDI_DIR				P4DIR
#define SDI_SEL0			P4SEL0
#define SDI_SEL1			P4SEL1
#define SDI_OUT				P4OUT
#define SDI_OUTS			(DIR_OUT | TX_A1)

#define COM_SDI				COMM_A1
#define COM_USB				COMM_A0



void DelayMs(WORD num_ms);

#define MAGIC_NUMBER    	0xA5A5


typedef struct t_sensor {	// address
	BYTE	addr;
	BYTE	flags;			// FL_USE_CRC| FL_MEASURE_TYPE
	WORD	measurement; 	// The measurement number.
	WORD	period;			// number off seconds between samples, 0 for not active.
	WORD	sec;			// current second count.
	WORD	period_concurrent;	// Time returned by concurrent command.
	BOOL	immediate;		// set if you want to read immediately.
	BYTE	phase;			// concurrent phase:
} Sensor;

typedef struct t_sensors {
	Sensor		sensors[MAX_SENSORS];
	int 		num_sensors;
	BOOL 		run;
} Sensors;

Sensors gSensors;

typedef struct t_sensor_flash {
	BYTE	addr;			//
	BYTE	flags;			//
	WORD	measurement; 	// The measurement number.
	WORD	period;			// number off seconds between samples, 0 for not active.
} SensorFlash;

// MUST BE WORD ALIGNED!!!
typedef struct t_flashvar { 					// all variables stored in flash
	WORD			magic;						// the magic number
	BYTE			okay_on;
	BYTE			num_sensors;				// number of defined sensors
	BYTE			echo;
	BYTE			transparent_mode;
	BYTE			reserved1;
	BYTE			reserved2;
	SensorFlash		sensors[MAX_SENSORS];
} FlashVars;

FlashVars  gFlashVars;




#endif // #ifndef SYSDEP_H
