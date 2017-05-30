#include <p18cxxx.h>		// includes the processor specific file according to -pxxxx option on the cmd lin
#include <math.h>
#include <delays.h>
#include <timers.h>
#include <usart.h>
#include <adc.h>
#include "types.h"
#include "serial.h"

#define RX_PPM

// some utility macros
#define Set(S,b) 		((uint8)(S|=(1<<b)))
#define Clear(S,b) 		((uint8)(S&=(~(1<<b))))
#define IsSet(S,b) 		((uint8)((S>>b)&1))
#define IsClear(S,b) 	((uint8)(!(S>>b)&1))
#define Invert(S,b) 	((uint8)(S^=(1<<b)))

#define LED_1 PORTCbits.RC0
#define LED_2 PORTCbits.RC1
#define LED_ON	1
#define LED_OFF	0

#define i2c_SCL	PORTCbits.RC3
#define i2C_SDA	PORTCbits.RC4
#define	I2C_ACK		0
#define	I2C_NACK	1

// From irq.c 
#define MAX_RX_CH 9
#define MAX_OUT_CH 8		// All PWM out CH are on PortB, RB6 & RB7 are not available when programmed with Debugger,
//#define MAX_OUT_CH 6	
#define MIN_RX_SYNC_PAUSE 1250  // 1250 *4us = 5ms  
#define MAX_RX_SYNC_PAUSE 4000  //  4000 *4us = 16  ms
extern volatile uns8 PWM_Val[10];	
extern volatile uns8 PWM_Pause;
extern volatile uns8 TimeTick1ms;
extern volatile uns16 PID_Delay;
extern volatile uns8 RXCh_Val[MAX_RX_CH];
extern uns8 RX_Num_Channels;		// This stores the number of channels decoded in a frame -- might be useful to outside code 
extern uns8 volatile RxFrameErr;	// count number of bad RX frames or loss of signal 	
extern uns8 volatile _NoSignal ;

// Based on: TMR0 at 4us and terminal count of: 0xff == 256 * 4 us = 1.024ms 
// These limits are calculated so that pulses are exactly symmetrical around 1.5ms 
// The Maximum limit can be raised to a upper limit of ~248 for bigger PWM range (input & output)

// NOTE: Raising _Maximum too high will result in TMR0 (+ISR processing time) to overrun TMR2 on channels set to _Maximum which 
//  in effect will leave that channels output signal Permanently high. 

#define	_Minimum	0 	// -100%  == 1.024ms  
#define _Maximum	238 // +100% ==   238 x 4us + 1024 us = 1976 us 
// For asymetrical  PWM around neutral  _Maximum can be raised up to 248.
// Higer than that and the  PWM ISR routines can not handle it!
#define _Neutral_Out    (_Maximum/2)  // 0%  ==  119 x 4us + 1024 = 1500us
#define _Neutral_In 	119  // this is (1500 ms -1024ms )  /4 us  for input channels  = 119 . This is always the case for RX 


// check the PPM RX and motor values
#if _Minimum >= _Maximum
#error _Minimum < _Maximum!
#endif

#if (_Maximum < _Neutral_Out)
#error Maximum < _Neutral_Out!
#endif

// ADC Channels
#define ADCPORTCONFIG 0b00001010 // 0x0a VCC & VSS == ref, AN0-AN4 as inputs to ADC 
#define ADC_CH_0 0b00000011 // select AN0  leaving ADC power on
#define ADC_CH_1 0b00000111 // select AN1  leaving ADC power on
#define ADC_CH_2 0b00001011 // select AN2  leaving ADC power on
#define ADC_CH_3 0b00001111 // AN3
#define ADC_CH_4 0b00010011 // AN4


// Parameters for UART port

#ifdef FOSC_16
#define _ClkOut		(160/4)		// 16.0 MHz quartz 
#elif defined FOSC_32
#define _ClkOut		(320/4)		
#endif



// Prototypes
extern 	void Delay_ms( uns8);
extern 	short PID_Wait( short loop_delay );
extern	void WriteEE(uns8, int8);
extern	int8 ReadEE(uns8);
extern  int16 ADC(uint8);
extern 	void InitADC(void);



