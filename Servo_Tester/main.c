#include "config.h" 	// Chip Config pragmas must be executed before any other includes take place because of redefinition of pragma attribute keywords
#include "main.h"
#include "math.h"

 
void InitDefault_PWM( void )
{
	// all outputs netral 
	PWM_Val[0] = _Neutral_Out;  
	PWM_Val[1] = _Neutral_Out; 	
	PWM_Val[2] = _Minimum; 
	PWM_Val[3] = _Neutral_Out; 
	PWM_Val[4] = _Maximum;
	PWM_Val[5] = _Maximum;
	PWM_Val[6] = _Neutral_Out;
	PWM_Val[7] = _Neutral_Out;
	PWM_Pause = 10;		// initially until the eeprom is read
}

void Set_PWM_From_RC_input(void)
{	
	PWM_Val[0] = RXCh_Val[0];
	PWM_Val[1] = RXCh_Val[1];
	PWM_Val[2] = RXCh_Val[2];
	PWM_Val[3] = RXCh_Val[3];
	PWM_Val[4] = RXCh_Val[4];
	PWM_Val[5] = RXCh_Val[5];
	PWM_Val[6] = RXCh_Val[6];
	PWM_Val[7] = RXCh_Val[7];	
}	

void Init_HW ( void )
{
	// general ports setup
	TRISA = 0xff;		// all inputs
	InitADC();			// Setup ADC clock and mode etc
	InitUART();
	InitDefault_PWM();  // Setup PWM array with meaning ful values

	// This setup is used to gain two more Servo PWM signals on RB6 and RB7 
	// Note: Signals are not present when programmed via debug mode 
	PORTB = 0b00000000;	// All Low initially 
	TRISB = 0b00000000;	// All outputs -- Servo PWMs

	PORTC = 0b01000000;	// all outputs to low, except TxD 
	TRISC = 0b10100100;	// RC7, RC5, RC2 are inputs

//  ??????????  check for needed	
SSPSTATbits.CKE = 1;// default I2C - enable SMBus thresholds for 3.3V 

	
// Timer0 setup     ~~~~~~~~~~~~~~ Used for 1ms Timetick and PWM generation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifdef FOSC_16		
	T0CON =  0b00000011;	// TMR0 w/ int clock, 1:16 presc (see _PreScale0),  Ticks at 4us, interrupts every 1024us , for Xtal 16Mhz, no PLL, FOSC = 16Mhz 
#elif defined FOSC_32
	T0CON =  0b00000100;	// TMR0 w/ int clock, 1:32 presc (see _PreScale0),  Ticks at 4us, interrupts every 1024us , for Xtal 8Mhz, PLLx4, Fosc =32Mhz	
#endif
	
	T0CONbits.T08BIT = 1; 	// function in 8 bit mode 
	T0CONbits.TMR0ON = 1;   // Enable the timer
	
	INTCONbits.T0IF = 0;
	INTCONbits.T0IE = 1;
	INTCON2bits.RBPU = 1;				// enable weak pullups  -- 
	
// Timer1 setup       ~~~~~~~~~~ Used to capture RX signal ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	TMR1H = 0;
	TMR1L = 0;
	
	T1CON = 0b10110001;	// enable TMR1, 1:8 prescaler at Foac 16Mhz ==  run with 2us clock  == maxrun of 0xffff x 2us = 130ms
						// Fosc 32 Mhz = run at  1us clock ==  maxrun 65us
						// See ifdefs in IRQ.c  for dealing with differen Fosc rates
	PIR1bits.TMR1IF = 0;
	PIE1bits.TMR1IE = 1;

	// Capture 
	CCP1CON = 0b00000100;	// capture mode: every falling edge
	PIE1bits.CCP1IE = 1;
	
// Timer2 setup   ~~~~~~~~~~~~~~~ Used for PWM generation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if FOSC_16		
	T2CON = 0b00000111; // enable the timer at 4us per tick, i.e Fosc/4/16 for Xtal 16Mhz No PLL , Fosc = 16Mhz
#elif defined FOSC_32	
	T2CON = 0b00001111; // enable the timer at 4us per tick, i.e Fosc/4/32for Xtal 8Mhz PLL 4x , Fosc = 32Mhz
#endif

	PR2 = 0xFF; 
	PIR1bits.TMR2IF = 0;
	PIE1bits.TMR2IE = 0;		// is beeing started by Tmr0 ISR

	// Setting up interrupt priorities 
	RCONbits.IPEN =1;			// Priority schema 
	INTCON2bits.TMR0IP  = 1;	// timer 0 == high interrup priority , used for 1ms tick used inPWM 
	IPR1bits.CCP1IP 	= 0;    // Capture 1 interrup = low Priority	used for RX signal capture 
	IPR1bits.TMR1IP		= 0;	// and timer 1 overflow = low priority 	used for RX signal timeout
	IPR1bits.TMR2IP     = 1;	// High Priortity -- PWM variable part 	
	
	// Enable global interrrupts 
	INTCONbits.GIEL   = 1;		// low priority  interrupts enable
	INTCONbits.GIEH   = 1; 		// High priority  interrupts enable
}	





#define ADC_AVERAGE_CNT 100


void main(void)
{  
	unsigned char pos1 = _Neutral_Out;
	unsigned char pos0 = _Neutral_Out;
	signed char dir = 1;
	
	Init_HW();
	LED_1 = LED_2 = LED_OFF;
	
	while (1)
	{
				
		pos0 = ADC(ADC_CH_0)/4;		 		// controlled by potentiometer on ch0
		if (pos0 > _Maximum)
			pos0 = _Maximum;
			
		PWM_Val[0] = pos0;
		PWM_Val[1] = _Maximum - pos0;				// inverse from ch0
		
		Delay_ms(8);
		
		if (pos1 == _Maximum )
		{
			dir = -1;
			LED_1 = !LED_1;
		}
		if (pos1 == _Minimum )
			dir = 1;
		
		// wig-wag		
		pos1 += dir;
		PWM_Val[5] = pos1;
			
	}	  // End  while (1)
}

