#include "config.h" 	// Chip Config pragmas must be executed before any other includes take place because of redefinition of pragma attribute keywords
#include "main.h"
#include "math.h"

 
void InitDefault_PWM( void )
{
	// all motors off -- Cam netral 
	PWM_Val[0] = _Neutral_Out;  
	PWM_Val[1] = _Neutral_Out; 	
	PWM_Val[2] = _Neutral_Out; 
	PWM_Val[3] = _Neutral_Out; 
	PWM_Val[4] = _Neutral_Out;
	PWM_Val[5] = _Neutral_Out;
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

// LM134 temp Probe : R_set = 152 Ohm, RL = 8.2K
// Iset - 227e-6 * Temp(K)/Rset 
// For temp=400degK  Iset == 227e-6 *400 /152 = 0.59736e-3
// U Rl  at 400degK == 8.2e3 * 0.5973e-3 = 4.898V == 1003.7 bits adc
// 1 bit ADC = 4.898/(5/1024) = 0.3986 deg K
//#define Temp_Gain 0.3986   // Calculated value 
#define Temp_Gain 0.3967 //Calibarted value

// PID (K)oefficients 
#define Kp 100.0
#define Ki 0.05
#define Kd 0.0
#define P_dt 5000	// in 1.024 ms
#define Temp_setpoint 85.0		// 85C is 185F


#define ADC_AVERAGE_CNT 100

// PID 
float P = 0.0;
float I = 0.0;
float D = 0.0;
float p_last = 0.0;
unsigned short temp_ADC_avg[ADC_AVERAGE_CNT];


const char SetPoint[] = "\r\nSetpoint deg F: ";
const char temp1cp[] = "\r\nTemp1 deg F: ";
const char temp2cp[] = "    Temp2 deg F: ";

void main(void)
{  
	
	float P_error;
	float P_last;
	short P_drive;
	
	unsigned short i;
	unsigned short avg_ndx = 0;
	unsigned long  Temp_ADC;		// note: Must be long to prevent overrun when adding up average
	unsigned char  tempF;
	float Temp_C;
	
	Init_HW();
	LED_1 = LED_2 = LED_ON;
	PID_Delay = P_dt;
	
	tempF =  Temp_setpoint *9/5+32;
			
	TxText( SetPoint);
	TxValU( tempF );
	while (1)
	{
				
		PWM_Val[0] = ADC(ADC_CH_0)/4;		 		// controlled by potentiometer on ch0
		
		temp_ADC_avg[avg_ndx++] = ADC(ADC_CH_4);	//  collect a running average of temp measurements 
		
		if (avg_ndx >= ADC_AVERAGE_CNT)
			avg_ndx = 0;
			 
		
		if (PID_Wait( P_dt) )	// wait until next PID time step 
			continue; 
			
// PID stuff from here on down	
		
		LED_1 = !LED_1;
		LED_2 = !LED_1;
		
		for ( i = 0, Temp_ADC = 0; i < ADC_AVERAGE_CNT; i++)
			Temp_ADC += temp_ADC_avg[i];
		
		Temp_ADC /= i;	

		
		Temp_C = (Temp_ADC * Temp_Gain) - 273.15;  // LM134: R_set = 152Oh, R_L 8.2K, ADC_Vref = VDD/VSS == 5V
		

		if ( Temp_C < Temp_setpoint)		// digital control, damper either open or closed, no PID
			PWM_Val[1] =0xff;
		else 
			PWM_Val[1] =0x0;
			
		P_error = Temp_setpoint - Temp_C;
		
		// the proportional term
		P = Kp *P_error;
		
		// The integral term
		I += P_error * Ki * P_dt;
		
		// Prevent windup 
		if ( I > _Maximum )
			I = _Maximum;
		else if (I < -_Maximum )
			I = -_Maximum;	
		
		// The derivative term -- directly from plant output instead from error
		D = Kd * (Temp_C-P_last)/P_dt;
		P_last = Temp_C;
		
		// the drive	
		//P_drive = P + I + D;
		// Proportinal only 
		P_drive = P ;
		
		P_drive =128-P_drive; 	// shift to servo middle position 
				
		if (P_drive < _Minimum) 	// limit to what can be driven -- needed to be done here since P_drive is signed and PWM_Value is uns8
			P_drive = _Minimum;
			 					
		 if (P_drive >_Maximum)
		 	P_drive = _Maximum;
		 
		// Outputs 2 & 3 for P only drive	
		PWM_Val[3] = PWM_Val[2] = 0xff - (unsigned char)P_drive; 	 // invert control sense
	 
		
		// Prop and Integral 
		P_drive = P + I ;
		
		P_drive =128-P_drive; 	// shift to servo middle position 
		
		
		if (P_drive < _Minimum) 	// limit to what can be driven -- needed to be done here since P_drive is signed and PWM_Value is uns8
			P_drive = _Minimum;
			 					
		 if (P_drive >_Maximum)
		 	P_drive = _Maximum;
		// outputs 4 & 5 for PI drive
		PWM_Val[5] = PWM_Val[4] = 0xff - (unsigned char)P_drive; 	 // invert control sense
		
		// Second temp value on AN3 input - RA3
	//	Temp_ADC = ADC(ADC_CH_3);	
	//	Temp_C = (Temp_ADC * Temp_Gain) - 273.15;  // LM134: R_set = 152Oh, R_L 8.2K, ADC_Vref = VDD/VSS == 5V
		
		if (Temp_C >0 )
			tempF = Temp_C*9/5+32; 	// F in integer 
		else 
			tempF = 0;
		
		TxText(temp1cp);
		TxValU( tempF );
		
		// Second temp value on AN3 input - RA3
		Temp_ADC = ADC(ADC_CH_3);	
		Temp_C = (Temp_ADC * Temp_Gain) - 273.15;  // LM134: R_set = 152Oh, R_L 8.2K, ADC_Vref = VDD/VSS == 5V
		if (Temp_C > 0 )
			tempF = Temp_C*9/5+32; 	// F in integer 
		else 
			tempF = 0;
			
		TxText(temp2cp);
		TxValU( tempF );
		
		 	
	}
	
  // End  while (1)
}

