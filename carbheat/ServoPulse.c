#include "pic.h" 
/* 
 + Simple pulse generator for a RC servo signal to move the servo from full left to full right
 + position.
 + Using a PIC 12F675 device GPIO5 is providing the servo pulse
 + while GPIO3 and is externally pulled up and connected via a switch to gnd.
*/  

// Program the OSCCAL instruction at 0x3ff with the help of the PicKit2 tool
// otherwise the target will not start up

// Do not read the device with MasterClear_enable set to internal, everything will come back as 0

// Configuration = 0x194

void          
main ( void )    
{    
	int i; 

	TRISIO = 0x1A; 		// Outputs: GP5,GP2,GP0
	WPU   = 0x00;		// Weak pull up off
	ANSEL = 0x00;		// ADC Analog inputs all off
	VRCON = 0x81;		// reference voltage at ~1.406V
	WPU   = 0xff;
	GPPU = 0;			//global enable pull ups on inputs
	
	CMCON = 0x03;		// Comperator input on GP1, output on GP2, ref from internal 

	while (1)
	{	
		GPIO5 = 0;		// Servpo signal low level
	

		for ( i = 1 ; i <1500; i++ ) ;		// delay for pause
	
		if (GPIO4  && ! ( CMCON & 0x40 && GPIO3 == 0) )		// Manual switch or Auto and Temp below freezing
		{
			i = 75;		// Delay for pulse ( Short) about 840us
			GPIO0 = 0;	// LED off Indicating cold air
			VRCON = 0x81; // == 1.406v
		}
		else
		{
			i = 180;	// Delay for pulse ( Long ) about 2 ms
			GPIO0=1; 	// LED on, indicating hot hair
			// VRCON = 0xf7; // == 1.458V -- Creating a hystereses 
			if (GPIO3 == 0 )
				VRCON = 0x82; //  to lock in the trigger until swtch is moved from Auto/Arm mode
			else
				VRCON = 0x81; // not in Auto/Arm mode, don't add hystereses 
			
		}
		
		GPIO5 = 1;		// Servo pulse high level
	
	
		while (i--) ;
		
	}
}
	