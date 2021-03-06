

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include <spi.h>
#ifdef jkgjhgjhjhgjhgjhgjhgjhgjhg
#pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
#pragma config CPUDIV   = OSC1_PLL2	
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON		//USB Voltage Regulator
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
#pragma config CCP2MX   = ON
#pragma config STVREN   = ON
#pragma config LVP      = OFF
#pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
#pragma config CP2      = OFF
#pragma config CP3      = OFF
#pragma config CPB      = OFF
#pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
#pragma config WRT2     = OFF
#pragma config WRT3     = OFF
//      #pragma config WRTB     = ON       // Boot Block Write Protection for bootloader use
#pragma config WRTB     = OFF
#pragma config WRTC     = OFF
#pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
#pragma config EBTR2    = OFF
#pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF
#endif 
/** V A R I A B L E S ********************************************************/
#pragma udata



/** V E C T O R  R E M A P P I N G *******************************************/

extern void _startup (void);        // See c018i.c in your C18 compiler dir
#pragma code _RESET_INTERRUPT_VECTOR = 0x000020
void _reset (void)
{
    _asm goto _startup _endasm
}
#pragma code

/** D E C L A R A T I O N S **************************************************/
#pragma code

void main(void)
{
	unsigned char byte1;

	int i;

	
    TRISBbits.TRISB2 = 0;
    LATBbits.LATB2  = 1;
    
    ADCON1 |= 0x0F;                 // Default all pins to digital
    i = 0;
    //    foo();
    //OpenSPI(SPI_FOSC_64, MODE_11, SMPMID);
    MyOpenSPI( 0x2);
 
    while(1)
    {
	//		byte1 = ReadSPI();
		byte1 = MyReadMedia();
		i++;

    } //end while
}//end main
