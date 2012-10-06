#include <p18cxxx.h>
#include <spi.h>                    //SPI library functions

unsigned char MyReadMedia(void)
{
	int i = 0;
		unsigned char f;
		
		f = SSPBUF;
	
		SSPBUF = 0xFF;                  // initiate bus cycle
  	 	while ( !SSPSTATbits.BF )	i++;
  	 	f = SSPBUF;
  	 	if (i >1000 ) i = 0;
  	 	return f;
}  	 		

void MyOpenSPI( unsigned char mode)
{
	SSPCON1 = 0x00;                 // power on state		
	SSPSTAT &=  0x3f ;               // power on state 

	SSPCON1bits.CKP = 1;        // clock idle state high
 	TRISBbits.TRISB1 = 0;       // define clock pin as output
 	TRISBbits.TRISB0 = 1;       // define SDI pin as input	
   	TRISCbits.TRISC7 = 0;       // define SDO pin as output
   	TRISBbits.TRISB2 = 0;    // Temp sensor CS 
   	LATBbits.LATB2 = 1;		// Disable CS for temp sensor


   	SSPCON1 |= mode;
	SSPCON1 |= SSPENB;              // enable synchronous serial port 

}
static int i ;
 void foo ( void ) 
 {
 	int timeout;
 	int ggg;

 volatile  unsigned char b;
 	
  	
    MyOpenSPI(0x2);

    for(timeout=0; timeout<10; timeout++)
    {

  ggg = 9;
    
  		b = MyReadMedia();


	ggg = 8;
         
 	}
 }