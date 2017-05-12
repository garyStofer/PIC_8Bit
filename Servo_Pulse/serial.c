// Serial support (RS232 option)
#include "main.h"
#include "serial.h"

// Init to 8n1 38Kbd 
void InitUART( void )
{
	uns16 volatile tmp;
	   // setup serial port for 8N1
	TXSTA = 0b00100100;	// async mode, BRGH = 1
	RCSTA = 0b10010000;	// receive mode
	SPBRG = _B38400;
	tmp = RCREG;		// read RCREG to empty FIFO
}	

// send a character to the serial port
void TxChar(char ch)
{
	while( PIR1bits.TXIF == 0 ) ;	// wait for transmit ready
	TXREG = ch;		// put new char
}

// transmit a fix text from a table
void TxText(const  char *pch)
{
	while( *pch != '\0' )
	{
		TxChar(*pch);
		pch++;
	}
}



// converts an umsigned byte to decimal and send it
void TxValU(uns8 v)
{	
	uns8 nival;

	nival = v;

	v = nival / 100;
	TxChar(v+'0');
	nival %= 100;		

	v = nival / 10;
	TxChar(v+'0');
	nival %= 10;

	TxChar(nival+'0');
}

// converts a nibble to HEX and sends it
void TxNibble(uns8 v)
{
	uns8 nival;

	nival = v + '0';
	if( nival > '9' )
		nival += 7;		// A to F
	TxChar(nival);
}

// converts an uns byte to HEX and sends it
void TxValH(uns8 v)
{
	TxNibble(v >> 4);
	TxNibble(v & 0x0f);
}

// converts an uns short to HEX and sends it
void TxValH16(uns16 v)
{
	TxValH(v >> 8);
	TxValH(v & 0xff);
} // TxValH16

// converts a signed byte to decimal and send it
void TxValS(int8 v)
{
	if( v < 0 )
	{
		TxChar('-');	// send sign
		v = -v;
	}
	else
		TxChar('+');	// send sign

	TxValU(v);
}

void TxNextLine(void)
{
	TxChar(0x0d);
	TxChar(0x0a);
}

// if a character is in the buffer
// return it. Else return the NUL character
char RxChar(void)
{
	uns8 ch;
	
	if( PIR1bits.RCIF )	// a character is waiting in the buffer
	{
		if( RCSTAbits.OERR || RCSTAbits.FERR )	// overrun or framing error?
		{
			RCSTAbits.CREN = 0;	// diable, then re-enable port to
			RCSTAbits.CREN = 1;	// reset OERR and FERR bit
			ch = RCREG;	// dummy read
		}
		else
		{
			ch = RCREG;	// get the character
			TxChar(ch);	// echo it
			return(ch);		// and return it
		}
	}
	return( '\0' );	// nothing in buffer
}


// enter an uintigned number 00 to 99
uns8 RxNumU(void)
{
	char ch;
	uns8 nival;

	nival = 0;
	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival = ch - '0';
	nival *= 10;
	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	return(nival);
}


short _NegIn;
// enter a signed number -99 to 99 (always 2 digits)!
int8 RxNumS(void)
{
	char ch;
	int8 nival;

	nival = 0;

	_NegIn = 0;
	do
	{
		ch = RxChar();
	}
	while( ((ch < '0') || (ch > '9')) &&
           (ch != '-') );
	if( ch == '-' )
	{
		_NegIn = 1;
		do
		{
			ch = RxChar();
		}
		while( (ch < '0') || (ch > '9') );
	}
	nival = ch - '0';
	nival *= 10;

	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	if( _NegIn )
		nival = -nival;
	return(nival);
}







