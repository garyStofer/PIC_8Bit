#include "main.h"

#ifdef EE_read_write_used

int8 ReadEE(uns8 addr)
{
	int8 b;

	EEADR = addr;
	EECON1bits.EEPGD = false;
	EECON1bits.RD = true;
	b=EEDATA;
	EECON1 = 0;
	return(b);	
} // ReadEE

void WriteEE(uns8 addr, int8 d)
{
	int8 rd;
	
	rd = ReadEE(addr);
	if ( rd != d )						// avoid redundant writes
	{
		EEDATA = d;				
		EEADR = addr;
		EECON1bits.EEPGD = false;
		EECON1bits.WREN = true;

		EECON2 = 0x55;
		EECON2 = 0xaa;
		EECON1bits.WR = true;
		while(EECON1bits.WR);

		EECON1bits.WREN = false;
	}

} // WriteEE



void ProgRegister(void)
{
	uns8 IRQStat;
	
	EECON1bits.EEPGD = 0;
	IRQStat = INTCON;
	INTCON &= 0x3F;					// Disable both High and low IRQ's 

	EECON1bits.WREN = 1;			// enable eeprom writes
	EECON2 = 0x55;					// Set programming key sequence
	EECON2 = 0xAA;
	EECON1bits.WR = 1;				// start write cycle

	INTCON = IRQStat; 				// Re-enable IRQ's again 

	while( EECON1bits.WR == 1 );	// wait to complete
	EECON1bits.WREN = 0;			// disable EEPROM write
}

#endif
