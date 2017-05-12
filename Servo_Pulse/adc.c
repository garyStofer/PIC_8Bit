#include "main.h"

int16 ADC( uns8  ch) // 
{	
	ADCON0 = ch ; 
	while( ADCON0bits.GO ) 
	{ /* wait */}     // wait to complete
	
	return ADRES;
}	

void InitADC()
{
 OpenADC(
#ifdef FOSC_32	
		  ADC_FOSC_32 & 
#else
		  ADC_FOSC_16 &
#endif
          ADC_RIGHT_JUST &
          ADC_12_TAD,				// this is Fosc/4/ADC_FOSC_16*ADC_12_TAD = 48us
		  ADC_CH0 &
		  ADC_INT_OFF &
          ADC_VREFPLUS_VDD &
          ADC_VREFMINUS_VSS,	  
          ADCPORTCONFIG);
} // InitADC


