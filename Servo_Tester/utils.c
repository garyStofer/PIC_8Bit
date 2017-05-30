// Utilities and subroutines
#include "main.h"


// Blocking wait 
void Delay_ms( uns8 ms_dur)	//Note: can only do delay up to 255 ms 
{
	uns8 t_end;
	t_end = TimeTick1ms + ms_dur; 	 
	while (TimeTick1ms != t_end)
	;
}


// indicates if PID is still sleeping
short PID_Wait( short sleep_time )		// 1.024 ms interrupt counts PID_Delay down 
{
#ifdef Scope_PID_idle_time	
		LED_2 = 1; 			// Scope probe to measure the loop idle time
#endif	
	if (PID_Delay  )  	// wait here until delay time is up 
		return 1;
	{ /* wait */ }

	PID_Delay = sleep_time; 	// reset Delay ( in ms )
#ifdef Scope_PID_idle_time
	LED_2 = 0;
#endif	
return 0;
} 

