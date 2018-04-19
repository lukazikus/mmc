/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Program created by Jiachen Zhang on 2014-08-12.
// Program modified by Jiachen Zhang on 2014-08-13~20.
// Program modified by Jiachen Zhang on 2015-08-04.
// Program modified by ... on ... (Please follow this format to add any following modification info.)
//

#include "s826_subroutine.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBROUTINE

// Board Initialization
// Output: errcode.

bool flag_range_changed = true;
int prev_range = 8;
int count_range = 16;

int s826_init(void)
{
	int errcode = S826_ERR_OK;

	return errcode;
}

int s826_close(void)
{
	return 0;
}

// Test whether the output voltage is inside the range or not.
// Input       : rangeCode: 0: 0 +5V; 1: 0 +10V; 2: -5 +5V; 3:-10 +10V.
//               outputV  : Desired analog output voltage (can be positive and negative).
// Return Value: 0: inside the range; -1: outside the range.

short int rangeTest(uint rangeCode,double outputV)
{

	return 0;
}

// Set 1 AO channel.
// Input: board: board identifier.
//        chan : DAC channel # in the range 0 to 7.
//        rangeCode: 0: 0 +5V; 1: 0 +10V; 2: -5 +5V; 3:-10 +10V.
//        outputV: Desired analog output voltage (can be positive and negative).

int s826_aoPin(uint chan,uint rangeCode,double outputV)
{

	return 1;
}

// Set the 8 AO channel together.
// Input: board    : board identifier.
//        rangeCode: 0: 0 +5V; 1: 0 +10V; 2: -5 +5V; 3:-10 +10V.
//        outputV  : Desired analog output voltage (can be positive and negative).

int s826_aoWhole(uint rangeCode[8], double outputV[8])
{

	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ANAOLOG INPUT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Initialize one AI channel. Designate a slot for this channel.
// Input:
//	 chan   : Bit-wise. Each bit stands for one channel. The corresponding bit will be set if one channel is of interest.

int s826_aiInit(uint chan, uint range)
{
	uint errcode = S826_ERR_OK;

	// These ...Old variables are used to receive the existing setting of the ADC.
	//uint chanOld =0;
	//uint tsettleOld = 0;
	//uint rangeOld = 0;
	//X826(S826_AdcSlotConfigRead(board, slot, &chanOld, &tsettleOld, &rangeOld));
	//printf("Reading result %i, %i, %i\n",chan,tsettle,range);
	//if(chanOld != chan)
	//	printf("Warning: This AI slot has been assigned to channel %i!\n",chanOld);


	return errcode;
}

// Read the ADC value(s) of interested channel(s).
// Input:
//	 chan: Bit-wise. Each bit stands for one channel. The corresponding bit will be set if one channel is of interest.
//	 *aiV: Pointer to an array which is used to receive the ADC result.
// Output: errcode.

int s826_aiRead(uint chan,double *aiV)
{
	uint errcode = S826_ERR_OK;


	return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DIGITAL OUTPUT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Set 1 DO channel to be high/low voltage.
// Input:
//        chan   : DAC channel # in the range 0 to 23.
//        outputV: Desired digit output voltage: 0: Low; 1: High.
// Output: errcode.
// Note: Writing a '1' to the output register causes the I/O pin to be driven LOW,
//       whereas writing '0' allows the pin to be internally pulled up or driven HIGH or LOW by an external circuit.

int s826_doPin(uint chan,bool outputV)
{
	uint errcode = S826_ERR_OK;

	return errcode;
}

// Set all DO channel to be high/low voltage.
// Input:
//        outputV: Desired digit output voltage: 0: Low; 1: High. Bit-wise.
// Output: errcode.
// Note: Writing a '1' to the output register causes the I/O pin to be driven LOW,
//       whereas writing '0' allows the pin to be internally pulled up or driven HIGH or LOW by an external circuit.

int s826_doWhole(uint outputV)
{
	uint errcode = S826_ERR_OK;


	return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void print_current_time_with_ms (void)
{
    long            ms; // Milliseconds
    time_t          s;  // Seconds
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s  = spec.tv_sec;
    ms = spec.tv_nsec / 1.0e6; // Convert nanoseconds to milliseconds

    printf(" Current time: %"PRIdMAX".%03ld seconds since the Epoch\n",
           (intmax_t)s, ms);

}
*/

// Configure a counter channel to operate as a periodic timer and start it running.
#define TMR_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_START | S826_CM_OM_NOTZERO)

// Configure a counter channel to operate as a periodic timer and start it running.
int PeriodicTimerStart(uint counter, uint period)
{
    int errcode;

    return errcode;
}

// Halt channel operating as periodic timer.
int PeriodicTimerStop(uint counter)
{
    return 1;
}

// Wait for periodic timer event.
int PeriodicTimerWait(uint counter, uint *timestamp)
{

    return 1;
}

int waitUsePeriodicTimer(uint waitTime)
{

    return 1;
}
