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

int s826_init(void) {
	int errcode = S826_ERR_OK;

	int boardflags  = S826_SystemOpen();        		// open 826 driver and find all 826 boards
	//printf("The board number is %i\n",boardflags);

	if ((boardflags < 0) || ((boardflags & (1 << BOARD)) == 0))
	{
        	errcode = boardflags;                       // problem during open
		printf("Fatal Problem During Board Opening.\n");
	} else if (boardflags != (1<<BOARD))
		printf("Multiple boards are detected. Error may occur.\n");

	return errcode;
}

int s826_close(void) {
	S826_SystemClose();   // The return value always = 0.
	return 0;
}

// Test whether the output voltage is inside the range or not.
// Input       : rangeCode: 0: 0 +5V; 1: 0 +10V; 2: -5 +5V; 3:-10 +10V.
//               outputV  : Desired analog output voltage (can be positive and negative).
// Return Value: 0: inside the range; -1: outside the range.

short int rangeTest(uint rangeCode,double outputV) {
	switch(rangeCode)
		{
			case 0:
				if ((outputV>5)||(outputV<0)) return -1;

				break;
			case 1:
				if ((outputV>10)||(outputV<0)) return -1;

				break;
			case 2:
				if ((outputV>5)||(outputV<-5)) return -1;

				break;
			case 3:
				if ((outputV>10)||(outputV<-10)) return -1;

				break;
			default:
				printf("Error: Range selection is wrong.");
				return -1;
				break;
		}
	return 0;
}

// Set 1 AO channel.
// Input: board: board identifier.
//        chan : DAC channel # in the range 0 to 7.
//        rangeCode: 0: 0 +5V; 1: 0 +10V; 2: -5 +5V; 3:-10 +10V.
//        outputV: Desired analog output voltage (can be positive and negative).
int s826_aoPin(uint chan,uint rangeCode,double outputV) {
	int errcode   = S826_ERR_OK;
	int miniV     = 0;            // The lowest possible voltage under specific range setting.
	uint rangeV   = 0;            // Corresponding range voltage span under specific rangeCode.
	uint setpoint = 0;            // Value used in S826 subroutine.

	short int temp = rangeTest(rangeCode,outputV);
	if (temp == -1) {printf("Error: Output voltage is outside the range."); return -1;}

	switch(rangeCode) {
		case 0:
			miniV = 0;
			rangeV = 5;   // Span is 5V.
			break;
		case 1:
			miniV = 0;
			rangeV = 10;   // Span is 10V.
			break;
		case 2:
			if ((outputV>5)||(outputV<-5)) {printf("Error: Output voltage is outside the range.");return -1;}

			miniV = -5;
			rangeV = 10;   // Span is 10V.
			break;
		case 3:
			miniV = -10;
			rangeV = 20;   // Span is 20V.
			break;
		default:
			printf("Error: Range selection is wrong.");
			return -1;
			break;
	}

	if(rangeCode!=prev_range){

        if(count_range != 0){
        	count_range--;
        }else{
        flag_range_changed = true;
        count_range = 16; //# of channels

    	}
	}

	//printf("miniV is %i and rangeV is %i\n",miniV,rangeV);
	//printf("outputV is %f\n",outputV);
	//printf("Fractional result is %f\n",(outputV-miniV)/rangeV);

	setpoint = (uint)((outputV - miniV)/rangeV * 0xFFFF);   // Calc the corresponding setpoint value for the DAQ.
	//printf("setpoint value is 0x%x\n",setpoint);
	if(flag_range_changed){//added to fix issue with high frequency noise (resetting of amp after programming)
        X826(S826_DacRangeWrite(BOARD,chan,rangeCode,0));   // Program DAC output range.
        if(count_range == 0){
       		flag_range_changed = false;
  			prev_range = rangeCode;
		}
	}
	X826(S826_DacDataWrite(BOARD,chan,setpoint,0));     // Set the desired output voltage.
	return errcode;
}

// Set the 8 AO channel together.
// Input: board    : board identifier.
//        rangeCode: 0: 0 +5V; 1: 0 +10V; 2: -5 +5V; 3:-10 +10V.
//        outputV  : Desired analog output voltage (can be positive and negative).

int s826_aoWhole(uint rangeCode[8], double outputV[8]) {
	int errcode   = S826_ERR_OK;
	int miniV     = 0;            // The lowest possible voltage under specific range setting.
	uint rangeV   = 0;            // Corresponding range voltage span under specific rangeCode.
	uint setpoint = 0;            // Value used in S826 subroutine.
	short int temp = 0;
	int i;

	for(i=0;i<8;i++)
	{
		// Perform range test.
		temp = rangeTest(rangeCode[i],outputV[i]);
		if (temp == -1) {printf("Error: Output voltage is outside the range."); return -1;}

		switch(rangeCode[i])
		{
			case 0:
				miniV = 0;
				rangeV = 5;   // Span is 5V.
				break;
			case 1:
				miniV = 0;
				rangeV = 10;   // Span is 10V.
				break;
			case 2:
				miniV = -5;
				rangeV = 10;   // Span is 10V.
				break;
			case 3:
				miniV = -10;
				rangeV = 20;   // Span is 20V.
				break;
			default:
				printf("Error: Range selection is wrong.");
				return -1;
				break;
		}
		setpoint = (uint)((outputV[i] - miniV)/rangeV * 0xFFFF);   // Calc the corresponding setpoint value for the DAQ.
		X826( S826_DacRangeWrite(BOARD, i, rangeCode[i], 0)   );      // program dac output range

		X826( S826_DacDataWrite(BOARD, i, setpoint, 0)  );
	}

	return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ANAOLOG INPUT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Initialize one AI channel. Designate a slot for this channel.
// Input:
//	 chan   : Bit-wise. Each bit stands for one channel. The corresponding bit will be set if one channel is of interest.

int s826_aiInit(uint chan, uint range) {
	uint errcode = S826_ERR_OK;

	// These ...Old variables are used to receive the existing setting of the ADC.
	//uint chanOld =0;
	//uint tsettleOld = 0;
	//uint rangeOld = 0;
	//X826(S826_AdcSlotConfigRead(board, slot, &chanOld, &tsettleOld, &rangeOld));
	//printf("Reading result %i, %i, %i\n",chan,tsettle,range);
	//if(chanOld != chan)
	//	printf("Warning: This AI slot has been assigned to channel %i!\n",chanOld);

	uint i = 0;
	for (;i<16;i++) X826( S826_AdcSlotConfigWrite(BOARD,i,i,TSETTLE,range) );   // Configure channel i to timeslot i.

	//uint slotlist = ((uint)1)<<slot;
	X826( S826_AdcSlotlistWrite(BOARD,chan,0) );  // Program the conversion slot list. Write Mode: 0 (write).

	// Disable hardware triggering.
	uint trigmode = 0;
	X826( S826_AdcTrigModeWrite(BOARD,trigmode) );
	//S826_AdcTrigModeRead(board,&trigmode);
	//printf("Trigmode is 0x%x.\n",trigmode);

	unsigned short int enable = 1;                   // Set to 1 to enable, or 0 to disable ADC conversion bursts.
	//uint enableRead = 0;
	X826( S826_AdcEnableWrite(BOARD,enable) );         // Enable or disable ADC conversions.
	//X826(S826_AdcEnableRead(board,&enableRead));
	//printf("The enable status is %i.\n",enableRead);

	return errcode;
}

// Read the ADC value(s) of interested channel(s).
// Input:
//	 chan: Bit-wise. Each bit stands for one channel. The corresponding bit will be set if one channel is of interest.
//	 *aiV: Pointer to an array which is used to receive the ADC result.
// Output: errcode.

int s826_aiRead(uint chan,double *aiV) {
	uint errcode = S826_ERR_OK;

	//uint status = 0;
	int buf[16];
	uint tstamp[16];
	//uint slotlist = ((uint)1)<<slot;
	uint tmax = 1000;

	//uint halfRangeV = 0;
	//time_t mytime;
	//mytime=time(NULL);
	//printf("slotlist is %i.\n",slotlist);
	//X826(S826_AdcSlotConfigRead(board,slot,&chan,&tsettle,&rangeCode));
	//printf("I am here. S826_AdcSlotConfigRead. The rangeCode is %i.\n",rangeCode);
	//uint i = 1;
	//for(;i<100;i++)
	//{
	//	X826(S826_AdcStatusRead(board,&status));
	//	printf("Status is %i.\n",status);
		//delay(100);
	//	usleep(1000);
	//}
	//if ((status >> slot) && 1)
	//{
	X826( S826_AdcRead(BOARD, buf, tstamp, &chan, tmax) );

	//printf("I am here. S826_AdcRead.\n");
        //printf("The time is now %s", ctime(&mytime));
	//print_current_time_with_ms ();
	//}
	//printf("buf is %i",buf[0]);
	uint bufNew = 0;
	double resolution = 0;   // Resolution.
	uint chanOld    = 0;   // Buffer to store the AI channel #.
	uint tsettleOld = 0;   // Buffer to store the settle-time (uS).
	uint rangeCodeOld   = 0;   // Buffer to store the range-code (0: +-10V; 1: +-5V; 2: +-2V; 3: +-1V).

	uint i = 0;
	for (;i<16;i++)
	{
		if (chan & (1<<i))
		{
			bufNew = buf[i] & 0xFFFF;
			//printf("The bufNew is 0x%x.\n",bufNew);
			X826( S826_AdcSlotConfigRead(BOARD,i,&chanOld,&tsettleOld,&rangeCodeOld) );

			switch (rangeCodeOld)
			{
				case 0:
					//halfRangeV = 10;
					resolution = 20.0 / 65536.0;
					break;
				case 1:
					//halfRangeV = 5;
					resolution = 10.0 / 65536.0;
					break;
				case 2:
					//halfRangeV = 2;
					resolution = 4.0 / 65536.0;
					break;
				case 3:
					//halfRangeV = 1;
					resolution = 2.0 / 65536.0;
					break;
				//default:
				//	resolution = 20 / 2^16;
				//	break;
			}
			if (bufNew & 0x8000) // The reading is negative.
			{
				//printf("The reading is negative.\n");
				//actualV = - (~(bufNew - 1))/0x7FFF * halfRangeV;
				//actualV = ((~(bufNew - 1))&0xFFFF) * resolution;
				aiV[i] = ((~(bufNew - 1))&0xFFFF) * resolution;
				//printf("Test: actualV is %fV.\n",actualV);
				aiV[i] = -1 * aiV[i];
				//printf("The reading is %f.\n",aiV[i]);
			} else 		     // The reading is positive.
			{
				//printf("The reading is positive.\n");
				aiV[i] = bufNew * resolution;
				//printf("The reading is %f.\n",aiV[i]);
			}
		}
	}
	//printf("The resolution is %f.\n",resolution);
	//printf("The AI signal is 0x%x, i.e. %fV.\n",bufNew,actualV);
	//fprintf(fp," %f\n",actualV);

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
	uint64 dio = DIO(chan);
	uint data[2] = DIOMASK(dio);
	uint mode;

	//printf("outputV is %i, decision is %i.\n",outputV,((outputV == 0) || (outputV == 1)));

	// Verify the outputV.
	//if (!((outputV == 0) || (outputV == 1))) {printf("Error in s826_doPin about variable outputV.\n"); return -1;}

	mode = 2 - ((uint)outputV);    // mode 1: Clear (program to '0') all register bits that have '1' in the corresponding data bit. All other register bits are unaffected.
				 // mode 2: Set (program to '1') all register bits that have '1' in the corresponding data bit. All other register bits are unaffected.

	//X826( S826_DioOutputSourceWrite(BOARD, DIOMASK((uint64)0)) );   // Assigns the signal sources for all DIO pins.
	X826(S826_DioOutputWrite(BOARD,data,mode));

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

	//uint64 dio = DIO(chan);
	//outputV = ~outputV;
	uint data[2] = DIOMASK((uint64)outputV);
	uint mode = 0;

	//printf("outputV is %i, decision is %i.\n",outputV,((outputV == 0) || (outputV == 1)));

	// Verify the outputV.
	//if (!((outputV == 0) || (outputV == 1))) {printf("Error in s826_doPin about variable outputV.\n"); return -1;}

	//mode = 2 - 1*outputV;    // mode 1: Clear (program to '0') all register bits that have '1' in the corresponding data bit. All other register bits are unaffected.
				 // mode 2: Set (program to '1') all register bits that have '1' in the corresponding data bit. All other register bits are unaffected.

	//X826( S826_DioOutputSourceWrite(BOARD, DIOMASK((uint64)0)) );

	X826(S826_DioOutputWrite(BOARD,data,mode));
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
    X826( S826_CounterStateWrite(BOARD, counter, 0)                         );     // halt channel if it's running
    X826( S826_CounterModeWrite(BOARD, counter, TMR_MODE)                   );     // configure counter as periodic timer
    X826( S826_CounterSnapshotConfigWrite(BOARD, counter, 4, S826_BITWRITE) );     // capture snapshots at zero counts
    X826( S826_CounterPreloadWrite(BOARD, counter, 0, period)               );     // timer period in microseconds
    X826( S826_CounterStateWrite(BOARD, counter, 1)                         );     // start timer
    return errcode;
}

// Halt channel operating as periodic timer.
int PeriodicTimerStop(uint counter)
{
    return S826_CounterStateWrite(BOARD, counter, 0);
}

// Wait for periodic timer event.
int PeriodicTimerWait(uint counter, uint *timestamp)
{
    uint counts, tstamp, reason;    // counter snapshot
    int errcode = S826_CounterSnapshotRead(BOARD, counter, &counts, &tstamp, &reason, S826_WAIT_INFINITE);    // wait for timer snapshot
    if (timestamp != NULL)
        *timestamp = tstamp;
    return errcode;
}

int waitUsePeriodicTimer(uint waitTime)
{
    uint counter    = 0;            // counter channel to use as a periodic timer (0 to 5)
    uint period     = waitTime;       // timer interval in microseconds (500000 = 0.5 seconds)

    int i;
    int errcode;
    uint tstamp;    // counter snapshot's timestamp
    //uint counts;

    //printf("\nDemoPeriodicTimer\n");

    X826( PeriodicTimerStart(counter, period) );     // configure counter as periodic timer and start it running.

    //for (i = 0; i < 10; i++)                                // repeat a few times ...
    //{
        X826( PeriodicTimerWait(counter, &tstamp) );     // wait for timer snapshot
        //X826( S826_CounterRead(board, counter, &counts) );      // get counts - just to exercise CounterRead
        //printf("timestamp:%u\n", (uint) tstamp); // report timestamp
    //}

    X826( PeriodicTimerStop(counter) );              // halt timer
    return errcode;
}
