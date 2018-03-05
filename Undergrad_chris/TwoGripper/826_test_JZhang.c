////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Program created by Jiachen Zhang on 2014-08-12.
// Program modified by Jiachen Zhang on 2014-08-13~19.
// Program modified by ... on ... (Please follow this format to add any following modification info.)
// 

//#ifndef _LINUX
//#include "..\826api.h"
//#else
//#include "826api.h"
//#endif

#include "s826_subroutine.h"
//#include "stdio.h"
//#include "stlib.h"

/*
#include "826api.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ERROR HANDLING
// These examples employ very simple error handling: if an error is detected, the example functions will immediately return an error code.
// This behavior may not be suitable for some real-world applications but it makes the code easier to read and understand. In a real
// application, it's likely that additional actions would need to be performed. The examples use the following X826 macro to handle API
// function errors; it calls an API function and stores the returned value in errcode, then returns immediately if an error was detected.

#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array
*/


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEMO: GENERATE SINE WAVE OUTPUT
// Resources: Counter channel (used as periodic timer), analog output channel.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PI  3.1415926535
 int i=0;
 double Vout=0.000;
 FILE *fp;

/*
static int DemoSinewaveGenerator(uint board,uint aout)
{
    // Configuration values
    double freq     = 50.0;         // sine frequency in Hz
    uint amplitude  = 32767;        // peak amplitude (32767=10V). Must be < 32768
    uint tsample    = 250;          // sample time in microseconds. Must be fast enough to meet nyquist
    double duration = 5.0;          // waveform duration in seconds
    //uint aout       = 0;            // output sine wave on thiS 826_WAIT_INFINITEs dac channel
    uint counter    = 0;            // use this counter channel as dac sample timer

    uint tstamp;                    // snapshot timestamp
    uint tbegin;                    // timestamp at t0
    double runtime;                 // runtime in seconds
    uint dacval;                    // dac setpoint
    int errcode;

    printf("\nDemoSinewaveGenerator\n");

    X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_10_10, 0)   );      // program dac output range: -10V to +10V
    X826( PeriodicTimerStart(board, counter, tsample)               );      // configure counter0 as periodic timer and start it running
    X826( PeriodicTimerWait(board, counter, &tstamp)                );      // wait for first sampluint diomask[]  = DIOMASK( DIO(diochan) );e time (t0)
    tbegin = tstamp;                                                        // get t0 timestamp

    do {                                                                    // repeat until waveform duration has elapsed:

        runtime = (double)(tstamp - tbegin) / 1000000.0;                        // compute time elapsed since t0
        dacval  = (uint)(32768 + amplitude * sin(2.0 * PI * freq * runtime));   // compute sinewave amplitude based on elapsed time

        X826( S826_DacDataWrite(board, aout, dacval, 0)  );         // output next sinewave sample
        errcode = PeriodicTimerWait(board, counter, &tstamp);         // wait for next sample time
        if (errcode == -15) {
            printf("overflow, make thread priority higher and/or use low latency kernel\n");
        } else if (errcode != 0) {
            printf("\nERROR: %d\n", errcode); return errcode;
        }
    }
    while (runtime < duration);

    X826( PeriodicTimerStop(board, counter));                               // halt periodic timer
    printf("\nDemoSinewaveGenerator complete[%d]\n", errcode);
    return errcode;
}
*/


// Waiting Function
// Input:
// 	 waitTime: microsecond.




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAM
// Resources:
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	// The "short int" is assumed to have 2 Bytes. If not, give a warning.
	int temp = sizeof(short int);
	if ((int)sizeof(short int) != 2) printf("Warning: The short int type is not as desired. Its # of bit is currently %i.\n",temp);
	/*
	printf("uint %i\n",(int)sizeof(uint));
	printf("short int %i\n",(int)sizeof(short int));
	printf("char %i\n",(int)sizeof(char));
	printf("long %i\n",(int)sizeof(long));
	printf("float %i\n",(int)sizeof(float));
	*/

    //uint board      = 0;                        // change this if you want to use other than board number 0
    int errcode     = S826_ERR_OK;
    int initFlag = s826_init();
    //int boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards

	int rc;
	uint diochan    = 0;                        // detect edges on this dio channel (0 to 47)
	//uint diomask[]  = DIOMASK( DIO(diochan) );  // bit mask for edge detecting dio
	uint diomask[]  = DIOMASK( (uint64)0 );	
	uint mode = 0;

	// Test the DIO. set 0x555555
	//uint data[] = DIOMASK((uint64)0x555555);
	// Test the DIO. set 0xAAAAAA
	//uint data[] = DIOMASK((uint64)0xAAAAAA);
	uint data[]     = DIOMASK(DIO(21));
	
	// DO Pin subroutine test.
	uint chan = 0;
	//uint outputV = 0;
	//s826_doPin(board,chan,outputV);
	s826_doPin(20 , 0); 
s826_doPin(21 , 0); 
s826_doPin(22 , 0); 

	// DO Whole subroutine test.
	//uint outputV = 0xAAAAAA;
	//s826_doWhole(board,outputV);
    
    //if (boardflags < 0)
    //    errcode = boardflags;                       // problem during open
    //else if ((boardflags & (1 << board)) == 0)
    //    printf("TARGET BOARD NOT FOUND\n");         // driver didn't find board you want to use
    //else
    //{

	    // enable writing to safe mode registers
	    //rc = S826_SafeWrenWrite(board, 0x02);
	    //if (rc != 0) {
		//printf("failed to enable wren for watchdog\n");
		//return rc;
	    //}

		//S826_DioOutputSourceWrite(board, diomask);

		//S826_DioOutputWrite(
		// 			board,   // board identifier
		// 			data,    // pointer to DIO data
		// 			mode);   // 0=write, 1=clear bits, 2=set bits
		uint aoChan  = 0;
		uint aoRange = 2;   // 2: -5 ~ +5 V.
		
		X826(s826_aoPin(aoChan,aoRange,5));

		aoChan = 1;
		X826(s826_aoPin(aoChan,aoRange,-5));
		//uint range[]    = {0,1,2,3,0,1,2,3};
		//uint setpoint[] = {0x8000,0x8000,0x8000,0x8000,0x8000,0x8000,0x8000,0x8000};
		//double outputV[] = {3,8,-2,-7,1,9,2,-5};
		//X826(s826_aoWhole(board,range,outputV))
		//X826( DemoSinewaveGenerator(board,7)  );      // analog sinewave output
	//int i = 0;
	//for(i=0;i<16;i++)
	//{	
	uint aiChan = 1 | (1<<11);   // Set AI0 and AI-11.
	printf("aiChan is 0x%x.\n",aiChan);
	double aiV[16];             // Buffer to receive the AI reading.

	uint aiSlot = 0;
	uint tsettle = 500;   // settling time in microseconds 
	uint aiRangeCode = 1; // 1: -5 ~ +5 V.

	fp=fopen("myf.txt","w");
	//print the frequency in file
        //fprintf(fp,"10\n");//the value referred to waitUsePeriodicTimer
		X826( s826_aiInit(aiChan,aiRangeCode) );
	      
	for(i=0;i<100;i++)
	{
		X826( s826_aiRead(aiChan,aiV) );
                //Vout=s826_aiRead(aiChan,aiV);
		//usleep(1e9);
		printf("The channel0 value is %f, and channel 3 value is %f.\n",aiV[0],aiV[11]);
		fprintf(fp,"%f\n",aiV[11]);
		//X826(s826_aoPin(0,aoRange,5-i));
		//X826(s826_aoPin(1,aoRange,i-5));

		waitUsePeriodicTimer(5e5);//1e5:10hz  5e4:20hz  2e4:50hz 1e4:100hz
		//printf("%f\n",Vout);
		//fprintf(fp,"%f\n",Vout);
                //fprintf(fp,"%.3f",Vout);
		//fprintf(fp,"hello");
	}
	
	fclose(fp);
	//}

	
    //}



    switch (errcode)
    {
    case S826_ERR_OK:           break;
    case S826_ERR_BOARD:        printf("Illegal board number"); break;
    case S826_ERR_VALUE:        printf("Illegal argument"); break;
    case S826_ERR_NOTREADY:     printf("Device not ready or timeout"); break;
    case S826_ERR_CANCELLED:    printf("Wait cancelled"); break;
    case S826_ERR_DRIVER:       printf("Driver call failed"); break;
    case S826_ERR_MISSEDTRIG:   printf("Missed adc trigger"); break;
    case S826_ERR_DUPADDR:      printf("Two boards have same number"); break;S826_SafeWrenWrite(BOARD, 0x02);
    case S826_ERR_BOARDCLOSED:  printf("Board not open"); break;
    case S826_ERR_CREATEMUTEX:  printf("Can't create mutex"); break;
    case S826_ERR_MEMORYMAP:    printf("Can't map board"); break;
    default:                    printf("Unknown error"); break;
    }

	    
	//#ifndef _LINUX	
	//    printf("\nKeypress to exit ...\n\n");
	//    while (!_kbhit());
	//    _getch();
	//#endif

        //S826_SystemClose();
	s826_close();

	return 0;
}

