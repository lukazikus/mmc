////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Program created by Jiachen Zhang on 2014-08-12.
// Program modified by Jiachen Zhang on 2014-08-13~20.
// Program modified by ... on ... (Please follow this format to add any following modification info.)

#ifndef S826_SUBROUTINE_H
#define S826_SUBROUTINE_H

#include "s826api.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>       // For struct timespec.
#include <inttypes.h>
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

#define NELEMS(x)  (sizeof(x) / sizeof(x[0]))

#define BOARD   (0)     // The board identifier is assumed to be always 0.
//#define TSETTLE (500)   // ADC. Settling time in microseconds.
#define TSETTLE (0)															// ADC. Settling time in microseconds. For reading a single channel, no settling time is needed
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define _POSIX_C_SOURCE 200809L   //use the function gettime

// SUBROUTINE
int s826_init(void);
int s826_close(void);
int s826_aoPin(uint chan,uint rangeCode,double outputV);
int s826_aoWhole(uint rangeCode[8], double outputV[8]);

// Analog Input
int s826_aiInit(uint chan,uint range);
int s826_aiRead(uint chan,double *aiV);

// DIGITAL OUPUT
int s826_doPin(uint chan,bool outputV) ;                              // Set 1 DO channel to be high/low voltage.
int s826_doWhole(uint outputV);                                       // Set all DO channel to be high/low voltage.

void print_current_time_with_ms (void);
int waitUsePeriodicTimer(uint waitTime);

//FILE *fp;

#endif
