////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Program created by Jiachen Zhang on 2014-08-21.
// Program modified by Jiachen Zhang on 2014-08-21~28.
// Program modified by ... on ... (Please follow this format to add any following modification info.)
// 

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "s826_subroutine.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOTOR PIN CONNECTION

// Channel 1: Pin 0 to 5; Channel 2: Pin 6 to 11; Channel 3: Pin 12 to 17; Channel 4: Pin 18 to 23.

#define _motor_enable (0)   // Motor enable DO Pin.

#define _motor_dir    (1)   // Motor direction control DO pin.

#define _motor_ms_1   (2)   // Motor step size control DO pin_1.
#define _motor_ms_2   (3)   // Motor step size control DO pin_2.
#define _motor_ms_3   (4)   // Motor step size control DO pin_3.

#define _motor_stp    (5)   // Motor step control DO pin.

#define STEPMIN         (800) // Used in currentControl function.

// SUBROUTINES
int motorInit(void);
int motorClose(void);

int motorGo(int motnum, int step_size, int number_of_steps, int torque);
int motorAllGo(uint *stepSize,int *nStep,int torque);

int motorStall(int motnum);
int motorRelease(int motnum);


#endif
