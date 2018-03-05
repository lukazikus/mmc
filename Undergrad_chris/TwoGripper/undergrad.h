#ifndef UNDERGRAD
#define UNDERGRAD

#include "classDefJZ.h"
#include "coilFieldControl.h"	// controlling DAQ board
#include "math_subroutine.h"
#include "multiagent.h"			// set desired pair orientation and separation
#include "optimizationagents.h"
#include "vision.h"             // required for vision-based feedback control
#include "PageGeneralControl.h"


int undergrad_start_stop_demo (int input);  // start/stop the demonstration thread

#endif
