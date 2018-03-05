#ifndef TWOGRIPPER
#define TWOGRIPPER

#include "classDefJZ.h"
#include "coilFieldControl.h"	// controlling DAQ board
#include "math_subroutine.h"
#include "multiagent.h"			// set desired pair orientation and separation
#include "optimizationagents.h"
#include "vision.h"             // required for vision-based feedback control

// Functins
int twoGripper_start_or_stop (int d);   // start or stop 2 grippers thread
int twoGripper_define_cargo_pos(void);  // define cargo positions by mouse clicking
int twoGripper_start_or_stop_path_follow (int input);
int twoGripper_drop_gripper (void);
#endif
