// This file contains the declarations of the public available functions about Magnetic Gripper Control.

#ifndef GRIPPER
#define GRIPPER

#include "math.h"               // math related functions and values
#include "s826_subroutine.h"    // s826 board subroutines
#include "vision.h"             // required for vision-based feedback control

int gripper_initRotationDemo(void);          // initialize the gripper's rotation demo thread

int gripper_setGrippingMotion(float d);      // change the level of gripping

int gripper_setLX(float d);                  // Planar Locomotion
int gripper_setLY(float d);


#endif
