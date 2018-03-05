#ifndef TWOSWIMMERS
#define TWOSWIMMERS

#include "constantValue.h"     // Values of the constants used
#include "vision.h"
#include "AccelStepper.h"
#include "s826_subroutine.h"
#include "math_subroutine.h"
#include "coilFieldControl.h"

int initiate_UT_follow(void);
int set_angle_difference(float);
int stop_everything(void);

int mS_getNetMagnetizationDir(void);
int set_netmagnetizationdir(float d);
int set_netmagnetization2nddir(float d);

#endif // TWOSWIMMERS
