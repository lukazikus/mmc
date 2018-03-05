#ifndef MULTISWIMMERTRIAL
#define MULTISWIMMERTRIAL

#include "constantValue.h"     // Values of the constants used
#include "vision.h"
#include "AccelStepper.h"
#include "s826_subroutine.h"
# include "math_subroutine.h"


int initiate_multiswimmer_trial(void);
int set_angle_difference(float);
int stop_everything(void);

int mS_getNetMagnetizationDir(void);
int set_netmagnetizationdir(float d);
int set_netmagnetizationdir2(float d);

#endif // MULTISWIMMERTRIAL
