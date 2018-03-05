#ifndef OPTIMIZATION_AGENTS
#define OPTIMIZATION_AGENTS

#include "coilFieldControl.h"
#include "constantValue.h"     // Values of the constants used
#include "vision.h"
#include "AccelStepper.h"
#include "s826_subroutine.h"
# include "math_subroutine.h"
#include "multiagent.h"
#include <stdio.h>
#include <math.h>

#include "pthread.h"
#include "math.h"

////    This part includes threads
//int stop_everything_MA(void);
int opt2agent_law_start(void);    // 2-Agent optimization law
//int getCtrlType(void);
//float* getDesiredVal(void);
int optim_stop_thread (void);
float* getDesiredVal_opt(void);


////    This part includes declaration of referenced functions
void set_Bmag_MA_to_opt(float d);                                                   // assign a value to external magentic field mag-parameter in P_control design

int set_pullingCtrl_to(void);                                                 // to initiate pulling thread : drag coefficient and magnetic moments
void set_pair_ratios_to(float);                                       // To set ratios between coils in multi_agent
void set_SATadj_to(float);                                       // To set coil saturation in multi_agent
void set_integralGain_MA_to(float);                                // To  adjust integral gain
void set_zfieldAdj_MA_to(float);                                // To adjust z-field in constrained
int optim_pause_thread (bool input);				// pause thread for open-loop action

#endif
