#ifndef COILROBOT
#define COILROBOT

#include "AccelStepper.h"
#include "coilFieldControl.h"
#include "constantValue.h"     // Values of the constants used
#include "vision.h"
#include "math_subroutine.h"
#include "optimizationagents.h"

// NOTE: Setting            The settings for image detection of binary controller : min radius: 10   blur size: 6 or 7  swap threshold: 25 to 35  swap ratio: 4

//    This part includes threads
int initiate_UT_follow(void);
int set_angle_difference(float);
int Binary_control_law(void);      // Binary and P-controller
float* getDesiredVal(void);
float*  getfieldAngMA(void);

// This part includes declaration of referenced functions
int set_K_Pctrl_to(float d);
int set_K_Constrainedctrl_to(float d);
int set_Factor_Constrainedctrl_to(float d);
int set_ROTperiod_to(int d);
void set_Pctrl_flag_on(int d);
void set_Pctrl_Straight_flag_on(int d);

void set_Bmag_MA_to(float d);                                                   // assign a value to external magentic field mag-parameter in P_control design

float dis_cal_fun_MA (float, float);

void set_des_angMA_to(float d);                                                   // assign a value to external magentic field mag-parameter in P_control design

int multiagent_stop_thread (void);												// stop thread
int multiAgent_set_desired_rotation_and_sep (float input0, float input1, float input2, float input3, float input4, float input5);	// set the desired pair rotation (phi) and separation, called by twoGripper.c
int multiAgent_pause_thread (bool input);				// pause thread for open-loop action
void set_pullinconstant_MA_to(float d);                                       // To  adjust pulling gain
float multiagent_get_global_field_angle (void);
int enable_or_disable_gradient (int input);
#endif
