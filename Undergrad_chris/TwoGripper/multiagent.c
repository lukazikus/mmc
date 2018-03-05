#include "multiagent.h"

static int fGradient = 1;                           // flag for whether or not use gradient
static bool fThread = true;                         // only 1 thread per file
static bool fPause = false;                             // pause thread for open-loop action
static float presSep = 0.0, presOri = 0.0;
static float nextSep = 0.0, nextOri = 0.0;
static float vecLength = 0.0, vecOri = 0.0;          // vec. from present to next com
static float pullGain = 9.0;                        // gain for calc. field gradient for pulling

static float gradientFieldVolt[4] = {0,0,0,0};                      // gradient field signal volt.
static float uniformFieldVolt[4] = {0,0,0,0};                   // uniform component of magnetic field, x, y, z
static float uniformFieldAmp = 7;                               // amp. of uniform magnetic field in mT
static float outputVolt[6] = {0,0,0,0,0,0};                         // output voltage to amplifiers
static float  saturationGradientFieldVolt = 0.5;                 // saturation field strength for gradient generation

static float globalFieldAngle = 0.0;                    // global magneitc field angle
static float globalFieldAngleMemo = 0.0;
static float psi_0 = 54.72 * M_PI / 180.0;              // Lagrange angle; P_controller variable

static int fPController            = 0;                 // flag for P controller; default disable mode
static int fRotationLessController = 1;                 // rotationless controller flag

static float PControlCoeff = 15;

static int N_traj= 0;

// Adaptive control variables for multi-agent project
static float AdaptLambda_MA = 12;
static float AdaptK_MA = 12;
static float set_des_angMA_var = 45;

static float AdaptGammaF_MA = 1000;
static float AdaptGammaM_MA = 1000;
static float Adapt_sigma=0;
static float Adapt_sigma_sat=0;
static float AdaptPhi_MA = 1;
static float Adapt_gamma = 1;
static float Adapt_xdoubledotd = 0;
static float Adapt_xdotd = 0;
static float Adapt_xerrordot = 0;
static float Adapt_xerror = 0;
//static double Adapt_ahat[2]={0,0};
static double Adapt_xdot_MA =0;
static double force_MA=0;
static double check_der=0;
static double Adapt_updateHatparam[2]={0,0};
static double Adaptcontrollaw = 0;
static double prev_law= 0;
static float Actualdist= 0;
static int frame_counter = 0;
static int ctrl_Type_OUT= 0;
static int ctrl_Type_MA= 0;
static float Des_OUT[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
static float field_OUT[1] = {0};
static double fieldVec[3] = {0,0,0};
static double crossvec[3]= {0,0,0};

static float localFieldAngle = 0.0;                 // [0, pi/2]
static float K_contrain_MA = 1e-1;
static float pure_angle_error=0;
static int ROT_sign_MA = 1;
static int ROT_period_MA=0;
//static float enorm_MA;                          // SCATTERING AREA threshold variable
static float Pregion_percent = 0.3;               // Pregion (%)

/// Optimization variables
static double  f_const = -3e-7;                 // force constant
static double  moment = 1e-6;                   // magnetic moment  default 5e-6:: m = 50 means 50*1e-7
static double  Viscos = 1e-3;                   // Viscosity of liquid
static double  ShellRad = 2e-3;                 // Shell Radius

/// Regulator value assignment
static float Reg1 = 3.5;             // regulator 1 in logistic function
static float Reg2 = 1e3;             // regulator 1 in logistic function
static float stepsizemultiple = 1;   // to decrease step-length in optimization program
static float factorP = 1;      // constrained problem weight of separation over orientation

/// Constrained controller variables
static double forceV [2]={0,0};
static double error_vec_angle=0;
static double norm_E_vec=0;
static double force_vec_mag=0;

int multiagent_stop_thread (void) {
    fThread = false;
    return 1;
}

//  Change the field magnitude changed by spin button "moldfieldmag"
void set_Pctrl_flag_on (int d) {
    if (d == 1)
        fPController = 1;
    else if (d == 0)
        fPController = 0;
}

//  Change the field magnitude changed by spin button "moldfieldmag"
void set_Pctrl_Straight_flag_on(int d) {
    if(d == 1) {
        fRotationLessController = 1;
    } else if(d == 0) {
        fRotationLessController = 0;
    }
}

//  Change the field magnitude changed by spin button "moldfieldmag"
int set_K_Pctrl_to(float d) {
    PControlCoeff = d;
    return 0;
}

//  Change the field magnitude changed by spin button "moldfieldmag"
int set_K_Constrainedctrl_to(float d) {
    K_contrain_MA = -1*pow(10,d);    // default scale    k = -d*1e-5
    return 0;
}

//  Change the factor value in constrained problem
int set_Factor_Constrainedctrl_to(float d) {
    factorP = d*1;
    return 0;
}

int set_ROTperiod_to(int d) {
    ROT_period_MA = d;
    return 0;
}

void set_Bmag_MA_to(float d) {                                                   // assign a value to external magentic field mag-parameter in P_control design
    uniformFieldAmp = d;
}

void set_AdaptiveLambda_MA_to(float d) {                                                      // assign a value to Lambda in Adaptive control design
    AdaptLambda_MA = d;
}

void set_AdaptiveK_MA_to(float d) {                                                      // assign a value to K in Adaptive control design
    AdaptK_MA = d;
}

void set_des_angMA_to(float d) {                                                      // assign a value to GammaFluid in Adaptive control design
    set_des_angMA_var = d;
}

void set_AdaptiveGammaF_MA_to(float d) {                                                     // assign a value to GammaFluid in Adaptive control design
    AdaptGammaF_MA = d;
}

void set_AdaptiveGammaM_MA_to(float d)                                                      // assign a value to GammaM in Adaptive control design
{
    AdaptGammaM_MA = d;
}

void set_AdaptivePhi_MA_to(float d) {                                                        // assign a value to Phi in Adaptive control design
    AdaptPhi_MA = d;
}

float * getDesiredVal(void) {
    Des_OUT[0] = nextSep;
    Des_OUT[1] = nextOri;
    Des_OUT[2] = globalFieldAngle;
    Des_OUT[3] = 0.0;
    Des_OUT[4] = localFieldAngle;       // in plane angle psi
    Des_OUT[5] = N_traj;
    Des_OUT[6] = M_PI_2;        // out of plane angle alpha
    Des_OUT[7] = forceV[0];             //  output force x-component
    Des_OUT[8] = forceV[1];             // output force y-component
    Des_OUT[9] = pure_angle_error;
    Des_OUT[10] = error_vec_angle;      // error vec angle
    Des_OUT[11] = norm_E_vec;           // error vec magnitude
    Des_OUT[12] = force_vec_mag;        // force vec magnitude
    return Des_OUT;
}

float *  getfieldAngMA(void) {
    field_OUT[0] = globalFieldAngle;
    return field_OUT;
}

float multiagent_get_global_field_angle (void) {
    return globalFieldAngle;
}

// clculate distance between current position and desired distance
static float calc_dis_to_desired_sep (float goal_dis_keeper, float dis_coor_MARead) {
    float current_dis_MA = goal_dis_keeper - dis_coor_MARead;               // TODO: why use present - target
    return current_dis_MA;
}

static int staturate_gradient_field_signal () {
    if (gradientFieldVolt[0] > saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[0] = saturationGradientFieldVolt;
        printf("Saturated +X1 \n");
    }
    if (gradientFieldVolt[0] < -saturationGradientFieldVolt) {// to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[0] = -saturationGradientFieldVolt;
        printf("Saturated -X1 \n");
    }

    if (gradientFieldVolt[1]>saturationGradientFieldVolt) // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
    {
        gradientFieldVolt[1] = saturationGradientFieldVolt;
        printf("Saturated +X2 \n");
    }
    if (gradientFieldVolt[1]<-saturationGradientFieldVolt) // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
    {
        gradientFieldVolt[1] = -saturationGradientFieldVolt;
        printf("Saturated -X2 \n");
    }
    if (gradientFieldVolt[2] > saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[2] = saturationGradientFieldVolt;
        printf("Saturated +Y1 \n");
    }
    if (gradientFieldVolt[2] < -saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[2] = -saturationGradientFieldVolt;
        printf("Saturated -Y1 \n");
    }

    if (gradientFieldVolt[3] > saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[3] = saturationGradientFieldVolt;
        printf("Saturated +Y2 \n");
    }
    if (gradientFieldVolt[3] < -saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[3] = -saturationGradientFieldVolt;
        printf("Saturated -Y2 \n");
    }
    return 1;
}
static int add_constant_and_gradient_field_signal () {
    if (uniformFieldVolt[0] >= 0)
        outputVolt[0] = uniformFieldVolt[0] + gradientFieldVolt[0];                                        // coil 1.0
    else
        outputVolt[0] = uniformFieldVolt[0] - gradientFieldVolt[0];
    if (uniformFieldVolt[1] >= 0)
        outputVolt[1] = uniformFieldVolt[1] - gradientFieldVolt[1];                                        // coil 1.1
    else
        outputVolt[1] = uniformFieldVolt[1] + gradientFieldVolt[1];
    if (uniformFieldVolt[2] >= 0)
        outputVolt[2] = uniformFieldVolt[2] + gradientFieldVolt[2];                                        // coil 2.0
    else
        outputVolt[2] = uniformFieldVolt[2] - gradientFieldVolt[2];
    if (uniformFieldVolt[3] >= 0)
        outputVolt[3] = uniformFieldVolt[3] - gradientFieldVolt[3];                                        // coil 2.1
    else
        outputVolt[3] = uniformFieldVolt[3] + gradientFieldVolt[3];
    return 1;
}

static int output_signal_to_amplifier (void) {
    s826_aoPin(6, 2, 0);                     // z-bottom 5.276  1st amplifier
    s826_aoPin(5, 2, 0);                     // z-top  4.44      2nd amplifier
    s826_aoPin(1, 2, outputVolt[3]);         // y-right  4.96     3rd amplifier
    s826_aoPin(4, 2, outputVolt[2]);         // y-left    5.296   4th amplifier
    s826_aoPin(3, 2, outputVolt[1]);         // -x coil  5.14     5th amplifier
    s826_aoPin(0, 2, outputVolt[0]);         // +x coil     5.32    6th amplifier
    printf("output voltage %.2f, %.2f, %.2f, %.2f, 0, 0\n",outputVolt[0],
            outputVolt[1],outputVolt[2],outputVolt[3]);
    return 1;
}

// To implement Binary and P-control  law :: attraction or repulsion
static void * Thread_low_layer_controller (void * threadid) {
    printf("@ the Beginning of Thread_low_layer_controller.\n");

    float pullConst = 0.0;                  // for pulling amp.
    float angleError=0.0, angleErrorMemo = 0.0;                         // rotational error for P-controller

    float disToDesiredSep = 0;               // distance of present to desired sep.
    int counter = 0;
    float PControlAngle = 0.0;                                  // local angle obtained from P controller

    while (fThread == true) {
        //center_coor_MA[0] = getCenterPointCoor_MA();          // vision.c
        while (fPause);                                         // pause if open-loop action is undergoing

        disToDesiredSep = calc_dis_to_desired_sep (nextSep, presSep);
        printf("present sep. is %.2f, goal is %.2f\n", presSep, nextSep);
        // binary controller
        if (disToDesiredSep > 0) {                              // present sep. < desired sep.
            if (abs_f(adjust_angle_range(M_PI_2 - localFieldAngle)) <= M_PI_2)
                localFieldAngle = M_PI_2;
            else
                localFieldAngle = -1.0 * M_PI_2;                    // pure repulsion
        } else if (disToDesiredSep < 0) {                       // present sep. > desired sep.
            localFieldAngle  = 0.0;                             // pure attraction
        }

        // rotation-less P-controller starts here
        if ((abs (disToDesiredSep) < psi_0 / (PControlCoeff * 1e-3)) && fRotationLessController) {
            angleError = adjust_angle_range (nextOri - presOri);      // deviation from \phi angle here shown by pi/4
            printf("present angle %.2f, desiredAngle %.2f, angleError %.2f\n", presOri * 180/M_PI, nextOri * 180/M_PI, angleError * 180/M_PI);

            if (angleError > 0) {
                if (ROT_sign_MA == 1)           // previous angleError > 0
                    ROT_sign_MA = 1;
                else {                          // previous angleError < 0
                    if (abs_f(angleError) > 5.0/180.0 * M_PI)
                        ROT_sign_MA = 1;
                }
            } else {
                if ((ROT_sign_MA == 1) && (abs_f(angleError) > 5.0/180.0 * M_PI))           // previous angleError > 0
                    ROT_sign_MA = -1;
                else if (ROT_sign_MA == -1)     // previous angleError < 0
                    ROT_sign_MA = -1;
            }

            localFieldAngle = psi_0 + disToDesiredSep * PControlCoeff * 1e-3;

            // this part of code is added to tell user if the gail is too high to avoid binary behaviour
            if (localFieldAngle > M_PI_2)
                localFieldAngle = M_PI_2;
            else if (localFieldAngle < 0)
                localFieldAngle = 0;
            else
                localFieldAngle = ROT_sign_MA * localFieldAngle;
        }

        globalFieldAngle = adjust_angle_range (presOri + localFieldAngle);
        // pulling
        if (fGradient == 1)
            pullConst = 1e-2 * pullGain * vecLength / 11.4;
        else
            pullConst = 0.0;
        gradientFieldVolt[0] = pullConst * cos(vecOri);
        gradientFieldVolt[1] = gradientFieldVolt[0];
        gradientFieldVolt[2] = pullConst * sin(vecOri);
        gradientFieldVolt[3] = gradientFieldVolt[2];

        while (get_abs_angle_diff(globalFieldAngle, globalFieldAngleMemo) > M_PI_2 / 3.0) {
            globalFieldAngleMemo = globalFieldAngleMemo + get_sign (adjust_angle_range (globalFieldAngle - globalFieldAngleMemo)) * M_PI_2 / 3.0;
            uniformFieldVolt[0] = uniformFieldAmp * cos(globalFieldAngleMemo) / 5.32;         // x-left
            uniformFieldVolt[1] = uniformFieldAmp * cos(globalFieldAngleMemo) / 5.14;         // x-right
            uniformFieldVolt[2] = uniformFieldAmp * sin(globalFieldAngleMemo) / 5.2;          // y-left
            uniformFieldVolt[3] = uniformFieldAmp * sin(globalFieldAngleMemo) / 5.13;          // y-right
            if (fabs(32*uniformFieldVolt[0]) < 1e-5 || fabs(32*uniformFieldVolt[1]) < 1e-5) {// to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
                gradientFieldVolt[0] = 0;
                gradientFieldVolt[1] = 0;
            }
            if (fabs(32*uniformFieldVolt[2]) < 1e-5 || fabs(32*uniformFieldVolt[3]) < 1e-5) { // to avoid singularities in Y  :: it means field is only in Y-direction, and so the only choice for pulling too
                gradientFieldVolt[2] = 0;
                gradientFieldVolt[3] = 0;
            }
            staturate_gradient_field_signal ();
            add_constant_and_gradient_field_signal ();
            output_signal_to_amplifier ();
            printf("damping output field change . . .\n");
            set_agentControlDisplay ( getDesiredVal());             // set values to display in vision.c
            //printf("!!!!!!obtained size %d, actual size %d.\n", (int)sizeof(getDesiredVal()), (int)sizeof(Des_OUT));
            usleep(2e4);
        }
        uniformFieldVolt[0] = uniformFieldAmp * cos(globalFieldAngle) / 5.32;         // x-left
        uniformFieldVolt[1] = uniformFieldAmp * cos(globalFieldAngle) / 5.14;         // x-right
        uniformFieldVolt[2] = uniformFieldAmp * sin(globalFieldAngle) / 5.2;          // y-left
        uniformFieldVolt[3] = uniformFieldAmp * sin(globalFieldAngle) / 5.13;          // y-right

        if (fabs(32*uniformFieldVolt[0]) < 1e-5 || fabs(32*uniformFieldVolt[1]) < 1e-5) {// to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
            gradientFieldVolt[0] = 0;
            gradientFieldVolt[1] = 0;
        }
        if (fabs(32*uniformFieldVolt[2]) < 1e-5 || fabs(32*uniformFieldVolt[3]) < 1e-5) { // to avoid singularities in Y  :: it means field is only in Y-direction, and so the only choice for pulling too
            gradientFieldVolt[2] = 0;
            gradientFieldVolt[3] = 0;
        }
        staturate_gradient_field_signal ();
        add_constant_and_gradient_field_signal ();
        output_signal_to_amplifier ();
        set_agentControlDisplay ( getDesiredVal());             // set values to display in vision.c
        //printf("!!!!!!obtained size %d, actual size %d.\n", (int)sizeof(getDesiredVal()), (int)sizeof(Des_OUT));
        usleep(1e4);
        counter++;
    }
    coilCurrentClear();
    printf("@ the End of Thread_low_layer_controller.\n");
}

// BINARY and P-control FINISHES HERE
int Binary_control_law (void) {
  fThread = true;     // to get the center of object
  pthread_t Binary_control_law_Thread;
  printf("before init. binary control law\n");
  pthread_create(&Binary_control_law_Thread, NULL, Thread_low_layer_controller, NULL);
  return 1;
}

// set the desired pair rotation (phi) and separation, called by twoGripper.c
int multiAgent_set_desired_rotation_and_sep (float input0, float input1, float input2, float input3, float input4, float input5) {
    presSep = input0;
    presOri = input1;
    nextSep = input2;
    nextOri = input3;
    vecLength = input4;
    vecOri = input5;
    return 1;
}

// pause thread for open-loop action
int multiAgent_pause_thread (bool input) {
    fPause = input;
    return 1;
}

void set_pullinconstant_MA_to(float d) {                                     // To  adjust gain in pulling
   pullGain = d;
}

int enable_or_disable_gradient (int input) {
    fGradient = input;
    return 1;
}
