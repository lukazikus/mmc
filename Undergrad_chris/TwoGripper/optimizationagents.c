////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Program created by Mohammad on 2015-06-25.
// The purpose is to align permanent magnets placed in shell-mold along a certain direction.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "optimizationagents.h"

static bool fThread = true;              // TO enable pulling control thread
static bool fPause = false;              // pause thread for open-loop action
static float outputVolt[6] = {0,0,0,0,0,0};                         // output voltage to amplifiers
static float uniformFieldVolt[6] = {0,0,0,0,0,0};                   // uniform component of magnetic field, x, y, z
static float gradientFieldVolt[4] = {0,0,0,0};                      // gradient field signal volt.
static float pullGain = 5.0;                        // gain for calc. field gradient for pulling

static int N_traj= 0;
static float angle_des=0;
static float alpha_angle=0;
static int* center_coor_MA[1];
static int* goalMouse_MA[1];
static float* dis_coor_MA[1];
static float Target_dis_MA_new=0;
static float cal_dis_MA=0;
static float B_strength_MA = 7;
static float Des_OUT[7] = {0,0,0,0,0,0,0};

static float field_angle_MA_Binary=0;
static float input_angleVAr_MA=0;
static float field_angle_MA_local=0;

/// Optimization variables
static float  Coilpair_ratio_MA = 1;          // ratio between the pair of coils
static float  saturationGradientFieldVolt = 0.5;                 // saturation field strength for gradient generation
static float  integral_k = 1.2e-8;            // ratio between the pair of coils
static float  AdjZ = 1;                 // adj z-field to make rotation possible
static float* des_valMA[1];

float*  getDesiredVal_opt(void) {
    Des_OUT[0] = Target_dis_MA_new;
    //Des_OUT[0] = Target_dis_MA;
    Des_OUT[1] = angle_des;   // uncomment for multi-agent
    //Des_OUT[1] = M_PI_2;   // uncomment for optimization
    Des_OUT[2] = field_angle_MA_Binary;
    Des_OUT[3] = input_angleVAr_MA;
    Des_OUT[4] = field_angle_MA_local;
    Des_OUT[5] = N_traj;
    Des_OUT[6] = alpha_angle;    // out of plane angle alpha

    return Des_OUT;
}

int optim_stop_thread (void) {
    fThread = false;
    return 1;
}

void set_Bmag_MA_to_opt(float d) {                                                  // assign a value to external magentic field mag-parameter in P_control design
    B_strength_MA = d;
}

void  set_pair_ratios_to(float d) {                                                // To set desired pulling angle in multi-agent
	Coilpair_ratio_MA = d;
}

void  set_SATadj_to(float d) {                                                // To set desired saturation field in multi-agent
	saturationGradientFieldVolt = d;
}

void  set_integralGain_MA_to(float d)                                         // To  adjust integral gain
{
	//integral_k = d;
		integral_k = d*1e-8;
}

void  set_zfieldAdj_MA_to(float d)                                         // To  adjust integral gain
{
	//integral_k = d;
		AdjZ = d*1;
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
    s826_aoPin(2, 2, 0);                     // z-bottom 5.276  1st amplifier
    s826_aoPin(5, 2, 0);                     // z-top  4.44      2nd amplifier
    s826_aoPin(1, 2, outputVolt[3]);         // y-right  4.96     3rd amplifier
    s826_aoPin(4, 2, outputVolt[2]);         // y-left    5.296   4th amplifier
    s826_aoPin(3, 2, outputVolt[1]);         // -x coil  5.14     5th amplifier
    s826_aoPin(0, 2, outputVolt[0]);         // +x coil     5.32    6th amplifier
    printf("output voltage %.2f, %.2f, %.2f, %.2f, 0, 0\n",outputVolt[0],
            outputVolt[1],outputVolt[2],outputVolt[3]);
    return 1;
}
// Create the driving field signal:   uniform field taken from multiagent.m and summed up with a saturated gradient field
static void * Thread_pulling_control (void * threadid) {
    printf("@ the Beginning of Thread_pulling_control. \n");

    float globalFieldAngle = 0.0, globalFieldAngleMemo = 0.0;                   // output field angle for constant field and its memory
    float angle_des=0, pull_const=0, errorPull=0, COM_coorX=0, COM_coorY=0, cycleAmp=0;
    double pulling_angMA=0, pulling_angDeg=0;
    double time_next=0, time_now=0, norm_vecP[2]={0,0}, Exp_MA=0;
    int countme=0;
    double center_coor_MAx=0;
    double center_coor_MAy=0;
    double goalMouse_MAx=0;
    double goalMouse_MAy=0;

    while (fThread == true) {
        while (fPause);
        goalMouse_MA[0] = getGoalPointCoor();               // get mouse key goal point; vision.c
        //dis_coor_MA[0] =  getDistanceCoor();
        center_coor_MA[0] =  getCenterPointCoor_MA();      // uncomment only for multi-agent P-controller

        // Center of Mass coordinates
        COM_coorX = 0.5 * (center_coor_MA[0][0] + center_coor_MA[0][3]);
        COM_coorY = 0.5 * (center_coor_MA[0][1] + center_coor_MA[0][4]);
        errorPull = sqrt(pow(goalMouse_MA[0][0] - COM_coorX, 2) + pow(goalMouse_MA[0][1] - COM_coorY, 2));      // dis. from present COM to goal COM
        pulling_angMA = atan2((float)(goalMouse_MA[0][1] - COM_coorY), (float)(goalMouse_MA[0][0] - COM_coorX));   // to conform the reversal between force and field coordinate

        // integral action
        pull_const = 1e-2 * pullGain * errorPull;   //  P plus PI controller for pulling

        des_valMA[0] = getDesiredVal();              // uncomment for optimization application

         //pull_const = 0.0;                       // test the orientatin control
         gradientFieldVolt[0] = pull_const * cos(pulling_angMA) / (Coilpair_ratio_MA*11.4);
         gradientFieldVolt[1] = pull_const * cos(pulling_angMA) / (Coilpair_ratio_MA*11.4);
         gradientFieldVolt[2] = pull_const * sin(pulling_angMA) / (Coilpair_ratio_MA*11.4);
         gradientFieldVolt[3] = pull_const * sin(pulling_angMA) / (Coilpair_ratio_MA*11.4);

         //globalFieldAngle = dis_coor_MA[0][1] + angle_des;
         globalFieldAngle = des_valMA[0][2];
         printf("uniform field angle %.2f, pulling angle %.2f\n", globalFieldAngle * 180/M_PI, pulling_angMA * 180 / M_PI);

         // generating z-field with amplitude z_field and frequency of freqZ
         //pull_const = 0;  /// important!!!!!!!!!!!!!!!!

         while (get_abs_angle_diff(globalFieldAngle, globalFieldAngleMemo) > M_PI_2 / 3.0) {
             globalFieldAngleMemo = globalFieldAngleMemo + get_sign (adjust_angle_range (globalFieldAngle - globalFieldAngleMemo)) * M_PI_2 / 3.0;
             uniformFieldVolt[0] = B_strength_MA * cos(globalFieldAngleMemo) / 5.32;         // x-left
             uniformFieldVolt[1] = B_strength_MA * cos(globalFieldAngleMemo) / 5.14;         // x-right
             uniformFieldVolt[2] = B_strength_MA * sin(globalFieldAngleMemo) / 5.2;          // y-left
             uniformFieldVolt[3] = B_strength_MA * sin(globalFieldAngleMemo) / 5.2;          // y-right

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
             usleep(1e4);
         }
         /// Load signal to hardware

        //printf("angle_des %.2f, readangle %.2f, globalFieldAngle angle %.2f\n",angle_des, dis_coor_MA[0][1], globalFieldAngle);
        //printf("field const voltage %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",uniformFieldVolt[0],
                //uniformFieldVolt[1],uniformFieldVolt[2],uniformFieldVolt[3],
                //uniformFieldVolt[4],uniformFieldVolt[5]);
        uniformFieldVolt[0] = B_strength_MA * cos(globalFieldAngle) / 5.32;         // x-left
        uniformFieldVolt[1] = B_strength_MA * cos(globalFieldAngle) / 5.14;         // x-right
        uniformFieldVolt[2] = B_strength_MA * sin(globalFieldAngle) / 5.2;          // y-left
        uniformFieldVolt[3] = B_strength_MA * sin(globalFieldAngle) / 5.2;          // y-right
        // assume desired force of 1 uN

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
        usleep(1e5);
    }
    coilCurrentClear();
}

int set_pullingCtrl_to(void) {
    fThread = true;     // to pull agents
    pthread_t PullingCTRL_Thread;
    pthread_create( &PullingCTRL_Thread, NULL, Thread_pulling_control, NULL);
    return 1;
}

// pause thread for open-loop action
int optim_pause_thread (bool input) {
    fPause = input;
    return 1;
}
