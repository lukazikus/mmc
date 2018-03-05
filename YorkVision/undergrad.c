#include "undergrad.h"

static int fThread = 0;
static float gradientDir = 0.0;                 // gradient angle (radian) w.r.t. +x
static int fGradient = 0;                       // whether or not apply a gradient
static float gradientAmpX = 7;
static float gradientAmpY = 3;

//// Global MMC variables (altered by threads)
static float PgainMMC = 10;   // P gain in MMC
static float IgainMMC = 10;   // I gain in MMC
static float DgainMMC = 10;   // D gain in MMC
static int MMCThread = 0;   // MMC flag :: initially disabled


static float outputVolt[6] = {0,0,0,0,0,0};                         // output voltage to amplifiers
static float uniformFieldVolt[6] = {0,0,0,0,0,0};                   // uniform component of magnetic field, x, y, z
static float gradientFieldVolt[4] = {0,0,0,0};                      // gradient field signal volt.
static float pullGain = 5.0;                        // gain for calc. field gradient for pulling

static int N_traj= 0;
static float angle_des=0;
static float alpha_angle=0;

static float* center_coor_MA[1]; // Current location of microrobot
static int* goalMouse_MA[1]; // Goal location of next position for microrobot
static float* dis_coor_MA[1];
static float Target_dis_MA_new=0;
static float cal_dis_MA=0;
static float B_strength_MA = 7; // [mT]
static float Des_OUT[7] = {0,0,0,0,0,0,0};

static float field_angle_MA_Binary=0;
static float input_angleVAr_MA=0;
static float field_angle_MA_local=0;

static float  Coilpair_ratio_MA = 1;          // ratio between the pair of coils
static float  saturationGradientFieldVolt = 0.5;                 // saturation field strength for gradient generation
static float  integral_k = 1.2e-8;            // ratio between the pair of coils
static float* des_valMA[1];

static int saturate_gradient_field_signal () {
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
        // printf("Saturated +Y1 \n");
    }
    if (gradientFieldVolt[2] < -saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[2] = -saturationGradientFieldVolt;
        // printf("Saturated -Y1 \n");
    }

    if (gradientFieldVolt[3] > saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[3] = saturationGradientFieldVolt;
        // printf("Saturated +Y2 \n");
    }
    if (gradientFieldVolt[3] < -saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[3] = -saturationGradientFieldVolt;
        // printf("Saturated -Y2 \n");
    }
    return 1;
}
/*
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
*/

static int add_constant_and_gradient_field_signal () {
    if (gradientFieldVolt[0] >= 0) {
        outputVolt[0] = gradientFieldVolt[0];
        outputVolt[1] = 0;
    } else {
        outputVolt[0] = 0;
        outputVolt[1] = gradientFieldVolt[0];
    }

    if (gradientFieldVolt[2] >= 0) {
        outputVolt[2] = gradientFieldVolt[2];
        outputVolt[3] = 0;
    } else {
        outputVolt[2] = 0;
        outputVolt[3] = gradientFieldVolt[2];
    }

    return 1;
}

static int fVibrate = 1;
static int output_signal_to_amplifier (void) {
    /*
    if (fVibrate)
        fVibrate = 0;
    else
        fVibrate = 1;
    */
    /*
    s826_aoPin(2, 2, fVibrate * 0.1);                     // z-bottom 5.276  1st amplifier
    s826_aoPin(5, 2, 0);                     // z-top  4.44      2nd amplifier
    s826_aoPin(1, 2, outputVolt[3]);         // y-right  4.96     3rd amplifier
    s826_aoPin(4, 2, outputVolt[2]);         // y-left    5.296   4th amplifier
    s826_aoPin(3, 2, outputVolt[1]);         // -x coil  5.14     5th amplifier
    s826_aoPin(0, 2, outputVolt[0]);         // +x coil     5.32    6th amplifier
    */

    s826_aoPin(2, 2, 0);                     // z-bottom 5.276  1st amplifier
    s826_aoPin(5, 2, 0);                     // z-top  4.44      2nd amplifier
    s826_aoPin(1, 2, outputVolt[3]);         // y-right  4.96     3rd amplifier
    s826_aoPin(4, 2, outputVolt[2]);         // y-left    5.296   4th amplifier
    s826_aoPin(3, 2, outputVolt[1]);         // -x coil  5.14     5th amplifier
    s826_aoPin(0, 2, outputVolt[0]);         // +x coil     5.32    6th amplifier


    // printf("output voltage %.2f, %.2f, %.2f, %.2f, 0, 0\n",outputVolt[0],
            // outputVolt[1],outputVolt[2],outputVolt[3]);
    return 1;
}

static void * autonomy_thread(void * threadid){
    // Pulling thread
    printf("MMC_thread starts\n");

    float globalFieldAngle = 0.0, globalFieldAngleMemo = 0.0;                   // output field angle for constant field and its memory
    float angle_des=0, pull_const=0, int_const=0, der_const=0, errorPull=0, errorSum=0, errorDiff=0, COM_coorX=0, COM_coorY=0, cycleAmp=0;
    double pulling_angMA=0, pulling_angDeg=0;
    double time_next=0, time_now=0, norm_vecP[2]={0,0}, Exp_MA=0;
    int countme=0;
    double center_coor_MAx=0;
    double center_coor_MAy=0;
    double goalMouse_MAx=0;
    double goalMouse_MAy=0;

    // RUNNING
    while ( MMCThread ) {
        printf("Executing task \n");
        center_coor_MA[0] = get_coords(); // Get updated coordinates of camera
        // printf("%f, %f", center_coor_MA[0][0], center_coor_MA[0][1]);
        goalMouse_MA[0] = getGoalPointCoor();               // get mouse key goal point; vision.c
        // printf("%d, %d  \n", goalMouse_MA[0][0], goalMouse_MA[0][1]);
        //usleep(1e5);

        // Center of Mass coordinates
        COM_coorX = center_coor_MA[0][0];
        COM_coorY = center_coor_MA[0][1];
        errorPull = sqrt(pow(goalMouse_MA[0][0] - COM_coorX, 2) + pow(goalMouse_MA[0][1] - COM_coorY, 2));      // dis. from present COM to goal COM
        pulling_angMA = atan2((float)(goalMouse_MA[0][1] - COM_coorY), (float)(goalMouse_MA[0][0] - COM_coorX));   // to conform the reversal between force and field coordinate
        //printf("(X,Y) = (%f, %f),  (E1,E2) = (%f, %f)\n",COM_coorX, COM_coorY, errorPull, pulling_angMA);

        // PID control
        pull_const = 1e-2 * PgainMMC; // * errorPull;   //  Proportional term
        int_const = 1e-2 * IgainMMC * errorSum; // Integral term
        der_const = 1e-2 * DgainMMC * errorDiff;

        //pull_const = 0.0;                       // test the orientation control
        gradientFieldVolt[0] = pull_const * cos(pulling_angMA) / (Coilpair_ratio_MA*11.4); // Outer coils
        gradientFieldVolt[1] = pull_const * cos(pulling_angMA) / (Coilpair_ratio_MA*11.4);
        gradientFieldVolt[2] = pull_const * sin(pulling_angMA) / (Coilpair_ratio_MA*20); // Inner coils
        gradientFieldVolt[3] = pull_const * sin(pulling_angMA) / (Coilpair_ratio_MA*20);
        // printf("pulling angle %.3f\n", pulling_angMA * 180 / M_PI);

        // printf("Gradient Fields: %f, %f, %f, %f\n", gradientFieldVolt[0], gradientFieldVolt[1], gradientFieldVolt[2], gradientFieldVolt[3]);

        globalFieldAngle = pulling_angMA;
        // printf("uniform field angle %.2f, pulling angle %.2f\n", globalFieldAngle * 180/M_PI, pulling_angMA * 180 / M_PI);

        // generating z-field with amplitude z_field and frequency of freqZ
        //pull_const = 0;  /// important!!!!!!!!!!!!!!!!

        /* TODO: if statement
        uniformFieldVolt[0] = B_strength_MA * cos(globalFieldAngle) / 5.32;         // x-left
        uniformFieldVolt[1] = B_strength_MA * cos(globalFieldAngle) / 5.14;         // x-right
        uniformFieldVolt[2] = B_strength_MA * sin(globalFieldAngle) / 5.2;          // y-left
        uniformFieldVolt[3] = B_strength_MA * sin(globalFieldAngle) / 5.2;          // y-right


        // assume desired force of 1 uN

        if (fabs(32*uniformFieldVolt[0]) < 1e-5 || fabs(32*uniformFieldVolt[1]) < 1e-5) {// to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
            gradientFieldVolt[0] = 0;
            gradientFieldVolt[1] = 0;
            printf("x coil set to zero \n");
        }

        if (fabs(32*uniformFieldVolt[2]) < 1e-5 || fabs(32*uniformFieldVolt[3]) < 1e-5) { // to avoid singularities in Y  :: it means field is only in Y-direction, and so the only choice for pulling too
            gradientFieldVolt[2] = 0;
            gradientFieldVolt[3] = 0;
        }
        */

        //saturate_gradient_field_signal (); // Saturate gradient field signal to limit it
        add_constant_and_gradient_field_signal (); // Add gradient and uniform field
        output_signal_to_amplifier (); // Write to amplifier to output current
        usleep(5e4);
}

coilCurrentClear();
printf("MMC_thread ended\n");

}

static void * keyboard_thread (void * threadid) {
    printf("keyboard_thread started\n");
    coilCurrentClear();
    float px=0.0, nx=0.0,py=0.0,ny=0.0,pz=0.0,nz=0.0;
    float xAmp=0.0, yAmp=0.0, zAmp=0.0;
    while ( fThread ) {

    // goalMouse[0] = getGoalPointCoor();               // get mouse key goal point; vision.c
    // center_coor[0] =  getCenterPointCoor_MA();      // get the robot position

    float goal_pose[3] = {500,650,0}; // Goal pose: x,y,theta

    // calculates states errors
    // errorPull = sqrt(pow(goalMouse_MA[0][0] - center_coor[0][0], 2) + pow(goalMouse_MA[0][1] - center_coor[0][1], 2));      // dis. from current position of robot to goal position
    // pulling_ang = atan2((float)(goalMouse_MA[0][1] - center_coor[0][1]), (float)(goalMouse_MA[0][0] - center_coor[0][0]));   // pulling angle
    // // P-control law
    // pull_const = pullGain * errorPull;   //  P plus PI controller for pulling
    // xAmp = pull_const * cos(pulling_ang)+uniform_mag*cos(headingM);  // calculates the current in x-direction to generate desired force_x
    // yAmp = pull_const * sin(pulling_ang)+uniform_mag*sin(headingM);   // calculates the current in y-direction to generate desired force_y

    //printf("field gradient is %.2f\n", gradientDir*180.0/M_PI);
    xAmp = gradientAmpX * cos(gradientDir) ;
    yAmp = gradientAmpX * sin(gradientDir);
    if (xAmp >=0) {
        px = xAmp;
        nx = 0.0;
    } else {
        px = 0.0;
        nx = xAmp;
    }
    if (yAmp >=0) {
        py = yAmp;
        ny = 0.0;
    } else {
        py = 0.0;
        ny = yAmp;
    }
    if (fGradient == 1){
        amplifier_set_each_coil_current (px, nx, py, ny, pz, nz);
    }else{
        coilCurrentClear();
    }
    usleep(1e4);
    }
    coilCurrentClear();
    printf("keyboard_thread ended\n");
}

// toggle the keyboard detection thread
int undergrad_keyboard_init_stop(int d) {
    if (d==1){
        fThread = 1;
        pthread_t keyboardThread;
        pthread_create(&keyboardThread, NULL, keyboard_thread, NULL);
    } else
        fThread = 0;

    return 1;
}

// set gradient dir using keyboard
int undergrad_set_dir(int d) {
    switch (d) {
        case 0: gradientDir = 0.0; fGradient = 1;  break;
        case 1: gradientDir = M_PI_2; fGradient = 1;   break;
        case 2: gradientDir = M_PI; fGradient = 1; break;
        case 3: gradientDir = -M_PI_2; fGradient = 1; break;
        case 4: gradientDir = M_PI_4; fGradient = 1; break;
        case 5: gradientDir =  3.0 * M_PI_4; fGradient = 1; break;
        case 6: gradientDir = -M_PI_4; fGradient = 1; break;
        case 7: gradientDir = -3.0 * M_PI_4; fGradient = 1; break;
        case -1: gradientDir = 0.0; fGradient = 0; break;
    }
    printf("field gradient is %.2f\n", gradientDir*180.0/M_PI);
    return 1;
}

static void * walk_thread (void * threadid) {
    printf("walk_thread started\n");
    coilCurrentClear();
    float px=0.0, nx=0.0,py=0.0,ny=0.0,pz=0.0,nz=0.0;
    float xAmp=0.0, yAmp=0.0, zAmp=0.0;
    float theta = M_PI_4;
    float delta = M_PI_2 / 6.0;
    int index = 0;
    float amp = 5;
    while ( fThread ) {
        xAmp = amp * cos(theta + delta*sin(index/40.0*2*M_PI)) * cos(gradientDir + delta*cos(index/40.0*2*M_PI));
        yAmp = amp * cos(theta + delta*sin(index/40.0*2*M_PI)) * sin(gradientDir + delta*cos(index/40.0*2*M_PI));
        zAmp = amp * sin(theta + delta*sin(index/40.0*2*M_PI));
        px = xAmp;
        nx = xAmp;
        py = yAmp;
        ny = yAmp;
        pz = zAmp;
        nz = zAmp;
        if (fGradient == 1)
            amplifier_set_each_coil_current (px, nx, py, ny, pz, nz);
        index ++;
        if (index >= 40)    index = 0;
        usleep(1e4);
    }
    coilCurrentClear();
    printf("walk_thread ended\n");
}

// toggle the walk thread
int undergrad_walk_init_stop (int d) {
    if (d==1){
        if (fThread == 1) {
            fThread = 0;
            usleep(1e6);
        }
        fThread = 1;
        pthread_t walkThread;
        pthread_create(&walkThread, NULL, walk_thread, NULL);
    } else
        fThread = 0;

    return 1;
}

int undergrad_set_x_gradient(float d) {
    gradientAmpX = d;
    return 1;
}

int undergrad_set_y_gradient(float d) {
    gradientAmpY = d;
    return 1;
}

////////   MMC functions that run after interacting with GUI  //////
int setPgain_MMC(float d) { // Set P gain
    PgainMMC = d;
    return 1;
}

// MMC thread
int on_startMMC_Thread (void) {
        MMCThread = 1;
        pthread_t MMC_Thread;
        pthread_create(&MMC_Thread, NULL, autonomy_thread, NULL);
    return 1;
}
