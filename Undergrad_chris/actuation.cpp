#include "actuation.hpp"
#include "undergrad.h"

static int fThread = 0;             // flag of thread running
static int directionCode = 1;      // -1: neutral; 0: +x; 1: +y; 2: -x; 3: -y
static int fKey = 0;                // if a key has been pressed, request reviewing direction
static float angle_heading = 0;     // Angle in degrees to be directly written to the set angle function
static bool undergrad_control = 1;  // Flag for using undergrad path planning and tracking control
static float *output_signals[1];       // This variable will be set to undergrad's get_output_signals function
static int actuationThread = 0;   // Actuation flag :: initially disabled


/* send signal to amplifiers */
#define Coil_PX 0
#define Coil_NX 3
#define Coil_PY 4
#define Coil_NY 1
#define Coil_PZ 2
#define Coil_NZ 5

void my_sleep (unsigned msec) {
    struct timespec req, rem;
    int err;
    req.tv_sec = msec / 1000;
    req.tv_nsec = (msec % 1000) * 1000000;
    while ((req.tv_sec != 0) || (req.tv_nsec != 0)) {
        if (nanosleep(&req, &rem) == 0)
            break;
        err = errno;
        // Interrupted; continue
        if (err == EINTR) {
            req.tv_sec = rem.tv_sec;
            req.tv_nsec = rem.tv_nsec;
        }
        // Unhandleable error (DEFAULT (bad pointer), EINVAL (bad timeval in tv_nsec), or ENOSYS (function not supported))
        break;
    }
}

/* get present time of day */
double get_present_time (void) {
    struct timeval presentTime;
    gettimeofday( &presentTime, NULL );
    double returnVal = (double) presentTime.tv_sec + presentTime.tv_usec * 1e-6;        // second
    return returnVal;
}


/* class definition */
/* constructor */
Coil_System :: Coil_System ( void ) {
    for (int i = 0; i < 6; i ++) {
        gradientV[i] = 0;
    }
    for (int i = 0; i < 3; i ++) {
        uniformV[i] = 0;
    }
    angle = 0;
    angleOld = 0;
    fGradient = 0;
    /* ensure robot is init. aligned with +x */
    for (int i = 0; i < 10; i ++) {
        uniformV[0] = i * 0.1;
        output_signal ();
        my_sleep(100);
    }
    //uniformV[0] = 0;
    //output_signal ();
}

void Coil_System :: set_uniform_field_volt ( float data[3] ) {
    for (int i = 0; i < 3; i ++) {
        uniformV[i] = data[i];
    }
}

/* set the z field strength to tilt robot */
void Coil_System :: set_z_field_volt (float data) {
    uniformV[2] = data;
}

void Coil_System :: set_gradient_field_volt ( float data[6] ) {
    for (int i = 0; i < 6; i ++) {
        gradientV[i] = data[i];
    }
}

void Coil_System :: output_signal ( void ) { // IMPORTANT: This is where we choose between using Jiachen's or Chris's control signals
    if(undergrad_control){
        output_signals[0] = get_output_signals(); // Get output signals from undergrad path planning/tracking
        for (int i = 0; i < 4; i ++) {
            outputV[i] = output_signals[0][i];
            printf("Output %d: %f\n", i, output_signals[0][i]);
        }
        for (int i = 4; i < 6; i ++) {
            outputV[i] =                uniformV[i/2];
        }
    }
    else if (fGradient) {
        for (int i = 0; i < 6; i ++) {
            outputV[i] = gradientV[i] + uniformV[i/2]; // Set pairs of coils to the same uniform field
        }
    } else {
        for (int i = 0; i < 6; i ++) {
            outputV[i] =                uniformV[i/2];
        }
    }

    /* apply gradient */
    /*
    if (outputV[0] >=0) {
        s826_aoPin(Coil_PX, 2, 2*outputV[0]);
        s826_aoPin(Coil_NX, 2, 0);
    } else {
        s826_aoPin(Coil_PX, 2, 0);
        s826_aoPin(Coil_NX, 2, 2*outputV[0]);
    }

    if (outputV[2] >=0) {
        s826_aoPin(Coil_PY, 2, 2*outputV[2]);
        s826_aoPin(Coil_NY, 2, 0);
    } else {
        s826_aoPin(Coil_PY, 2, 0);
        s826_aoPin(Coil_NY, 2, 2*outputV[2]);
    }

    if (outputV[4] >=0) {
        s826_aoPin(Coil_PZ, 2, 2*outputV[4]);
        s826_aoPin(Coil_NZ, 2, 0);
    } else {
        s826_aoPin(Coil_PZ, 2, 0);
        s826_aoPin(Coil_NZ, 2, 2*outputV[4]);
    }
    */

    /* apply uniform field */

    s826_aoPin(Coil_PX, 2, outputV[0]); // 0
    s826_aoPin(Coil_NX, 2, outputV[1]); // 3
    s826_aoPin(Coil_PY, 2, outputV[2]); // 4
    s826_aoPin(Coil_NY, 2, outputV[3]); // 1
    s826_aoPin(Coil_PZ, 2, outputV[4]); // 2
    s826_aoPin(Coil_NZ, 2, outputV[5]); // 5

    //printf("output %.3f %.3f %.3f %.3f %.3f %.3f.\n", outputV[0], outputV[1], outputV[2],outputV[3], outputV[4], outputV[5]);
    fGradient = 0;
}

void Coil_System :: set_angle (float data) {
    angleOld = angle;           // remember the last angle
    angle = data;
}

void Coil_System :: rotate_to_new_angle ( void ) {
    // angle is in range of [0, 360)
    float del = angle - angleOld; // Delta in angle
    if (del > 180)
        del = del - 360;
    if (del < -180)
        del = del + 360;

    int step = 1;
    if (del < 0)
        step = -1;

    /* gently rotate robot */
    uniformV[2] = 0;
    for (int i = 0; i < 6; i ++) // Set all gradients in field to 0
        gradientV[i] = 0;
    for (int i = 0; i < abs(del); i ++) {
        angleOld = angleOld + step;
        uniformV[0] = cosd(angleOld) * 1; // Set heading angle in increments
        uniformV[1] = sind(angleOld) * 1;
        uniformV[2] = 0.0;
        output_signal ();
        my_sleep(10);
    }
    angleOld = angle;
    /* calc. gradient using new angle */
    float gradientX = 1 * cosd(angle);
    float gradientY = 1 * sind(angle);
    for (int i = 0; i < 6; i ++) {
        gradientV[i] = 0;
    }
    if (gradientX >= 0)
        gradientV[0] = gradientX;
    else
        gradientV[1] = gradientX;
    if (gradientY >= 0)
        gradientV[2] = gradientY;
    else
        gradientV[3] = gradientY;
}

void Coil_System :: stop_output (void) {
    for (int i = 0; i < 6; i ++) {
        gradientV[i] = 0;
    }
    for (int i = 0; i < 3; i ++) {
        uniformV[i] = 0;
    }
    angle = 0;
    angleOld = 0;
    output_signal ();
}

void Coil_System :: add_gradient_output (void) {
    fGradient = 1;
}

static void send_signal_to_amplifier (float outputV[3]) {
    s826_aoPin(Coil_PX, 2, outputV[0]);
    s826_aoPin(Coil_NX, 2, outputV[0]);
    s826_aoPin(Coil_PY, 2, outputV[1]);
    s826_aoPin(Coil_NY, 2, outputV[1]);
    s826_aoPin(Coil_PZ, 2, outputV[2]);
    s826_aoPin(Coil_NZ, 2, outputV[2]);
    //printf("output %.3f %.3f %.3f.\n", outputV[0], outputV[1], outputV[2]);
}

/* damp rotation, otherwise robot will fly */
static void damp_rotation (float oldOutput[3], float newOutput[3]) {
    float del[3] = {0,0,0};
    for (int i = 0; i < 3; i ++)
        del[i] = 0.01 * (newOutput[i] - oldOutput[i]);

    for (int i = 0; i < 100; i ++) {
        for (int j = 0; j < 3; j ++)
            oldOutput[j] = oldOutput[j] + del[j];
        send_signal_to_amplifier (oldOutput);
        my_sleep(10);
    }
    printf("at the end of damp\n");
}

/* thread of actuation */
static void* actuation_THREAD ( void *threadid ) {
    printf("at the start of actuation_THREAD.\n");
    //int initFlag = s826_init();
    //printf("init result %d\n", initFlag);

    /* variable definition */
    static Coil_System coil;
    float freq = 10;                // frequency of vibration
    float periodTime = 1.0 / freq;  // time per period
    float tiltAngle = 20;           // tilting angle of degrees
    float ampXY = 1;                // field amplitude in voltage
    float ampZ  = ampXY * tand(tiltAngle);
    // float outputV[3] = {0,0,0};     // output voltage for x, y, z
    float outputVZ = 0;
    float angle = 0;                // moving angle in x-y plane

    double startTime = 0, presentTime = 0, timeElapsed = 0;
    startTime = get_present_time ();
    while (fThread) {
        presentTime = get_present_time ();
        timeElapsed = presentTime - startTime;
        if (timeElapsed >= periodTime) {
            while (timeElapsed >= periodTime) {
                timeElapsed = timeElapsed - periodTime;
            }
            startTime = presentTime - timeElapsed;
        }
        //printf("time: %.3f\n", timeElapsed);

        /* calc. field in x-y directions */
        // if (fKey) {
        //     if (directionCode == -1) {
        //         coil.stop_output ();
        //     } else {
        //         coil.set_angle (directionCode * 90.0);
        //         coil.rotate_to_new_angle ();
        //     }
        //     fKey = 0;                   // reset key flag
        // }else{
        //     if(angle_heading == -1){
        //         coil.stop_output();
        //     }else{
        //         coil.set_angle(angle_heading);
        //         coil.rotate_to_new_angle();
        //     }
        // }

        /* decide z field amplitude based on time */
        if (directionCode != -1) {
            outputVZ = -1.0 * ampZ * timeElapsed / periodTime;         // make the object tail tilt up
            coil.set_z_field_volt (outputVZ);
        }

        /* apply gradient when robot slips */
        if (timeElapsed >= 0.8 * periodTime) {
            // output gradient along moving direction
            //coil.add_gradient_output();
        }
        coil.output_signal ();
        //send_signal_to_amplifier (outputV);
    //    my_sleep(10);
    }

    //outputV[0] = 0;outputV[1] = 0;outputV[2] = 0;
    //send_signal_to_amplifier (outputV);
    coil.stop_output();
    //s826_close();
    printf("at the end of actuation_THREAD.\n");
}

/* start or stop the actuation thread */
void on_tB_actuation_Thread (void) {
    actuationThread = 1;
    pthread_t actuationThread;
    pthread_create( &actuationThread, NULL, actuation_THREAD, NULL);  //start actuation thread
}

void set_directionCode (int keycode) {
    directionCode = keycode;
    switch (directionCode) {
        case -1: printf("neutral\n"); break;
        case  0: printf("right\n"); break;
        case  1: printf("up\n"); break;
        case  2: printf("left\n"); break;
        case  3: printf("down\n"); break;
    }
    fKey = 1;                   // a key is pressed
}
void set_direction (float angle_set){
    angle_heading = angle_set;
    fKey = 0;
}
