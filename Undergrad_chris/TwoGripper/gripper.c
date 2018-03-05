#include "gripper.h"

/////////////////////////////////////////////////////////
// Locally Available Variables
/////////////////////////////////////////////////////////

static int f_gRD = 0;                                           // flag for gripper's rotation demo thread

static float v_gL = 0.0;                                        // variable: gripping level
static float v_lLX = 0.0;                                       // variable: locomotion level along x-axis
static float v_lLY = 0.0;                                       // variable: locomotion level along y-axis

/////////////////////////////////////////////////////////
// Locally Available Sub-Functions
/////////////////////////////////////////////////////////

/// Update Output Voltages
static int outputV(void) {
    float temp = v_gL + v_lLX;
    if (temp > 3.0)
        temp = 3.0;                                             // ensure the output voltage is within the range
    s826_aoPin( 0 , 2,  temp  );                                 // update voltage along x-axis; channel-0 corresponds to +x

    s826_aoPin( 1 , 2,  v_gL  );                                 // update voltage along x-axis; channel-1 corresponds to -x

    s826_aoPin( 2 , 2, -v_lLY );                               // update voltage along y-axis; channel-2 corresponds to +y

    return 1;
}

/////////////////////////////////////////////////////////
// Rotation Demo Thread
/////////////////////////////////////////////////////////
static void* gRD_thread ( void * threadid )
{
    printf("@ the Beginning of gRD_thread.\n");

    f_gRD = 1;                                              // mark the flag that the thread is running

    /// Close the gripper
    int i = 0;
    for (i=0; i<300; i++)                                   // total 3 s
    {
        s826_aoPin( 2 , 2, i*0.01 );                        // +z
        usleep(1e4);                                        // wait for 0.01 s
    }

    /// Rotate the gripper around -y
    for (i=0; i<300; i++)                                   // total 3 s (3/4 cycle)
    {
        s826_aoPin( 0 , 2, 3*sin(M_PI/200 * i) );           // x dir.
        s826_aoPin( 2 , 2, 3*cos(M_PI/200 * i) );           // z dir.
        usleep(1e4);                                        // wait for 0.01 s
    }

    /// Rotate the gripper around +z; counter-clockwise (+x -> +y -> -x -> -y)
    for (i=0; i<400; i++)                                   // total 4 s
    {
        s826_aoPin( 0 , 2, -3*cos(M_PI/200 * i) );           // x dir.
        s826_aoPin( 1 , 2, 3*sin(M_PI/200 * i) );           // y dir.
        usleep(1e4);                                        // wait for 0.01 s
    }

    /// Reset output voltages
    s826_aoPin( 0 , 2, 0 );
    s826_aoPin( 1 , 2, 0 );
    s826_aoPin( 2 , 2, 0 );

    f_gRD = 0;                                              // mark the flag that the thread ends
    printf("@ the End of gRD_thread.\n");
}

int gripper_initRotationDemo(void)                           // initialize the gripper's rotation demo thread
{
    if (f_gRD)
    {
        printf("Gripper Rotation Thread is already running.\n");
        return 0;
    }
    else
    {
        pthread_t gRD;
        pthread_create( & gRD, NULL, gRD_thread, NULL);     // start gripper's rotation demo thread
        return 1;
    }
}

/////////////////////////////////////////////////////////
// Gripping Motion
/////////////////////////////////////////////////////////
int gripper_setGrippingMotion(float d)
{
    v_gL = -3.0 * d;                                         // determine the gripping level
    outputV();                                              // update the output voltage
    return 1;
}

/////////////////////////////////////////////////////////
// Planar Locomotion
/////////////////////////////////////////////////////////
int gripper_setLX(float d)
{
    v_lLX = 3.0 * d;
    outputV();                                              // update the output voltage
    return 1;
}

int gripper_setLY(float d)
{
    v_lLY = 3.0 * d;
    outputV();                                              // update the output voltage
    return 1;
}
