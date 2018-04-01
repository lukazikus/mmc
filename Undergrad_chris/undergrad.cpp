#include "undergrad.h"
using namespace std;
using namespace cv;

/*
Occupancy Grid Scheme:
0 - Occupied by robot
1 - Unoccupied
2 - Occupied by cargo
3 - Occupied by arena
*/

static int fThread = 0;
static float gradientDir = 0.0;                 // gradient angle (radian) w.r.t. +x
static int fGradient = 0;                       // whether or not apply a gradient
static float gradientAmpX = 7;
static float gradienstAmpY = 3;

// Global MMC variables
static float PgainMMC = 10;   // P gain in MMC
static float IgainMMC = 0;   // I gain in MMC
static float DgainMMC = 0;   // D gain in MMC
static int MMCThread = 0;   // MMC flag :: initially disabled
static int gradient_on = 1; // 1 means gradient field is on
static int uniform_on = 0; // 0 means uniform field is on
static float thresh = 2.0; // Distance in pixels that center of mass of robot can be from goal location
static bool autonomous = 0; // IMPORTANT Flag for whether robot should follow autonomous path or just go wherever the mouse click tells it to
static bool mountable = 0; // Flag for when the robot is able to mount to the cargo
static int state = -1; // 0 means go towards cargo, 1 means orient to cargo, 2 means move towards cargo, 3 means go to goal position, 4 means orient properly
static float goal_orientation; // Goal orientation that the cargo should take on
static float freq = 10;                // frequency of vibration
static int num_points = 6; // Number of points required for calibration
static int thickness; // Thickness of walls in pixels
static int cargo_type = 0; // 0 for circle, 1 for rectangle, 2 for triangle

// Tilt angle variables
static int fVibrate = 1; // Flag for whether the robot should conduct stick slip motion
float tiltAngle = 20;           // tilting angle of degrees
float ampXY = 1;                // field amplitude in voltage
float ampZ  = ampXY * tand(tiltAngle);

// Output voltage variables for coil (gradient vs. uniform field)
static float outputVolt[6] = {0,0,0,0,0,0};                         // output voltage to amplifiers
static float uniformFieldVolt[6] = {0,0,0,0,0,0};                   // uniform component of magnetic field, x, y, z
static float gradientFieldVolt[4] = {0,0,0,0};                      // gradient field signal voltage
static float pullGain = 5.0;                        // Gain for calculating field gradient for pulling
static float B_strength_MA = 7; // Strength of magnetic field in mT
static float Coilpair_ratio_MA = 1; // ratio between the pair of coils
static float  saturationGradientFieldVolt = 1; // saturation field strength for gradient generation

// Path Planning variables
static float* robot_pos[1]; // Current location of microrobot
static float* cargo_pos[1]; // Goal location of cargo
static float* goal_pos[1]; // Goal location of next position for microrobot to align with cargo at first
static float* click_pos[1]; // Location to actually have the robot move towards
static float look_ahead = 40; // Distance away from goal position that we should click on the screen
static float tol = 5; // Distance behind cargo that robot's centre of mass should go to

static int saturate_gradient_field_signal () {
    if (gradientFieldVolt[0] > saturationGradientFieldVolt) { // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[0] = saturationGradientFieldVolt;
        //  printf("Saturated +X1 \n");
    }
    if (gradientFieldVolt[0] < -saturationGradientFieldVolt) {// to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
        gradientFieldVolt[0] = -saturationGradientFieldVolt;
        //  printf("Saturated -X1 \n");
    }

    if (gradientFieldVolt[1]>saturationGradientFieldVolt) // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
    {
        gradientFieldVolt[1] = saturationGradientFieldVolt;
        // printf("Saturated +X2 \n");
    }
    if (gradientFieldVolt[1]<-saturationGradientFieldVolt) // to avoid singularities in X  default constant 0.5 :: it means field is only in X-direction, and so the only choice for pulling too
    {
        gradientFieldVolt[1] = -saturationGradientFieldVolt;
        //  printf("Saturated -X2 \n");
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

// /*
static int add_constant_and_gradient_field_signal () {
    if (gradientFieldVolt[0] >= 0) {
        outputVolt[0] = gradientFieldVolt[0];
        outputVolt[1] = 0;
    } else {
        outputVolt[0] = 0;
        outputVolt[1] = gradientFieldVolt[1];
    }

    if (gradientFieldVolt[2] >= 0) {
        outputVolt[2] = gradientFieldVolt[2];
        outputVolt[3] = 0;
    } else {
        outputVolt[2] = 0;
        outputVolt[3] = gradientFieldVolt[3];
    }

    outputVolt[4] = uniformFieldVolt[4];
    outputVolt[5] = uniformFieldVolt[5];

    return 1;
}
// */

static int output_signal_to_amplifier (void) {
    s826_aoPin(2, 2, fVibrate ? outputVolt[5] : 0);        // z-bottom 5.276  1st amplifier
    s826_aoPin(5, 2, fVibrate ? outputVolt[4] : 0);        // z-top  4.44     2nd amplifier
    s826_aoPin(1, 2, outputVolt[3]);         // y-right  4.96     3rd amplifier
    s826_aoPin(4, 2, outputVolt[2]);         // y-left    5.296   4th amplifier
    s826_aoPin(3, 2, outputVolt[1]);         // -x coil  5.14     5th amplifier
    s826_aoPin(0, 2, outputVolt[0]);         // +x coil     5.32    6th amplifier
    // printf("output voltage %.2f, %.2f, %.2f, %.2f,  %.2f,  %.2f\n",outputVolt[0],outputVolt[1],outputVolt[2],outputVolt[3],outputVolt[4],outputVolt[5]);
    return 1;
}

static void* autonomy_thread(void * threadid){
    // Pulling thread
    printf("MMC_thread started\n");

    float globalFieldAngle = 0.0, globalFieldAngleMemo = 0.0;                   // output field angle for constant field and its memory
    float pull_const=0, int_const=0, der_const=0, errorPull=0, errorPullPrev=0, errorSum=0,\
          errorDiff=0, COM_coorX=0, COM_coorY=0, GOAL_coorX=0, GOAL_coorY=0;
    double pulling_angle=0;
    float periodTime = 1.0 / freq;  // time per period

    double startTime = 0, presentTime = 0, timeElapsed = 0; // Time variables
    startTime = get_present_time ();
    float outputVZ = 0; // Output z axis voltage

    // Path Planning Variables
    Pair src; // Current start location of robot
    Pair dest; // Current goal location of robot
    Pair next_pos; // Next location in stack that robot should move towards
    stack<Pair> Path; // Stack of positions to be generated by A*

    // Run the main MMC thread (after clicking Start Control)
    while ( MMCThread ) {
        // printf("Executing task \n");
        robot_pos[0] = get_robot_pose(); // Get updated coordinates of robot in image frame
        cargo_pos[0] = get_cargo_pose(); // Get mouse key goal point in image frame
        goal_pos[0] = getGoalPointCoor(); // Initialize goal variable to avoid seg faults

        if(state == -1){ // Calibration phase, get arena boundary points and occupancy grid
            // Create Occupancy Grid based on arena and cargo (These variables should come from CV)
            float radius = 30; // Size of cargo (to be approximated as a square for occupancy grid)
            Point points[num_points];
            points[0].x = 10; points[0].y = 400; // Boundary corner points
            points[1].x = 600; points[2].y = 10;
            points[2].x = 350; points[3].y = 400;
            points[3].x = 350; points[4].y = 360; // Walls
            points[4].x = 350; points[5].y = 320;
            points[5].x = 350; points[6].y = 380;

            int** occ_grid = new int*[ROW]; 
            for(int i = 0; i < ROW; ++i){
                occ_grid[i] = new int[COL];
            }
            for(int i = 0; i < ROW; ++i){
                for(int j = 0; j < COL; ++j){
                    occ_grid[i][j] = 1;
                }
            }
            create_og(occ_grid, points, thickness, cargo_pos, radius);

            // Set source and destination of path that robot should traverse
            src = make_pair(robot_pos[0][1],robot_pos[0][0]);
            dest = make_pair(cargo_pos[0][1],cargo_pos[0][0]);
            calc_dest(cargo_pos, tol, dest); // Update detination based on cargo's pose

            // printf("src: (%d,%d), dest: (%d,%d)\n", src.first,src.second,dest.first,dest.second);
            aStarSearch(occ_grid, src, dest, Path); // Run A* and generate Path

            // Free memory allocated for occ_grid
            for(int i = 0; i < ROW; ++i) {
                delete [] occ_grid[i];
            }
            delete [] occ_grid;

            state = 0;
        }else if(autonomous && state == 0){ // Extract next position to go to from Path
            if(Path.empty()){ // Reached destination in path
                state = 2;
                continue;
            }
            next_pos = Path.top(); Path.pop(); // Extract next postion to go to
            calc_clickPos(robot_pos[0], next_pos, look_ahead, click_pos[0]); // Compute next position to click
            fVibrate = 1; // Vibrate to let robot move
            
            state = 1;
        }else if(autonomous && state == 1){
            if(errorPull < thresh){
                state = 0;
            }
        }else if(autonomous && state == 1){ // Turn towards cargo
            // globalFieldAngle = cargo_pos[0][2]; // Align to cargo
            goal_pos[0][0] = robot_pos[0][0] + cos(cargo_pos[0][2]); // Dummy goal position so that robot orients towards cargo
            goal_pos[0][1] = robot_pos[0][1] + sin(cargo_pos[0][2]);
            fVibrate = 0; // No need to vibrate when just aligning to goal pose
            usleep(2000); // Give 2s for the microrobot to orient
            state = 2;
        }else if(autonomous && state == 2){ // Go towards cargo
            // globalFieldAngle = pulling_angle;
            goal_pos[0] = cargo_pos[0]; // Set goal position exactly equal to target position
            fVibrate = 1; // Vibrate to let robot move
            if(errorPull < thresh){
                state = 3;
            }
        }else if(autonomous && state == 3){ // Go towards goal
            // globalFieldAngle = pulling_angle;
            goal_pos[0] = getGoalPointCoor(); // Final goal is wherever the mouse clicks
            fVibrate = 1; // Vibrate to let robot move
            if(errorPull < thresh){
                state = 4;
            }
        }else if(autonomous && state == 4){ // Align to proper orientation
            // globalFieldAngle = goal_orientation; // Align to cargo
            goal_pos[0][0] = robot_pos[0][0] + cos(cargo_pos[0][2]); // Dummy goal position so that robot orients towards cargo
            goal_pos[0][1] = robot_pos[0][1] + sin(cargo_pos[0][2]);
            fVibrate = 0; // No need to vibrate when just aligning to goal pose
            usleep(2000); // Give 2s for the microrobot to orient
            MMCThread = 0; // Break out of while loop
        }else{
            click_pos[0] = getGoalPointCoor(); // Use for positioning robot wherever the mouse clicks
            // printf("%f,%f\n",goal_pos[0][0], goal_pos[0][1]);
        }

        //WAYPOINT GENERATION BASED ON CURRENT PATH NODE
        // Waypoint following
        COM_coorX = robot_pos[0][0]; // Current location of robot
        COM_coorY = robot_pos[0][1];
        GOAL_coorX = click_pos[0][0]; // Next location of robot based on click position
        GOAL_coorY = click_pos[0][1];

        errorPullPrev = errorPull; // Keep track of previous error for derivative computation
        errorPull = sqrt(pow(GOAL_coorX - COM_coorX, 2) + pow(GOAL_coorY - COM_coorY, 2)); // Error
        errorSum = errorSum + errorPull; // Approximate integral of error
        errorDiff = errorPull - errorPullPrev; // Approximate derivative of error
        pulling_angle = atan2((float)(GOAL_coorY - COM_coorY), (float)(GOAL_coorX - COM_coorX));
        // printf("(X,Y) = (%f, %f),  (E1,E2) = (%f, %f)\n",COM_coorX, COM_coorY, errorPull, pulling_angle); // Debug statement

        // PID control
        pull_const = 1e-2 * PgainMMC * errorPull; // Proportional term
        int_const = 1e-2 * IgainMMC * errorSum; // Integral term
        der_const = 1e-2 * DgainMMC * errorDiff; // Derivative term

        // Use Gradient Field
        gradientFieldVolt[0] = (pull_const+int_const+der_const) * cos(pulling_angle) / (Coilpair_ratio_MA*11.4); // Outer coils
        gradientFieldVolt[1] = (pull_const+int_const+der_const) * cos(pulling_angle) / (Coilpair_ratio_MA*11.4);
        gradientFieldVolt[2] = (pull_const+int_const+der_const) * sin(pulling_angle) / (Coilpair_ratio_MA*20); // Inner coils
        gradientFieldVolt[3] = (pull_const+int_const+der_const) * sin(pulling_angle) / (Coilpair_ratio_MA*20);

        // Use Uniform Field
        // uniformFieldVolt[0] = B_strength_MA * cos(globalFieldAngle) / 5.32;         // x-left
        // uniformFieldVolt[1] = B_strength_MA * cos(globalFieldAngle) / 5.32;         // x-right
        // uniformFieldVolt[2] = B_strength_MA * sin(globalFieldAngle) / 5.2;          // y-left
        // uniformFieldVolt[3] = B_strength_MA * sin(globalFieldAngle) / 5.2;          // y-right

        // Create stick slip motion with varying z coil actuation
        presentTime = get_present_time ();
        timeElapsed = presentTime - startTime;
        if(timeElapsed >= periodTime){
            timeElapsed = timeElapsed - periodTime*(int)((presentTime - startTime)/periodTime); // Bring timeElapsed into period range
            startTime = presentTime - timeElapsed;
        }

        outputVZ = -1.0 * ampZ * timeElapsed / periodTime;         // make the object tail tilt up
        // printf("AmpZ: %.2f, Time Elapsed: %.2f, Period Time: %.2f\n", ampZ, timeElapsed, periodTime);
        uniformFieldVolt[4] = outputVZ; // Set z axis voltages
        uniformFieldVolt[5] = outputVZ;

        // Actuate coils
        saturate_gradient_field_signal (); // Saturate gradient field signal to limit it
        add_constant_and_gradient_field_signal (); // Add gradient and uniform field
        output_signal_to_amplifier (); // Write to amplifier to output current (note this is not necessary if used with actuation.cpp)
        usleep(5e4);
    }

    // Finish MMC thread and turn off all coils
    coilCurrentClear();
    printf("MMC_thread ended\n");
}

// Get present time of day (from actuation.cpp)
double get_present_time (void) {
    struct timeval presentTime;
    gettimeofday( &presentTime, NULL );
    double returnVal = (double) presentTime.tv_sec + presentTime.tv_usec * 1e-6;  // second
    return returnVal;
}

////////   MMC functions that run after interacting with GUI  //////
int setPgain_MMC(float d) { // Set P gain
    PgainMMC = d;
    return 1;
}

////////   MMC functions that run after interacting with GUI  //////
int setIgain_MMC(float d) { // Set I gain
    IgainMMC = d;
    return 1;
}

////////   MMC functions that run after interacting with GUI  //////
int setDgain_MMC(float d) { // Set D gain
    DgainMMC = d;
    return 1;
}

void create_og(int** occ_grid, Point points[], int thickness, float** cargo_pos, float radius){
    // Create obstacles where arena boundaries are
    int x_avg = (points[0].x + points[1].x)/2;
    for(int i = points[0].y; i >= points[1].y; i--){ // Draw 0s on left wall
        for(int j = points[0].x-thickness/2; j < points[0].x+thickness/2; j++){
            occ_grid[i][j] = 0;
        }
    }

    for(int i = points[0].x; i <= points[1].x; i--){ // Draw 0s on bottom wall
        for(int j = points[1].y-thickness/2; j < points[1].y+thickness/2; j++){
            occ_grid[j][i] = 0;
        }
    }

    for(int i = points[1].y; i <= points[0].y; i++){ // Draw 0s on right wall
        for(int j = points[1].x-thickness/2; j < points[1].x+thickness/2; j++){
            occ_grid[i][j] = 0;
        }
    }

    for(int i = points[1].x; i >= points[0].x; i--){ // Draw 0s on top wall
        for(int j = points[0].y-thickness/2; j < points[0].y+thickness/2; j++){
            occ_grid[j][i] = 0;
        }
    }

    for(int i = points[0].y; i >= points[2].y; i--){ // Draw 0s on center wall
        for(int j = x_avg - thickness/2; j < x_avg + thickness/2; j++){
            occ_grid[i][j] = 0;
        }
    }

    for(int i = points[3].y; i >= points[4].y; i--){ // Draw 0s on center wall
        for(int j = x_avg - thickness/2; j < x_avg + thickness/2; j++){
            occ_grid[i][j] = 0;
        }
    }

    // Create obstacles where cargo is
    for(int i = cargo_pos[0][0]-radius; i < cargo_pos[0][0]+radius; i++){
        for(int j = cargo_pos[0][1] - radius; j < cargo_pos[0][1] + radius; j++){
            occ_grid[j][i] = 0;
        }
    }
}

void calc_clickPos(float* robot_pos, Pair next_pos,  float look_ahead, float* click_pos){
    // Calculate updated mouse position for each node in Path
    float heading = atan2(next_pos.first - robot_pos[1], next_pos.second - robot_pos[0]);
    click_pos[0] = next_pos.second + look_ahead*cos(heading);
    click_pos[1] = next_pos.first + look_ahead*sin(heading);
}

void calc_dest(float** carg_pos, float tol, Pair &dest){
    // Calculate actual destination that the robot shoulds go towards based on cargo's radius
    float heading = cargo_pos[0][2];
    dest.second = cargo_pos[0][0] - tol*sin(cargo_pos[0][2]); 
    dest.first = cargo_pos[0][1] - tol*cos(cargo_pos[0][2]);
}

// MMC thread
int on_startMMC_Thread (void) {
        MMCThread = 1;
        pthread_t MMC_Thread;
        pthread_create(&MMC_Thread, NULL, autonomy_thread, NULL);
    return 1;
}

int on_stopMMC_Thread(void){
    MMCThread = 0;
    s826_aoPin(2, 2, 0);         // z-bottom 5.276  1st amplifier
    s826_aoPin(5, 2, 0);         // z-top  4.44      2nd amplifier
    s826_aoPin(1, 2, 0);         // -y-right  4.96     3rd amplifier
    s826_aoPin(4, 2, 0);         // +y-left    5.296   4th amplifier
    s826_aoPin(3, 2, 0);         // -x coil  5.14     5th amplifier
    s826_aoPin(0, 2, 0);         // +x coil     5.32    6th amplifier
}
