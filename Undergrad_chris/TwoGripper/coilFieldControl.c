#include "coilFieldControl.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gobla Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint flag_pathFollow = 0;            // 0: inactive; 1: active
uint flag_UT_follow  = 0;            // 1: follow UT path
uint flag_actuation_A_control = 0;   // 1: perform actuation amplitude feedback control

int single_goalCoor_pathFollow[2];
//int array_goalCoor_pathFollow[] =

static float swimmer_dirAdjust_bias = 0, swimmer_dirAdjust_angle = 0;  // Bias used to compensate the net magnetization. Initial no bias. Angle in radians.
static float swimmer_dirAdjust_angle_I = 0;                            // PI controller's integral part
static float P_coeff = 1.0, I_coeff = 1.0;                             // PI controller coefficients

//static float coil_current_x = 0, coil_current_y = 0, coil_current_z = 0;
static float coil_current_voltage[3] = {0,0,0};

static int swimmer_dir_reverse = 1;   // Different swimmers may swim to different dir. in the same rotating field, because of differernt magnetization.
                                      // This variable makes sure all swimmers swim to +x dir. when apply a positive voltage on the +x dir.

static uint flag_swimmer_actuation = 0;   // Flag: 1: swimmer is actuated.
static uint flag_2AgentCtrl = 0;         // Flag: 1: 2 Agents are actuated.
static uint dir_index_MA = 0;               // Desired agent direction. 0: +x, 1: +y, 2: -x, 3: -y      ::: multi-agent project



static uint dir_index = 0;                // Desired swimming direction. 0: +x, 1: +y, 2: -x, 3: -y
static int  swim_heading = 1;             // Swim forward or backward. 1: forward, -1: backward

static float swimmer_A_h = 3.0, swimmer_A_v = 3.0;
static float swimmer_h_bias = 0, swimmer_v_bias = 0;

//static uint swimmer_actuationThreadControl = 0;
static float swimmer_actuationFreq = 30.0;

static float swimmer_heading = 0;               // Heading dir. of swimmer.
static uint  swimmer_heading_1stTime = 1;       // 1: swimmer heading has not been previously determined
static int   swimming_dir = 1;                  // Swimming direction. 1: forward; -1: backward

static struct timeval start;

static int flag_multiSwimmer_stop = 0;
static int flag_2Agentctrl_stop = 0;


static int flag_swimmer_seriesTest = 0;
static int flag_swimmingSpeed      = 0;         // flag for swimming speed test
static int index_n_UT_follow = 1;               // desired number of "UT" path follows

static float swimmer_actuationAcontrol_low = 0.2;
static float swimmer_actuationAcontrol_high = 1.0;
static float swimmer_desiredHeading = 0.0;         // desired swimmer heading, pointing from current center point to goal point, unit: radian

static int flag_criticalCoorMonitor = 1;           // (UT following) to prevent spinning, once the swimmer reached the critical coor. of the goal point, it will move on to next goal point

// Cilia Global Variables
static int flag_ciliaTest = 0;



// To show field angle on top camera for multi-agent project
float field_angle_MA;      // in radians!

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutine: Get Current Time in Seconds
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static double get_currentTime(void)
{
    gettimeofday(&start, NULL);
    double l_time = (double) start.tv_sec + start.tv_usec*1e-6 ;   // seconds
    return l_time;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Switch: Swimmer Heading Recording
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static FILE * l_heading_filePointer;

static int switch_swimmerHeadingRecord(int a_command)
{
    switch (a_command)
    {
        case 0:  fclose(l_heading_filePointer); break;
        case 1:  l_heading_filePointer = fopen("swimmerHeading.txt","w"); break;
        default: printf("Error in switch_swimmerHeadingRecord. - a_command out of range.\n");
    }
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint swimmer_heading_test_running = 0;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getGoalPointCoor_pathFollow(int index)
{
    if (index <= 10)
    {
        //single_goalCoor_pathFollow[0] = 50;
        //single_goalCoor_pathFollow[1] = 440 - 40 * index;
        single_goalCoor_pathFollow[0] = 80;
        single_goalCoor_pathFollow[1] = 400 - 32 * index;
    } else if (index <=20)
    {
        //single_goalCoor_pathFollow[0] = 50 + 27 * (index - 10);
        //single_goalCoor_pathFollow[1] = 40;
        single_goalCoor_pathFollow[0] = 80 + 24 * (index - 10);
        single_goalCoor_pathFollow[1] = 80;
    } else if (index <= 30)
    {
        //single_goalCoor_pathFollow[0] = 320;
        //single_goalCoor_pathFollow[1] = 40 + 40 * (index - 20);
        single_goalCoor_pathFollow[0] = 320;
        single_goalCoor_pathFollow[1] = 80 + 32 * (index - 20);
    } else if (index <= 40)
    {
        //single_goalCoor_pathFollow[0] = 320 + 27 * (index - 30);
        //single_goalCoor_pathFollow[1] = 440;
        single_goalCoor_pathFollow[0] = 320 + 24 * (index - 30);
        single_goalCoor_pathFollow[1] = 400;
    } else if (index <= 45)
    {
        //single_goalCoor_pathFollow[0] = 590 - 27 * (index - 40);
        //single_goalCoor_pathFollow[1] = 440;
        single_goalCoor_pathFollow[0] = 560 - 24 * (index - 40);
        single_goalCoor_pathFollow[1] = 400;
    } else if (index <= 55)
    {
        //single_goalCoor_pathFollow[0] = 455;
        //single_goalCoor_pathFollow[1] = 440 - 40 * (index - 45);
        single_goalCoor_pathFollow[0] = 440;
        single_goalCoor_pathFollow[1] = 400 - 32 * (index - 45);
    }

    single_goalCoor_pathFollow[1] = 480 - single_goalCoor_pathFollow[1];   // Positive y direction is different
    setMouse(0, 1, single_goalCoor_pathFollow);   // Set middle mouse click to be the goal point coor.
    //return single_goalCoor_pathFollow;
}

int getGoalPointCoor_UT_follow(int a_index)
{
    switch (a_index)
    {
        case 0: single_goalCoor_pathFollow[0] = 80;
                single_goalCoor_pathFollow[1] = 400;
                break;
        case 1: single_goalCoor_pathFollow[0] = 80;
                single_goalCoor_pathFollow[1] = 80;
                break;
        case 2: single_goalCoor_pathFollow[0] = 320;
                single_goalCoor_pathFollow[1] = 80;
                break;
        case 3: single_goalCoor_pathFollow[0] = 320;
                single_goalCoor_pathFollow[1] = 400;
                break;
        case 4: single_goalCoor_pathFollow[0] = 560;
                single_goalCoor_pathFollow[1] = 400;
                break;
        case 5: single_goalCoor_pathFollow[0] = 440;
                single_goalCoor_pathFollow[1] = 400;
                break;
        case 6: single_goalCoor_pathFollow[0] = 440;
                single_goalCoor_pathFollow[1] = 80;
                break;
        default: printf("Error: getGoalPointCoor_UT_follow() - a_index is %d.\n", a_index);
    }

    single_goalCoor_pathFollow[1] = 480 - single_goalCoor_pathFollow[1];   // Positive y direction is different
    setMouse(0, 1, single_goalCoor_pathFollow);   // Set middle mouse click to be the goal point coor.
    return 1;
}

/// Update the swimmer's heading according to camera image
// Input: a_flag: 1: use a_heading as expected heading. 0 : determine heading according to previous value
//                The swimmer turns so fast that diff. of headings in 2 continuous frames may be > PI/2
//                Use a_flag when applying a static field. Because the expected heading is known.



int update_swimmer_heading(int a_flag, float a_heading)
{
    float *l_rectSideP  = get_CenterP_rect_short_side_coor_array();  // 2 center points of the rect.'s short sides. {point-1.x, point-1,y, point-2.x, point-2.y}
    int   *centerP_coor = getCenterPointCoor();
    //printf("centerP is %d, %d.\n", centerP_coor[0], centerP_coor[1]);
    float l_heading_radian[2];
    float l_heading_abs_diff[2];
    int l_i = 0;
    float l_heading_ref = 0;              // Reference value for determining swimmer's heading.
    if (a_flag)
        l_heading_ref = a_heading;        // Use expected heading as ref.
    else
        l_heading_ref = swimmer_heading;  // Use previous heading as ref.

    l_heading_ref = adjust_angle_range (l_heading_ref);  // make sure the ref. angle is in the range of (-PI, PI]

    float temp1, temp2;
    for (; l_i < 2; l_i++)
    {
        temp1 = l_rectSideP[l_i*2 + 1] - centerP_coor[1];
        temp2 = l_rectSideP[l_i*2 + 0] - centerP_coor[0];
        //l_heading_radian[l_i] = atan2(l_rectSideP[l_i*2 + 1] - centerP_coor[1], l_rectSideP[l_i*2 + 0]-centerP_coor[0]);
        l_heading_radian[l_i] = atan2(temp1, temp2);
        //printf("temp1 is %.1f, temp2 is %.1f, and heading is %.1f.\n", temp1, temp2, l_heading_radian[l_i]);
    }
    //printf("heading 1: %.1f and heading 2: %.1f. sideP1x: %.1f, sideP1y: %.1f, sideP2x: %.1f, sideP2y: %.1f\n", l_heading_radian[0], l_heading_radian[1], l_rectSideP[0],l_rectSideP[1],l_rectSideP[2],l_rectSideP[3]);

    if (swimmer_heading_1stTime)                                    // If swimmer's heading has not been determined yet, set the heading as the one pointing to righthandside
    {
        if (l_rectSideP[0] > l_rectSideP[2])                        // If point-1 at the right hand side of point-2 ...
            swimmer_heading = l_heading_radian[0];
        else
            swimmer_heading = l_heading_radian[1];

        swimmer_heading_1stTime = 0;
    }
    else
    {
        /// Make sure data is inside range before calc.
        for (l_i = 0; l_i < 2; l_i++)
        {
            l_heading_abs_diff[l_i] = l_heading_radian[l_i] - l_heading_ref;
            // Make sure the heading diff. is positive
            if (l_heading_abs_diff[l_i] < 0)
                l_heading_abs_diff[l_i] = - l_heading_abs_diff[l_i];
            // Make sure the heading diff. is smallest
            if (l_heading_abs_diff[l_i] > M_PI)
                l_heading_abs_diff[l_i] = 2*M_PI - l_heading_abs_diff[l_i];
        }

        if (l_heading_abs_diff[0] < l_heading_abs_diff[1])
            swimmer_heading = l_heading_radian[0];
        else
            swimmer_heading = l_heading_radian[1];
    }
    //printf("swimmer_heading candidates are %.1f and %.1f.\n", l_heading_radian[0] *180/M_PI, l_heading_radian[1]*180/M_PI);
    //if ( (l_heading_ref > M_PI) || (l_heading_ref <= -M_PI) )
    //        printf("Error in l_heading_ref. it is %.3f.  and swimmer heading is %.3f.\n", l_heading_ref, swimmer_heading);
    //printf("swimmer heading is %.3f.\n", swimmer_heading);
    ///
    if ( ( (swimmer_heading - swimmer_desiredHeading) < 0.02 ) && ( (swimmer_heading - swimmer_desiredHeading) > -0.02 ) )
        printf("Warning! Swimmer heading is close to desired heading. swimmer heading is %.3f.\n", swimmer_heading);
    ///

    fprintf(l_heading_filePointer, "%.3f %.3f %.3f %d %.3f %.3f %.3f %.3f %.3f %d %d\n", l_heading_radian[0], l_heading_radian[1], l_heading_ref, a_flag, a_heading, l_rectSideP[0],l_rectSideP[1],l_rectSideP[2],l_rectSideP[3],centerP_coor[0],centerP_coor[1]);

    return 0;
}

// Arguments: 3 variables in spherical coor. l_r: output amplitude;  l_phi: angle w.r.t vertical axis [0,PI]
//int update_coil_current(float l_r, float l_theta, float l_phi)
// a_theta: desired swimmer's heading in x-y plane [0, 2PI);
// a_flag: 1: use swimmer_heading as l_theta; 0: use a_theta as l_theta
int update_coil_current(int a_flag, float a_theta, float a_phi)
{
    int l_aoRange = 2;   // 2: -5 ~ +5 V.
    float l_bias_x = 0, l_bias_y = 0;
    float l_theta = 0, l_phi = a_phi;
    update_swimmer_heading(0,0);
    if (a_flag)
    {

        l_theta = swimmer_heading;
    }
    else   // If specified
        l_theta = a_theta;

    l_bias_x = swimmer_dirAdjust_bias * cos(l_theta + swimmer_dirAdjust_angle);
    l_bias_y = swimmer_dirAdjust_bias * sin(l_theta + swimmer_dirAdjust_angle);
    //printf("Bias is x: %.1f and y: %.1f.\n", bias_x, bias_y);
    //printf("Output is x: %.1f and y: %.1f.\n",coil_current_x + bias_x,coil_current_y + bias_y);

    // Rotating field set according to swimmer's geometric heading
    coil_current_voltage[0] = swimmer_A_h * sin(l_phi) * swimming_dir * cos(swimmer_heading) + l_bias_x;
    coil_current_voltage[1] = swimmer_A_h * sin(l_phi) * swimming_dir * sin(swimmer_heading) + l_bias_y;
    coil_current_voltage[2] = swimmer_A_v * cos(l_phi);

	s826_aoPin( 0 , l_aoRange, coil_current_voltage[0]);
	s826_aoPin( 1 , l_aoRange, coil_current_voltage[1]);
	s826_aoPin( 2 , l_aoRange, coil_current_voltage[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutine: Apply a Static Field (usually used to steer swimmer)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int apply_static_coil_current(float a_A, float a_theta, float a_phi)
{
    int l_aoRange = 2;   // 2: -5 ~ +5 V.

    coil_current_voltage[0] = a_A * sin(a_phi) * cos(a_theta);
    coil_current_voltage[1] = a_A * sin(a_phi) * sin(a_theta);
    coil_current_voltage[2] = a_A * cos(a_phi);

	s826_aoPin( 0 , l_aoRange, coil_current_voltage[0]);
	s826_aoPin( 1 , l_aoRange, coil_current_voltage[1]);
	s826_aoPin( 2 , l_aoRange, coil_current_voltage[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread: Generate Rotating Field
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* swimmer_rotate_field_thread (void*threadid)
{
	printf("@ the Beginning of swimmer_rotate_field_thread.\n");
	//int errcode     = S826_ERR_OK;
    //char str_centerPointCoor[50];
	//usleep(1e4);

	int temp_dir = dir_index;

	uint aoChan_horizontal = 0;   // Horizontal Channel
	//uint aoChan_adjust     = 2;   // Field adjustment channel
	//uint aoRange = 2;   // 2: -5 ~ +5 V.

	struct timeval start;
	double time_elapsed, time_now, time_variable = 0;                                 //time in seconds
	gettimeofday(&start, NULL);
	double time_init = (double) start.tv_sec + start.tv_usec*1e-6 ;         // Initial time in seconds.

	float outputV_horizontal = 0, outputV_vertical = 0;
    float dirAdjust_bias_x = 0, dirAdjust_bias_y = 0;                // Directional bias

    float *centerP_rect_short_side_coor_array;  // 2 center points of the rect.'s short sides. {point-1.x, point-1,y, point-2.x, point-2.y}
    float orientation_angle;
    int *centerP_coor;

    float l_theta = 0, l_phi = 0;
    float l_bias_x = 0, l_bias_y = 0;

	while ( flag_swimmer_actuation == 1 )
	{
        //printf("Inside the while loop.\n");
        /*
		if (swimmer_controlLoop == 1)   // If the control loop is turned on...
		{	if (centerP.x > 400)
			{
				dirIndex = 2;       // -x
				//printf("change to -x direction.\n");
			}
			else if (centerP.x < 150)
			{
				dirIndex = 0;  // +x
				//printf("change to +x direction.\n");
			}

			strcpy (str_centerPointCoor, "Center Point Coor. x: ");
			char str_temp[4];
			sprintf(str_temp, "%d ", centerP.x);
			strcat (str_centerPointCoor, str_temp);
			strcat (str_centerPointCoor, "y: ");
			sprintf(str_temp, "%d", centerP.y);
			strcat (str_centerPointCoor, str_temp);
			// may cause errors
			//gdk_threads_enter();
			//gtk_label_set_text (centerPointCoor, str_centerPointCoor);
			//gdk_threads_leave();
		}
        */

		if (temp_dir != dir_index)   // If direction is changed...
		{
            switch(dir_index)
			{
				case 0: aoChan_horizontal = 0; //aoChan_adjust = 2;
                        swim_heading =  1;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
                        break;
				case 1: aoChan_horizontal = 1; //aoChan_adjust = 0;
                        swim_heading =  1;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
                        break;
				case 2: aoChan_horizontal = 0; //aoChan_adjust = 2;
                        swim_heading = -1;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
                        break;
				case 3: aoChan_horizontal = 1; //aoChan_adjust = 0;
                        swim_heading = -1;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
                        break;
			}
            printf("marker.\n");
			// If the change is from x to y or from y to x, turn the swimmer
            if ( (temp_dir - dir_index != 2) && (temp_dir - dir_index != -2) )   // If the change is from x to y or from y to x...
            {
                coilCurrentClear();
                //coil_current_voltage[aoChan_horizontal] = swim_heading * 2.0;
                //update_coil_current();
                //update_coil_current(2.0, dir_index * M_PI/2, M_PI/2);
                l_theta = dir_index * M_PI/2;
                printf("theta is %.3f.\n", l_theta);
                l_phi = M_PI/2;

                //update_coil_current(0, l_theta, M_PI/2);
                l_bias_x = swimmer_dirAdjust_bias * cos(l_theta + swimmer_dirAdjust_angle);
                l_bias_y = swimmer_dirAdjust_bias * sin(l_theta + swimmer_dirAdjust_angle);
                //printf("Bias is x: %.1f and y: %.1f.\n", bias_x, bias_y);
                //printf("Output is x: %.1f and y: %.1f.\n",coil_current_x + bias_x,coil_current_y + bias_y);

                // Rotating field set according to swimmer's geometric heading
                coil_current_voltage[0] = swimmer_A_h * sin(l_phi) * swimming_dir * cos(l_theta) + l_bias_x;
                coil_current_voltage[1] = swimmer_A_h * sin(l_phi) * swimming_dir * sin(l_theta) + l_bias_y;
                coil_current_voltage[2] = swimmer_A_v * cos(l_phi);

                s826_aoPin( 0 , 2, coil_current_voltage[0]);
                s826_aoPin( 1 , 2, coil_current_voltage[1]);
                s826_aoPin( 2 , 2, coil_current_voltage[2]);

                usleep(3e5);
            }

            temp_dir = dir_index;
		}

		gettimeofday(&start, NULL);
		time_now = (double) start.tv_sec + start.tv_usec*1e-6;
		time_elapsed = time_now - time_init;                         //elapsed time in seconds
		time_variable = time_variable + swim_heading * time_elapsed;
		//time_variable = time_variable + time_elapsed;
		time_init = time_now;

		//outputV_horizontal = 3.0 * sin( 2*M_PI * AO_sinFreq * time_variable) + 0.5;  // Weak Offset
		//outputV_horizontal = 2.5 * sin( 2*M_PI * AO_sinFreq * time_variable) + 1.0;   // Strong Offset
		//outputV_horizontal = 3.5 * sin( 2*M_PI * AO_sinFreq * time_variable);
		//outputV_vertical   = 3.5 * cos( 2*M_PI * AO_sinFreq * time_variable);

        //outputV_horizontal = swimmer_A_h * sin( 2*M_PI * AO_sinFreq * time_variable) + swimmer_dir_bias;  // For under surface swimmer
        //outputV_vertical   = swimmer_A_v * cos( 2*M_PI * AO_sinFreq * time_variable) + swimmer_vertical_bias;

        //dirAdjust_bias_x = swimmer_dirAdjust_bias * cos(swimmer_dirAdjust_angle);
        //dirAdjust_bias_y = swimmer_dirAdjust_bias * sin(swimmer_dirAdjust_angle);

        //coil_current_voltage[aoChan_horizontal]     = swimmer_A_h * sin( 2*M_PI * swimmer_actuationFreq * time_variable) + swimmer_h_bias + dirAdjust_bias_x;  // For under surface swimmer
        l_phi = 2 * M_PI * swimmer_actuationFreq * time_variable;

        //if (!aoChan_horizontal)
        //    coil_current_voltage[1] =  dirAdjust_bias_y;
        //else
        //    coil_current_voltage[0] = -dirAdjust_bias_y;

        //coil_current_voltage[2]                     = swimmer_A_v * cos( 2*M_PI * swimmer_actuationFreq * time_variable) + swimmer_v_bias;
        //printf("Ready to update coil current.\n");
        //usleep(1e6);

        //update_coil_current(0, l_theta, l_phi);
        l_bias_x = swimmer_dirAdjust_bias * cos(l_theta + swimmer_dirAdjust_angle);
        l_bias_y = swimmer_dirAdjust_bias * sin(l_theta + swimmer_dirAdjust_angle);
        //printf("Bias is x: %.1f and y: %.1f.\n", bias_x, bias_y);
        //printf("Output is x: %.1f and y: %.1f.\n",coil_current_x + bias_x,coil_current_y + bias_y);

        // Rotating field set according to swimmer's geometric heading
        coil_current_voltage[0] = swimmer_A_h * sin(l_phi) * swimming_dir * cos(l_theta) + l_bias_x;
        coil_current_voltage[1] = swimmer_A_h * sin(l_phi) * swimming_dir * sin(l_theta) + l_bias_y;
        coil_current_voltage[2] = swimmer_A_v * cos(l_phi);

        s826_aoPin( 0 , 2, coil_current_voltage[0]);
        s826_aoPin( 1 , 2, coil_current_voltage[1]);
        s826_aoPin( 2 , 2, coil_current_voltage[2]);

        centerP_coor = getCenterPointCoor();
        centerP_rect_short_side_coor_array = get_CenterP_rect_short_side_coor_array();  // Get coor. of center point of 2 short sides
        orientation_angle = atan2(centerP_rect_short_side_coor_array[1] - centerP_coor[1], centerP_rect_short_side_coor_array[0]-centerP_coor[0]);
        //printf("orientation_angle is %.1f.\n", orientation_angle * 180 / M_PI);

		usleep(1e3);
	}

	// Reset all 3 coil currents to 0.
	coilCurrentClear();

	dir_index = 0;
	swim_heading = 1;

	printf("@ the End of swimmer_rotate_field_thread.\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutine: Calc. Diff. between Swimmer's Heading & Desired Orientation
//      Input: a_angle_radian: desired orientation in radians (-M_PI, M_PI]
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float get_orientation_diff(float a_angle_radian)
{
    /*
    int *centerP_coor = getCenterPointCoor();
    int *goalP_coor   = getGoalPointCoor();

    float x_distance = goalP_coor[0] - centerP_coor[0];
    float y_distance = goalP_coor[1] - centerP_coor[1];
    float total_distance = pow(x_distance, 2) + pow(y_distance, 2);  // Distance square
    float angle_radian = atan2(y_distance, x_distance);
    */

    update_swimmer_heading(0,0);
    float orientation_diff = 0;
    float l_angle_radian = a_angle_radian;

    orientation_diff = l_angle_radian - swimmer_heading;   // Difference between the desired orientation and current orientation
    if (orientation_diff > M_PI)
        orientation_diff = orientation_diff - 2*M_PI;
    else if (orientation_diff < -M_PI)
        orientation_diff = orientation_diff + 2*M_PI;

    if ( (orientation_diff > M_PI) || (orientation_diff < -M_PI) )
        printf("Error, orientation_diff is %.3f.\n", orientation_diff);

    return orientation_diff;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TEST: Swimmer Heading Test Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* swimmer_heading_test_thread (void*threadid)
{
    printf("@ the Beginning of swimmer_heading_test_thread.\n");

    //struct timeval start;
	double l_time_now, time_elapsed, time_variable = 0;                       //time in seconds
	//gettimeofday(&start, NULL);
	//double time_init = (double) start.tv_sec + start.tv_usec*1e-6 ;
    double l_time_init = get_currentTime();

    double l_para = 0;   // Parameter in sine and cos functions
    //float l_amplitude_h = 0;

    /// Rotate swimmer along +x dir.
    //coil_current_voltage[0] = 3.0;
    //coil_current_voltage[1] = 0;
    //coil_current_voltage[2] = 0;
    update_coil_current(0, 0 + swimmer_dirAdjust_angle, M_PI/2);
    usleep(5e5);
    coilCurrentClear();

    /// Open a file to record data
    //FILE *fp;
    //fp = fopen("headingTestRecord.txt","w");

    int *centerP_coor;
    float l_theta = 0, l_phi = 0;

    while (swimmer_heading_test_running)
    {
        //gettimeofday(&start, NULL);
        //time_now      = (double) start.tv_sec + start.tv_usec*1e-6 ;    // Current time
        l_time_now    = get_currentTime();
        time_elapsed  = l_time_now - l_time_init;
        time_variable = time_variable + time_elapsed;
        l_time_init   = l_time_now;                                      // Update initial time

        l_para = 2*M_PI * swimmer_actuationFreq * time_variable;

        //l_amplitude_h = swimmer_A_h * sin( l_para);
        // Rotate field along the swimmer's heading
        //update_swimmer_heading();
        //coil_current_voltage[0] = l_amplitude_h * cos(swimmer_heading) + swimmer_dirAdjust_bias * cos(swimmer_heading + swimmer_dirAdjust_angle);
        //coil_current_voltage[1] = l_amplitude_h * sin(swimmer_heading) + swimmer_dirAdjust_bias * sin(swimmer_heading + swimmer_dirAdjust_angle);

        //coil_current_voltage[2] = swimmer_A_v * cos(l_para);
        //l_theta =
        //update_coil_current(1, 0, l_para);   // Use swimmer_heading as theta
        update_coil_current(0, 0, l_para);    // Desired theta is 0

        /// Record center point and heading
        //centerP_coor = getCenterPointCoor();
        //fprintf(fp, "%d %d %.3f\n", centerP_coor[0], centerP_coor[1], swimmer_heading);

        usleep(1e3);
    }

    coilCurrentClear();

    //fclose(fp);   // Close file

    printf("@ the End of swimmer_heading_test_thread.\n");
}

// a_orientation_1 (-M_PI, M_PI], a_orientation_2 (-M_PI, M_PI]
float get_abs_orientation_diff(float a_orientation_1, float a_orientation_2)
{
    float l_diff = a_orientation_1 - a_orientation_2;
    if (l_diff < 0)
        l_diff = -l_diff;
    if (l_diff > M_PI)
        l_diff = 2*M_PI - l_diff;

    return l_diff;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Multiple Swimmer Test Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* multiple_swimmer_thread (void*threadid)
{
	printf("@ the Beginning of multiple_swimmer_thread.\n");

    double l_time_init = 0, l_time_now = 0, l_time_variable = 0;
    int l_dir = dir_index;
    int l_h_channel = 0, l_swimming_dir = 1;
    int l_x_dir = 1, l_y_dir = 1;
    //float l_bias = 0.5;
    float l_bias = swimmer_dirAdjust_bias;

    l_time_init = get_currentTime();

    coilCurrentClear();

    while(flag_swimmer_actuation == 1)
	{
        l_time_now      = get_currentTime();
        l_time_variable = l_time_variable + l_time_now - l_time_init;
        l_time_init     = l_time_now;

        if (l_dir != dir_index)   // If direvction is changed...
		{
            switch(dir_index)
			{
				case 0: l_h_channel = 0;
                        l_swimming_dir =  1;
                        flag_multiSwimmer_stop = 0;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
                        break;
				case 1: l_h_channel = 1;
                        l_swimming_dir =  1;
                        flag_multiSwimmer_stop = 0;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
                        break;
				case 2: l_h_channel = 0;
                        l_swimming_dir = -1;
                        flag_multiSwimmer_stop = 0;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
                        break;
				case 3: l_h_channel = 1;
                        l_swimming_dir = -1;
                        flag_multiSwimmer_stop = 0;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
                        break;
                case 4: l_x_dir = 1;
                        l_y_dir = 1;
                        flag_multiSwimmer_stop = 0;
                        printf("set flag_multiSwimmer_stop to 0.\n");
                        break;
                case 5: l_x_dir = -1;
                        l_y_dir = 1;
                        flag_multiSwimmer_stop = 0;
                        printf("set flag_multiSwimmer_stop to 0.\n");
                        break;
                case 6: l_x_dir = -1;
                        l_y_dir = -1;
                        flag_multiSwimmer_stop = 0;
                        printf("set flag_multiSwimmer_stop to 0.\n");
                        break;
                case 7: l_x_dir = 1;
                        l_y_dir = -1;
                        flag_multiSwimmer_stop = 0;
                        printf("set flag_multiSwimmer_stop to 0.\n");
                        break;
			}
            l_dir = dir_index;
		}

        if (!flag_multiSwimmer_stop)
        {
            if (l_dir < 4)
            {
                coil_current_voltage[l_h_channel]     = swimmer_A_h * l_swimming_dir * sin(2*M_PI*swimmer_actuationFreq*l_time_variable);
                coil_current_voltage[1 - l_h_channel] = 0;
                coil_current_voltage[2]               = swimmer_A_v *                  cos(2*M_PI*swimmer_actuationFreq*l_time_variable);
            }
            else
            {
                coil_current_voltage[0] = swimmer_A_h * sin(M_PI/4.0) * l_x_dir * sin(2*M_PI*swimmer_actuationFreq*l_time_variable);
                coil_current_voltage[1] = swimmer_A_h * sin(M_PI/4.0) * l_y_dir * sin(2*M_PI*swimmer_actuationFreq*l_time_variable);
                coil_current_voltage[2] = swimmer_A_v *                           cos(2*M_PI*swimmer_actuationFreq*l_time_variable);
            }

            coil_current_voltage[0] = coil_current_voltage[0] + l_bias;   // Bias is always applied along +x dir.

            s826_aoPin(0, 2, coil_current_voltage[0]);
            s826_aoPin(1, 2, coil_current_voltage[1]);
            s826_aoPin(2, 2, coil_current_voltage[2]);
        }
        else
        {
            coilCurrentClear();
            s826_aoPin(0, 2, l_bias);
        }
        usleep(1e3);
	}
    coilCurrentClear();
    printf("@ the End of multiple_swimmer_thread.\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread: Magnetization Heading Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int get_netMagneticMomentHeading(float *a_heading)
{
    coilCurrentClear();
    coil_current_voltage[0] = 1.0;
    coil_current_voltage[1] = 0.0;
    coil_current_voltage[2] = 0.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);

    usleep(3e6);
    printf("before update swimmer heading.\n");
    update_swimmer_heading(1,0);
    printf("before recording.\n");
    //float l_heading_1 = swimmer_heading;
    a_heading[0] = swimmer_heading;

    printf("at the beginning of 2nd test.\n");
    coilCurrentClear();
    coil_current_voltage[0] = 0.0;
    coil_current_voltage[1] = 1.0;
    coil_current_voltage[2] = 0.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);

    usleep(3e6);
    update_swimmer_heading(1, M_PI_2);
    //float l_heading_2 = swimmer_heading;
    a_heading[1] = swimmer_heading;

    coilCurrentClear();
    return 1;
}

void* swimmer_magnetizationHeading_thread (void*threadid)
{
    printf("@ the Beginning of swimmer_magnetizationHeading_thread.\n");

    float l_heading[2];
    get_netMagneticMomentHeading(l_heading);
    /*
    coilCurrentClear();
    coil_current_voltage[0] = 1.0;
    coil_current_voltage[1] = 0.0;
    coil_current_voltage[2] = 0.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);

    usleep(3e6);
    update_swimmer_heading(1,0);
    float l_heading_1 = swimmer_heading;

    coilCurrentClear();
    coil_current_voltage[0] = 0.0;
    coil_current_voltage[1] = 1.0;
    coil_current_voltage[2] = 0.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);

    usleep(3e6);
    update_swimmer_heading(1, M_PI_2);
    float l_heading_2 = swimmer_heading;
    */

    FILE *l_fp;
    l_fp = fopen("magnetizationHeading.txt","w");
    fprintf(l_fp, "%.3f %.3f\n", l_heading[0], l_heading[1]);
    fclose(l_fp);
    //coilCurrentClear();
    printf("@ the End of swimmer_magnetizationHeading_thread.\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer Point Following Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* swimmer_point_following_thread (void*threadid)
{
	printf("@ the Beginning of swimmer_point_following_thread.\n");

    int *centerP_coor, *goalP_coor;           // Pointer to the array storing coordinates of center point.
    double l_period = 0;                       // Period of rotating field
    float orientation_diff  = 0;                // Difference between the sample's orientation and the angle from center point to goal point

    int temp_goalP_storage[2] = {320,240};    // Store last goal point coor.

    float x_distance=0, y_distance=0, total_distance=0;   // Distance between current position and goal position
    float threshold = 100;                                // Tolerance for position following: 10 pixel
    //float threshold = 25;                                   // 5 pixels
    //float threshold = 49;                                     // 7 pixels
    //float threshold_pathFollow = 100;                       //
    //float threshold_pathFollow = 400;   // Movement of swimmer is not continuous
    int l_threshold_criticalCoor = 10;    // threshold used for critical coor. judgement
    float threshold_pathFollow = 900;
    float actuation_A_control_lowBound = 400;   // Low and High limits of actuation amplitude control
    //float actuation_A_control_highBound = 10000;
    float actuation_A_control_highBound = 22500;

    //float angle_radian = 0;
    float l_d_heading = 0;     // Desired swimmer heading
    float bias_x = 0, bias_y = 0;

    struct timeval start;
	double time_now, time_elapsed, time_variable = 0;                       //time in seconds
	gettimeofday(&start, NULL);
	double time_init = (double) start.tv_sec + start.tv_usec*1e-6 ;         // Initial time in seconds.

    uint index_pathFollow     = 0;   // Index of the path following mark point
    uint index_pathFollow_max = 55;  // # of points in the path following mark point array
    /// For UT Follow
    uint index_UT_follow = 0;
    uint index_UT_follow_max = 6;

    if (flag_pathFollow)
        getGoalPointCoor_pathFollow(index_pathFollow);

    int l_index_n_UT_follow = 0;
    // If in UT follow mode, set the first goal point
    if (flag_UT_follow)
    {
        getGoalPointCoor_UT_follow(index_UT_follow);
        l_index_n_UT_follow = 0;
    }

    uint flag_goal_reached = 1; // 1: has reached the goal point. Initial value is 1 because we need to turn the sample at the very beginning

    /// Actuation Field Strength
    //float l_swimmer_A_high = 2.5;
    //float l_swimmer_A_low  = 0.5;
    //float l_swimmer_A_high = 1.0;
    //float l_swimmer_A_low  = 0.2;
    float l_swimmer_A_high = 0.5;
    float l_swimmer_A_low  = 0.1;

    FILE * l_marker_filePointer = fopen("Swimmer_marker.txt","w");   // open file for recording
    FILE * l_PI_filePointer = fopen("PI_parameter.txt","w");   // open file for recording


    int l_flag_criticalCoorReached = 0;   // (UT following) to prevent spinning
    int l_critical_coorValue = 0.0;
    int l_critical_coorIndex = 0;         // 0: x; 1: y
    float l_goalP_d_x = 0.0;              // distance between 2 goal points
    float l_goalP_d_y = 0.0;
    int l_critical_coorValue_temp = 0;

    float threshold_PI_control = 5000;
    float l_swimmer_heading = 0;
	while(flag_swimmer_actuation == 1)
	{
        // Tell the vision.c to record the center point of sample
        switch_record_centerP(1);
        //
        centerP_coor = getCenterPointCoor();   // in vision.c, get the center point of swimmer
        goalP_coor   = getGoalPointCoor();

        x_distance = goalP_coor[0] - centerP_coor[0];
        y_distance = goalP_coor[1] - centerP_coor[1];
        total_distance = pow(x_distance, 2) + pow(y_distance, 2);  // Distance square
        swimmer_desiredHeading = atan2(y_distance, x_distance);

        /// Critical Coor. Monitoring

        if ( (temp_goalP_storage[0] == goalP_coor[0]) && (temp_goalP_storage[1] == goalP_coor[1]) )                       // if goal point is not changed ... use AND !!!
        {
            l_critical_coorValue_temp = temp_goalP_storage[l_critical_coorIndex] - l_critical_coorValue * l_threshold_criticalCoor - centerP_coor[l_critical_coorIndex];
            if ( ( ( l_critical_coorValue_temp > 0 ) && (l_critical_coorValue == -1) ) || ( ( l_critical_coorValue_temp < 0 ) && (l_critical_coorValue == 1) ) )
            {
                printf("coorValue_temp is %d and critical_coorValue is %d. temp goal is %d %d.\n", l_critical_coorValue_temp, l_critical_coorValue, temp_goalP_storage[0], temp_goalP_storage[1]);
                l_flag_criticalCoorReached = 1;
            }
        }
        ///

        update_swimmer_heading(0,0);

        if (total_distance > threshold && ( (!flag_criticalCoorMonitor) || (!l_flag_criticalCoorReached) ) )   // if swimmer has not reached its goal AND no critical coor. monitoring or critical coor. has not been reached
        {

            // Actuation Amplitude Feedback Control
            if (flag_actuation_A_control)
            {
                if (total_distance < actuation_A_control_lowBound)
                {
                    //swimmer_A_h = 0.5;
                    //swimmer_A_v = 0.5;
                    //swimmer_A_h = l_swimmer_A_low;
                    //swimmer_A_v = l_swimmer_A_low;
                    swimmer_A_h = swimmer_actuationAcontrol_low;
                }
                else if (total_distance < actuation_A_control_highBound)
                {
                    //swimmer_A_h = 1.0/4800.0 * (total_distance - actuation_A_control_lowBound) + 0.5;
                    //swimmer_A_h = (l_swimmer_A_high - l_swimmer_A_low)/(actuation_A_control_highBound - actuation_A_control_lowBound) * (total_distance - actuation_A_control_lowBound) + l_swimmer_A_low;
                    //swimmer_A_v = swimmer_A_h;
                    swimmer_A_h = (swimmer_actuationAcontrol_high - swimmer_actuationAcontrol_low) / (actuation_A_control_highBound - actuation_A_control_lowBound) * (total_distance - actuation_A_control_lowBound) + swimmer_actuationAcontrol_low;
                }
                else
                {
                    //swimmer_A_h = 2.5;
                    //swimmer_A_v = 2.5;
                    //swimmer_A_h = l_swimmer_A_high;
                    //swimmer_A_v = l_swimmer_A_high;
                    swimmer_A_h = swimmer_actuationAcontrol_high;
                }
                swimmer_A_v = swimmer_A_h;
                swimmer_dirAdjust_bias = swimmer_A_h / 5.0;
            }

            gettimeofday(&start, NULL);
            time_now = (double) start.tv_sec + start.tv_usec*1e-6;
            time_elapsed = time_now - time_init;                         //elapsed time in seconds
            time_variable = time_variable + time_elapsed;
            time_init = time_now;

            //printf("flag goal reached is %d.\n", flag_goal_reached);
            if ( (flag_goal_reached) || (temp_goalP_storage[0] != goalP_coor[0]) || (temp_goalP_storage[1] != goalP_coor[1]) )   // If this is at the beginning of a rotating field ...
            {
                printf("Inside loop.\n");
                //printf("dirAdjust_bias_x is %.2f, and dirAdjust_bias_y is %.2f.\n", dirAdjust_bias_x, dirAdjust_bias_y);

                if ( (temp_goalP_storage[0] != goalP_coor[0]) || (temp_goalP_storage[1] != goalP_coor[1]) )   // if goal point has changed ...
                {
                    l_goalP_d_x = fabs(goalP_coor[0] - centerP_coor[0]);                 // distance between the x-coor. of the new and old goal points
                    l_goalP_d_y = fabs(goalP_coor[1] - centerP_coor[1]);                 // distance between the y-coor. of the new and old goal points
                    l_critical_coorIndex = (l_goalP_d_x > l_goalP_d_y) ? 0:1;
                    l_critical_coorValue = ( (goalP_coor[l_critical_coorIndex] - centerP_coor[l_critical_coorIndex]) > 0 ) ? 1:-1;   // if critical coor. of goal point is bigger than current position, set 1
                    printf("critical coor index is %d and critical coor value is %d.\n",l_critical_coorIndex, l_critical_coorValue);
                    temp_goalP_storage[0] = goalP_coor[0];
                    temp_goalP_storage[1] = goalP_coor[1];
                }

                coilCurrentClear();
                fprintf(l_marker_filePointer, "%.3f %d\n", time_init, 0);   // record current clear time

                if (!flag_goal_reached)  // If swimmer has reached its destination, it is not swimming now.
                {
                    usleep(5e5);         // If goal changes while swimming, wait until the swimmer stops
                    centerP_coor = getCenterPointCoor();   // in vision.c, get current center point of swimmer
                    x_distance = goalP_coor[0] - centerP_coor[0];
                    y_distance = goalP_coor[1] - centerP_coor[1];
                    swimmer_desiredHeading = atan2(y_distance, x_distance);
                }

                //update_coil_current(0, angle_radian, PI/2);
                update_swimmer_heading(0,0);
                orientation_diff = get_abs_orientation_diff(swimmer_heading, swimmer_desiredHeading);

                if ( orientation_diff < (M_PI/2) )
                {
                    //apply_static_coil_current(3.0, angle_radian + swimmer_dirAdjust_angle, M_PI/2);   // Steer swimmer towards goal point
                    swimming_dir = 1;   // Swim forward
                }
                else
                {
                    //apply_static_coil_current(3.0, angle_radian + M_PI + swimmer_dirAdjust_angle, M_PI/2);
                    swimming_dir = -1;   // Swim backward
                }
                if (swimming_dir == 1)
                    l_d_heading = swimmer_desiredHeading;
                else if (swimming_dir == -1)   // If swim backward
                {
                    l_d_heading = swimmer_desiredHeading + M_PI;        // (0, 2*M_PI]
                    if (l_d_heading > M_PI)
                        l_d_heading = l_d_heading - 2*M_PI;   // (-M_PI, M_PI]
                }
                //apply_static_coil_current(3.0, l_d_heading + swimmer_dirAdjust_angle, M_PI/2);   // steer swimmer
                apply_static_coil_current(l_swimmer_A_high/2.0, l_d_heading + swimmer_dirAdjust_angle, M_PI/2);
                l_swimmer_heading = swimmer_heading;                                           // record current swimmer heading
                //printf("swimming direction is %d.\n",swimming_dir);
                usleep(1e5);                                                                   // wait 0.1 s
                time_variable = 0;
                gettimeofday(&start, NULL);
                time_init = (double) start.tv_sec + start.tv_usec*1e-6;
                do
                {
                    //update_swimmer_heading(1, l_d_heading + swimmer_dirAdjust_angle);
                    update_swimmer_heading(1, l_d_heading);                                    // DO NOT include the directional adjust angle!!!
                    if (get_abs_orientation_diff(swimmer_heading, l_swimmer_heading) < 0.02)   // if the swimmer is not turning anymore ...
                        break;
                    gettimeofday(&start, NULL);
                    time_now = (double) start.tv_sec + start.tv_usec*1e-6;
                    //time_elapsed = time_now - time_init;                         //elapsed time in seconds
                    //time_variable = time_variable + time_elapsed;
                    time_variable = time_now - time_init;

                    if (time_variable > 0.3)                                       // if time has passed 0.3 s ...
                        break;
                    l_swimmer_heading = swimmer_heading;
                    usleep(1e5);
                } while (get_abs_orientation_diff(swimmer_heading, l_d_heading) > 0.02 );
                time_init = time_now;                                                   // update time init
                time_variable = 0;
                coilCurrentClear();
                fprintf(l_marker_filePointer, "%.3f %d\n", time_init, 1);               // record end time
                flag_goal_reached = 0;
                /*
                if (total_distance > threshold_M_PI_control)
                {
                    printf("PI control update.\n");
                    do
                    {
                        gettimeofday(&start, NULL);
                        time_now = (double) start.tv_sec + start.tv_usec*1e-6;
                        time_elapsed = time_now - time_init;                         //elapsed time in seconds
                        time_variable = time_variable + time_elapsed;
                        time_init = time_now;

                        //coil_current_voltage[0] = turn_swimmer * cos(angle_radian + swimmer_dirAdjust_angle);
                        //coil_current_voltage[1] = turn_swimmer * sin(angle_radian + swimmer_dirAdjust_angle);

                        //printf("swimmer_dirAdjust_angle: %.1f; x-coil: %.1f; y-coil: %.1f.\n", swimmer_dirAdjust_angle, coil_current_voltage[0], coil_current_voltage[1]);

                        update_coil_current(0, angle_radian, M_PI/2);
                        usleep(1e5);


                        orientation_diff = get_orientation_diff();
                        swimmer_dirAdjust_angle_I = swimmer_dirAdjust_angle_I + I_coeff * orientation_diff * time_elapsed;
                        swimmer_dirAdjust_angle = P_coeff * orientation_diff + swimmer_dirAdjust_angle_I;

                        printf("Angle difference is %.1f. Directional adjust angle is integral: %.1f and total %.1f.\n", orientation_diff*180/M_PI, swimmer_dirAdjust_angle_I*180/M_PI, swimmer_dirAdjust_angle*180/M_PI);

                    } while ( (orientation_diff > 0.05) || (orientation_diff < -0.05) );
                    printf("OUT: %.1f.\n", orientation_diff);
                    usleep(5e5);
                }
                */

                //gettimeofday(&start, NULL);
                //time_init = (double) start.tv_sec + start.tv_usec*1e-6;
            }
            else
            {
                //gettimeofday(&start, NULL);
                //time_now = (double) start.tv_sec + start.tv_usec*1e-6;
                //time_elapsed = time_now - time_init;                         //elapsed time in seconds
                //time_variable = time_variable + time_elapsed;
                //time_init = time_now;

                //bias_x = swimmer_h_bias * cos(angle_radian);
                //bias_y = swimmer_h_bias * sin(angle_radian);
                //amplitude_h = swimmer_A_h * sin( 2*M_PI * swimmer_actuationFreq * time_variable) + swimmer_h_bias;
                //coil_current_voltage[0] = amplitude_h * cos(angle_radian) + dirAdjust_bias_x;
                //coil_current_voltage[1] = amplitude_h * sin(angle_radian) + dirAdjust_bias_y;
                //amplitude_x = (swimmer_A_h + swimmer_h_bias) * cos(angle_radian);
                //amplitude_y = (swimmer_A_h + swimmer_h_bias) * sin(angle_radian);
                //outputV_horizontal = 2.5 * sin( 2*M_PI * AO_sinFreq * time_variable) + 1.0;   // Strong Offset
                //outputV_horizontal = 3.5 * sin( 2*M_PI * AO_sinFreq * time_variable);
                //coil_current_voltage[2] = swimmer_dir_reverse * swimmer_A_v * cos( 2*M_PI * swimmer_actuationFreq * time_variable) + swimmer_v_bias;

                if (swimmer_actuationFreq > 0)
                {
                    l_period = 1 / swimmer_actuationFreq;
                    if (time_variable > l_period)
                    {
                        //time_variable = time_variable % l_period;   // Prevent time_variable explode
                        time_variable = fmod(time_variable, l_period);
                    }
                }
                else
                    printf("Error: swimmer_actuationFreq = 0.\n");

                // TODO: consider the swimming direction.
                if (swimming_dir == 1)
                    l_d_heading = swimmer_desiredHeading;
                else if (swimming_dir == -1)   // If swim backward
                {
                    l_d_heading = swimmer_desiredHeading + M_PI;        // (0, 2*M_PI]
                    if (l_d_heading > M_PI)
                        l_d_heading = l_d_heading - 2*M_PI;   // (-M_PI, M_PI]
                }

                update_coil_current(0, l_d_heading, 2*M_PI * swimmer_actuationFreq * time_variable);

                if (flag_pathFollow && (total_distance < threshold_pathFollow))   // If in path following mode...
                {
                    index_pathFollow = index_pathFollow + 1;
                    if (index_pathFollow > index_pathFollow_max)   // If the swimmer has reached the end of the path...
                    {
                        index_pathFollow = 0;
                        usleep(1e6);
                    }
                    getGoalPointCoor_pathFollow(index_pathFollow);
                }

                orientation_diff = get_orientation_diff(l_d_heading);

                // M_PI controller
                //printf("Total distance is %.1f and threshold_PI_control is %.1f.\n", total_distance, threshold_PI_control);
                if (total_distance > threshold_PI_control)   // When the sample is close to the destination, the angle can change very quickly
                {
                    //printf("Inside PI update.\n");
                    swimmer_dirAdjust_angle_I = swimmer_dirAdjust_angle_I + I_coeff * orientation_diff * time_elapsed;
                    swimmer_dirAdjust_angle_I = adjust_angle_range(swimmer_dirAdjust_angle_I);                          // (-PI, PI]
                    swimmer_dirAdjust_angle   = P_coeff * orientation_diff + swimmer_dirAdjust_angle_I;
                    swimmer_dirAdjust_angle   = adjust_angle_range(swimmer_dirAdjust_angle);                            // (-PI, PI]

                    ///
                    fprintf(l_PI_filePointer, "%.3f %.3f %.3f %.3f\n", orientation_diff, P_coeff * orientation_diff, swimmer_dirAdjust_angle_I, swimmer_dirAdjust_angle);
                    ///
                }
                //printf("Angle difference is %.1f. Directional adjust angle is integral: %.1f and total %.1f.\n", orientation_diff*180/PI, swimmer_dirAdjust_angle_I*180/PI, swimmer_dirAdjust_angle*180/PI);
                usleep(1e3);
            }
        }
        else
        {                        // If the swimmer is very close to the destination...
            coilCurrentClear();
            flag_goal_reached = 1;
            if (flag_UT_follow)
            {
                index_UT_follow = index_UT_follow + 1;
                if (index_UT_follow > index_UT_follow_max)         // If the swimmer has reached the end of the path "UT"...
                {
                    l_index_n_UT_follow = l_index_n_UT_follow + 1; // increase the index by 1 because the sample has finished one time
                    if (l_index_n_UT_follow < index_n_UT_follow)   // If sample has swam desired times of "UT", stop the thread
                    {
                        index_UT_follow = 0;

                        //usleep(2e6);
                    }
                    else
                        flag_swimmer_actuation = 0;
                }
                getGoalPointCoor_UT_follow(index_UT_follow);   // update goal point
            }
            l_flag_criticalCoorReached = 0;
        }
    }

    // Tell the vision.c to STOP recording the center point of sample
    switch_record_centerP(0);
    //

    fclose(l_PI_filePointer);
    fclose(l_marker_filePointer);                                          // close marker file

	coilCurrentClear();   // Reset all 3 coil currents to 0.
    swimmer_dirAdjust_angle_I = 0;
    swimmer_dirAdjust_angle   = 0;
	printf("@ the End of swimmer_point_following_thread.\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize & Stop Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int init_swimmer_thread(void)
{
    //printf("@ the Beginning of init_swimmer_thread.\n");
	pthread_t swimmer_thread;
    //swimmer_actuationThreadControl = 1;
    if (flag_swimmer_actuation)
    {
        flag_swimmer_actuation = 0;
        usleep(5e5);
    }
    flag_swimmer_actuation = 1;
    pthread_create(&swimmer_thread, NULL, swimmer_point_following_thread, NULL);  //start swimmer thread
    return 1;
}

int stop_swimmer_actuation_thread(void)
{
	//swimmer_actuationThreadControl = 0;
	flag_swimmer_actuation = 0;
	return 1;
}

int init_rotate_field_thread(void)
{
    //printf("@ the Beginning of init_swimmer_thread.\n");
	pthread_t rotate_field_thread;
	if (flag_swimmer_actuation)
    {
        flag_swimmer_actuation = 0;
        usleep(5e5);
    }
    flag_swimmer_actuation = 1;
    pthread_create(&rotate_field_thread, NULL, swimmer_rotate_field_thread, NULL);  //start swimmer thread
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize & Stop Path Following
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int init_swimmer_pathFollow(void)
{
    //printf("@ the Beginning of init_swimmer_thread.\n");
    flag_pathFollow = 1;
	pthread_t swimmer_thread;
    //swimmer_actuationThreadControl = 1;
    if (flag_swimmer_actuation)
    {
        flag_swimmer_actuation = 0;
        usleep(5e5);
    }
    flag_swimmer_actuation = 1;
    pthread_create(&swimmer_thread, NULL, swimmer_point_following_thread, NULL);  //start swimmer thread
    return 1;
}

int stop_swimmer_pathFollow(void)
{
	//swimmer_actuationThreadControl = 0;
	flag_swimmer_actuation = 0;
	flag_pathFollow = 0;
	return 1;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Coil Current Clear
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int coilCurrentClear(void) {
    coil_current_voltage[0] = 0;
    coil_current_voltage[1] = 0;
    coil_current_voltage[2] = 0;
    //update_coil_current();

	uint aoRange = 2;   // 2: -5 ~ +5 V.
    s826_aoPin( 0 , aoRange, 0);
	s826_aoPin( 1 , aoRange, 0);
	s826_aoPin( 2 , aoRange, 0);
    s826_aoPin( 3 , aoRange, 0);
    s826_aoPin( 4 , aoRange, 0);
    s826_aoPin( 5 , aoRange, 0);
    s826_aoPin( 6 , aoRange, 0);
    s826_aoPin( 7 , aoRange, 0);
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Rotating Freq.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_swimmerActuationFreq(float freq)
{
    swimmer_actuationFreq = freq;
    return 0;
}

int set_swimmer_dirAdjust_angle(float d)
{
    swimmer_dirAdjust_angle = d * M_PI / 180;  // Convert from degree to radian
    //update_coil_current(0,PI/2);
    s826_aoPin(0, 2, swimmer_dirAdjust_bias * cos(swimmer_dirAdjust_angle));
    s826_aoPin(1, 2, swimmer_dirAdjust_bias * sin(swimmer_dirAdjust_angle));
}

int set_swimmer_dirAdjust_amp(float d)
{
    swimmer_dirAdjust_bias = d;
    //update_coil_current(0,Pi/2);
    s826_aoPin(0, 2, swimmer_dirAdjust_bias * cos(swimmer_dirAdjust_angle));
    s826_aoPin(1, 2, swimmer_dirAdjust_bias * sin(swimmer_dirAdjust_angle));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Direct Control Coil Current
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int set_coil_current_to (int index, float d)
{
    //printf("Inside set_coil_current_to.\n");
    coil_current_voltage[index] = d;
    switch (index)
    {
        case 0: s826_aoPin( 0 , 2, d); //printf("coil_current_x is set to be %.1f.\n",coil_current_x);
                break;
        case 1: s826_aoPin( 1 , 2, d); //printf("coil_current_y is set to be %.1f.\n",coil_current_y);
                break;
        case 2: s826_aoPin( 2 , 2, d); //printf("coil_current_z is set to be %.1f.\n",coil_current_z);
                break;
    }
    //update_coil_current();
}

int set_swimmer_swim_dir_reverse(int d)
{
    if (d == 1)
        swimmer_dir_reverse = -1;
    else
        swimmer_dir_reverse =  1;
}

/* Get current coil current control voltage value */
float get_coil_current(int index)
{
    float returnValue = 0;

    if ( (index > 2) || (index < 0) )
        printf("Error in get_coil_current.\n");
    else
        returnValue = coil_current_voltage[index];

    return returnValue;
}


int set_directional_index(int d)
{
    dir_index = d;
    return 0;
}

/// Get the driving dir. index
int get_directional_index(void)
{
    return dir_index;
}

int set_swimmer_h_bias(float d)
{
    swimmer_h_bias = d;
    return 0;
}

int set_swimmer_v_bias(float d)
{
    swimmer_v_bias = d;
    return 0;
}

int set_swimmer_A_h (float d)
{
    swimmer_A_h = d;
    return 0;
}

int set_swimmer_A_v (float d)
{
    swimmer_A_v = d;
    return 0;
}

//////////////
// Swimmer Heading test

int init_swimmerHeadingTest (void)
{
    swimmer_heading_test_running = 1;
    pthread_t swimmer_heading_thread;

    pthread_create(&swimmer_heading_thread, NULL, swimmer_heading_test_thread, NULL);  //start swimmer thread
    return 0;
}

int stop_swimmerHeadingTest (void)
{
    swimmer_heading_test_running = 0;
    return 0;
}

float get_dirAdjust_angle(void)
{
    return swimmer_dirAdjust_angle;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread: Swimming Speed Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void * swimmer_speed_thread (void * threadid)
{
	printf("@ the Beginning of swimmer_speed_thread.\n");

    swimmer_dirAdjust_bias = swimmer_A_h / 5.0;                 // set bias field strength based on actuation strength

	//FILE * l_fileP = fopen("Swimmer_swimmingSpeed.txt","w");    // open file to record centre point

	double time_now, time_elapsed, l_time_variable = 0;         //time in seconds
	double time_init = get_currentTime();                       // initial time in seconds.
    float l_d_heading = 0.0;                                    // which dir. to apply the bias field
    int * l_centerP_coor;

    FILE *l_fileP = fopen("Swimmer_speedTest.txt","w");
    fprintf(l_fileP, "%.2f %.2f %.2f %.3f %.2f\n", swimmer_A_h, swimmer_A_v, swimmer_dirAdjust_bias, l_d_heading, swimmer_actuationFreq);   // record control para

    int l_index = 0;
    float *l_rectSideP;
    float l_swimmerLength = 0;
    /// Get the swimmer's length
    for (l_index = 0; l_index < 3; l_index++)
    {
        l_rectSideP     = get_CenterP_rect_short_side_coor_array();
        l_swimmerLength = sqrt( pow( (l_rectSideP[2] - l_rectSideP[0]), 2) + pow( (l_rectSideP[3] - l_rectSideP[1]), 2) );
        fprintf(l_fileP, "%.3f\n", l_swimmerLength);   // record swimmer's length in pixels
        usleep(1e6);
    }
    fclose(l_fileP);    // close file
    //update_coil_current(0, l_d_heading, M_PI_2);                // steer swimmer first
    apply_static_coil_current(1, l_d_heading, M_PI_2);            // USE THIS FUNCTION TO STEER SWIMMER
    usleep(5e5);
    coilCurrentClear();

    switch_record_centerP(1);                                   // enable centre point recording
    switch_swimmerHeadingRecord(1);
    while (flag_swimmingSpeed)
    {
        l_centerP_coor  = getCenterPointCoor();                 // get current centre point

        /// If the swimmer has reached the boundaries of the field of view ...
        if ( (l_centerP_coor[0] > 540) || (l_centerP_coor[0] < 100) || (l_centerP_coor[1] < 100) || (l_centerP_coor[1] > 380) )
        {
            flag_swimmingSpeed = 0;
            switch_record_centerP(0);                           // disable centre point recording
            switch_swimmerHeadingRecord(0);
            break;
        }

        time_now        = get_currentTime();
        time_elapsed    = time_now - time_init;                   //elapsed time in seconds
        l_time_variable = l_time_variable + time_elapsed;
        time_init       = time_now;

        update_coil_current(0, l_d_heading, 2 * M_PI * swimmer_actuationFreq * l_time_variable);
        //fprintf(l_fileP, "%.3f %.3f %.3f %.3f\n", orientation_diff, P_coeff * orientation_diff, swimmer_dirAdjust_angle_I, swimmer_dirAdjust_angle);

        usleep(1e3);                                              // delay
    }

	//fclose(l_fileP);
    coilCurrentClear();    // clear coil current

	printf("@ the End of swimmer_speed_thread.\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Swimming Speed Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int initiate_swimmingSpeedTest(void)
{
    if (flag_swimmingSpeed)
    {
        flag_swimmer_actuation = 0;
        usleep(5e5);
    }
    flag_swimmingSpeed = 1;
    pthread_t swimmingSpeed_thread;
    pthread_create( &swimmingSpeed_thread, NULL, swimmer_speed_thread, NULL);  //start swimmer thread
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: UMultiple Swimmer Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_swimmer_multiSwimmer(int d)
{
    if (d == 1)
    {
        if (flag_swimmer_actuation)
        {
            flag_swimmer_actuation = 0;
            usleep(5e5);
        }
        flag_swimmer_actuation = 1;
        pthread_t multiSwimmer_thread;
        pthread_create(&multiSwimmer_thread, NULL, multiple_swimmer_thread, NULL);  //start swimmer thread
    }
    else
        flag_swimmer_actuation = 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: UT Follow
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int init_UT_follow (void)
{
    if (flag_swimmer_actuation)
    {
        flag_swimmer_actuation = 0;
        printf("Stopping current swimmer actuation thread ...\n");
        usleep(5e5);
    }
    flag_swimmer_actuation = 1;
    flag_UT_follow = 1;
    pthread_t swimmer_UT_follow;

    ///
    switch_swimmerHeadingRecord(1);
    ///

    pthread_create( &swimmer_UT_follow, NULL, swimmer_point_following_thread, NULL);  //start swimmer thread
    return 1;
}

int stop_UT_follow (void)
{
    flag_swimmer_actuation = 0;
    flag_UT_follow = 0;

    ///
    switch_swimmerHeadingRecord(0);
    ///

    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Actuation Amplitude Feedback Control
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_flag_actuation_A_control (int d)
{
    flag_actuation_A_control = d;
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Dir. Angle PI Control Parameters
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_swimmer_proportional(float d)
{
    P_coeff = d;
    return 1;
}

int set_swimmer_integral(float d)
{
    I_coeff = d;
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Swimmer Heading
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float get_swimmer_heading (void)
{
    return swimmer_heading;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Swimming Dir.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int get_swimming_dir(void)
{
    return swimming_dir;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Multiple Swimmers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int stop_actuation_field(void)   // Will not stop the thread
{
    flag_multiSwimmer_stop = 1;
    return 1;
}

int click_swimmer_magnetizationHeading(void)
{
    pthread_t swimmer_magnetizationHeading;
    pthread_create( &swimmer_magnetizationHeading, NULL, swimmer_magnetizationHeading_thread, NULL);  //start swimmer thread
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer: Series Test Button Clicked
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* swimmer_series_test_thread (void*threadid)
{
    printf("@ the Beginning of swimmer_series_test_thread.\n");

    float *l_rectSideP; //  = get_CenterP_rect_short_side_coor_array();  // 2 center points of the rect.'s short sides. {point-1.x, point-1,y, point-2.x, point-2.y}

    //float l_constant[] = {1.0, 3.0, 4.0, 7.0};
    //l_rectSideP = l_constant;

    int l_index = 0;
    float l_swimmerLength = 0;

    FILE *l_fileP;
    l_fileP = fopen("Swimmer_SeriesTest.txt","w");

    /// Get the swimmer's length
    for (l_index = 0; l_index < 3; l_index++)
    {
        l_rectSideP  = get_CenterP_rect_short_side_coor_array();
        l_swimmerLength = sqrt( pow( (l_rectSideP[2] - l_rectSideP[0]), 2) + pow( (l_rectSideP[3] - l_rectSideP[1]), 2) );
        fprintf(l_fileP, "%.3f\n", l_swimmerLength);   // record swimmer's length in pixels
        usleep(1e6);
    }

    //l_swimmerLength = l_swimmerLength / 10.0;
    //printf("Result is %.3f.\n", l_swimmerLength);

    /// Get the swimmer's net magnetic moment dir.
    switch_swimmerHeadingRecord(1);   // start recording swimmer's heading
    float l_magnetizationHeading[2];
    for (l_index = 0; l_index < 3; l_index++)
    {
        get_netMagneticMomentHeading(l_magnetizationHeading);
        fprintf(l_fileP, "%.3f %.3f\n", l_magnetizationHeading[0], ( l_magnetizationHeading[1] - M_PI_2 ) );
    }

    /// "UT" following
    index_n_UT_follow = 3;   // ask the sample to follow the path for 3 times
    flag_swimmer_actuation = 1;
    flag_UT_follow = 1;

    pthread_t swimmer_UT_follow;
    pthread_create( &swimmer_UT_follow, NULL, swimmer_point_following_thread, NULL);  //start swimmer thread

    while(flag_swimmer_actuation)
    {
        usleep(1e6);
    }
    switch_swimmerHeadingRecord(0);   // stop recording swimmer's heading

    fclose(l_fileP);
    printf("@ the End of swimmer_series_test_thread.\n");
}


int swimmer_series_test(void)
{
    // Measure length
    // measure net magnetic moment dir.
    // measure swimming dir.
    // measure overshot
    if (!flag_swimmer_seriesTest)
    {
        pthread_t swimmer_seriesTest;
        pthread_create( &swimmer_seriesTest, NULL, swimmer_series_test_thread, NULL);  //start swimmer thread
    }
    else
        printf("Series test is already running.\n");
    return 1;
}

int set_swimmer_actuationAcontrol_low(float d)
{
    swimmer_actuationAcontrol_low = d;
}

int set_swimmer_actuationAcontrol_high(float d)
{
    swimmer_actuationAcontrol_high = d;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cilia: Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void* cilia_testThread (void*threadid)
{
    printf("@ the Beginning of cilia_testThread.\n");

    while(flag_ciliaTest)
    {
        coilCurrentClear();
        s826_aoPin( 0 , 2, -2.5);                                // -2 in x
        usleep(5e5);
        coilCurrentClear();
        s826_aoPin( 1 , 2, -0.6);
        usleep(5e5);
        coilCurrentClear();
        usleep(5e5);
        coilCurrentClear();
        s826_aoPin( 1 , 2,  0.3);
        usleep(5e5);
    }

    coilCurrentClear();
    printf("@ the End of cilia_testThread.\n");
}


int cilia_buttonToggled (int d)
{
    if (d==1)
        if (!flag_ciliaTest)
        {
            flag_ciliaTest = 1;
            pthread_t cilia_test;
            pthread_create( &cilia_test, NULL, cilia_testThread, NULL);  //start swimmer thread
        }
        else
            printf("Cilia test is already running.\n");
    else
        flag_ciliaTest = 0;

    return 1;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Two agent control stop button is pressed
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int stop_twoagentctrl_field(void)   // Will not stop the thread
{
    flag_2Agentctrl_stop = 1;
    return 1;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: set agent control field values
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_directional_index_MA(int d)
{
    dir_index_MA = d;
    return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Two agent control Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* twoAgent_control_thread (void*threadid)
{
	printf("@ the Beginning of two-agent control thread.\n");

    double l_time_init = 0, l_time_now = 0, l_time_variable = 0;
    int l_dir = dir_index_MA;
    int l_h_channel = 0, l_agent_dir = 1;
    int l_x_dir_MA = 1, l_y_dir_MA = 1;
    //float l_bias = 0.5;
    float l_bias = swimmer_dirAdjust_bias;

    l_time_init = get_currentTime();

    coilCurrentClear();

    while(flag_2AgentCtrl == 1)
	{
        l_time_now      = get_currentTime();
        l_time_variable = l_time_variable + l_time_now - l_time_init;
        l_time_init     = l_time_now;

        if (l_dir != dir_index_MA)   // If direcction is changed...
		{
            switch(dir_index_MA)
			{
				case 0: l_h_channel = 0;
                        l_agent_dir =  1;
                        flag_2Agentctrl_stop = 0;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
                        break;
				case 1: l_h_channel = 1;
                        l_agent_dir =  1;
                        flag_2Agentctrl_stop = 0;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
                        break;
				case 2: l_h_channel = 0;
                        l_agent_dir = -1;
                        flag_2Agentctrl_stop = 0;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
                        break;
				case 3: l_h_channel = 1;
                        l_agent_dir = -1;
                        flag_2Agentctrl_stop = 0;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
                        break;
                case 4: l_x_dir_MA = 1;
                        l_y_dir_MA = 1;
                        flag_2Agentctrl_stop = 0;
                        printf("set flag_2Agentctrl_stop to 0.\n");
                        break;
                case 5: l_x_dir_MA = -1;
                        l_y_dir_MA = 1;
                        flag_2Agentctrl_stop = 0;
                        printf("set flag_2Agentctrl_stop to 0.\n");
                        break;
                case 6: l_x_dir_MA = -1;
                        l_y_dir_MA = -1;
                        flag_2Agentctrl_stop = 0;
                        printf("set flag_2Agentctrl_stop to 0.\n");
                        break;
                case 7: l_x_dir_MA = 1;
                        l_y_dir_MA = -1;
                        flag_2Agentctrl_stop = 0;
                        printf("set flag_2Agentctrl_stop to 0.\n");
                        break;
			}
            l_dir = dir_index_MA;
		}

        if (!flag_2Agentctrl_stop)
        {
            if (l_dir < 4)
            {
            //    coil_current_voltage[l_h_channel]     = swimmer_A_h * l_agent_dir * sin(2*M_PI*swimmer_actuationFreq*l_time_variable);
            //   coil_current_voltage[1 - l_h_channel] = 0;
           //     coil_current_voltage[2]               = swimmer_A_v *                  cos(2*M_PI*swimmer_actuationFreq*l_time_variable);

           coil_current_voltage[l_h_channel] = 1 * l_agent_dir;               // enable the desired coil current   :: 1V is given to +1 or -1 direction
           coil_current_voltage[1 - l_h_channel] = 0;                        // clear the other coil voltage

            }
            else
            {
                coil_current_voltage[0] = 1 * l_x_dir_MA;
                coil_current_voltage[1] = 1 * l_y_dir_MA;

            }

           field_angle_MA =  atan2(coil_current_voltage[1], coil_current_voltage[0]);    // in radians!

            coil_current_voltage[0] = coil_current_voltage[0] + l_bias;   // Bias is always applied along +x dir.

            s826_aoPin(0, 2, coil_current_voltage[0]);
            s826_aoPin(1, 2, coil_current_voltage[1]);
         //   s826_aoPin(2, 2, coil_current_voltage[2]);
        }
        else
        {
            coilCurrentClear();
       //     s826_aoPin(0, 2, l_bias);
        }
        usleep(1e3);
	}
    coilCurrentClear();
    printf("@ the End of two agent control thread.\n");
}





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Multi-agent 2 Agent Control pressed created by Mohammad
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_2Agentctrl (int d)
{
    if (d == 1)
    {
        if (flag_2AgentCtrl)
        {
            flag_2AgentCtrl = 0;
            usleep(5e5);
        }
        flag_2AgentCtrl = 1;
        pthread_t twoAgentctrl_thread;
        pthread_create(&twoAgentctrl_thread, NULL, twoAgent_control_thread, NULL);  //starts two agent control thread
    }
    else
        flag_2AgentCtrl = 0;
}
