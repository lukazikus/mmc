
#include "twoswimmers.h"
#include "pthread.h"
#include "math.h"

static int position =0;
static float angle;
static float orientation;
static float swimmer_act_low = 0.2;                  //DC Bias Minimum Voltage
static float swimmer_act_high = 1.0;                 //DC Bias Maximum Voltage
static float swimmer_act_mid = 0.5;
static float x_freq = 10, y_freq = 15;
static float swimmer_heading;
static int flag = 1;
static float coil_current_voltage[3];
static float magnet_dir[2];
static float AC_voltage = 0.6;
static float percent_reduction = 0.5;
static int* center_coor[2];
static int goal_coor[2][2];
static int start_coor [2][2];
static int dest_coor[2][2];
static float DCx, DCy, dir;
static float speed_factor;
static float swimmer_actuationFreq =   30;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutine: Get Current Time in Seconds
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct timeval start;

static double get_currentTime(void)
{
    gettimeofday(&start, NULL);
    double l_time = (double) start.tv_sec + start.tv_usec*1e-6 ;   // seconds
    return l_time;
}

// Gives required orientation using present heading and desired heading between (0,PI) and (0,-PI)
static float get_required_orientation(float a_orientation_1, float a_orientation_2)
{
    float l_sum = a_orientation_1 + a_orientation_2;
    if (l_sum > M_PI)
    {
        l_sum = -(l_sum-M_PI);
    }
    else if (l_sum < -M_PI)
    {
        l_sum = 2*M_PI + l_sum;
    }

    return l_sum;
}

////////////////////////////////////////////////////// Uncomment these two functions when in use ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// sets magnetization direction for the two swimmers using spin button on the UI ///////////////////////////////////////
/*
 int set_netmagnetizationdir(float d)
{
    magnet_dir[0] = d*M_PI/180;
    return 1;
}*/

 int set_netmagnetization2nddir(float d)
{
    magnet_dir[1] = d*M_PI/180;
    return 1;
}

static int get_GoalPointCoor(int a_index, int swimmer_index)
{
    switch (swimmer_index)
    {
    case 0 :    switch (a_index)
                {
                    case 0: goal_coor[swimmer_index][0] = 80;
                            goal_coor[swimmer_index][1] = 400;
                            break;
                    case 1: goal_coor[swimmer_index][0] = 80;
                            goal_coor[swimmer_index][1] = 140;
                            break;
                    case 2: goal_coor[swimmer_index][0] = 80;
                            goal_coor[swimmer_index][1] = 80;
                            break;
                    case 3: goal_coor[swimmer_index][0] = 320;
                            goal_coor[swimmer_index][1] = 80;
                            break;
                    case 4: goal_coor[swimmer_index][0] = 320;
                            goal_coor[swimmer_index][1] = 400;
                            break;

                }
    case 1 :     switch (a_index)
                {
                    case 0: goal_coor[swimmer_index][0] = 320;
                            goal_coor[swimmer_index][1] = 400;
                            break;
                    case 1: goal_coor[swimmer_index][0] = 560;
                            goal_coor[swimmer_index][1] = 400;
                            break;
                    case 2: goal_coor[swimmer_index][0] = 440;
                            goal_coor[swimmer_index][1] = 400;
                            break;
                    case 3: goal_coor[swimmer_index][0] = 440;
                            goal_coor[swimmer_index][1] = 80;
                            break;

                }
    }
    return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////   Copied from Jiachen's Code     ///////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static float get_realtime_heading_angle(float a_flag, float a_heading)
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

    //{
        if (l_rectSideP[0] > l_rectSideP[2])                        // If point-1 at the right hand side of point-2 ...
            swimmer_heading = l_heading_radian[0];
        else
            swimmer_heading = l_heading_radian[1];

    //    swimmer_heading_1stTime = 0;
    //}
    /*
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
    */

    return 0;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////  Getting the net magnetization direction //////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    //printf("before update swimmer heading.\n");
    get_realtime_heading_angle(1,0);
    //printf("before recording.\n");
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
    get_realtime_heading_angle(1, M_PI_2);
    //float l_heading_2 = swimmer_heading;
    a_heading[1] = swimmer_heading;

    coilCurrentClear();
    return 1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Bring to start position ///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int bring_to_start_position(int s_index)
{
    printf("m4\n");

    center_coor[0] = getCenterPointCoor();
    center_coor[1] = get2ndCenterPointCoor();

    int x_distance  =    dest_coor[s_index][0]-center_coor[s_index][0];
    int y_distance  =    dest_coor[s_index][1]-center_coor[s_index][1];
    double time_now=0,   time_init=0,    time_tot=0,    time_prev=0,    time_next=0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////// Only focussing on the x- and y- motion ////////////////////////////////////////////////////////
    /////////////////////////// First X-motion ////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (x_distance > 0)
    {
        dir   =   1;
    }
    else if (x_distance < 0)
    {
        dir   =   -1;
    }
    else if (x_distance == 0)
    {
        dir = 0;
    }



    DCx = AC_voltage * percent_reduction * cos(magnet_dir[s_index]) ;
    DCy = AC_voltage * percent_reduction * sin(magnet_dir[s_index]) ;
    coilCurrentClear();
    coil_current_voltage[0] = DCx;
    coil_current_voltage[1] = DCy;
    coil_current_voltage[2] = 0.0;

    printf("DCx = %f, DCy = %f",DCx,DCy);

    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);
    printf("x_distance = %d, y_distance = %d, center_coor[0] = (%d,%d), center_coor[1] = (%d,%d) \n", x_distance, y_distance, center_coor[0][0],center_coor[0][1], center_coor[1][0], center_coor[1][1] );

    usleep (1e6);




    time_init = get_currentTime();
    while ((x_distance > 10 || x_distance < -10) &&  flag==1)
    {   //printf("m7\n");
        //printf("x_distance = %d, y_distance = %d, center_coor[0] = (%d,%d), center_coor[1] = (%d,%d) \n", x_distance, y_distance, center_coor[0][0],center_coor[0][1], center_coor[1][0], center_coor[1][1] );

        //////////////////////////////////////// Deciding the direction of the swimmer movement ///////////////////////////////
        if (x_distance > 0)
        {
            dir   =   1;
        }
        else if (x_distance < 0)
        {
            dir   =   -1;
        }
        else if (x_distance == 0)
        {
            dir = 0;
        }


        if ((dir*x_distance) > 100)
        {
            speed_factor = 1;
        }
        else if ((dir*x_distance)<100 && (dir*x_distance)>10)
        {
            speed_factor = 0.6;
        }
        else if ((dir*x_distance) <10)
        {
            speed_factor = 0.2;
        }

        time_now = get_currentTime();

        time_tot = time_now - time_init;

        coil_current_voltage[0] = AC_voltage * speed_factor * dir * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCx;
        coil_current_voltage[1] = DCy;
        coil_current_voltage[2] = AC_voltage * speed_factor * dir * cos(2*M_PI*swimmer_actuationFreq*time_tot);
        s826_aoPin(0, 2, coil_current_voltage[0]);
        s826_aoPin(1, 2, coil_current_voltage[1]);
        s826_aoPin(2, 2, coil_current_voltage[2]);
        usleep (1e3);
        if (s_index == 0)
        {
            center_coor[s_index] =    getCenterPointCoor();
        }
        else if (s_index == 1)
        {
            center_coor[s_index] =    get2ndCenterPointCoor();
        }
        x_distance  =    dest_coor[s_index][0]-center_coor[s_index][0];
        y_distance  =    dest_coor[s_index][1]-center_coor[s_index][1];

    }

        ///////////////////////////// Then Y-motion //////////////////////////////////////////////////////////////////////////////
        if (y_distance > 0)
        {
            dir   =   1;
        }
        else if (y_distance < 0)
        {
            dir   =   -1;
        }
        else if (y_distance == 0)
        {
            dir = 0;
        }

        DCx = AC_voltage * percent_reduction * sin(magnet_dir[s_index]);
        DCy = AC_voltage * percent_reduction * cos(magnet_dir[s_index]);
        coilCurrentClear();
        coil_current_voltage[0] = DCx;
        coil_current_voltage[1] = DCy;
        coil_current_voltage[2]=0.0;

        s826_aoPin(0, 2, coil_current_voltage[0]);
        s826_aoPin(1, 2, coil_current_voltage[1]);
        s826_aoPin(2, 2, coil_current_voltage[2]);

        usleep (1e6);
        printf("DCx = %f , DCy = %f", DCx, DCy);
    while ( ((y_distance) > 10 ||(y_distance)<-10) && flag==1)
    {
            //printf("x_distance = %d, y_distance = %d, center_coor[0] = (%d,%d), center_coor[1] = (%d,%d) \n", x_distance, y_distance, center_coor[0][0],center_coor[0][1], center_coor[1][0], center_coor[1][1] );

        //////////////////////////////////////// Deciding the direction of the swimmer movement ///////////////////////////////
        if (y_distance > 0)
        {
            dir   =   1;
        }
        else if (y_distance < 0)
        {
            dir   =   -1;
        }
        else if (y_distance == 0)
        {
            dir = 0;
        }


        if ((dir*y_distance) > 100)
        {
            speed_factor = 1;
        }
        else if ((dir*y_distance)<100 && (dir*y_distance)>20)
        {
            speed_factor = 0.6;
        }
        else if ((dir*y_distance) <20)
        {
            speed_factor = 0.2;
        }

        time_now = get_currentTime();

        time_tot = time_now - time_init;

        coil_current_voltage[0] = DCx;
        coil_current_voltage[1] = AC_voltage * speed_factor * dir * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCy;
        coil_current_voltage[2] = AC_voltage * speed_factor * dir * cos(2*M_PI*swimmer_actuationFreq*time_tot);
        s826_aoPin(0, 2, coil_current_voltage[0]);
        s826_aoPin(1, 2, coil_current_voltage[1]);
        s826_aoPin(2, 2, coil_current_voltage[2]);
        usleep (1e3);
        if (s_index == 0)
        {
            center_coor[s_index] =    getCenterPointCoor();
        }
        else if (s_index == 1)
        {
            center_coor[s_index] =    get2ndCenterPointCoor();
        }
        x_distance  =    dest_coor[s_index][0]-center_coor[s_index][0];
        y_distance  =    dest_coor[s_index][1]-center_coor[s_index][1];

    }
    if (flag == 0)
    {
        coilCurrentClear();
    }
    return 1;

}

//////////////////////////////////////////////////////End of bring_to_start_position function ////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////Sending to goal_coordinate function ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int change_position(void)
{
        float x_distance1, y_distance1, x_distance2, y_distance2;
        float distance1, distance2;
        float dcangle1, dcangle_deg1, dcangle2, dcangle_deg2;            ////// Decided by the angle needed to be aligned with swimmer 1
        float dir_x1, dir_y1, dir_x2, dir_y2;
        float ac_angle;
        float ymov,xmov;
        int flag_reached = 0;
        int swim_x, swim_y;
        double time_now=0,   time_init=0,    time_tot=0,    time_prev=0,    time_next=0;

        center_coor[0] = getCenterPointCoor();
        center_coor[1] = get2ndCenterPointCoor();

        x_distance1 = goal_coor[0][0] - center_coor[0][0];
        y_distance1 = goal_coor[0][1] - center_coor[0][1];
        x_distance2 = goal_coor[1][0] - center_coor[1][0];
        y_distance2 = goal_coor[1][1] - center_coor[1][1];

        distance1 = sqrt(pow(x_distance1,2)+pow(y_distance1,2));
        distance2 = sqrt(pow(x_distance2,2)+pow(y_distance2,2));


        /////////////////////////// Deciphering the direction in which the swimmer1 is moving //////////////////////////////
        dcangle1 = atan2 (y_distance1, x_distance1);
        dcangle_deg1 = dcangle1 * 180 / M_PI;
        if (dcangle_deg1 < 20 && dcangle_deg1 > -20)
        {
            if (x_distance1 > 0)
            {
                dir_x1 = 1;
            }
            else if (x_distance1 < 0)
            {
                dir_x1 = -1;
            }
            dir_y1 = 0;
        }
        else if (y_distance1 > 0)
        {
            dir_x1 = 0;
            dir_y1 = 1;
        }
        else if (y_distance1 < 0)
        {
            dir_x1 = 0;
            dir_y1 = -1;
        }

        /////////////////////////// Deciphering the direction in which the swimmer2 is moving //////////////////////////////
        dcangle2 = atan2 (y_distance2, x_distance2);
        dcangle_deg2 = dcangle2 * 180 / M_PI;
        if (dcangle_deg2 < 20 && dcangle_deg2 > -20)
        {
            if (x_distance2 > 0)
            {
                dir_x2 = 1;
            }
            else if (x_distance2 < 0)
            {
                dir_x2 = -1;
            }
            dir_y2 = 0;
        }
        else if (y_distance2 > 0)
        {
            dir_x2 = 0;
            dir_y2 = 1;
        }
        else if (y_distance2 < 0)
        {
            dir_x2 = 0;
            dir_y2 = -1;
        }


        ///////////////////////////////// Finding the angle of AC field needed for moving both the swimmers simultaneously to their destination /////////
        if (dir_x1 != 0)
        {
            xmov = x_distance1;
            swim_x = 0;
        }
        else if (dir_x2 != 0)
        {
            xmov = x_distance2;
            swim_x = 1;
        }

        if (dir_y1 != 0)
        {
            ymov = y_distance1;
            swim_y = 0;
        }
        else if (dir_y2 != 0)
        {
            ymov = y_distance2;
            swim_y = 1;
        }

        ac_angle = atan2(ymov,xmov);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////// Applying DC field ////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        DCx = AC_voltage * percent_reduction * dir_x1;
        DCy = AC_voltage * percent_reduction * dir_y1;
        coilCurrentClear();

        coil_current_voltage[0] = DCx;
        coil_current_voltage[1] = DCy;
        coil_current_voltage[2]=0.0;

        s826_aoPin(0, 2, coil_current_voltage[0]);
        s826_aoPin(1, 2, coil_current_voltage[1]);
        s826_aoPin(2, 2, coil_current_voltage[2]);

        usleep (1e6);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////// Applying AC field ////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        while (flag_reached == 1)
        {
            if (xmov > 100 || ymov > 100)
            {
                speed_factor = 1;
            }
            else if ((xmov<100 && xmov>20)||(ymov<100 && ymov>20))
            {
                speed_factor = 0.6;
            }
            else if (xmov <20)
            {
                speed_factor = 0.2;
            }

            time_now = get_currentTime();
            //time_tot = time_tot + time_now - time_init;
            //time_init = time_now;
            //time_next = time_tot;

            //usleep (1e6);
            time_tot = time_now - time_init;

            coil_current_voltage[0] = AC_voltage * speed_factor * cos(ac_angle) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCx;
            coil_current_voltage[1] = AC_voltage * speed_factor * sin(ac_angle) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCy;
            coil_current_voltage[2] = AC_voltage * speed_factor *                 cos(2*M_PI*swimmer_actuationFreq*time_tot);
            s826_aoPin(0, 2, coil_current_voltage[0]);
            s826_aoPin(1, 2, coil_current_voltage[1]);
            s826_aoPin(2, 2, coil_current_voltage[2]);
            usleep (1e3);

            center_coor[0] =    getCenterPointCoor();                              //How does it work?
            center_coor[1] =    get2ndCenterPointCoor();


            xmov  =    goal_coor[swim_x][0]-center_coor[swim_x][0];
            ymov  =    goal_coor[swim_y][1]-center_coor[swim_y][1];

            if ((xmov<10||xmov> -10) || (ymov<10||ymov> -10))
            {
                flag_reached = 1;
            }

            //distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));
            //angle_needed  =   atan2(y_distance,x_distance);
            //start_flag=0;


        }
            return 1;
}






///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////Main function ///////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void * UT_follower ( void * threadid )
{

    printf("m1\n");
    start_coor[0][0] = 80;
    start_coor[0][1] = 400;
    start_coor[1][0] = 320;
    start_coor[1][1] = 400;
    dest_coor[0][0] = 80;
    dest_coor[0][1] = 400;
    dest_coor[1][0] = 320;
    dest_coor[1][1] = 400;

    printf("m2\n");
    bring_to_start_position(0);
    printf("m3\n");
    bring_to_start_position(1);

    ///////////////////// Following left line of U and (top line of T + Returning to center) //////////////////////////////////////////////////////////
if (flag ==1)
{
    get_GoalPointCoor(0,0);
    get_GoalPointCoor(0,1);

    change_position();

    get_GoalPointCoor(1,0);
    get_GoalPointCoor(1,1);

    change_position();
}


}

int initiate_UT_follow (void)
{
    flag = 1;
    pthread_t UT_follow_thread;
    pthread_create( &UT_follow_thread, NULL, UT_follower, NULL);

    //printf("Thread created...\n");
    return 0;
}
/*
 int stop_everything(void)
{
    flag = 0;
    coilCurrentClear();
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Net Magnetization Dir. & Store It
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void * mS_getNetMDirTHREAD ( void*threadid )
{
    printf("@ the Beginning of mS_getNetMDirTHREAD.\n");

    FILE *l_file = fopen("mS_netMagnetizationDir.txt","w");

    float l_netMDir[2];
    get_netMagneticMomentHeading(l_netMDir);
    float a_x = l_netMDir[0]*180/M_PI;
    float a_y = l_netMDir[1]*180/M_PI;
        int l_index = 0;
    float l_swimmerLength = 0;
    float* l_rectSideP;

    /// Get the swimmer's length
    for (l_index = 0; l_index < 3; l_index++)
    {
        l_rectSideP  = get_CenterP_rect_short_side_coor_array();
        l_swimmerLength = sqrt( pow( (l_rectSideP[2] - l_rectSideP[0]), 2) + pow( (l_rectSideP[3] - l_rectSideP[1]), 2) );
        usleep(1e6);
    }

    fprintf(l_file,"%.4f, %.4f, %.3f \n",a_x,a_y,l_swimmerLength);    // store the heading for net magnetization test

    fclose(l_file);

    printf("@ the End of mS_getNetMDirTHREAD.\n");
}

int mS_getNetMagnetizationDir(void)
{
    pthread_t mS_getNetMDirThread;
    pthread_create( &mS_getNetMDirThread, NULL, mS_getNetMDirTHREAD, NULL);
    return 1;
}*/
