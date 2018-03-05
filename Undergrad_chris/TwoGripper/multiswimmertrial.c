#include "multiswimmertrial.h"
#include "pthread.h"
#include "math.h"
//static int center_coor[2]={10,10};
static int goal1[2]={40,30};
static int goal2[2]={100,100};
static int position =0;
static float angle;
static float orientation;
static float swimmer_act_low = 0.2;                  //DC Bias Minimum Voltage
static float swimmer_act_high = 1.0;                 //DC Bias Maximum Voltage
static float swimmer_act_mid = 0.5;
static float x_freq = 10, y_freq = 15;
static float swimmer_heading;
static float angle_difference = -M_PI;
static int flag = 1;
static float coil_current_voltage[3];
static float magnet_dir;
static float AC_voltage = 0.6;
static float percent_reduction = 0.5;
static int* center_coor;
static int goal_coor[2];
static float swimmer_actuationFreq =   60;             // For consistency, the frequency has been considered to be 60 for small swimmer and 30 for big swimmers

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

//Function to give fake coordinates to the micro-swimmer
static int * get_centerCoor(void)
{
    double l_time = get_currentTime();
    float x_freq = 10, y_freq = 15;
    int l_x = 320 * sin(l_time * x_freq) + 320;
    int l_y = 240 * sin(l_time * y_freq) + 240;
    center_coor[0] = l_x;
    center_coor[1] = l_y;
    return center_coor;
}

int* fakecenter_coor()
{
    switch(position)
    {
        case 0: center_coor[0]=10;
                center_coor[1]=10;
                break;
        case 1: center_coor[0]=goal1[0];
                center_coor[1]=goal1[1];
                break;
        case 2: center_coor[0]=goal2[0];
                center_coor[1]=goal2[1];
                break;
    }
        return center_coor;
}

float fake_orient()
{
    switch(position)
    {
        case 0: orientation = 2*PI/3;
        break;
        case 1: orientation = atan(2/3);
        break;
        case 2: orientation = atan(6/7);
        break;
    }
    return orientation;
}

float dc_anglecalc(int center[2])                          //DC angle required for the case
{
    float ang, xdist, ydist;
    xdist=goal1[0]-center[0];
    ydist=goal1[1]-center[1];
    ang=atan(ydist/xdist);
    return ang;
}

float dist_calc(int center1[2])                          //DC angle required for the case
{
    float xdist, ydist, dist;
    xdist=goal1[0]-center1[0];
    ydist=goal1[1]-center1[1];
    dist=sqrt(xdist*xdist+ydist*ydist);
    return dist;
}
/*
void* fake_function_trial(void*threadid)
{
    printf("@Starting the fake function...\n");
    int goal=0;
    int i=1;
    int flag_reached=0;
    int* centercoordinates;
    float angle;
    float distance;
    float angle12;
    float angle23;
    float swimmer_A;
    float swimmer_B;
    float DCbiasx, DCbiasy;
    float coil_current_voltage[3];
    float DCangle;
    float actuation_A_control_lowBound = 400;   // Low and High limits of actuation amplitude control
    float actuation_A_control_highBound = 22500;
    double time_now=0,   time_init=0,    time_tot=0;
    centercoordinates=get_centerCoor();
    while(!flag_reached && i <= 100)
    {
        time_now = get_currentTime();
        time_tot = time_tot + time_now - time_init;
        time_init = time_now;

        centercoordinates=fakecenter_coor();
        angle=fake_orient();
        angle12=atan(2/3);
        angle23=atan(6/7);
        DCangle=dc_anglecalc(centercoordinates);
        distance=dist_calc(centercoordinates);

        if (distance < actuation_A_control_lowBound)
                {
                    swimmer_A = swimmer_act_low;
                }
        else if (distance < actuation_A_control_highBound)
                {
                    swimmer_A = (swimmer_act_high - swimmer_act_low) /
                    (actuation_A_control_highBound - actuation_A_control_lowBound) * (distance - actuation_A_control_lowBound) + swimmer_act_low;
                }
        else
                {
                    swimmer_A = swimmer_act_high;
                }
                swimmer_B = swimmer_A / 5.0;

        DCbiasx = swimmer_B * cos(DCangle);
        DCbiasy = swimmer_B * sin(DCangle);

        // Rotating field set according to swimmer's geometric heading
        coil_current_voltage[0] = swimmer_A * cos(DCangle)*sin(2*PI*x_freq*time_tot) + DCbiasx;
        coil_current_voltage[1] = swimmer_A * sin(DCangle)*cos(2*PI*y_freq*time_tot) + DCbiasy;
        coil_current_voltage[2] = 0;

        printf("%f, %f, %f \n", coil_current_voltage[0], coil_current_voltage[1], coil_current_voltage[2]);
        //printf("Center coordinates: %d, %d \n", center_coor[0],center_coor[1]);
        distance=dist_calc(centercoordinates);
        if (distance<10)
        {
            flag_reached=1;
        }
        usleep(1e3);
        i++;
    }


}
*/

/*void* magnet_control(void*threadid)
{
    double time_now=0,   time_init=0,    time_tot=0;
    float coil_current_voltage[3];
    time_now = get_currentTime();
    time_tot = time_tot + time_now - time_init;
    time_init = time_now;

    coilCurrentClear();
    coil_current_voltage[0]=1.0;
    coil_current_voltage[1]=0.0;
    coil_current_voltage[2]=0.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);
    usleep(1e7);

    coilCurrentClear();
    coil_current_voltage[0]=0.0;
    coil_current_voltage[1]=1.0;
    coil_current_voltage[2]=0.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);
    usleep(1e7);

    coilCurrentClear();

    coil_current_voltage[0]=0.0;
    coil_current_voltage[1]=0.0;
    coil_current_voltage[2]=1.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);
    usleep(1e7);

    coilCurrentClear();

    coil_current_voltage[0]=0.7071;
    coil_current_voltage[1]=0.7071;
    coil_current_voltage[2]=0.0;
    s826_aoPin(0, 2, coil_current_voltage[0]);
    s826_aoPin(1, 2, coil_current_voltage[1]);
    s826_aoPin(2, 2, coil_current_voltage[2]);
    usleep(1e7);

    coilCurrentClear();
    int i=0;
    while(i<10000)
    {
        time_now = get_currentTime();
        time_tot = time_tot + time_now - time_init;
        time_init = time_now;
        coil_current_voltage[0] = 0.5*sin(2*PI*x_freq*time_tot);
        coil_current_voltage[1] = 0.5*cos(2*PI*y_freq*time_tot);
        coil_current_voltage[2] = 0;
        printf("%f %f %f \n",coil_current_voltage[0],coil_current_voltage[1],coil_current_voltage[2]);
        s826_aoPin(0, 2, coil_current_voltage[0]);
        s826_aoPin(1, 2, coil_current_voltage[1]);
        s826_aoPin(2, 2, coil_current_voltage[2]);
        usleep(1e6);
        coilCurrentClear();
        i++;
    }


}*/

////////////////////////////////////////////////////Gives required orientation using present heading and desired heading between (0,PI) and (0,-PI)///////////////////////

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

///////////////////////////Copied from Jiachen's Code/////////////////////////

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
    //printf("heading 1: %.1f and heading 2: %.1f. sideP1x: %.1f, sideP1y: %.1f, sideP2x: %.1f, sideP2y: %.1f\n", l_heading_radian[0], l_heading_radian[1], l_rectSideP[0],l_rectSideP[1],l_rectSideP[2],l_rectSideP[3]);

    //if (swimmer_heading_1stTime)                                    // If swimmer's heading has not been determined yet, set the heading as the one pointing to righthandside
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
    //printf("swimmer_heading candidates are %.1f and %.1f.\n", l_heading_radian[0] *180/M_PI, l_heading_radian[1]*180/M_PI);
    //if ( (l_heading_ref > M_PI) || (l_heading_ref <= -M_PI) )
    //        printf("Error in l_heading_ref. it is %.3f.  and swimmer heading is %.3f.\n", l_heading_ref, swimmer_heading);
    //printf("swimmer heading is %.3f.\n", swimmer_heading);
    ////
   // if ( ( (swimmer_heading - swimmer_desiredHeading) < 0.02 ) && ( (swimmer_heading - swimmer_desiredHeading) > -0.02 ) )
    //    printf("Warning! Swimmer heading is close to desired heading. swimmer heading is %.3f.\n", swimmer_heading);
    ////

    //fprintf(l_heading_filePointer, "%.3f %.3f %.3f %d %.3f %.3f %.3f %.3f %.3f %d %d\n", l_heading_radian[0], l_heading_radian[1], l_heading_ref, a_flag, a_heading, l_rectSideP[0],l_rectSideP[1],l_rectSideP[2],l_rectSideP[3],centerP_coor[0],centerP_coor[1]);

    return 0;
}


int set_angle_difference(float p)
{
    angle_difference = angle_difference;
    return 0;
}

int set_netmagnetizationdir(float d)
{
    magnet_dir = d*M_PI/180;
    return 1;
}

static int get_netMagneticMomentHeading(float *a_heading)
{
    coilCurrentClear();
    coil_current_voltage[0] = 0.3;
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
    coil_current_voltage[1] = 0.3;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer Velocity Characterization
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void * swimmer_characterization ( void * threadid )
{
    printf("@ the Beginning of swimmer_characterization.\n");

        FILE *centercoor_swimmer = fopen("centercoor_swimmer.txt","w");

    for (percent_reduction = 0.2; percent_reduction < 1; percent_reduction = percent_reduction + 0.2)
    {
    //float x_distance, y_distance, distance;
    float internal_flag = 1;
    double orient;
    //float DC_bias;
    float start_flag = 1;
    printf("%.4f.\n",magnet_dir);
    float DC_bias_x = AC_voltage * percent_reduction * cos(magnet_dir);
    float DC_bias_y = AC_voltage * percent_reduction * sin(magnet_dir);
    //float   angle_diff       =    set_angle_difference();
    float speed_factor = 1;
    //goal_coor   =    getGoalPointCoor();                                //How does it work?
    //center_coor =    getCenterPointCoor();                              //How does it work?
    //x_distance  =    goal_coor[0]-center_coor[0];
    //y_distance  =    goal_coor[1]-center_coor[1];
    //distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));

    double time_now=0,   time_init=0,    time_tot=0,    time_prev=0,    time_next=0;

                       // printf("D coil speed_factor \n");

                center_coor =    getCenterPointCoor();                              //How does it work?

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////??////////////////////////////////////////////Bringing the swimmer to center ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    int start_coor[2] = {300,250};
                                                               // printf("D coil speed_factor1\n");

                    float x_distance  =    start_coor[0]-center_coor[0];
                    float y_distance  =    start_coor[1]-center_coor[1];

                                                             //   printf("D coil speed_factor2\n");

                    float distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));
                    float angle_needed  =   atan2(y_distance,x_distance);
                    float dcangle   =   get_required_orientation (angle_needed, magnet_dir);
                                                               // printf("D coil speed_factor3\n");

                    float DCx   =   AC_voltage * percent_reduction * cos(dcangle);
                    float DCy   =   AC_voltage * percent_reduction * sin(dcangle);
                    //usleep(1e6);
                                            //printf("D coil speed_factor4\n");

                    coilCurrentClear();
                    coil_current_voltage[0] = DCx;
                    coil_current_voltage[1] = DCy;
                    coil_current_voltage[2]=0.0;
                    printf("DCangle: %f \n", dcangle*180/M_PI);

                    s826_aoPin(0, 2, coil_current_voltage[0]);
                    s826_aoPin(1, 2, coil_current_voltage[1]);
                    s826_aoPin(2, 2, coil_current_voltage[2]);

                    usleep (1e6);
                      //  printf("D coil speed_factor");

//////////////////////////////////////////////////////////Reducing the speed when swimmer is close to its desired position/////////////////////////////////////////////////////////////
                    while (distance > 25 && flag==1)
                    {
                      //  printf("D coil speed_factor");
                        if (distance > 100)
                        {
                            speed_factor = 1;
                        }
                        else if (distance<100 && distance>20)
                        {
                            speed_factor = 0.6;
                        }
                        else if (distance <20)
                        {
                            speed_factor = 0.2;
                        }

                        time_now = get_currentTime();
                //time_tot = time_tot + time_now - time_init;
                //time_init = time_now;
                //time_next = time_tot;

                        //usleep (1e6);
                        time_tot = time_now - time_init;

                        coil_current_voltage[0] = AC_voltage * speed_factor * cos(angle_needed) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCx;
                        coil_current_voltage[1] = AC_voltage * speed_factor * sin(angle_needed) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCy;
                        coil_current_voltage[2] = AC_voltage * speed_factor *                     cos(2*M_PI*swimmer_actuationFreq*time_tot);
                        s826_aoPin(0, 2, coil_current_voltage[0]);
                        s826_aoPin(1, 2, coil_current_voltage[1]);
                        s826_aoPin(2, 2, coil_current_voltage[2]);
                        usleep (1e3);
                        center_coor =    getCenterPointCoor();                              //How does it work?
                        x_distance  =    start_coor[0]-center_coor[0];
                        y_distance  =    start_coor[1]-center_coor[1];
                        distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));
                        angle_needed  =   atan2(y_distance,x_distance);

                    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    ////////// Get Net Magnetization Direction ////////////////



    /////////Applying DC Bias for orienting it in the X- direction/////////////////



            coilCurrentClear();
            coil_current_voltage[0] = DC_bias_x;
            coil_current_voltage[1] = DC_bias_y;
            coil_current_voltage[2]=0.0;
            s826_aoPin(0, 2, coil_current_voltage[0]);
            s826_aoPin(1, 2, coil_current_voltage[1]);
            s826_aoPin(2, 2, coil_current_voltage[2]);
            usleep(3e6);
            int i=0;
            int rev=1;
            //time_now = get_currentTime();
            //time_tot = time_tot + time_now - time_init;
            //time_init = time_now;
            //time_prev = time_tot;
            //time_next = time_tot;

            time_init = get_currentTime();


            //angle_difference = 0;

            while(flag == 1 && internal_flag == 1)
            {
                time_now = get_currentTime();
                //time_tot = time_tot + time_now - time_init;
                //time_init = time_now;
                //time_next = time_tot;

                time_tot = time_now - time_init;

                center_coor =    getCenterPointCoor();                              //How does it work?

                //float   heading_now      =    get_realtime_heading_angle();
                //float   heading_req      =    atan2  (y_distance ,x_distance);
                ///////////////////////Applying rotating field in the desired angle///////////////////////////////////////////////////////////////////////

                coil_current_voltage[0] = AC_voltage * cos(angle_difference) * rev * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DC_bias_x;
                coil_current_voltage[1] = AC_voltage * sin(angle_difference) * rev * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DC_bias_y;
                coil_current_voltage[2] = AC_voltage *                         rev * cos(2*M_PI*swimmer_actuationFreq*time_tot);
                s826_aoPin(0, 2, coil_current_voltage[0]);
                s826_aoPin(1, 2, coil_current_voltage[1]);
                s826_aoPin(2, 2, coil_current_voltage[2]);

                //printf("%f, %f, %f \n", coil_current_voltage[0], coil_current_voltage[1], coil_current_voltage[2]);
                //printf("%f \n",angle_difference);
                fprintf(centercoor_swimmer,"%f, %f, %f, %.4f, %d, %d \n", AC_voltage, percent_reduction, angle_difference*180/M_PI, time_tot,center_coor[0],center_coor[1]);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Bringing back the swimmer to the centre of workspace///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                usleep(1e3);
                if (center_coor[0] <= 100 || center_coor[0] >= 500 || center_coor[1] <= 50 || center_coor[1] >= 450 || time_tot > 5 )
                {
                    //start_coor[2] = {300,250};
                    x_distance  =    start_coor[0]-center_coor[0];
                    y_distance  =    start_coor[1]-center_coor[1];
                    distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));
                    angle_needed  =   atan2(y_distance,x_distance);
                    dcangle   =   get_required_orientation (angle_needed, magnet_dir);
                    DCx   =   AC_voltage * percent_reduction * cos(dcangle);
                    DCy   =   AC_voltage * percent_reduction * sin(dcangle);
                    //usleep(1e6);
                    coilCurrentClear();
                    coil_current_voltage[0] = DCx;
                    coil_current_voltage[1] = DCy;
                    coil_current_voltage[2]=0.0;
                    printf("DCangle: %f \n", dcangle*180/M_PI);

                    s826_aoPin(0, 2, coil_current_voltage[0]);
                    s826_aoPin(1, 2, coil_current_voltage[1]);
                    s826_aoPin(2, 2, coil_current_voltage[2]);

                    usleep (1e6);

//////////////////////////////////////////////////////////Reducing the speed when swimmer is close to its desired position/////////////////////////////////////////////////////////////
                    while (distance > 23 && flag==1)
                    {
                        //printf("DCangle: %f Distance: %.3f , Angle_needed: %f , volt1: %f  volt2: %f  volt3: %f speed_factor = %f \n", dcangle*180/M_PI, distance, angle_needed, coil_current_voltage[0], coil_current_voltage[1], coil_current_voltage[2], speed_factor);
                        if (distance > 100)
                        {
                            speed_factor = 1;
                        }
                        else if (distance<100 && distance>20)
                        {
                            speed_factor = 0.6;
                        }
                        else if (distance <20)
                        {
                            speed_factor = 0.2;
                        }

                        time_now = get_currentTime();
                //time_tot = time_tot + time_now - time_init;
                //time_init = time_now;
                //time_next = time_tot;

                        //usleep (1e6);
                        time_tot = time_now - time_init;

                        coil_current_voltage[0] = AC_voltage * speed_factor * cos(angle_needed) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCx;
                        coil_current_voltage[1] = AC_voltage * speed_factor * sin(angle_needed) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCy;
                        coil_current_voltage[2] = AC_voltage *                     rev * cos(2*M_PI*swimmer_actuationFreq*time_tot);
                        s826_aoPin(0, 2, coil_current_voltage[0]);
                        s826_aoPin(1, 2, coil_current_voltage[1]);
                        s826_aoPin(2, 2, coil_current_voltage[2]);
                        usleep (1e3);
                        center_coor =    getCenterPointCoor();                              //How does it work?
                        x_distance  =    start_coor[0]-center_coor[0];
                        y_distance  =    start_coor[1]-center_coor[1];
                        distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));
                        angle_needed  =   atan2(y_distance,x_distance);
                        start_flag=0;

                    }
                    if (start_flag ==0 && angle_difference != -M_PI)
                    {
                        start_flag = 1;
                        time_tot = 0;
                        time_init = time_now;
                        angle_difference = angle_difference + 10*M_PI/180;
                        coilCurrentClear();
                    }

                        coilCurrentClear();
                        coil_current_voltage[0] = DC_bias_x;
                        coil_current_voltage[1] = DC_bias_y;
                        coil_current_voltage[2] = 0.0;
                        s826_aoPin(0, 2, coil_current_voltage[0]);
                        s826_aoPin(1, 2, coil_current_voltage[1]);
                        s826_aoPin(2, 2, coil_current_voltage[2]);
                        usleep(1e6);


                }


                if (angle_difference > M_PI)
                {
                    internal_flag = 0;
                    angle_difference = -M_PI;

                }
            }

    }

    fclose(centercoor_swimmer);
    coilCurrentClear();

    printf("@ the End of swimmer_characterization.\n");
}
/*
void * b_magnitude_verification ( void * threadid )
{
    printf("@ the Beginning of b_magnitude_verification.\n");

        FILE *b_magnitude = fopen("B_magnitude.txt","w");


    //float x_distance, y_distance, distance;
    float internal_flag = 1;
    double orient;
    //float DC_bias;
    float start_flag = 1;
    printf("%.4f.\n",magnet_dir);
    float DC_bias_x = AC_voltage * percent_reduction * cos(magnet_dir);
    float DC_bias_y = AC_voltage * percent_reduction * sin(magnet_dir);
    //float   angle_diff       =    set_angle_difference();
    float swimmer_actuationFreq =   30;
    //goal_coor   =    getGoalPointCoor();                                //How does it work?
    //center_coor =    getCenterPointCoor();                              //How does it work?
    //x_distance  =    goal_coor[0]-center_coor[0];
    //y_distance  =    goal_coor[1]-center_coor[1];
    //distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));

    double time_now=0,   time_init=0,    time_tot=0,    time_prev=0,    time_next=0;




    ////////// Get Net Magnetization Direction ////////////////

    //float l_netMDir[2];
    //get_netMagneticMomentHeading(l_netMDir);
    fprintf(b_magnitude,"%.4f |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| \n", magnet_dir);    // store the heading for net magnetization test

    /////////Applying DC Bias for orienting it in the X- direction/////////////////



            coilCurrentClear();
            coil_current_voltage[0] = DC_bias_x;
            coil_current_voltage[1] = DC_bias_y;
            coil_current_voltage[2]=0.0;
            s826_aoPin(0, 2, coil_current_voltage[0]);
            s826_aoPin(1, 2, coil_current_voltage[1]);
            s826_aoPin(2, 2, coil_current_voltage[2]);
            usleep(3e6);
            int i=0;
            int rev=1;
            float speed_factor = 1;
            //time_now = get_currentTime();
            //time_tot = time_tot + time_now - time_init;
            //time_init = time_now;
            //time_prev = time_tot;
            //time_next = time_tot;

            time_init = get_currentTime();


            //angle_difference = 0;

            while(flag == 1 && internal_flag == 1 && AC_voltage <2)
            {
                time_now = get_currentTime();
                //time_tot = time_tot + time_now - time_init;
                //time_init = time_now;
                //time_next = time_tot;

                time_tot = time_now - time_init;

                center_coor =    getCenterPointCoor();                              //How does it work?

                //float   heading_now      =    get_realtime_heading_angle();
                //float   heading_req      =    atan2  (y_distance ,x_distance);
                ///////////////////////Applying rotating field in the desired angle///////////////////////////////////////////////////////////////////////

                coil_current_voltage[0] = AC_voltage * cos(angle_difference) * rev * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DC_bias_x;
                coil_current_voltage[1] = AC_voltage * sin(angle_difference) * rev * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DC_bias_y;
                coil_current_voltage[2] = AC_voltage *                         rev * cos(2*M_PI*swimmer_actuationFreq*time_tot);
                s826_aoPin(0, 2, coil_current_voltage[0]);
                s826_aoPin(1, 2, coil_current_voltage[1]);
                s826_aoPin(2, 2, coil_current_voltage[2]);

                //printf("%f, %f, %f \n", coil_current_voltage[0], coil_current_voltage[1], coil_current_voltage[2]);
                //printf("%f \n",angle_difference);
                fprintf(b_magnitude,"%f, %f, %f, %.4f, %d, %d \n", AC_voltage, percent_reduction, angle_difference*180/M_PI, time_tot,center_coor[0],center_coor[1]);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Bringing back the swimmer to the centre of workspace///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                usleep(1e3);
                if (center_coor[0] <= 100 || center_coor[0] >= 500 || center_coor[1] <= 50 || center_coor[1] >= 450 || time_tot > 5 )
                {
                    int start_coor[2] = {350,250};
                    float x_distance  =    start_coor[0]-center_coor[0];
                    float y_distance  =    start_coor[1]-center_coor[1];
                    float distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));
                    float angle_needed  =   atan2(y_distance,x_distance);
                    float dcangle   =   get_required_orientation (angle_needed, magnet_dir);
                    float DCx   =   AC_voltage * percent_reduction * cos(dcangle);
                    float DCy   =   AC_voltage * percent_reduction * sin(dcangle);
                    //usleep(1e6);
                    coilCurrentClear();
                    coil_current_voltage[0] = DCx;
                    coil_current_voltage[1] = DCy;
                    coil_current_voltage[2]=0.0;
                    printf("DCangle: %f \n", dcangle*180/M_PI);

                    s826_aoPin(0, 2, coil_current_voltage[0]);
                    s826_aoPin(1, 2, coil_current_voltage[1]);
                    s826_aoPin(2, 2, coil_current_voltage[2]);

                    usleep (1e6);

//////////////////////////////////////////////////////////Reducing the speed when swimmer is close to its desired position/////////////////////////////////////////////////////////////
                    while (distance > 23 && flag==1)
                    {
                        //printf("DCangle: %f Distance: %.3f , Angle_needed: %f , volt1: %f  volt2: %f  volt3: %f speed_factor = %f \n", dcangle*180/M_PI, distance, angle_needed, coil_current_voltage[0], coil_current_voltage[1], coil_current_voltage[2], speed_factor);
                        if (distance > 100)
                        {
                            speed_factor = 1;
                        }
                        else if (distance<100 && distance>20)
                        {
                            speed_factor = 0.5;
                        }
                        else if (distance <20)
                        {
                            speed_factor = 0.2;
                        }

                        time_now = get_currentTime();
                //time_tot = time_tot + time_now - time_init;
                //time_init = time_now;
                //time_next = time_tot;

                        //usleep (1e6);
                        time_tot = time_now - time_init;

                        coil_current_voltage[0] = AC_voltage* speed_factor * cos(angle_needed) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCx;
                        coil_current_voltage[1] = AC_voltage* speed_factor * sin(angle_needed) * sin(2*M_PI*swimmer_actuationFreq*time_tot) + DCy;
                        coil_current_voltage[2] = AC_voltage*                     rev * cos(2*M_PI*swimmer_actuationFreq*time_tot);
                        s826_aoPin(0, 2, coil_current_voltage[0]);
                        s826_aoPin(1, 2, coil_current_voltage[1]);
                        s826_aoPin(2, 2, coil_current_voltage[2]);
                        usleep (1e3);
                        center_coor =    getCenterPointCoor();                              //How does it work?
                        x_distance  =    start_coor[0]-center_coor[0];
                        y_distance  =    start_coor[1]-center_coor[1];
                        distance    =    sqrt(pow(x_distance,2)+pow(y_distance,2));
                        angle_needed  =   atan2(y_distance,x_distance);
                        start_flag=0;

                    }

                     if (start_flag ==0)
                    {
                        start_flag = 1;
                        time_tot = 0;
                        time_init = time_now;
                        AC_voltage = AC_voltage +0.05;
                        coilCurrentClear();
                    }
                        coilCurrentClear();
                        coil_current_voltage[0] = DC_bias_x;
                        coil_current_voltage[1] = DC_bias_y;
                        coil_current_voltage[2] = 0.0;
                        s826_aoPin(0, 2, coil_current_voltage[0]);
                        s826_aoPin(1, 2, coil_current_voltage[1]);
                        s826_aoPin(2, 2, coil_current_voltage[2]);
                        usleep(1e6);

                }



    }

    fclose(b_magnitude);
    coilCurrentClear();

    printf("@ the End of swimmer_characterization.\n");
}
*/
int get_GoalPointCoor(int a_index)
{
    switch (a_index)
    {
        case 0: goal_coor[0] = 80;
                goal_coor[1] = 400;
                break;
        case 1: goal_coor[0] = 80;
                goal_coor[1] = 80;
                break;
        case 2: goal_coor[0] = 320;
                goal_coor[1] = 80;
                break;
        case 3: goal_coor[0] = 320;
                goal_coor[1] = 400;
                break;
        case 4: goal_coor[0] = 560;
                goal_coor[1] = 400;
                break;
        case 5: goal_coor[0] = 440;
                goal_coor[1] = 400;
                break;
        case 6: goal_coor[0] = 440;
                goal_coor[1] = 80;
                break;
        default: printf("Error: getGoalPointCoor_UT_follow() - a_index is %d.\n", a_index);
    }

    return 1;
}


int initiate_multiswimmer_trial(void)
{
    flag = 1;
    pthread_t multiswimmerthread_trial;
    pthread_create( &multiswimmerthread_trial, NULL, swimmer_characterization, NULL);

    //printf("Thread created...\n");
    return 0;
}

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
}
