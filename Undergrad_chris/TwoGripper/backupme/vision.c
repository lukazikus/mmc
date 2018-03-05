////////////////////////////////////////////////////////////////////////////////////////
// File      : vision.c
// Function  : Camera-related subroutines
// Edited by : Jiachen, Zhe
////////////////////////////////////////////////////////////////////////////////////////
#include "vision.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int show_3agent = 0;          // to detect three agents
int show_kalman = 0;            // Kalman multi-object tracking
int show_kalman_xz = 1;            // Kalman multi-object tracking
int currentIndex = 0;  // Kalman trace length
int currentIndex_xz = 0;  // Kalman trace length

int blobcolorthreshold = 0;        // toggle color threshold in Blob detection  :: dark or bright
int BlobMinTh_var = 0;             // assign min threshold in Blob detection
int BlobMaxTh_var = 200;            // assign max threshold in Blob detection
int BlobMinArea_var = 130;             // assign min threshold in Blob detection
int BlobMaxArea_var = 50000;            // assign max threshold in Blob detection

FWcamera cam, cam_xz; //see FWcamera.cpp

int width = 640;   //image width, pixels
int height = 480;  //image height, pixels
int depth = 1;     //depth of image
//unsigned char * image = NULL;
int killVisionThread = 1; //this stops our vision thread

GtkLabel *labelFPSreceive, *labelFPSreceive_xz;

static Point mouse, mouseC, mouseR;
static Point mouse_xz, mouseC_xz, mouseR_xz;

//ImageProcessor RobotTracker(width, height, 1);
static int detect = 1; //are we detecting object?
int cannyLow=100, cannyHigh=150; //thresholds for image processing filter
static int dilater = 1;
static int edgemap = 0, binary = 0; //are we performing edgemap calculations?
int visionParam1 = 65; //for processing. Used in threshold() and houghCircle().
int visionParam2 = 35; //for processing
int Hough_MinRAdius_var = 3;               //for processing. Used in houghCircle() indicating min radius of circles.
int Hough_MaxRAdius_var = 170;             //for processing. Used in houghCircle() indicating max radius of circles.
int Hough_canny1_var = 100;                //for processing. Used in houghCircle() indicating canny1 value.
int Hough_canny2_var = 9;                //for processing. Used in houghCircle() indicating canny2 value.
int Hough_minDis_var = 10;                 // minimum admissible distance between center points in hough circle transform
int Hough_Binary_adj = 100;               //for processing. Used in houghCircle() indicating the level of binary conversion.
int Hough_Erosion_adj = 2;               //for processing. Used in houghCircle() indicating the level of erosion conversion.
int Hough_dilation_adj = 2;               //for processing. Used in houghCircle() indicating the level of dilation conversion.
int showagentcirc = 0;           // Hough circle show multiagent
int show_agentSingle = 0;          // to show single agent
int RollAveEnable = 2;        // to enable rolling average filtering
int showagentpreprocessVar = 0;    // preprocessing in multiagent project
int showagentBlob = 0;         // Blob  circle show multiagent
int showagentBinary = 0;         // Binary show multiagent
int showagentcircularity = 0.9;         // Binary show multiagent
int showagentconvexity = 0.9;         // Binary show multiagent
int showagentinertia = 0.9;         // Binary show multiagent
int imcolor_enabled = 1;         // enable conversion to imcolor to draw field
int setBlursizeHough = 1;        // set Blur size in Hough transform                put 7 if you are using Median filter otherwise 16 to 25 is good
int swapthreshold = 15;         // set Binary Threshold in Hough transform
int swapratio = 4;         // set swap ratio in Hough transform
static int ControlType;
static int detect1 = 0; //are we detecting object?
static float fpsReceive; //frames per second of video
static int flag_centerP_record = 0; // record center point or not. 1: yest, 0: no

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DOM project created by Mohammad
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int detect_region[4] = {0,0,0,0};				// storage for detect region {x, y, x_max, y_max}
/// Valve
static int flag_valve_detect = 1;				// flag for beads detectin in valve proj.
static int valve_region_value[2] = {0,0};				// regional density value
static int valve_detect_region[4] = {0,0,0,0};	// detect region
int f_bead = 0;				// flags: if a new frame has arrived; if a bead is detected
static Mat flow_reg, register_reg;			// current images of flow-region and register-region
static Mat flow_reg_pre, register_reg_pre, register_reg_k_pre, background, backImage, foreground;			// previous images of flow-region and register-region
static Mat threshold_output;    // for threshold and rectangle detection
static uchar threshold_DOM = 6;						// threshold of image diff. for detecting bead
static int flag_ROI = 0;     // permits to read the image created by ROI
static int flag_clear = 1;     // permits to read the image created by ROI
static bool flag_Hough = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int cannyLow_xz=100, cannyHigh_xz=150; //thresholds for image processing filter
static int dilater_xz = 1;
static int edgemap_xz = 0, binary_xz = 0; //are we performing edgemap calculations?
int visionParam1_xz = 65; //for processing
int visionParam2_xz = 35; //for processing
static int detect_xz = 0; //are we detecting object?
static int sidecam_on = 1; //is the sidecam capturing? Default: YES
static float fpsReceive_xz; //frames per second of video
static int topcam_on = 1; //is the sidecam capturing? Default: YES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// magnet detection variables -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int magnetdetection = 0;
bool showbox = true;
bool draw_annotation = true;
bool showprocess = false;
bool draw_roi = true;
bool draw_points = true;
int closediameter = 6;
int needle_thick = 30;
double magnet_threshold = 48;
float needle_x, needle_y;
float m_x, m_y, m_a = 0.0, m_x_history[6] = {0,0,0,0,0,0}, m_y_history[6] = {0,0,0,0,0,0}, m_a_history[6] = {0,0,0,0,0,0}; // historical value of magnet centre (m_x, m_y) and angle m_a
float m_x_temp, m_y_temp, m_a_temp;
float magnet_area = 0;
float trust_area = 0;
bool flag_magnet_sampled = false;
float Mwidth, Mlength;
bool drawMagetization = 0, showdestination = 1, showfielddirection = 1;
int counter = 0;
float m_aPre = -90;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This thread runs the image processing functions:
// initialize camera, receive and display frames, process images
Mat img_m_color_for_display, img_m_color_for_display2;
static float pix2mm_MA = 0.015;
static float* des_val_READ[1];
static float  Display_angle;
static float  Display_angle_xz;
static float des_DisVar_MA = 105;
static float des_angVar_MA;
static float des_angPull_MA = 45;
static char goal_str_MA[15];
static char goal_str_MATraj[15];
static float pairdist_mag = 100, pairdist_angle=0, pairdist_magORG=100, pairdist_angleORG=0, check_relative_dis=100, pairdist_mag_xz = 100, pairdist_angle_xz=0;
static float pairdist_mag13 = 100, pairdist_angle13=0, pairdist_mag23 = 100, pairdist_angle23=0;   // three agents separations
static float inputangle_MA=0;
//////////////////////////////////// DC and AC arrow print  -- Piyush ////////////////////////////////////////////////////////
int dc_arrow_active = 0;
int ac_arrow_active = 0;
int s1_magnet_dir_active = 0;
int s2_magnet_dir_active = 0;
int s1_heading_dir_active = 0;
int s2_heading_dir_active = 0;
static Point prev_centerP_adjusted, prev_centerP_adjusted_2;
static int frame_index_arrow = 0;
static Point prev_centerP_adjusted_MA, prev_centerP_adjusted_2_MA;    // for multi_agent
static int frame_index_arrow_MA = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This thread runs the image processing functions:
// initialize camera, receive and display frames, process images

Point centerP_adjusted, centerP_adjusted_xz;
Point centerP_adjusted_MA, centerP_adjusted_MA_xz;    // multi-agent adjusted y direction center point

Point2f centerP_rect_short_side[2];   // 2 center points of the rect.'s short sides.
Point2f centerP_rect_short_side_xz[2];   // 2 center points of the rect.'s short sides.


Point2f second_centerP_rect_short_side[2];   // 2 center points of the rect.'s short sides.


static Point centerP_adjusted_2,centerP_adjusted_2_xz;


/////////     Multiagent points        ////////////////
static Point center_m_MA;                   // center variable in Hough circle detection
static Point largest_center_MA;             // Largest center variable in Hough circle detection
static Point secondlargest_center_MA;             // Second Largest center variable in Hough circle detection
static Point largest_center_MA_xz;             // Largest center variable in Hough circle detection
static Point secondlargest_center_MA_xz;             // Second Largest center variable in Hough circle detection
static Point COM_MA;
static Point COM_MA_xz;                                   // Center-of-mass definitio                     // Center-of-mass definition
static Point rolling_MA_first;                       // first rolling avergae point
static Point rolling_MA_second;                       // second rolling avergae point
static Point thirdlargest_center_MA;             // Third Largest center variable in Hough circle detection
static Point first_orient_center_MA;             // first orient circle multi-agent
static Point second_orient_center_MA;             // second orient circle multi-agent
static Point third_orient_center_MA;             // second orient circle multi-agent
static Point first_agent_center;              // first agent center recoreded variable
static Point second_agent_center;              // second agent center recoreded variable
static Point third_agent_center;              // second agent center recoreded variable
static Point first_agent_center_xz;              // first agent center recoreded variable
static Point second_agent_center_xz;              // second agent center recoreded variable
static Point third_agent_center_xz;              // second agent center recoreded variable
static Point first_agent_center_MA;              // first agent center recoreded variable
static Point second_agent_center_MA;              // second agent center recoreded variable
static Point third_agent_center_MA;              // second agent center recoreded variable

static float  dist_orient_MA_largest;                 // orientation marker for the largest
static float  dist_orient_MA_second_largest;           // orientation marker for the second largest

static Point first_agent_center_PAST;             // PAST Largest center variable in Hough circle detection
static Point  second_agent_center_PAST;             // PAST second Largest center variable in Hough circle detection

static Point first_agent_center_2ndPAST;           // 2nd frame before last  or agent number 1
static Point second_agent_center_2ndPAST;           // 2nd frame before last for agent number 2

static Point first_agent_center_3rdPAST;           // 3rd frame before last  for agent number 1
static Point second_agent_center_3rdPAST;           // 3rd frame before last for agent number 2


static Point first_agent_center_4thPAST;           // 4th frame before last  for agent number 1
static Point second_agent_center_4thPAST;           // 4th frame before last for agent number 2

static Point first_agent_center_5thPAST;           // 5th frame before last  for agent number 1
static Point second_agent_center_5thPAST;           // 5th frame before last for agent number 2

static Point startP_MA; Point endP_MA; Point endP2_MA; Point endP3_MA;    // Drawing point multi-agent


static int radius_check_1;          // Hough circle radius swap
static int radius_check_2;          // Hough circle radius swap
static int radius_check_3;          // Hough circle radius swap


static double swimmer_angle = 0;

static int centerP_adjust_flag = 0;
static int centerPointCoorArray[2]  = {0,0};
static int centerPoint2CoorArray[2] = {0,0};
static int centerPointCoorArray_MA[9];                                // multiagent center point of agent 1, 2 AND their orientation circles
static float DistanceCoorArray_MA[6]  ;                                // multiagent center point of agent 1, 2 AND their orientation circles
static float VAR_CTRL_MA[3] ;                                // returns desired distance and pixel2mm conversionz
static int goalPointCoorArray[2]   = {320, 240};
static int goalPointCoorCArray[2]   = {320, 240};
static int goalPointCoorRArray[2]   = {320, 240};
static int goalPointCoorArray_xz[2]   = {320, 240};
static int goalPointCoorCArray_xz[2]   = {320, 240};
static int goalPointCoorRArray_xz[2]   = {320, 240};
static float rollAveXdiff;
static float rollAveYdiff;

static float centerP_rect_short_side_coor_array[4];   // point-1.x, point-1,y, point-2.x, point-2.y
static int flag_second_rect = 0;
static int flag_orinet_circ = 0;                          // orientation_circ detection flag, multi-agent (1:active mode toggle button)

//Variable for plot_line
bool plot_line_flag = false;
int line_x[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int line_y[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int line_R[6]={0,0,0,0,0,0};
int line_G[6]={0,0,0,0,0,0};
int line_B[6]={0,0,0,0,0,0};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Multi_agent window variables
// draw lines and and put texts
Point textpoint_MA;
Point textpointN_MA;

double fab_fontscale_MA = 0.6;
Scalar fab_color_MA = (50, 50, 0); // red
int fab_thickness_MA = 2;
//show angle and temperature
char angle_text_MA[51];
// Kalman variables
static int stateSize = 4;
static int measSize = 2;
static int contrSize = 0;
static bool found = false;
static int notFoundCount = 0;
static float exitX;
static float exitY;
static vector<int> not_assigned_tracks;
static int N =4;
static int M =4;
static vector< vector<double> > Cost(N,vector<double>(M));
static vector<int> assignment;
static float Accel_noise_mag;
static double dist_thres, dt = 0.016, dt_xz = 0.016;
static int maximum_allowed_skipped_frames, max_trace_length;
static float X=0,Y=0;
static float Xmeasured,Ymeasured,Rmeasured;
static float Xmeasured_xz,Ymeasured_xz,Rmeasured_xz;
RNG rng;
Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,255,255)};   // detection colors
int firstC = 1;   // initialization flag in Kalman filter
int largest_area = 0;
int largest_contour_index = 0;
int largest_radius_MA = 0;
int largest_radius_MA_index = 0;
int a_MA;
int second_largest_radius_MA = 0;
int third_largest_radius_MA = 0;
int second_largest_radius_MA_index = 0;
int third_largest_radius_MA_index = 0;
int first_orient_detected_MA_radius = 0;
int first_orient_detected_MA_index = 0;
int second_orient_detected_MA_radius = 0;
int second_orient_detected_MA_index = 0;
int third_orient_detected_MA_radius = 0;
int third_orient_detected_MA_index = 0;
int cirSize = 0;
int cirSize_xz = 0;

vector<Vec3f> circles_m; //for hough circle detection
vector<Vec3f> circles_m_xz; //for hough circle detection
Point temp_swap_center_MA;
int temp_swap_radius_MA;
/////////////////////////////   Checking the swap of coordinates ///////////////////////////
Point center_check_1;
Point center_check_2;
Point center_adjusted_check_1;
Point center_adjusted_check_2;
Point temp_swap_center;
// Extension for multi-agent
Point center_check_1_MA;
Point center_check_2_MA;
Point center_check_3_MA;
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat getImage(void)
{
	return img_m_color_for_display;
}

Mat getImage2(void)
{
	return img_m_color_for_display2;
}

int * getCenterPointCoor(void)
{
    centerPointCoorArray[0] = centerP_adjusted.x;
    centerPointCoorArray[1] = centerP_adjusted.y;
    return centerPointCoorArray;
}

int * get2ndCenterPointCoor(void)
{
    centerPoint2CoorArray[0] = centerP_adjusted_2.x;
    centerPoint2CoorArray[1] = centerP_adjusted_2.y;
    return centerPoint2CoorArray;
}

float * get_CenterP_rect_short_side_coor_array(void)
{
    centerP_rect_short_side_coor_array[0] = centerP_rect_short_side[0].x;
    centerP_rect_short_side_coor_array[1] = 480 - centerP_rect_short_side[0].y;  // !!! The y direction is inversed.

    centerP_rect_short_side_coor_array[2] = centerP_rect_short_side[1].x;
    centerP_rect_short_side_coor_array[3] = 480 - centerP_rect_short_side[1].y;
    return centerP_rect_short_side_coor_array;
}

int * getGoalPointCoor(void)
{
   // if(mouse.x>0)
 //   {
        goalPointCoorArray[0] = mouse.x;
        goalPointCoorArray[1] = 480 - mouse.y;   // Note the positive y dir.
 //   }
    return goalPointCoorArray;
}



// multi-agent center point read out for control purpose
int * getCenterPointCoor_MA(void)
{
    centerPointCoorArray_MA[0] = first_agent_center.x;
    centerPointCoorArray_MA[1] = 480 - first_agent_center.y;       // Note the positive y dir.
    centerPointCoorArray_MA[2] = 0;                                // z axis value
    centerPointCoorArray_MA[3] = second_agent_center.x;
    centerPointCoorArray_MA[4] = 480 - second_agent_center.y;      // Note the positive y dir.
    centerPointCoorArray_MA[5] = 0;                                // z axis value
    centerPointCoorArray_MA[6] = third_agent_center.x;
    centerPointCoorArray_MA[7] = 480 - third_agent_center.y;       // Note the positive y dir.
    centerPointCoorArray_MA[8] = 0;                                // z axis value

    return centerPointCoorArray_MA;
}

float * getDistanceCoor(void)
{

DistanceCoorArray_MA[0] = pairdist_mag;        // equivalent to r12
DistanceCoorArray_MA[1] = pairdist_angle;      // equivalent to phi12
DistanceCoorArray_MA[2] = pairdist_mag13;
DistanceCoorArray_MA[3] = pairdist_angle13;
DistanceCoorArray_MA[4] = pairdist_mag23;
DistanceCoorArray_MA[5] = pairdist_angle23;

    return DistanceCoorArray_MA;
}


float * getCTRL_MA(void)
{
    VAR_CTRL_MA[0] = des_DisVar_MA;
    VAR_CTRL_MA[1] = pix2mm_MA;
    VAR_CTRL_MA[2] = des_angVar_MA*M_PI/180;
    VAR_CTRL_MA[3] = des_angPull_MA*M_PI/180;
    return VAR_CTRL_MA;
}

///////////////////////////////////////////////////////////
//mouseC and mouseR are reversed                         //
//////////////////////////////////////////////////////////

int * getGoalPointCoorC(void)
{
    if(mouseR.x>0)
    {
        goalPointCoorCArray[0] = mouseR.x;
        goalPointCoorCArray[1] = 480 - mouseR.y;   // Note the positive y dir.
    }
    return goalPointCoorCArray;
}

int * getGoalPointCoorR(void)
{
    if(mouseC.x>0)
    {
        goalPointCoorRArray[0] = mouseC.x;
        goalPointCoorRArray[1] = 480 - mouseC.y;   // Note the positive y dir.
    }
    return goalPointCoorRArray;
}

int * getGoalPointCoor_xz(void)
{
    if(mouse_xz.x>0)
    {
        goalPointCoorArray_xz[0] = mouse_xz.x;
        goalPointCoorArray_xz[1] = 480 - mouse_xz.y;   // Note the positive y dir.
    }
    return goalPointCoorArray_xz;
}

///////////////////////////////////////////////////////////
//mouseC and mouseR are reversed                         //
//////////////////////////////////////////////////////////

int * getGoalPointCoorC_xz(void)
{
    if(mouseR_xz.x>0)
    {
        goalPointCoorCArray_xz[0] = mouseR_xz.x;
        goalPointCoorCArray_xz[1] = 480 - mouseR_xz.y;   // Note the positive y dir.
    }
    return goalPointCoorCArray_xz;
}

int * getGoalPointCoorR_xz(void)
{
    if(mouseC_xz.x>0)
    {
        goalPointCoorRArray[0] = mouseC.x;
        goalPointCoorRArray[1] = 480 - mouseC.y;   // Note the positive y dir.
    }
    return goalPointCoorRArray_xz;
}



// Get the orientation angle of the swimmer from the camera image.
double get_swimmer_angle()
{
    return swimmer_angle;
}


void initVision(void)
{
	pthread_t vthread, fthread, vthread_xz, fthread_xz;

	if(!cam_xz.initialize_xz()) //cam is instance of FWCamera, found in FWcamera.cpp
	{
		g_print("FW camera xz could not be found in initVision!!!\n");
		return;
	}
	usleep(1e5);
	if(!cam_xz.startGrabbingVideo_xz())
	{
		g_print("FW cam xz could not grab in initVision!!!\n");
		return;
	}

	if(killVisionThread == 0)
		g_print("Vision already running in initVision!!!\n");
	usleep(1e5);

	if(killVisionThread == 1)
  	{
  		killVisionThread = 0;
		//pthread_create(&fthread, NULL, FPSprint    , NULL);  //start frame print thread. Functionality??? comment this if you do not want to show frame per second
		pthread_create(&vthread_xz, NULL, visionThread_xz, NULL);  //start vision thread
	}

	// X-Z Camera
	if(topcam_on == 1) //if we are also using the top cam
	{
		usleep(2e5);
		printf("Before cam_xy.initialize_xz().\n");
		if(!cam.initialize()) //cam is instance of FWCamera, found in FWcamera.cpp
		{
			g_print("FW camera xy could not be found in initVision!!!\n");
			return;
		}
		usleep(1e5);

		if(!cam.startGrabbingVideo())
		{
			g_print("FW cam xy could not grab in initVision!!!\n");
			return;
		}

		usleep(1e5);
		//pthread_create(&fthread_xz, NULL, FPSprint_xz, NULL);  //start frame print thread
		pthread_create(&vthread, NULL, visionThread, NULL);  //start vision thread
	}
	return;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Frame Print Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This thread is not used? UI still displays images after this is commented out.
void* FPSprint(void*)
{
	char strReceive[50];
	char strReceive_xz[50];

	while(!killVisionThread)   //while the image processing is running //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
		//  int sprintf(char *str, const char *format, ...) sends formatted output to a string pointed to by str.
		sprintf(strReceive, "%.1f", fpsReceive); //writes into strRecieve the frames per second
		sprintf(strReceive_xz, "%.1f", fpsReceive_xz);
		gdk_threads_enter();
		gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive); //draw on the gui
		gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
		gdk_threads_leave();
		usleep((int)1e6); //sets frame rate display frequency
	}
	//printf("In the test zone!\n");
	sprintf(strReceive, "N/A"); //when we turn off the vision, write N/A into the gui
	sprintf(strReceive_xz, "N/A");
	gdk_threads_enter();
	gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive);
	gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
	gdk_threads_leave();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void vel_arrow(Mat Image,int c1,int c2,float vel,float Theta, CvScalar Color,int Size,int Thickness)
{
    cv::Point pt1,pt2;


    pt1.x=c1;
    pt1.y=c2;

    float u = vel * cos(Theta) * Size;
    float v = vel * Size * sin(Theta);

    pt2.x=c1+u;
    pt2.y=c2+v;

    cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line


    Size=(int)(5);


    if(Theta==M_PI/2 && pt1.y > pt2.y)
    {
        pt1.x=(int)(Size*cos(Theta)-Size*sin(Theta)+pt2.x);
        pt1.y=(int)(Size*sin(Theta)+Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line

        pt1.x=(int)(Size*cos(Theta)+Size*sin(Theta)+pt2.x);
        pt1.y=(int)(Size*sin(Theta)-Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line
    }
    else{
        pt1.x=(int)(-Size*cos(Theta)-Size*sin(Theta)+pt2.x);
        pt1.y=(int)(-Size*sin(Theta)+Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line

        pt1.x=(int)(-Size*cos(Theta)+Size*sin(Theta)+pt2.x);
        pt1.y=(int)(-Size*sin(Theta)-Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line
    }

}


void* visionThread(void*)
{
	printf("@ the Beginning of visionThread().\n");

	int index = 0;
	unsigned char *inImage;// = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	Mat img_m, img_m_color, img_m_gray;
	Mat threshold_output;    // for threshold and rectangle detection
	int i;
	timeval tStart, tEnd;
	float time;
	double current_time;
	float fpsVec[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex = 0;
  int frame_xz = 0;
  char str_xz[100];
  timeval tStart_xz, tEnd_xz;
  float time_xz;
  double current_time_xz;
  float fpsVec_xz[10] = {10,10,10,10,10,10,10,10,10,10};
  int fpsIndex_xz = 0;
  double time_current, time_elapsed, time_init, time_now,   time_tot,    time_prev,    time_next;

    struct timeval start;
    gettimeofday(&start, NULL);
    time_init = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time

    FILE *fp;
    fp = fopen("pointFollowRecord.txt","w");

// Kalman initialization for x-y vision
    //CTracker tracker(dt,50,100,20,50);     // initialization of tracker and Kalman filter  dt: sampling time 50:acceleration    100: dist_threshold    50:_maximum_allowed_skipped_frames  50:_max_trace_length
    currentIndex = 50;
    CTracker tracker(dt,50,65,50,currentIndex);     // initialization of tracker and Kalman filter  dt: sampling time 10:acceleration    100: dist_threshold    50:_maximum_allowed_skipped_frames  50:_max_trace_length


	while(!killVisionThread) //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
	    time_prev = get_currentTime();

        //printf("Marker 1.\n");
		//g_print("Got frame %d.   ", frame++);

		gettimeofday(&tStart, NULL);
		//usleep(6e4); //slow down vision thread

        // this function watis for a new frame, it takes a long time, so we can do some image processing in this thread
		inImage = cam.grabAframe(); //unsigned char *inImage;
		if(inImage == NULL)
		{
			g_print("Error in firewire stream! Reattempting...\n");
			usleep((int)1e3); // I don't know what the wait delay should be
		}

		img_m = Mat(height, width, CV_8UC1, inImage); //convert to Mat format


		//gettimeofday(&start, NULL);
		//time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time
		//time_elapsed = time_current - time_init;
		//time_current = time_init;
		//printf("in vision thread, time is %.5f.\n", time_elapsed);
		//

		if(edgemap==1)
		{
			Canny(img_m, img_m, cannyLow, cannyHigh, 3 ); //edge detect

			if(dilater>0) //if dilater = 0, just use original edgemap
			{
				dilate( img_m, img_m, Mat(), Point(-1, -1), dilater, 1, 1);
				//smooth( img_m, img_m, CV_MEDIAN, 5, 5);
				erode( img_m, img_m, Mat(), Point(-1, -1), dilater, 1, 1);
			}
		//	 	circle( img_m, MM, 10, Scalar(20,100,255) , -1, 8, 0 );          // Test Hough circle detection mode
		}

        //printf("Marker 2.\n");
/*
		if(detect == 1) //for threshold and bounding box detection
		{
			blur( img_m, threshold_output, Size(4,4) ); //blur image to remove small blips etc
			threshold( threshold_output, threshold_output, visionParam1, 255, THRESH_BINARY_INV );
			//adaptiveThreshold(img_m, threshold_output, 255,	ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,91,0);
			if(dilater>0) //if dilater = 0, just use original edgemap
			{
				dilate( threshold_output, threshold_output, Mat(), Point(-1, -1), dilater, 1, 1);
				erode( threshold_output, threshold_output, Mat(), Point(-1, -1), 2*dilater, 1, 1);
				dilate( threshold_output, threshold_output, Mat(), Point(-1, -1), dilater, 1, 1);
			}
			if(binary==1) //show binary image, don't do any more processing
			{
				cvtColor(threshold_output, img_m_color, CV_GRAY2BGR); //convert to color
				gdk_threads_enter();	//display video image in program window
				gtk_image_set_from_pixbuf(videoWindow, gdk_pixbuf_new_from_data(img_m_color.data, GDK_COLORSPACE_RGB, false, 8, img_m_color.cols, img_m_color.rows, img_m_color.step, NULL, NULL));
				gdk_threads_leave();
				continue; //don't do any more processing
			}
            cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color
		}
		else
			cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color anyways
*/

	if(flag_orinet_circ ==1) //if the detect? button is toggled for circle detection

		{
   des_val_READ[0] = getDesiredVal();     // uncomment for multi-agent application
   //  *****  des_val_READ[0] = getDesiredVal_opt();             // uncomment for optimization application
	// ***	sprintf(goal_str_MA, "%d pix = %3.1f mm", int(des_DisVar_MA), des_DisVar_MA*pix2mm_MA); //writes into strRecieve a the frames per second
    //sprintf(goal_str_MA, "Target r= %3.2f mm,  angle = %3.1f deg",  des_val_READ[0][0]*1000, des_val_READ[0][1]*180/M_PI); //writes into strRecieve a the frames per second
	 sprintf(goal_str_MA, " %3.2f mm,   %3.1f deg",  des_val_READ[0][0]*1000, des_val_READ[0][1]*180/M_PI); //writes into strRecieve a the frames per second

 // sprintf(goal_str_MA, "Desired r= %3.1f mm   N = %.0f",  des_DisVar_MA*pix2mm_MA, des_val_READ[0][5]); //writes into strRecieve a the frames per second
/// ***  sprintf(goal_str_MA, "moment angle = %3.0f degree",  des_angVar_MA); //writes into strRecieve a the frames per second
//  sprintf(goal_str_MATraj, "N = %.0f",  des_val_READ[0][5]); //writes into strRecieve a the frames per second

if (showagentcirc)    {

show_kalman = 1;    // uncomment to excluse Kalman filter detection!!!!!


		 ///   Hough circle algorithm starts here!!!!

          //  cvtColor(img_m, img_m_gray, CV_BGR2GRAY);      //convert to gray-scale recommended for hough-circle algorithm
       //     GaussianBlur(img_m, img_m_gray, Size(9, 9), 2, 2 );
         // threshold(img_m_gray, img_m_color, visionParam1, 255, THRESH_BINARY_INV );

         //   cvtColor(img_m, img_m, CV_RGB2GRAY);

blur( img_m, img_m_gray, Size(setBlursizeHough,setBlursizeHough) ); //blur image to remove small blips etc
threshold(img_m_gray, img_m_gray, Hough_Binary_adj, 255, THRESH_BINARY_INV);

/* past codes
          dilate( img_m_gray, img_m_gray, Mat(), Point(-1, -1), Hough_dilation_adj, 1, 1);

        erode( img_m_gray, img_m_gray, Mat(), Point(-1, -1), Hough_Erosion_adj, 1, 1);
         erode( img_m_gray, img_m_gray, Mat(), Point(-1, -1), Hough_Erosion_adj, 1, 1);

*/

    erode( img_m_gray, img_m_gray, Mat(), Point(-1, -1), 2*Hough_Erosion_adj, 1, 1);
    dilate( img_m_gray, img_m_gray, Mat(), Point(-1, -1), Hough_dilation_adj, 1, 1);

    HoughCircles( img_m_gray, circles_m, CV_HOUGH_GRADIENT, 1, Hough_minDis_var, Hough_canny1_var, Hough_canny2_var, Hough_MinRAdius_var, Hough_MaxRAdius_var); //find circles on image

		  //  cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color


//cirSize = circles_m.size();
//printf("number of circles =  %d  \n", cirSize);


if (showagentpreprocessVar == 1)
{
 	  		    cvtColor(img_m_gray, img_m_color, CV_GRAY2BGR); //convert to color

}

/// ORIGINAL Kalman file top-view cam
// Multiagent Kalman trial starts here

else if (show_kalman == 1)
{
    /// step-time calculation
        time_now = get_currentTime();
        time_tot = time_now - time_prev;     // equivalent to step-time dT ~ 16 mSec ! equivalent to 60 fps ::: essential for state estimation
     //   printf("Time = %f [s]", time_tot);
         dt = time_tot;                   // in seconds    // step-time calculation


/// Hough circle detection algorithm starts
cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color
/// center found by detection algorithms  starts
// sorting variables for Hough circle detection
 vector<Point>center_m_MA_index( circles_m.size() );
 vector<int>radius_m_MA_index( circles_m.size() );

cirSize = circles_m.size();
vector<Point2d> pts;


if ( cirSize>0 )
{
///////////////////////////      Find largest circle first
               for(i = 0; i < (circles_m.size()); i++)
                {
                Xmeasured = circles_m[i][0];
                Ymeasured = circles_m[i][1];
                Rmeasured = circles_m[i][2];
                pts.push_back(Point2d(Xmeasured,Ymeasured));
                }
	for(int i=0; i<pts.size(); i++)
	{
	//circle(img_m_color,pts[i],3,Scalar(255*(i == 2),255*(i == 0),255*(i == 1)),2,CV_AA);
    //circle(img_m_color,pts[i],circles_m[i][2],Scalar(255*(i == 2),255*(i == 0),255*(i == 1)),2,CV_AA);
   // circle(img_m_color,pts[i],3,Colors[i%6],2,CV_AA);
    // circle(img_m_color,pts[i],circles_m[i][2],Colors[i%6],2,CV_AA);
	}

/*
            meas.at<float>(0) = circles_m[largest_radius_MA_index][0];
            meas.at<float>(1) = circles_m[largest_radius_MA_index][1];
*/

//cirSize = pts.size();
//printf("number of circles =  %d   %f   %f\n",cirSize, pts[0].x,  pts[0].y);

//cirSize = circles_m.size();
//printf("number of circles =  %d  \n", cirSize);
//////////////////////////////////////////////////////////
/// center found by detection algorithms  ends






//	CTrackerM(dt,0.5,60.0,25,25);
	//CTracker tracker(dt,2,60,25,100);


	for(int i=0; i<pts.size(); i++)
	{
	//circle(img_m_color,pts[i],3,Scalar(0,255,0),1,CV_AA);
  //circle(img_m_color,pts[i],3,Scalar(255*(i == 2),255*(i == 0),255*(i == 1)),2,CV_AA);
  //circle(img_m_color,pts[i],circles_m[i][2],Scalar(255*(i == 2),255*(i == 0),255*(i == 1)),2,CV_AA);
	}


  if ( cirSize>1 )
  {


		tracker.Update(pts);

	//	cout << tracker.tracks.size()  << endl;

		for(int i=0;i<tracker.tracks.size();i++)
		{
			if(tracker.tracks[i]->trace.size()>1)
			{
				for(int j=0;j<tracker.tracks[i]->trace.size()-1;j++)
				{
					cv::line(img_m_color,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[i%6],2,CV_AA);    // default Colors[i%6]
					//circle(img_m_color,tracker.tracks[i]->trace[j],3,Scalar(255*(i == 2),255*(i == 0),255*(i == 1)),2,CV_AA);
				}
			}
		}


		// first and second circle assignments
		largest_center_MA = tracker.tracks[0]->trace[currentIndex-1];
		secondlargest_center_MA = tracker.tracks[1]->trace[currentIndex-1];
		//p thirdlargest_center_MA = tracker.tracks[2]->trace[currentIndex-1];


// uncomment if ( cirSize>1 )
// uncomment {
		/// the sketch of surrounding circles

/// First agent visualization

// BBBBB putText(img_m_color, "1", largest_center_MA, FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255,0,255), 2);

// BBBBB circle( img_m_color, largest_center_MA, 3, CV_RGB(255,0,255) , -1, 15, 0 );
//             circle( img_m_color, largest_center_MA, largest_radius_MA, CV_RGB(255,0,255) , 1, 8, 0 );
// ***      circle( img_m_color, largest_center_MA, des_DisVar_MA/2, CV_RGB(255,0,255) , 1, 30, 0 );
circle( img_m_color, largest_center_MA,  des_val_READ[0][0]*1000/(pix2mm_MA*2), CV_RGB(255,0,255) , 1, 30, 0 );


/// Second agent visualization

//   string box_text1 = format("%d", second_largest_radius_MA_index);
putText(img_m_color, "2", secondlargest_center_MA, FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0,255,0), 2);
// circle outline
circle( img_m_color, secondlargest_center_MA, 3, CV_RGB(0,255,0) , -1, 15, 0 );
 //           circle( img_m_color, secondlargest_center_MA, second_largest_radius_MA, CV_RGB(0,255,0) , 1, 8, 0 );
 //       circle( img_m_color, secondlargest_center_MA,  des_DisVar_MA/2, CV_RGB(0,255,0) , 1, 30, 0 );
circle( img_m_color, secondlargest_center_MA,   des_val_READ[0][0]*1000/(pix2mm_MA*2), CV_RGB(0,255,0) , 1, 30, 0 );




/*
/// Third agent visualization
putText(img_m_color, "3", thirdlargest_center_MA+Point(10, -25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200,10,50), 2);
circle( img_m_color, thirdlargest_center_MA, 3, Scalar(200,10,50) , -1, 15, 0 );
circle( img_m_color, thirdlargest_center_MA,   des_val_READ[0][0]*1000/(pix2mm_MA*2), Scalar(200,10,50) , 1, 30, 0 );
*/

// center of mass of the set

//pCOM_MA = Point( (thirdlargest_center_MA.x+secondlargest_center_MA.x+largest_center_MA.x)/3 , (thirdlargest_center_MA.y+secondlargest_center_MA.y +largest_center_MA.y)/3 );
COM_MA = Point( (secondlargest_center_MA.x+largest_center_MA.x)/2 , (secondlargest_center_MA.y +largest_center_MA.y)/2 );

 // center point demonstration
circle( img_m_color,  COM_MA, 4, CV_RGB(255,255,0) , -1, 15, 0 );


first_agent_center= largest_center_MA;
second_agent_center= secondlargest_center_MA;
//p third_agent_center= thirdlargest_center_MA;



pairdist_mag = sqrt((pow(first_agent_center.x - second_agent_center.x,2)+(pow(first_agent_center.y - second_agent_center.y,2))));
pairdist_angle = atan2( (second_agent_center.y - first_agent_center.y) , (second_agent_center.x - first_agent_center.x) );
vel_arrow(img_m_color,first_agent_center.x,first_agent_center.y,pairdist_mag,pairdist_angle,Scalar( 94, 206, 165 ),1,2);

/*
pairdist_mag13 = sqrt((pow(first_agent_center.x - third_agent_center.x,2)+(pow(first_agent_center.y - third_agent_center.y,2))));
pairdist_angle13 = atan2( (third_agent_center.y - first_agent_center.y) , (third_agent_center.x - first_agent_center.x) );
vel_arrow(img_m_color,first_agent_center.x,first_agent_center.y,pairdist_mag13,pairdist_angle13,Scalar( 165, 206, 94 ),1,2);


pairdist_mag23 = sqrt((pow(second_agent_center.x - third_agent_center.x,2)+(pow(second_agent_center.y - third_agent_center.y,2))));
pairdist_angle23 = atan2( (third_agent_center.y - second_agent_center.y) , (third_agent_center.x - second_agent_center.x) );
vel_arrow(img_m_color,second_agent_center.x,second_agent_center.y,pairdist_mag23,pairdist_angle23,Scalar( 94, 10, 165 ),1,2);

*/



Display_angle =  -des_val_READ[0][1];        // uncomment for optimization project


cv::line(img_m_color,COM_MA,
    Point(((second_agent_center.x+first_agent_center.x)/2)+(pairdist_mag/4)*cos(Display_angle),((second_agent_center.y+first_agent_center.y)/2)+(pairdist_mag/4)*sin(Display_angle)),Scalar(255,0,0),2,CV_AA);  //Draw Line

cv::line(img_m_color,COM_MA,
    Point(((second_agent_center.x+first_agent_center.x)/2)-(pairdist_mag/4)*cos(Display_angle),((second_agent_center.y+first_agent_center.y)/2)-(pairdist_mag/4)*sin(Display_angle)),Scalar(255,0,0),2,CV_AA);  //Draw Line


textpoint_MA.x=550; textpoint_MA.y=475;

putText( img_m_color, "Global", textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(255,0,0), fab_thickness_MA);

textpoint_MA.x=575; textpoint_MA.y=430;

vel_arrow(img_m_color, textpoint_MA.x,textpoint_MA.y, 20,des_val_READ[0][2],Scalar(255,0,0),1,2);       // global field direction
circle( img_m_color, textpoint_MA,  22, Scalar(255,0,0) , 1, 30, 0 );

textpoint_MA.x=460; textpoint_MA.y=475;

putText( img_m_color, "Local", textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(0,0,255), fab_thickness_MA);       // local field direction

textpoint_MA.x=480; textpoint_MA.y=430;

vel_arrow(img_m_color, textpoint_MA.x,textpoint_MA.y, 20,des_val_READ[0][3],Scalar(0,0,255),1,2);
circle( img_m_color, textpoint_MA,  22, Scalar(0,0,255) , 1, 30, 0 );

textpoint_MA.x=370; textpoint_MA.y=475;

putText( img_m_color, "Alpha", textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(255,255,0), fab_thickness_MA);       // local field direction

textpoint_MA.x=395; textpoint_MA.y=430;

circle( img_m_color, textpoint_MA,  22, Scalar(255,255,0) , 1, 30, 0 );    // alpha direction
vel_arrow(img_m_color, textpoint_MA.x,textpoint_MA.y, 20,des_val_READ[0][6],Scalar(255,255,0),1,2);

vel_arrow(img_m_color, first_agent_center.x,first_agent_center.y, 20,des_val_READ[0][2],Scalar(255,0,255),1,2);       // global field direction
vel_arrow(img_m_color, second_agent_center.x,second_agent_center.y, 20,des_val_READ[0][2],Scalar(0,255,0),1,2);       // global field direction
//p vel_arrow(img_m_color, third_agent_center.x,third_agent_center.y, 20,des_val_READ[0][2],Scalar(200,10,50),1,2);       // global field direction


textpoint_MA.x=5; textpoint_MA.y=450;
  putText( img_m_color, goal_str_MA, textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(255,0,0), fab_thickness_MA);

textpoint_MA.x=5; textpoint_MA.y=475;
putText( img_m_color, "Gradient Descent Optimization Control", textpoint_MA, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0), fab_thickness_MA);

}
}

}
 // Multiagent Kalman trial ends up here


 imcolor_enabled = 0;   // color image is already set
}



    if(imcolor_enabled == 1)
       {
            cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color
       }

		}


/////////////////    multi-agent detection ends up here     /////////////////
////////////////////////////////////////////////////////////////////////////


		if(flag_orinet_circ ==0  &&  show_kalman==0) //for threshold and bounding box detection
		{

			cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color

		}


		//draw mouse clicks:
		if(mouse.x>0)
			circle( img_m_color, mouse, 4, Scalar(  200, 50, 0), 2, 8, 0 );
		if(mouseR.x>0)
			circle( img_m_color, mouseR,4, Scalar( 20, 250, 300 ), 2, 8, 0 );
		if(mouseC.x>0)
			circle( img_m_color, mouseC, 4, Scalar( 220, 130, 100 ), 2, 8, 0 );

		//imshow("this is a test2",img_m);//display video image in separate cv window
		// compute 10 average fps

		img_m_color_for_display = img_m_color;   // Transfer the image data to the buffer for display

		//  Needed for Frame rate calculation of Top camera (xy)
		gettimeofday(&tEnd, NULL);
		current_time = ((double)tEnd.tv_sec + (double)tEnd.tv_usec*1e-6) ;
		time = (int)round( (((double)tEnd.tv_sec + (double)tEnd.tv_usec*1e-6) - ((double)tStart.tv_sec + (double)tStart.tv_usec*1e-6) )*1000.0);
		fpsVec[fpsIndex++] = 1000.0/time;
		if(fpsIndex > 9) fpsIndex = 0;
		fpsReceive = 0;
		for(int i = 0; i < 10; i++)
			fpsReceive += fpsVec[i];
		fpsReceive /= 10.0;
	} //end vision loop due to killVisionthread==1

	cam.stopGrabbingVideo();
	usleep ((int)1e5); //make sure that ImageProc_xz has closed
	cam.deinitialize();

	printf("@ the End of visionThread().\n");
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* visionThread_xz(void*)
{
	printf("In vision thread x-z.\n");

	unsigned char *inImage_xz;// = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	Mat img_m_xz, img_m_color_xz, img_m_gray_xz;

	Mat threshold_output_xz; //for threshold and rectangle detection

	int i;
	timeval tStart_xz, tEnd_xz;
	float time_xz;
	double current_time_xz;
	float fpsVec_xz[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex_xz = 0;

	// local variables - Zhe
	Mat img_m_xz_ori; // storing the original image of xz
	Mat img_m_xz_bi; // storing the binary image of xz
	Mat img_m_xz_temp;
	float magnet_maxarea, pre_area = 100.0;
	int ind_maxarea;
	bool contour_number = false; // whether the number of contours is greater than 2 ?

    double time_current_xz, time_elapsed_xz, time_init_xz, time_now_xz,   time_tot_xz,    time_prev_xz,    time_next_xz;;
	struct timeval start_xz;
	gettimeofday(&start_xz, NULL);
	time_init_xz = (double) start_xz.tv_sec + start_xz.tv_usec*1e-6 ; // Start time



// Kalman initialization for xz-vision
currentIndex_xz = 50;
CTracker tracker_xz(dt_xz,50,80,50,currentIndex_xz);     // initialization of tracker and Kalman filter  dt: sampling time 10:acceleration    100: dist_threshold    50:_maximum_allowed_skipped_frames  50:_max_trace_length
//CTracker tracker_xz(dt_xz,0,60.0,1000,1000);

	while(!killVisionThread) //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
		//g_print("Got frame %d.   ", frame++);

		gettimeofday(&tStart_xz, NULL);
		//usleep(6e4); //slow down vision thread

		inImage_xz = cam_xz.grabAframe_xz(); //unsigned char *inImage;

		if(inImage_xz == NULL)
		{
			g_print("Error in firewire stream xz! Reattempting...\n");
			usleep((int)1e3); // I don't know what the wait delay should be
		}

		img_m_xz = Mat(height, width, CV_8UC1, inImage_xz); //convert to Mat format


        cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color anyways


		img_m_color_for_display2 = img_m_color_xz;
		//gdk_threads_enter();	//display video image in program window
		//gtk_image_set_from_pixbuf(videoWindow2, gdk_pixbuf_new_from_data(img_m_color.data, GDK_COLORSPACE_RGB, false, 8, img_m_color.cols, img_m_color.rows, img_m_color.step, NULL, NULL));
		//gdk_threads_leave();

//imshow("this is a test2",img_m);//display video image in separate cv window

		// compute 10 average fps
		gettimeofday(&tEnd_xz, NULL);
		current_time_xz = ((double)tEnd_xz.tv_sec + (double)tEnd_xz.tv_usec*1e-6) ;
		time_xz = (int)round( (((double)tEnd_xz.tv_sec + (double)tEnd_xz.tv_usec*1e-6) - ((double)tStart_xz.tv_sec + (double)tStart_xz.tv_usec*1e-6) )*1000.0);
		fpsVec_xz[fpsIndex_xz++] = 1000.0/time_xz;
		if(fpsIndex_xz > 9) fpsIndex_xz = 0;
		fpsReceive_xz = 0;
		for(int i = 0; i < 10; i++)
			fpsReceive_xz += fpsVec_xz[i];
		fpsReceive_xz /= 10.0;
		//g_print("  %.1f fps yz\n", fpsReceive_xz); //we now do this in the gui using a separate thread

	} //end vision loop due to killVisionthread==1

	cam_xz.stopGrabbingVideo_xz();
	usleep ((int)1e5); //make sure that ImageProc_xz has closed
	cam.deinitialize(); //taken care of by top camera deinitialize call

	printf("@ the End of visionThread_xz().\n");
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stopVision(void)
{
	killVisionThread = 1;
}

GdkPixbuf *convertOpenCv2Gtk(IplImage *image) //Currently only RGB images with 8 bits per sample are supported. From http://subinsebastien.tumblr.com/post/2839808825/opencv-gtk-and-a-day
{
	IplImage *gtkMask;
//    gtkMask = cvLoadImage("testimage.jpg",CV_LOAD_IMAGE_UNCHANGED);

	gtkMask = image;

	cvCvtColor( image, gtkMask, CV_BGR2RGB );
    	GdkPixbuf *pix;
	gdk_threads_enter(); //may not need this here???
    	pix = gdk_pixbuf_new_from_data((guchar*)gtkMask->imageData,
              GDK_COLORSPACE_RGB,
              FALSE,
              gtkMask->depth,
              gtkMask->width,
              gtkMask->height,
              (gtkMask->widthStep),
              NULL,
              NULL);
	gdk_threads_leave();
    	return pix;
}



void set_edgemap(int d)
{
	edgemap = d;
}
void set_binary(int d)
{
	binary = d;
}
void setGain_vision(int d)
{
	cam.setGain(d);
}
void setShutter_vision(int d)
{
	cam.setShutter(d);
}
void setDilate_vision(int d)
{
	dilater = d; //for image processing on edgemap
}
void setvisionParam1_vision(int d)
{
	visionParam1 = d; //for image processing
}
void setvisionParam2_vision(int d)
{
	visionParam2 = d; //for image processing
}
void setcannyLow_vision(int d)
{
	cannyLow = d; //for image processing
}
void setcannyHigh_vision(int d)
{
	cannyHigh = d; //for image processing
}
void setdetect_vision(int d)
{
	detect1 = d; //for image processing
}

 void setvision_HoughMinRadius_vision(int d)
{
	Hough_MinRAdius_var = d;                  //for image processing to get min radius of Hough circle
}


 void setvision_HoughMaxRadius_vision(int d)
{
	Hough_MaxRAdius_var = d;                  //for image processing to get max radius of Hough circle
}

 void setvision_Houghcanny1_vision(int d)
{
	Hough_canny1_var = d;                  //for image processing to get canny1 of Hough circle
}


 void setvision_Houghcanny2_vision(int d)
{
	Hough_canny2_var = d;                  //for image processing to get canny1 of Hough circle
}


void setvision_HoughminDis_vision(int d)
{
	Hough_minDis_var = d;                  //for image processing to get min distance of Hough circle
}



 void 	setvision_HoughBinary(int d)
{
	Hough_Binary_adj = d;                  //for image processing to get binary adjustment of Hough circle
}

 void 	setvision_HoughErosion(int d)
{
	Hough_Erosion_adj = d;                  //for image processing to get erosion adjustment of Hough circle
}

void setvision_HoughDilation(int d)
{
	Hough_dilation_adj = d;                  //for image processing to get dilation adjustment of Hough circle
}


void set_agentshowcirc(int d)
{
	showagentcirc = d; //for showing magnet boundary box
}

void set_agentpreprocess(int d)
{
	showagentpreprocessVar = d; //for preprocessing box in multi-agent
}

void set_agentSingle(int d)
{
	show_agentSingle = d; //for showing magnet boundary box
}


void set_RollAve(int d)
{
	RollAveEnable = d; //for rolling average activation
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setSideCam_vision(int d)
{
	sidecam_on = d; //is the sidecam on?
}

void set_edgemap_xz(int d)
{
	edgemap_xz = d;
}
void set_binary_xz(int d)
{
	binary_xz = d;
}
void setGain_xz_vision(int d)
{
	cam.setGain_xz(d);
}
void setShutter_xz_vision(int d)
{
	cam.setShutter_xz(d);
}
void setDilate_xz_vision(int d)
{
	dilater_xz = d; //for image processing on edgemap
}
void setvisionParam1_xz_vision(int d)
{
	visionParam1_xz = d; //for image processing
}
void setvisionParam2_xz_vision(int d)
{
	visionParam2 = d; //for image processing
}
void setcannyLow_xz_vision(int d)
{
	cannyLow_xz = d; //for image processing
}
void setcannyHigh_xz_vision(int d)
{
	cannyHigh_xz = d; //for image processing
}
void setdetect_xz_vision(int d)
{
	detect_xz = d; //for image processing
}
void setsidecam_xz_vision(int d)
{
	sidecam_on = d; //is the sidecam on?
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//set magnet detection variables -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_magnetdetection(int d)
{
	magnetdetection = d; //are we performing magnet detection?
}
void set_showbox(int d)
{
	showbox = d; //for showing magnet boundary box
}
void set_showprocess(int d)
{
	showprocess = d; //for showing image processing results
}
void set_closediameter(int d)
{
    closediameter = d;
}
void set_showdestination(int d)
{
    showdestination = d;
}
void set_showfielddirection(int d)
{
    showfielddirection = d;
}

void reverse_magent()
{
    m_a = anglePlus_v(m_a, 180.0);

    for ( int i = 0; i<6; i++)
    {
        m_a_history[i] = m_a;
    }

}

void set_magnet_trust()
{
    trust_area  = magnet_area;
    flag_magnet_sampled = true;
}



void orientation_display(Point2f p1, Point2f p2, Point2f p3, Point2f p4)
{
    float m_x_sum = 0, m_y_sum = 0 , m_a_ave = 0;
    float diff1x = p1.x-p2.x, diff2x = p2.x-p3.x, diff1y = p1.y-p2.y, diff2y = p2.y-p3.y;
    float distance1 = sqrt(diff1x*diff1x + diff1y*diff1y); // distance between 1 and 2
    float distance2 = sqrt(diff2x*diff2x + diff2y*diff2y); // distance between 2 and 3

    if (distance1 > distance2)
    {
        Mlength = distance1;
        Mwidth  = distance2;
        m_a_temp = atan2(diff1x, diff1y) * 180.0/PI;
    }
    else
    {
        Mlength = distance2;
        Mwidth  = distance1;
        m_a_temp = atan2(diff2x, diff2y) * 180.0/PI;
    }

    m_x_temp = (p1.x+p2.x+p3.x+p4.x)/4.0;
    m_y_temp = (p1.y+p2.y+p3.y+p4.y)/4.0;

    // decide based on the previous angle between 2 possible directions
    float m_aOp = anglePlus_v( m_a_temp, 180.0 ); // optional
    if ( abs( angleMinus_v(m_a_temp, m_a_history[5]) ) > abs( angleMinus_v(m_aOp, m_a_history[5]) ) )
        m_a_temp = m_aOp;

    ///////////////////////
    // whether to accept this m_a_temp based on the previous several values????????????
    ///////////////////////

    // update the historical data for m_x, m_y and m_a
    for ( int i = 0; i < 5; i++ )
    {
        m_x_history[i] = m_x_history[i+1];
        m_x_sum = m_x_sum + m_x_history[i];

        m_y_history[i] = m_y_history[i+1];
        m_y_sum = m_y_sum + m_y_history[i];

        m_a_history[i] = m_a_history[i+1];
    }
    m_x_history[5] = m_x_temp;
    m_x_sum = m_x_sum + m_x_history[5];
    m_y_history[5] = m_y_temp;
    m_y_sum = m_y_sum + m_y_history[5];
    m_a_history[5] = m_a_temp;
    m_a_ave = angleMiddle_v( angleMiddle_v(m_a_history[0],m_a_history[2]), angleMiddle_v(m_a_history[3],m_a_history[5]) );

    // output m_x, m_y and m_a;
    m_x = m_x_sum/6.0;
    m_y = m_y_sum/6.0;
    m_a = m_a_ave;
    //printf(" (%.2f  %.2f)  %.2f\n", m_x_sum/6.0, m_y_sum/6.0, m_a_sum/6.0);
}



float anglePlus_v( float a, float b) // rotate angle a by angle b => angle c, wrap c in (-pi, pi]. "v" means vision.c
{
    float c = a + b;
    while ( c > 180 )   { c = c - 360.0; }
    while ( c <= -180 ) { c = c + 360.0; }
    return c;
}

float angleMinus_v( float c, float a) // rotate angle a by angle b => angle c, wrap b in (-pi, pi]. "v" means vision.c
{
    float b = c - a;
    while ( b > 180 )   { b = b - 360.0; }
    while ( b <= -180 ) { b = b + 360.0; }
    return b;
}

float angleMiddle_v( float a, float b) // the middle angle between angle a and b. "v" means vision.c
{
    float d,m;
    d = angleMinus_v(b, a);
    m = anglePlus_v(a, d/2);
    while ( m > 180 )   { m = m - 360.0; }
    while ( m <= -180 ) { m = m + 360.0; }
    return m;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
{
	switch(whichScreen)
	{
		case 0: //top screen
			switch(whichMouse)
			{
				case 1: //left mouse
					mouse.x 		= mouseClick[0];
					mouse.y 		= mouseClick[1];
					break;
				case 2: //right mouse
					mouseR.x 		= mouseClick[0];
					mouseR.y		= mouseClick[1];
					break;
				case 3: //center mouse
					mouseC.x 		= mouseClick[0];
					mouseC.y 		= mouseClick[1];
					break;
			}
			break;
		case 1: //side screen
			switch(whichMouse)
			{
				case 1: //left mouse
					mouse_xz.x 		= mouseClick[0];
					mouse_xz.y 		= mouseClick[1];
					break;
				case 2: //right mouse
					mouseR_xz.x 		= mouseClick[0];
					mouseR_xz.y 		= mouseClick[1];
					break;
				case 3: //center mouse
					mouseC_xz.x 		= mouseClick[0];
					mouseC_xz.y		= mouseClick[1];
					break;
			}
			break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Set 2nd Rect. Requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_2nd_rect (int d)
{
    if (d == 1)
        flag_second_rect = 1;
    else
        flag_second_rect = 0;

    //printf("flag_second_rect is set to be %d.\n",flag_second_rect);
    return 1;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Set 2nd Rect. Requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_1st_orient_circ (int d)
{
    if (d == 1)
        flag_orinet_circ = 1;
    else
        flag_orinet_circ = 0;

    // printf("flag_second_rect is set to be %d.\n",flag_orinet_circ);
    return 1;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Enable or Disable Centre Point Recording
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int switch_record_centerP(int argu)
{
    flag_centerP_record = argu;
}


void plot_line(bool flag, int lx[], int ly[], int lR[], int lG[], int lB[])
{
    //for a line from (x1 y1) to (x2 y2)
    //lx holds the xcoords of each line (x1 x2, x1 x2, ...)
    //ly holds the ycoords of each line (y1 y2, y1 y2, ...)
    //lR, lG, and lB hold the RGB values for each line

    plot_line_flag = flag;

    for(int count=0; count<12; count++)
    {
        line_x[count]=lx[count];
        line_y[count]=480-ly[count];

        if (count <6)
        {
            line_R[count]=lR[count];
            line_G[count]=lG[count];
            line_B[count]=lB[count];
        }

    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Blob detection color threshold adjustment  (to detect dark or bright circles)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void set_blob_color_adj(int d)
{
	blobcolorthreshold = d; //for image processing: Blob detection
}

 void set_BlobMinTh (int d)
{
	BlobMinTh_var = d;                  //for image processing to get min Blob binary threshold
}


 void set_BlobMaxTh (int d)
{
	BlobMaxTh_var = d;                 //for image processing to get max Blob binary threshold
}


 void set_BlobMinArea (int d)
{
	BlobMinArea_var = d;                 //for image processing to get min Blob Area
}


 void set_BlobMaxArea (int d)
{
	BlobMaxArea_var = d;                 //for image processing to get min Blob Area
}

void set_Blobdet (int d)                    //for showing Blobs
{
	showagentBlob = d;
}

void set_BinarydetectMA (int d)                    //for showing Binary
{
	showagentBinary = d;
}

void set_Blobcircularity (int d)                    //for showing circularity
{
	showagentcircularity = d;
}


void set_Blobconvexity (int d)                    //for showing convexity
{
	showagentconvexity = d;
}


void set_Blobinertia (int d)                    //for showing inertia
{
	showagentinertia = d;
}


void set_HoughBlur (int d)                    // To set Blur Marker Size in Hough transform
{
	setBlursizeHough = d;
}


void set_HoughMindistance (int d)                    // To set Binary Threshold in Hough transform
{
	swapthreshold = d;
}



void set_HoughSwapratio (int d)                    // To swapratio in Hough transform
{
	swapratio = d;
}


void  set_pix2mm_to (float d)                   // convert mm to pix
{
	pix2mm_MA = d;
}


void  set_des_goalMA_to (float d)                   // set desired distance
{
	des_DisVar_MA = d;
}

void  set_des_angMA_to(float d)                                                // To set desired angle in multi-agent
{
	des_angVar_MA = d;
}


void  set_des_PullangMA_to(float d)                                                // To set desired pulling angle in multi-agent
{
	des_angPull_MA = d;
}





int set_bead_detect_region( int * arr )
{
    flag_ROI = 1;
	int index = 0;
	for ( index = 0; index < 4; index ++ ) {
		valve_detect_region[index] = arr[index];
	}
	return 1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define Detection Region
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void* Define_Detect_Region_Thread (void*threadid)
{
	printf("@ the Beginning of Define_Detect_Region_Thread.\n");
	int click_count = 0;					// how many points have been specified
	int x_coor[4] = {0,0,0,0};
	int y_coor[4] = {0,0,0,0};
	int * mouse;
	int * mouse_init;
	int mouse_init_x, mouse_init_y;
	int flag = 0;							// new click found flag
	/// Store Init Mouse Value. Has to store values from the array (mouse_init). Beacuse the array's value changes with the value in vision.c.
	mouse_init = getGoalPointCoor_xz();				// initial mouse value
	mouse_init_x = mouse_init[0];
	mouse_init_y = mouse_init[1];
	/// While Loop
	while ( click_count < 4 )
	{
		// check mouse click and count how many clicked
		mouse = getGoalPointCoor_xz();
		//printf("mouse is set to be [%d, %d]. init %d %d\n", mouse[0], mouse[1], mouse_init_x, mouse_init_y);
		if ( click_count == 0 ) {									// special case for first click
			if ( ( mouse[0] != mouse_init_x ) || ( mouse[1] != mouse_init_y ) )
				flag = 1;
		} else {													// not the first click
			if ( ( mouse[0] != x_coor[click_count-1] ) || ( mouse[1] != y_coor[click_count-1] ) )
				flag = 1;
		}

		/// Record Detected Point
		if ( flag == 1 ) {
			x_coor[click_count] = mouse[0];
			y_coor[click_count] = mouse[1];
			flag = 0;
			click_count ++;
		}
		//printf("Current Detection Region [%d, %d, %d, %d], count %d.\n", detect_region[0], detect_region[1],detect_region[2],detect_region[3], click_count);
		//usleep(1e6);
	}
	/// Extract Detection Region from Clicked Points
	detect_region[0] = (int) min_array ( x_coor, 4 );
	detect_region[3] = 480 - (int) min_array ( y_coor, 4 );
	detect_region[2] = (int) max_array ( x_coor, 4 );
	detect_region[1] = 480 - (int) max_array ( y_coor, 4 );
	///
	    flag_clear = 1;
	set_bead_detect_region(detect_region);
	printf("Final Detection Region [%d, %d, %d, %d].\n", detect_region[0], detect_region[1],detect_region[2],detect_region[3]);  // gives you the top left corner and bottom right corner of the rectangle
}

int valve_define_detect_region( void )
{
	pthread_t define_detect_region_thread;
	pthread_create(&define_detect_region_thread, NULL, Define_Detect_Region_Thread, NULL);
	return 1;
}
