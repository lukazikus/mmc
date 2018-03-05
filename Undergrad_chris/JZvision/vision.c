#include "vision.h"

static bool fDisplayValue = false;                              // protection of display values
static int fShowCargo = 1;                                      // whether or not show the detected cargo
static Image_JZ presFrame;                                      // present obtained frame
static Point_JZ cargoOne(0,0), cargoTwo(0,0);                   // cargo detected from frames
static Point_JZ cargoOneMemo(0,0), cargoTwoMemo(0,0);           // cargo memories

static Point agentOneMemo(0.0), agentTwoMemo(0,0);                              // memory of 2 agents, used to avoid switching
FWcamera cam; //see FWcamera.cpp

int width = 640, height = 480;   //image width & height, pixels
int depth = 1;                   //depth of image

int killVisionThread = 1; //this stops our vision thread

GtkLabel *labelFPSreceive, *labelFPSreceive_xz;

static Point mouse, mouseC, mouseR;
static Point mouse_xz, mouseC_xz, mouseR_xz;

static int detect = 1; //are we detecting object?
int cannyLow=100, cannyHigh=150; //thresholds for image processing filter
static int dilater = 1;
static int edgemap = 0, binary = 0; //are we performing edgemap calculations?
int visionParam1 = 65; //for processing. Used in threshold() and houghCircle().
int visionParam2 = 35; //for processing
int Hough_MinRAdius_var = 3;                //for processing. Used in houghCircle() indicating min radius of circles.
int Hough_MaxRAdius_var = 170;              //for processing. Used in houghCircle() indicating max radius of circles.
int Hough_canny1_var = 100;                 //for processing. Used in houghCircle() indicating canny1 value.
int Hough_canny2_var = 10;                  //for processing. Used in houghCircle() indicating canny2 value.
int Hough_minDis_var = 20;                  // minimum admissible distance between center points in hough circle transform
int binaryThreshold = 90;                   // indicating the level of binary conversion.
int Hough_Erosion_adj = 0;                  //for processing. Used in houghCircle() indicating the level of erosion conversion.
int Hough_dilation_adj = 6;                 //for processing. Used in houghCircle() indicating the level of dilation conversion.
int showagentcirc = 1;                      // Hough circle show multiagent
int showagentpreprocessVar = 0;    // preprocessing in multiagent project
int imcolor_enabled = 1;            // enable conversion to imcolor to draw field
int setBlursizeHough = 10;          // set Blur size in Hough transform                put 7 if you are using Median filter otherwise 16 to 25 is good
int swapthreshold = 15;             // set Binary Threshold in Hough transform
int swapratio = 4;                  // set swap ratio in Hough transform
static float fpsReceive; //frames per second of video
static int flag_centerP_record = 0; // record center point or not. 1: yest, 0: no

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int edgemap_xz = 0, binary_xz = 0; //are we performing edgemap calculations?
int visionParam1_xz = 65; //for processing
int visionParam2_xz = 35; //for processing
static int topcam_on = 1; //is the sidecam capturing? Default: YES
static int sidecam_on = 0; //is the sidecam capturing? Default: YES
static float fpsReceive_xz; //frames per second of video

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
bool showdestination = 1, showfielddirection = 1;

Mat img_m_color_for_display, img_m_color_for_display2;
static float pix2mm_MA = 0.015;

//static float* des_val_READ[1];
static float agentControlDisplay[13] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

static float  Display_angle;
static float  Display_angle_xz;
static float des_DisVar_MA = 105;
static float des_angVar_MA;
static float des_angPull_MA = 45;
static char goal_str_MA[15];
static char goal_str_MATraj[15];
static float pairdist_mag = 100, pairdist_angle=0;
static float pairdist_mag13 = 100, pairdist_angle13=0, pairdist_mag23 = 100, pairdist_angle23=0;   // three agents separations
static float inputangle_MA=0;
//////////////////////////////////// DC and AC arrow print  -- Piyush ////////////////////////////////////////////////////////
int dc_arrow_active = 0;
int ac_arrow_active = 0;
int s1_magnet_dir_active = 0;
int s2_magnet_dir_active = 0;
int s1_heading_dir_active = 0;
int s2_heading_dir_active = 0;
static int frame_index_arrow = 0;
static int frame_index_arrow_MA = 0;

Point centerP_adjusted;
Point centerP_adjusted_MA, centerP_adjusted_MA_xz;    // multi-agent adjusted y direction center point

Point2f centerP_rect_short_side[2];   // 2 center points of the rect.'s short sides.

static Point centerP_adjusted_2;

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
static Point agentOne(0,0);              // first agent center recoreded variable
static Point agentTwo(0,0);              // second agent center recoreded variable
static Point third_agent_center;              // second agent center recoreded variable

static int centerP_adjust_flag = 0;
static int centerPointCoorArray[2]  = {0,0};
static int centerPoint2CoorArray[2] = {0,0};
static int centerPointCoorArray_MA[9];                       // multiagent center point of agent 1, 2 AND their orientation circles
static float DistanceCoorArray_MA[6]  ;                      // multiagent center point of agent 1, 2 AND their orientation circles
static float VAR_CTRL_MA[3] ;                                // returns desired distance and pixel2mm conversionz
static int goalPointCoorArray[2]   = {320, 240};
static int goalPointCoorCArray[2]   = {320, 240};
static int goalPointCoorRArray[2]   = {320, 240};
static int goalPointCoorArray_xz[2]   = {320, 240};
static int goalPointCoorCArray_xz[2]   = {320, 240};
static int goalPointCoorRArray_xz[2]   = {320, 240};

static float centerP_rect_short_side_coor_array[4];   // point-1.x, point-1,y, point-2.x, point-2.y
static int flag_second_rect = 0;
static int flag_orinet_circ = 1;                          // orientation_circ detection flag, multi-agent (1:active mode toggle button)

//Variable for plot_line
bool fPlotLine = false;
Point_JZ line1Pt1(0,0), line1Pt2(0,0), line2Pt1(0,0), line2Pt2(0,0);        // 2 lines connecting grippers to cargoes/destinations

Point textpoint_MA;

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
static int N =4;
static int M =4;
static float Accel_noise_mag;
static double dist_thres, dt = 0.016, dt_xz = 0.016;
static int maximum_allowed_skipped_frames, max_trace_length;
static float X=0,Y=0;
static float Xmeasured,Ymeasured,Rmeasured;
static float Xmeasured_xz,Ymeasured_xz,Rmeasured_xz;
RNG rng;
Scalar Colors[]={Scalar(255,0,255),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,255,255)};   // detection colors
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
static double get_currentTime(void) {
    gettimeofday(&start, NULL);
    double l_time = (double) start.tv_sec + start.tv_usec*1e-6 ;   // seconds
    return l_time;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat getImage(void) {
	return img_m_color_for_display;
}

Mat getImage2(void) {
	return img_m_color_for_display2;
}

int * getCenterPointCoor(void) {
    centerPointCoorArray[0] = centerP_adjusted.x;
    centerPointCoorArray[1] = centerP_adjusted.y;
    return centerPointCoorArray;
}

int * get2ndCenterPointCoor(void) {
    centerPoint2CoorArray[0] = centerP_adjusted_2.x;
    centerPoint2CoorArray[1] = centerP_adjusted_2.y;
    return centerPoint2CoorArray;
}

float * get_CenterP_rect_short_side_coor_array(void) {
    centerP_rect_short_side_coor_array[0] = centerP_rect_short_side[0].x;
    centerP_rect_short_side_coor_array[1] = 480 - centerP_rect_short_side[0].y;  // !!! The y direction is inversed.

    centerP_rect_short_side_coor_array[2] = centerP_rect_short_side[1].x;
    centerP_rect_short_side_coor_array[3] = 480 - centerP_rect_short_side[1].y;
    return centerP_rect_short_side_coor_array;
}

int * getGoalPointCoor(void) {
    if(mouse.x>0) {
        goalPointCoorArray[0] = mouse.x;
        goalPointCoorArray[1] = 480 - mouse.y;   // Note the positive y dir.
    }
    return goalPointCoorArray;
}

int vision_return_cargo_pos (Point_JZ &pt1, Point_JZ &pt2) {
    pt1.x = cargoOne.x;
    pt1.y = 480 - cargoOne.y;
    pt2.x = cargoTwo.x;
    pt2.y = 480 - cargoTwo.y;
    return 1;
}

// Jiachen: return multi-agent pos in frame
int return_multi_agent_pos (Point_JZ &pt1, Point_JZ &pt2) {
    pt1.x = agentOne.x;
    pt1.y = 480 - agentOne.y;
    pt2.x = agentTwo.x;
    pt2.y = 480 - agentTwo.y;
    return 1;
}

int return_single_agent_pos(Point_JZ &pt) {
    pt.x = agentOne.x;
    pt.y = 480 - agentOne.y;
    return 1;
}

// multi-agent center point read out for control purpose
int * getCenterPointCoor_MA(void) {
  centerPointCoorArray_MA[0] = agentOne.x;
  centerPointCoorArray_MA[1] = 480 - agentOne.y;       // Note the positive y dir.
  centerPointCoorArray_MA[2] = 0;                                // z axis value
  centerPointCoorArray_MA[3] = agentTwo.x;
  centerPointCoorArray_MA[4] = 480 - agentTwo.y;      // Note the positive y dir.
  centerPointCoorArray_MA[5] = 0;                                // z axis value
  centerPointCoorArray_MA[6] = third_agent_center.x;
  centerPointCoorArray_MA[7] = 480 - third_agent_center.y;       // Note the positive y dir.
  centerPointCoorArray_MA[8] = 0;                                // z axis value

  return centerPointCoorArray_MA;
}

float * getDistanceCoor (void) {
    DistanceCoorArray_MA[0] = pairdist_mag;        // equivalent to r12
    DistanceCoorArray_MA[1] = atan2( (-agentTwo.y + agentOne.y), (agentTwo.x - agentOne.x) );  // note the increasing dir. of y coor.
    DistanceCoorArray_MA[2] = pairdist_mag13;
    DistanceCoorArray_MA[3] = pairdist_angle13;
    DistanceCoorArray_MA[4] = pairdist_mag23;
    DistanceCoorArray_MA[5] = pairdist_angle23;

    return DistanceCoorArray_MA;
}

float * getCTRL_MA (void) {
    VAR_CTRL_MA[0] = des_DisVar_MA;                     // desired separation between 2 agents
    VAR_CTRL_MA[1] = pix2mm_MA;
    VAR_CTRL_MA[2] = des_angVar_MA*M_PI/180;
    VAR_CTRL_MA[3] = des_angPull_MA*M_PI/180;
    return VAR_CTRL_MA;
}

///////////////////////////////////////////////////////////
//mouseC and mouseR are reversed                         //
//////////////////////////////////////////////////////////
int * getGoalPointCoorC(void) {
    if(mouseR.x>0) {
        goalPointCoorCArray[0] = mouseR.x;
        goalPointCoorCArray[1] = 480 - mouseR.y;   // Note the positive y dir.
    }
    return goalPointCoorCArray;
}

int * getGoalPointCoorR(void) {
    if(mouseC.x>0) {
        goalPointCoorRArray[0] = mouseC.x;
        goalPointCoorRArray[1] = 480 - mouseC.y;   // Note the positive y dir.
    }
    return goalPointCoorRArray;
}

int * getGoalPointCoor_xz(void) {
    if(mouse_xz.x>0) {
        goalPointCoorArray_xz[0] = mouse_xz.x;
        goalPointCoorArray_xz[1] = 480 - mouse_xz.y;   // Note the positive y dir.
    }
    return goalPointCoorArray_xz;
}

///////////////////////////////////////////////////////////
//mouseC and mouseR are reversed                         //
//////////////////////////////////////////////////////////

int * getGoalPointCoorC_xz(void) {
    if(mouseR_xz.x>0) {
        goalPointCoorCArray_xz[0] = mouseR_xz.x;
        goalPointCoorCArray_xz[1] = 480 - mouseR_xz.y;   // Note the positive y dir.
    }
    return goalPointCoorCArray_xz;
}

int * getGoalPointCoorR_xz(void) {
    if(mouseC_xz.x>0) {
        goalPointCoorRArray[0] = mouseC.x;
        goalPointCoorRArray[1] = 480 - mouseC.y;   // Note the positive y dir.
    }
    return goalPointCoorRArray_xz;
}

// Frame Print Thread
// This thread is not used? UI still displays images after this is commented out.
void* FPSprint(void*) {
	char strReceive[50];
	char strReceive_xz[50];

	while(!killVisionThread)   //while the image processing is running //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
		//  int sprintf(char *str, const char *format, ...) sends formatted output to a string pointed to by str.
		sprintf(strReceive, "%.1f", fpsReceive); //writes into strRecieve the frames per second
		sprintf(strReceive_xz, "%.1f", fpsReceive_xz);
		gdk_threads_enter();
		// gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive); //draw on the gui
		// gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
		gdk_threads_leave();
		usleep((int)1e6); //sets frame rate display frequency
	}
	//printf("In the test zone!\n");
	sprintf(strReceive, "N/A"); //when we turn off the vision, write N/A into the gui
	sprintf(strReceive_xz, "N/A");
	gdk_threads_enter();
	// gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive);
	// gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
	gdk_threads_leave();
}

void vel_arrow (Mat Image,int c1,int c2,float vel,float Theta, CvScalar Color,int Size,int Thickness) {
    cv::Point pt1,pt2;

    pt1.x=c1;
    pt1.y=c2;

    float u = vel * cos(Theta) * Size;
    float v = vel * Size * sin(Theta);

    pt2.x=c1+u;
    pt2.y=c2+v;

    cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line

    Size=(int)(5);

    if(Theta==M_PI/2 && pt1.y > pt2.y) {
        pt1.x=(int)(Size*cos(Theta)-Size*sin(Theta)+pt2.x);
        pt1.y=(int)(Size*sin(Theta)+Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line

        pt1.x=(int)(Size*cos(Theta)+Size*sin(Theta)+pt2.x);
        pt1.y=(int)(Size*sin(Theta)-Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line
    } else{
        pt1.x=(int)(-Size*cos(Theta)-Size*sin(Theta)+pt2.x);
        pt1.y=(int)(-Size*sin(Theta)+Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line

        pt1.x=(int)(-Size*cos(Theta)+Size*sin(Theta)+pt2.x);
        pt1.y=(int)(-Size*sin(Theta)-Size*cos(Theta)+pt2.y);
        cv::line(Image,pt1,pt2,Color,Thickness,8);  //Draw Line
    }
}

// main vision thread
static void * visionThread(void *) {
    printf("@ the Beginning of visionThread().\n");

    Mat imgTemp;                        // temporal img variable
    int iLargestContour = 0;
    int iSecondContour = 0;
	unsigned char *inImage;            // = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	//Mat img_m, img_m_color, img_m_gray;
    Mat img_m_color;
	Mat threshold_output;              // for threshold and rectangle detection
	timeval tStart, tEnd;
	float time;
	double current_time;
	float fpsVec[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex = 0;

    while (!killVisionThread) {
		gettimeofday(&tStart, NULL);

        // this function watis for a new frame, it takes a long time, so we can do some image processing in this thread
		inImage = cam.grabAframe(); //unsigned char *inImage;
		if(inImage == NULL) {
            g_print("Error in firewire stream! Reattempting...\n");
			usleep((int)1e3); // I don't know what the wait delay should be
		}
        img_m_color = Mat (height, width, CV_8UC1, inImage);
        presFrame.update (img_m_color);               // put newly obtained frame into the obj.
		//img_m = Mat(height, width, CV_8UC1, inImage); //convert to Mat format

        if (flag_orinet_circ == 1) {                   //if the detect? button is toggled for circle detection
            //des_val_READ[0] = getDesiredVal();          // uncomment for multi-agent application
            //sprintf(goal_str_MA, " %3.2f mm,   %3.1f deg",  des_val_READ[0][0]*1000, des_val_READ[0][1]*180/M_PI); //writes into strRecieve a the frames per second

            if (showagentcirc == 1) {
                //Mat   binaryImage;
                //Mat contourImage;
                presFrame.blurImg (setBlursizeHough);
                //blur (img_m, img_m_gray, Size(setBlursizeHough,setBlursizeHough) ); //blur image to remove small blips etc
                presFrame.binarizeImg (binaryThreshold);                        // apply binary threshold to make image black and white
                //threshold(img_m_gray, binaryImage, binaryThreshold, 255, THRESH_BINARY_INV);
                //erode( binaryImage, binaryImage, Mat(), Point(-1, -1), 2*Hough_Erosion_adj, 1, 1);
                presFrame.dilateImg (Hough_dilation_adj);
                //dilate (binaryImage, binaryImage, Mat(), Point(-1, -1), Hough_dilation_adj, 1, 1);
                //presFrame.img.copyTo (contourImage);
                //binaryImage.copyTo (contourImage);

                //vector<vector<Point> > 	tv_contours;
                //vector<Vec4i>          	hierarchy;
                presFrame.getContours ();
                //findContours (contourImage, tv_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); //find contours, will destroy input image
                //int nContour = tv_contours.size();
                /*
                int nContour = presFrame.contours.size();
            	if (nContour > 1) {                                            // if more than 1 contour is detected
                    double area[nContour];
                    for (int i = 0; i < nContour; i++) {
                        //area[i] = contourArea( tv_contours[i], false );
                        area[i] = contourArea (presFrame.contours[i], false);
                    }
                    iLargestContour = 0;                                        // init. largest contour index is 0
                    for (int j = 1; j < nContour; j ++) {
                        if (area[iLargestContour] < area[j]) {
                            iLargestContour = j;
                        }
                    }
                    if (iLargestContour != 0)
                        iSecondContour = 0;
                    else
                        iSecondContour = 1;
                    for (int j = 1; j < nContour; j ++) {
                        if (j != iLargestContour && area[iSecondContour] < area[j]) {
                            iSecondContour = j;
                        }
                    }
                    */
                int contourRes = presFrame.sortContour ();
                //printf("contourRes is %d.\n", contourRes);
                if (contourRes > 0) {                                          // if at least one agent is detected
                    Rect bounding;
            		//bounding = boundingRect( tv_contours[iLargestContour] );
                    bounding = boundingRect (presFrame.contours[presFrame.agentIndex[0]]);
                    agentOne.x = bounding.x + bounding.width  * 0.5;
                    agentOne.y = bounding.y + bounding.height * 0.5;
                    //printf("first agent is (%d, %d)\n", agentOne.x,agentOne.y);
                    //bounding = boundingRect( tv_contours[iSecondContour] );
                    if (contourRes > 1) {                                       // if >1 agents are detected
                        bounding = boundingRect (presFrame.contours[presFrame.agentIndex[1]]);
                        agentTwo.x = bounding.x + bounding.width  * 0.5;
                        agentTwo.y = bounding.y + bounding.height * 0.5;
                        float temp1, temp2;
                        temp1 = pow(agentOne.x - agentOneMemo.x, 2) + pow(agentOne.y - agentOneMemo.y, 2);
                        temp2 = pow(agentTwo.x - agentOneMemo.x, 2) + pow(agentTwo.y - agentOneMemo.y, 2);
                        if (temp1 > temp2) {
                            int temp3 = 0, temp4 = 0;
                            temp3 = agentOne.x;
                            temp4 = agentOne.y;
                            agentOne.x = agentTwo.x;
                            agentOne.y = agentTwo.y;
                            agentTwo.x = temp3;
                            agentTwo.y = temp4;
                        }
                        agentOneMemo.x = agentOne.x;
                        agentOneMemo.y = agentOne.y;
                        agentTwoMemo.x = agentTwo.x;
                        agentTwoMemo.y = agentTwo.y;
                        if (contourRes == 4) {
                            bounding = boundingRect (presFrame.contours[presFrame.agentIndex[2]]);
                            cargoOne.x = bounding.x + bounding.width  * 0.5;
                            cargoOne.y = bounding.y + bounding.height * 0.5;
                            bounding = boundingRect (presFrame.contours[presFrame.agentIndex[3]]);
                            cargoTwo.x = bounding.x + bounding.width  * 0.5;
                            cargoTwo.y = bounding.y + bounding.height * 0.5;
                            if (cargoOne.dis_to_another_pt(cargoOneMemo) > cargoTwo.dis_to_another_pt(cargoOneMemo)) {
                                Point_JZ tempPt (cargoOne);
                                cargoOne.update(cargoTwo);
                                cargoTwo.update(tempPt);
                            }
                            cargoOneMemo.update(cargoOne);              // update cargo one memory
                            cargoTwoMemo.update(cargoTwo);              // update cargo two memory
                            //printf("vision.c cargos are (%d, %d), (%d, %d)\n", cargoOne.x,cargoOne.y,cargoTwo.x,cargoTwo.y);
                        }
                    }
                } else {

                }
                if (showagentpreprocessVar == 1) {
                    cvtColor(presFrame.processedImg, img_m_color, CV_GRAY2BGR); //convert to color
                } else
                    cvtColor (presFrame.img, img_m_color, CV_GRAY2BGR);
                    //cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color

                putText(img_m_color, "1", agentOne, FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255,0,255), 2);
                //circle (img_m_color, agentOne, des_val_READ[0][0] * 0.5, CV_RGB(255,0,255) , 1, 30, 0 );

                while (fDisplayValue);
                circle (img_m_color, agentOne, agentControlDisplay[0] * 0.5, CV_RGB(255,0,255) , 1, 30, 0 );

                //putText(img_m_color, "2", agentTwo, FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0,255,0), 2);
                //circle (img_m_color, agentTwo, des_val_READ[0][0] * 0.5, CV_RGB(0,255,0) , 1, 30, 0 );
                //while (fDisplayValue);
                //circle (img_m_color, agentTwo, agentControlDisplay[0] * 0.5, CV_RGB(0,255,0) , 1, 30, 0 );
                if (fShowCargo == 1) {
                    circle (img_m_color, Point (cargoOne.x, cargoOne.y), 40, CV_RGB(255,0,0) , 1, 30, 0 );
                    //circle (img_m_color, Point (cargoTwo.x, cargoTwo.y), 20, CV_RGB(0,0,255) , 1, 30, 0 );
                }
                /*
                if (fPlotLine == 1) {                                           // draw line connecting grippers to cargoes/destinations
                    line(img_m_color, Point(line1Pt1.x,line1Pt1.y), Point(line1Pt2.x,line1Pt2.y), Scalar(200,  50,  0), 1, 8);
                    line(img_m_color, Point(line2Pt1.x,line2Pt1.y), Point(line2Pt2.x,line2Pt2.y), Scalar(250, 150, 50), 1, 8);
                }*/

                COM_MA = Point((agentTwo.x+agentOne.x)/2 , (agentTwo.y +agentOne.y)/2 );
                //circle( img_m_color,  COM_MA, 4, CV_RGB(255,255,0) , -1, 15, 0 );
                pairdist_mag = sqrt(pow(agentOne.x - agentTwo.x,2) + pow(agentOne.y - agentTwo.y,2));
                pairdist_angle = atan2( (agentTwo.y - agentOne.y), (agentTwo.x - agentOne.x) );  // note the increasing dir. of y coor.
                //vel_arrow (img_m_color, agentOne.x, agentOne.y, pairdist_mag, pairdist_angle, Scalar( 94, 206, 165 ), 1, 2);

                //Display_angle =  -des_val_READ[0][1];        // uncomment for optimization project
                while (fDisplayValue);
                Display_angle =  -agentControlDisplay[1];

                //cv::line(img_m_color,COM_MA,
                //    Point(((agentTwo.x+agentOne.x)/2)+(pairdist_mag/4)*cos(Display_angle),((agentTwo.y+agentOne.y)/2)+(pairdist_mag/4)*sin(Display_angle)),Scalar(255,0,0),2,CV_AA);  //Draw Line

                //cv::line(img_m_color,COM_MA,
                //    Point(((agentTwo.x+agentOne.x)/2)-(pairdist_mag/4)*cos(Display_angle),((agentTwo.y+agentOne.y)/2)-(pairdist_mag/4)*sin(Display_angle)),Scalar(255,0,0),2,CV_AA);  //Draw Line

                //textpoint_MA.x=550; textpoint_MA.y=475;
                //putText( img_m_color, "Global", textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(255,0,0), fab_thickness_MA);
                //textpoint_MA.x=575; textpoint_MA.y=430;
                //vel_arrow(img_m_color, textpoint_MA.x,textpoint_MA.y, 20, -des_val_READ[0][2],Scalar(255,0,0),1,2);       // global field direction
                //circle( img_m_color, textpoint_MA,  22, Scalar(255,0,0) , 1, 30, 0 );
                //textpoint_MA.x=460; textpoint_MA.y=475;
                //putText(img_m_color, "Local", textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(0,0,255), fab_thickness_MA);       // local field direction
                //textpoint_MA.x=480; textpoint_MA.y=430;
                //vel_arrow(img_m_color, textpoint_MA.x,textpoint_MA.y, 20,des_val_READ[0][3],Scalar(0,0,255),1,2);
                //circle( img_m_color, textpoint_MA,  22, Scalar(0,0,255) , 1, 30, 0 );
                //textpoint_MA.x=370; textpoint_MA.y=475;
                //putText( img_m_color, "Alpha", textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(255,255,0), fab_thickness_MA);       // local field direction
                //textpoint_MA.x=395; textpoint_MA.y=430;
                //circle( img_m_color, textpoint_MA,  22, Scalar(255,255,0) , 1, 30, 0 );    // alpha direction
                //vel_arrow(img_m_color, textpoint_MA.x,textpoint_MA.y, 20,des_val_READ[0][6],Scalar(255,255,0),1,2);
                //vel_arrow(img_m_color, agentOne.x,agentOne.y, 20,-des_val_READ[0][2],Scalar(255,0,255),1,2);       // global field direction
                //vel_arrow(img_m_color, agentTwo.x,agentTwo.y, 20,-des_val_READ[0][2],Scalar(0,255,0),1,2);       // global field direction
                //while (fDisplayValue);
                //vel_arrow(img_m_color, agentOne.x,agentOne.y, 20,-agentControlDisplay[2],Scalar(255,0,255),1,2);       // global field direction
                //vel_arrow(img_m_color, agentTwo.x,agentTwo.y, 20,-agentControlDisplay[2],Scalar(0,255,0),1,2);       // global field direction
                //p vel_arrow(img_m_color, third_agent_center.x,third_agent_center.y, 20,des_val_READ[0][2],Scalar(200,10,50),1,2);       // global field direction
                //textpoint_MA.x=5; textpoint_MA.y=450;
                //putText( img_m_color, goal_str_MA, textpoint_MA, FONT_HERSHEY_SIMPLEX, fab_fontscale_MA, Scalar(255,0,0), fab_thickness_MA);
                //textpoint_MA.x=5; textpoint_MA.y=475;
                //putText( img_m_color, "P-Control", textpoint_MA, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0), fab_thickness_MA);
                imcolor_enabled = 0;   // color image is already set
            }
            if(imcolor_enabled == 1) {
                //cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color
                cvtColor (presFrame.img, img_m_color, CV_GRAY2BGR);
            }
        }
		if(flag_orinet_circ ==0) //for threshold and bounding box detection
            cvtColor (presFrame.img, img_m_color, CV_GRAY2BGR);
			//cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color
		//draw mouse clicks:
		if(mouse.x>0)
			circle (img_m_color, mouse, 4, Scalar(  200, 50, 0), 2, 8, 0 );
		if(mouseR.x>0)
			circle (img_m_color, mouseR,4, Scalar( 20, 250, 300 ), 2, 8, 0 );
		if(mouseC.x>0)
			circle (img_m_color, mouseC, 4, Scalar( 220, 130, 100 ), 2, 8, 0 );

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

void stopVision(void) {
	killVisionThread = 1;
}

/*  commented by Mohammad to avoid crash
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
*/

void initVision(void) {
	pthread_t vthread, fthread, vthread_xz, fthread_xz;
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

	if(killVisionThread == 0)
		g_print("Vision already running in initVision!!!\n");
	usleep(1e5);

	if(killVisionThread == 1)
  	{
  		killVisionThread = 0;
	//	pthread_create(&fthread, NULL, FPSprint    , NULL);  //start frame print thread. Functionality??? comment this if you do not want to show frame per second   :: commnted by Mohammad
		pthread_create(&vthread, NULL, visionThread, NULL);  //start vision thread
	}
  if(topcam_on == 1) {//if we are also using the top cam
      usleep(2e5);
      printf("Before cam_xy.initialize_xy().\n");
      if(!cam.initialize()) {//cam is instance of FWCamera, found in FWcamera.cpp
          g_print("FW camera xy could not be found in initVision!!!\n");
          return;
      }
      usleep(1e5);
      if(!cam.startGrabbingVideo())	{
          g_print("FW cam xy could not grab in initVision!!!\n");
          return;
      }
      usleep(1e5);
      pthread_create(&vthread, NULL, visionThread, NULL);  //start vision thread
  }

}

void set_edgemap(int d) {
	edgemap = d;
}

void set_binary(int d) {
	binary = d;
}

void setGain_vision(int d) {
	cam.setGain(d);
}

void setShutter_vision(int d) {
	cam.setShutter(d);
}

void setDilate_vision(int d) {
	dilater = d; //for image processing on edgemap
}

void setvisionParam1_vision(int d) {
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

 void setvision_HoughMinRadius_vision(int d) {
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


void setvision_HoughminDis_vision(int d) {
	Hough_minDis_var = d;                  //for image processing to get min distance of Hough circle
}

void setvision_HoughBinary(int d) {
	binaryThreshold = d;
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

void set_agentpreprocess(int d) {
	showagentpreprocessVar = d; //for preprocessing box in multi-agent
}

// X-Z Camera
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

void setvisionParam1_xz_vision(int d)
{
	visionParam1_xz = d; //for image processing
}
void setvisionParam2_xz_vision(int d)
{
	visionParam2 = d; //for image processing
}

void setsidecam_xz_vision(int d)
{
	sidecam_on = d; //is the sidecam on?
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//set magnet detection variables -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_magnetdetection(int d) {
	magnetdetection = d; //are we performing magnet detection?
}
void set_showbox(int d) {
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

void reverse_magent() {
    m_a = anglePlus_v(m_a, 180.0);

    for ( int i = 0; i<6; i++) {
        m_a_history[i] = m_a;
    }
}

void set_magnet_trust() {
    trust_area  = magnet_area;
    flag_magnet_sampled = true;
}

void orientation_display(Point2f p1, Point2f p2, Point2f p3, Point2f p4) {
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

// set next center-of-mass of agent pair
int vision_set_next_com (Point_JZ &input) {
    mouse.x = input.x;
    mouse.y = 480 - input.y;
    return 1;
}

void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) { //click in pixels
	switch (whichScreen) {
		case 0: //top screen
			switch (whichMouse) {
				case 1: //left mouse
					mouse.x = mouseClick[0];
					mouse.y = mouseClick[1];
					break;
				case 2: //right mouse
					mouseR.x = mouseClick[0];
					mouseR.y = mouseClick[1];
					break;
				case 3: //center mouse
					mouseC.x = mouseClick[0];
					mouseC.y = mouseClick[1];
					break;
			}
			break;
		case 1: //side screen
			switch (whichMouse) {
				case 1: //left mouse
					mouse_xz.x = mouseClick[0];
					mouse_xz.y = mouseClick[1];
					break;
				case 2: //right mouse
					mouseR_xz.x = mouseClick[0];
					mouseR_xz.y = mouseClick[1];
					break;
				case 3: //center mouse
					mouseC_xz.x = mouseClick[0];
					mouseC_xz.y = mouseClick[1];
					break;
			}
			break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Set 2nd Rect. Requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int set_2nd_rect (int d) {
    if (d == 1)
        flag_second_rect = 1;
    else
        flag_second_rect = 0;
    //printf("flag_second_rect is set to be %d.\n",flag_second_rect);
    return 1;
}

int set_1st_orient_circ (int d) {
    if (d == 1)
        flag_orinet_circ = 1;
    else
        flag_orinet_circ = 0;
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Enable or Disable Centre Point Recording
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int switch_record_centerP(int argu) {
    flag_centerP_record = argu;
}

int vision_plot_line (bool flag, Point_JZ &input0, Point_JZ &input1, Point_JZ &input2, Point_JZ &input3) {
    fPlotLine = flag;
    line1Pt1.x = input0.x;
    line1Pt1.y = 480 - input0.y;
    line1Pt2.x = input1.x;
    line1Pt2.y = 480 - input1.y;
    line2Pt1.x = input2.x;
    line2Pt1.y = 480 - input2.y;
    line2Pt2.x = input3.x;
    line2Pt2.y = 480 - input3.y;

    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Blob detection color threshold adjustment  (to detect dark or bright circles)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_HoughBlur (int d) {                    // To set Blur Marker Size in Hough transform
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

// convert mm to pix
void  set_pix2mm_to (float d) {
	pix2mm_MA = d;
}

// set desired distance
void  set_des_goalMA_to (float d) {
	des_DisVar_MA = d;
}

void  set_des_PullangMA_to(float d)                                                // To set desired pulling angle in multi-agent
{
	des_angPull_MA = d;
}

// draw specified cargo positions on top-view
int vision_draw_cargo (int xCoor[2], int yCoor[2]) {
    mouseR.x = xCoor[0];
    mouseR.y = 480 - yCoor[0];
    mouseC.x = xCoor[1];
    mouseC.y = 480 - yCoor[1];
    return 1;
}

// set the flag for whether or not show the detected cargoes
int vision_set_show_cargo_flag (int input) {
    fShowCargo = input;
    return 1;
}

// get values to display from multiagent.c
int set_agentControlDisplay ( float * input) {
    fDisplayValue = true;
    //int n = sizeof(input) / sizeof(float);      // # of elements
    //printf("size of input %d, size of float %d\n", (int)sizeof(input), (int)sizeof(float));
    //if (n == 13) {
        for (int i = 0; i < 13; i ++)
            agentControlDisplay[i] = input[i];
        fDisplayValue = false;
        return 1;
    //} else {
        //printf("ERROR: vision.c -> set_agentControlDisplay input element mismatch. input is %d\n", n);
        //fDisplayValue = false;
        //usleep(1e6);
        //return 0;
    //}
}
