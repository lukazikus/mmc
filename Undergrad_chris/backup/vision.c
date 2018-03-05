// (08-15)
#include "vision.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int flag_newFrame = 0;													// (07-15) indicate whether or not a new frame has arrived

static int flag_DC_AC_angle = 0;												// (07-19) indicate whether or not plot DC & AC angles

/*
static int DC_line_x[2] = {0,0};												// 2 points of line representing DC & AC angles
static int DC_line_y[2] = {0,0};
static int AC_line_x[2] = {0,0};
static int AC_line_y[2] = {0,0};
*/

static Point DC_line_P_1, DC_line_P_2, AC_line_P_1, AC_line_P_2;				// 4 points of line representing DC & AC angles

FWcamera cam, cam_xz; //see FWcamera.cpp

int width = 640;   //image width, pixels
int height = 480;  //image height, pixels
int depth = 1;     //depth of image
//unsigned char * image = NULL;
int killVisionThread = 1; //this stops our vision thread

GtkWidget *labelFPSreceive, *labelFPSreceive_xz;

static Point mouse, mouseC, mouseR;
static Point mouse_xz, mouseC_xz, mouseR_xz;

//ImageProcessor RobotTracker(width, height, 1);

int cannyLow=100, cannyHigh=150; //thresholds for image processing filter
static int dilater = 1;
static int edgemap = 0, binary = 0; //are we performing edgemap calculations?
int visionParam1 = 65; //for processing. Used in threshold() and houghCircle().
int visionParam2 = 35; //for processing
int Hough_MinRAdius_var = 5;               //for processing. Used in houghCircle() indicating min radius of circles.
int Hough_MaxRAdius_var = 200;             //for processing. Used in houghCircle() indicating max radius of circles.
int showagentcirc = 1;
static int detect = 1; //are we detecting object?
static float fpsReceive; //frames per second of video

static int flag_centerP_record = 0; // record center point or not. 1: yest, 0: no

static int centerP_dataSafeLock = 0;  // 1: centerP is being changed, wait until it is done

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int cannyLow_xz=100, cannyHigh_xz=150; //thresholds for image processing filter
static int dilater_xz = 1;
static int edgemap_xz = 0, binary_xz = 0; //are we performing edgemap calculations?
int visionParam1_xz = 65; //for processing
int visionParam2_xz = 35; //for processing
static int detect_xz = 1; //are we detecting object?
static int sidecam_on = 1; //is the sidecam capturing? Default: YES
static float fpsReceive_xz; //frames per second of video
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// magnet detection variables -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int magnetdetection = 0;
int showbox = 1;
int showprocess = 0;
int closediameter = 14;
float m_x, m_y, m_a = 0.0, m_x_history[6] = {0,0,0,0,0,0}, m_y_history[6] = {0,0,0,0,0,0}, m_a_history[6] = {0,0,0,0,0,0}; // historical value of magnet centre (m_x, m_y) and angle m_a
float m_x_temp, m_y_temp, m_a_temp;
float Mwidth, Mlength;
bool drawMagetization = 0, showdestination = 1, showfielddirection = 1;
extern float fangle, destination_angle, v1, v2;
extern float current_temp;
extern char fab_status[];
extern char fab_time[];
extern float field_x, field_y, field_z, field_mag, field_angle;
extern bool KillInFab;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This thread runs the image processing functions:
// initialize camera, receive and display frames, process images

Point centerP_adjusted, centerP_adjusted_xz;
Point2f centerP_rect_short_side[4];//, centerP_rect_short_side_2[2];   							// 2 center points of the (1st and 2nd) rect.'s short sides.
Point2f centerP_rect_short_side_xz[2];   // 2 center points of the rect.'s short sides.


static Point centerP_adjusted_2,centerP_adjusted_2_xz;

static Point prev_centerP_adjusted, prev_centerP_adjusted_2;

static double swimmer_angle = 0;

static int centerP_adjust_flag = 0;
static int centerPointCoorArray[2]  = {0,0};
static int centerPoint2CoorArray[2] = {0,0};
static int goalPointCoorArray[2]   = {320, 240};
static int goalPointCoorCArray[2]   = {320, 240};
static int goalPointCoorRArray[2]   = {320, 240};
static int goalPointCoorArray_xz[2]   = {320, 240};
static int goalPointCoorCArray_xz[2]   = {320, 240};
static int goalPointCoorRArray_xz[2]   = {320, 240};

static float centerP_rect_short_side_coor_array[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//static float centerP_2nd_rect_short_side_coor_array[4];   // point-1.x, point-1,y, point-2.x, point-2.y

Mat img_m_color_for_display, img_m_color_for_display2;

static int flag_second_rect = 0;
static int flag_orinet_circ = 0;                          // orientation_circ detection flag, multi-agent (1:active mode toggle button)

//Variable for plot_line
bool plot_line_flag = false;
int line_x[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int line_y[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int line_R[6]={0,0,0,0,0,0};
int line_G[6]={0,0,0,0,0,0};
int line_B[6]={0,0,0,0,0,0};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// (07-15) See if new frame has arrived
int check_newFrame( void )
{
	return flag_newFrame;
}


Mat getImage(void)
{
	return img_m_color_for_display;
}
Mat getImage2(void)
{
	return img_m_color_for_display2;
}
/// Get Center Point of Rect. in Top View
int * getCenterPointCoor(void)
{
    while(centerP_dataSafeLock);                   // wait until change is done
    centerP_dataSafeLock = 1;                      // do not allow changing the center point when reading it
    centerPointCoorArray[0] = centerP_adjusted.x;
    centerPointCoorArray[1] = centerP_adjusted.y;
    centerP_dataSafeLock = 0;
    flag_newFrame = 0;															// (07-15) clear new frame flag
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
	if (flag_second_rect) {																	// IF the 2nd rec. is required ...
		centerP_rect_short_side_coor_array[4] = centerP_rect_short_side[2].x;
		centerP_rect_short_side_coor_array[5] = 480 - centerP_rect_short_side[2].y;  // !!! The y direction is inversed.
		centerP_rect_short_side_coor_array[6] = centerP_rect_short_side[3].x;
		centerP_rect_short_side_coor_array[7] = 480 - centerP_rect_short_side[3].y;
	}
    return centerP_rect_short_side_coor_array;
}
/// Get center points of rect. for determining headings
int JZ_get_heading_point ( float heading_point[] )
{
	heading_point[0] =       centerP_rect_short_side[0].x;
	heading_point[1] = 480 - centerP_rect_short_side[0].y;  								// !!! The y direction is inversed.
	heading_point[2] =       centerP_rect_short_side[1].x;
	heading_point[3] = 480 - centerP_rect_short_side[1].y;
	if (flag_second_rect) {																	// IF the 2nd rec. is required ...
		heading_point[4] =       centerP_rect_short_side[2].x;
		heading_point[5] = 480 - centerP_rect_short_side[2].y;								// !!! The y direction is inversed.
		heading_point[6] =       centerP_rect_short_side[3].x;
		heading_point[7] = 480 - centerP_rect_short_side[3].y;
	}
    return 1;
}
/*
/// Get the center points of the short sides of the 2nd rect.
float * get_CenterP_2nd_rect_short_side_coor_array ( void )
{
    centerP_2nd_rect_short_side_coor_array[0] = centerP_rect_short_side_2[0].x;
    centerP_2nd_rect_short_side_coor_array[1] = 480 - centerP_rect_short_side_2[0].y;  // !!! The y direction is inversed.
    centerP_2nd_rect_short_side_coor_array[2] = centerP_rect_short_side_2[1].x;
    centerP_2nd_rect_short_side_coor_array[3] = 480 - centerP_rect_short_side_2[1].y;
    return centerP_rect_short_side_coor_array;
}
*/
int * getGoalPointCoor(void)
{
    if(mouse.x>0)
    {
        goalPointCoorArray[0] = mouse.x;
        goalPointCoorArray[1] = 480 - mouse.y;   // Note the positive y dir.
    }
    return goalPointCoorArray;
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
/// Get the orientation angle of the swimmer from the camera image.
double get_swimmer_angle()
{
    return swimmer_angle;
}

void initVision(void)
{
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
		pthread_create(&fthread, NULL, FPSprint    , NULL);  //start frame print thread. Functionality??? comment this if you do not want to show frame per second
		pthread_create(&vthread, NULL, visionThread, NULL);  //start vision thread
	}

	// X-Z Camera
	if(sidecam_on == 1) //if we are using the side cam also
	{
		usleep(2e5);
		printf("Before cam_xz.initialize_xz().\n");
		if(!cam_xz.initialize_xz()) //cam is instance of FWCamera, found in FWcamera.cpp
		{
			g_print("FW camera yz could not be found in initVision!!!\n");
			return;
		}
		usleep(1e5);

		if(!cam_xz.startGrabbingVideo_xz())
		{
			g_print("FW cam yz could not grab in initVision!!!\n");
			return;
		}

		usleep(1e5);
		//pthread_create(&fthread_xz, NULL, FPSprint_xz, NULL);  //start frame print thread
		pthread_create(&vthread_xz, NULL, visionThread_xz, NULL);  //start vision thread
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

	while(!killVisionThread)   //while the image processing is running //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
		//  int sprintf(char *str, const char *format, ...) sends formatted output to a string pointed to by str.
		sprintf(strReceive, "%.1f", fpsReceive); //writes into strRecieve the frames per second

		gdk_threads_enter();
		gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive); //draw on the gui
		gdk_threads_leave();

		usleep((int)1e6); //sets frame rate display frequency
	}
	//printf("In the test zone!\n");
	sprintf(strReceive, "N/A"); //when we turn off the vision, write N/A into the gui
	gdk_threads_enter();
	gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive);
	gdk_threads_leave();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* FPSprint_xz(void*)
{
	char strReceive[50];

	while(!killVisionThread) //while the image processing is running
	{
		sprintf(strReceive, "%.1f", fpsReceive_xz); //writes into strRecieve a the frames per second
		gdk_threads_enter();
		gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive); //draw on the gui
		gdk_threads_leave();
		usleep((int)1e6); //sets frame rate display frequency
	}
	sprintf(strReceive, "N/A"); //when we turn off the vision, write N/A into the gui
	gdk_threads_enter();
	gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive);
	gdk_threads_leave();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* visionThread(void*)
{
	printf("@ the Beginning of visionThread().\n");

	int index = 0;
	unsigned char *inImage;// = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	Mat img_m, img_m_color, img_m_gray;
	// Explain: The class Mat represents an n-dimensional dense numerical single-channel or multi-channel array.
	//          It can be used to store real or complex-valued vectors and matrices, grayscale or color images, voxel volumes,
	//	    vector fields, point clouds, tensors, histograms (though, very high-dimensional histograms may be better stored in a SparseMat).
	vector<Vec3f> circles_m; //for hough circle detection

	Mat threshold_output;    // for threshold and rectangle detection
	vector<vector<Point> > contours; //for threshold and rectangle detection
	vector<Point> largest_contour;
	vector<Vec4i> hierarchy; //for threshold and rectangle detection

	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;
	double a;
	int second_largest_area = 0;
	int third_largest_area = 0;            // regarded as the first orientation circle in multi-agent
	int second_largest_contour_index = 0;
	RotatedRect second_rotated_bounding_rect;
	//Point centerP;

	int i;
	int frame = 0;
	char str[100];
	timeval tStart, tEnd;
	float time;
	double current_time;
	float fpsVec[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex = 0;

	// X-Z Camera
	unsigned char *inImage_xz;// = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	Mat img_m_xz, img_m_color_xz;
	vector<Vec3f> circles_m_xz; //for hough circle detection

	Mat threshold_output_xz; //for threshold and rectangle detection
	vector<vector<Point> > contours_xz; //for threshold and rectangle detection
	vector<Point> largest_contour_xz;
	vector<Vec4i> hierarchy_xz; //for threshold and rectangle detection


	//int largest_area_xz=0;
	//int largest_contour_index_xz = 0;
	//Rect bounding_rect_xz;
	//
	RotatedRect rotated_bounding_rect;
	//
	double a_xz;
	Point centerP;
    Point l_centerP_2;
    Point center_m;                   // center variable in Hough circle detection
    int radius_m;                   // radius variable in Hough circle detection
    Point MM;

	//int i;
	int frame_xz = 0;
	char str_xz[100];
	timeval tStart_xz, tEnd_xz;
	float time_xz;
	double current_time_xz;
	float fpsVec_xz[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex_xz = 0;
	//

	double time_current, time_elapsed, time_init;
	struct timeval start;
	gettimeofday(&start, NULL);
	time_init = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time

    FILE *fp;
    fp = fopen("pointFollowRecord.txt","w");

	while(!killVisionThread & !KillInFab) //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
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

        flag_newFrame = 1;																// (07-15) indicate new frame

		//gettimeofday(&start, NULL);
		//time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time
		//time_elapsed = time_current - time_init;
		//time_current = time_init;
		//printf("in vision thread, time is %.5f.\n", time_elapsed);
		//

		/*
		inImage_xz = cam_xz.grabAframe_xz();
		if(inImage_xz == NULL)
		{
			g_print("Error in firewire stream yz! Reattempting...\n");
			usleep((int)1e3); // I don't know what the wait delay should be
		}

		img_m_xz = Mat(height, width, CV_8UC1, inImage_xz); //convert to Mat format
		*/

		MM = Point(240,320);         // test point

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

/// 2D Multi-agent detection from top camera (xy)  created by Mohammad
/////////////////    multi-agent detection starts here     /////////////////
////////////////////////////////////////////////////////////////////////////

	if(flag_orinet_circ ==1) //if the detect? button is toggled for circle detection

		{
            blur( img_m, threshold_output, Size(4,4) ); //blur image to remove small blips etc
            threshold( threshold_output, threshold_output, visionParam1, 255, THRESH_BINARY_INV );
            cvtColor(threshold_output, img_m_color, CV_GRAY2BGR); //convert to color
        //    gdk_threads_enter();	//display video image in program window
     //       gtk_image_set_from_pixbuf(videoWindow, gdk_pixbuf_new_from_data(img_m_color.data, GDK_COLORSPACE_RGB, false, 8, img_m_color.cols, img_m_color.rows, img_m_color.step, NULL, NULL));
       //     gdk_threads_leave();
         //   continue; //don't do any more processing

		    /*
		 ///   Hough circle algorithm starts here!!!!

          //  cvtColor(img_m, img_m_gray, CV_BGR2GRAY);      //convert to gray-scale recommended for hough-circle algorithm
         // Reduce the noise so we avoid false circle detection
            GaussianBlur(img_m, img_m_gray, Size(9, 9), 2, 2 );
            threshold(img_m_gray, img_m_color, visionParam1, 255, THRESH_BINARY_INV );


		    //  threshold( threshold_output, img_m, visionParam1, 255, THRESH_BINARY_INV );
			HoughCircles( img_m_gray, circles_m, CV_HOUGH_GRADIENT, 1, (int)width/10, visionParam1, visionParam2,   Hough_MinRAdius_var, Hough_MaxRAdius_var); //find circles on image
		    cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color

			for( i = 0; i < circles_m.size(); i++ ) //process and print all circles from HoughCircles()
			{
			      	Point center_m(cvRound(circles_m[i][0]), cvRound(circles_m[i][1]));
			      	int radius_m = cvRound(circles_m[i][2]);

                                if (showagentcirc)
                                            {
                                                    // circle outline
                                                    Scalar color = Scalar( (int)i*250/(contours.size()+1), 255-(int)i*20/(contours.size()+1), (int)i*250/(contours.size()+1) );
                                                    circle( img_m_color, center_m, 3, Scalar(0,100,0) , -1, 8, 0 );
                                                    circle( img_m_color, center_m, radius_m, Scalar(0,0,200) , 1, 8, 0 );
                                                    // circle center
                                                    // g_print("m:   x: %d y: %d r: %d, index= %d \n",center_m.x,center_m.y, radius_m, i);
                                                   // fprintf(fp, "m:   x: %d y: %d r: %d\n",center_m.x,center_m.y, radius_m);   // record current time (sec), and center point coor.
                                            }
			}
					 ///   Hough circle algorithm ends up here!!!!

			*/
		}

/////////////////    multi-agent detection ends up here     /////////////////
////////////////////////////////////////////////////////////////////////////

		if(detect == 1) //for threshold and bounding box detection
		{

			blur( img_m, threshold_output, Size(4,4) ); //blur image to remove small blips etc
			// Explain: blur( src, dst, Size( i, i ), Point(-1,-1) );
			// We specify 4 arguments (more details, check the Reference):
			// 	src: Source image
			// 	dst: Destination image
			// 	Size( w,h ): Defines the size of the kernel to be used ( of width w pixels and height h pixels)
			// 	Point(-1, -1): Indicates where the anchor point (the pixel evaluated) is located with respect to the neighborhood.
			//                     If there is a negative value, then the center of the kernel is considered the anchor point.

			threshold( threshold_output, threshold_output, visionParam1, 255, THRESH_BINARY_INV );
			// Explain: C: double cvThreshold(const CvArr* src, CvArr* dst, double threshold, double max_value, int threshold_type)
			//	Parameters:
			//		src – input array (single-channel, 8-bit or 32-bit floating point).
			//		dst – output array of the same size and type as src.
			//		thresh – threshold value.
			//		maxval – maximum value to use with the THRESH_BINARY and THRESH_BINARY_INV thresholding types.
			//		type – thresholding type (see the details below).

			//adaptiveThreshold(img_m, threshold_output, 255,	ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,91,0);
			if(dilater>0) //if dilater = 0, just use original edgemap
			{
				dilate( threshold_output, threshold_output, Mat(), Point(-1, -1), dilater, 1, 1);
				// Explain: C: void cvDilate(const CvArr* src, CvArr* dst, IplConvKernel* element=NULL, int iterations=1 )
				// Parameters:
				// 	src – input image; the number of channels can be arbitrary, but the depth should be one of CV_8U, CV_16U, CV_16S, CV_32F` or ``CV_64F.
				//	dst – output image of the same size and type as src.
				//	element – structuring element used for dilation; if element=Mat() , a 3 x 3 rectangular structuring element is used.
				//	anchor – position of the anchor within the element; default value (-1, -1) means that the anchor is at the element center.
				//	iterations – number of times dilation is applied.
				//	borderType – pixel extrapolation method (see borderInterpolate() for details).
				//	borderValue – border value in case of a constant border (see createMorphologyFilter() for details).

				//smooth( img_m, img_m, CV_MEDIAN, 5, 5);
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

			//cvtColor(threshold_output, img_m_color, CV_GRAY2BGR); //convert to color
			findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); //find contours

			vector<vector<Point> > contours_poly( contours.size() );
			vector<Rect> boundRect( contours.size() );
			//
			vector<RotatedRect> minRect( contours.size() );
			//
			vector<Point2f>center( contours.size() );
			vector<float>radius( contours.size() );

            //printf("contours size is %d.\n", contours.size());

            if (contours.size() >= 1)   // if the camera detects rectangle ... sometimes the view is all white, no rectangles are detected
            {
                for( i = 0; i < contours.size(); i++ )
			   {
                    //
                    minRect[i] = minAreaRect( Mat(contours[i]) );
                    //
                    approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
			     	boundRect[i] = boundingRect( Mat(contours_poly[i]) );
			   }

                //first, find the largest contour    ...   Contour classification!
                largest_area = 0;

                for(i = 0; i< (contours.size()); i++ )
                {
                    a = contourArea(contours[i], false);           //  Find the area of contour
                    if(a > largest_area)                           // if current contour is bigger ...
                    {
                        largest_area = a;
                        largest_contour_index = i;                 //Store the index of largest contour
                    }
                }
                bounding_rect = boundingRect( contours[largest_contour_index] );                                       // Find the bounding rectangle for biggest contour
                //printf("Marker 5.\n");

                centerP = Point( bounding_rect.x + bounding_rect.width/2, bounding_rect.y + bounding_rect.height/2 );  // Center point of the bounding rectangle

                while (centerP_dataSafeLock);   // wait until reading is done
                centerP_dataSafeLock = 1;
                centerP_adjusted = Point( bounding_rect.x + bounding_rect.width/2, 480-(bounding_rect.y + bounding_rect.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
                centerP_dataSafeLock = 0;

                rotated_bounding_rect = minAreaRect( Mat( contours[largest_contour_index] ) );
            }

            /// Recording to File
            gettimeofday(&start, NULL);
            time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
            if (flag_centerP_record)
                fprintf(fp, "%.6f %d %d\n", time_current, centerP_adjusted.x, centerP_adjusted.y);   // record current time (sec), and center point coor.
            ///

			// If 2 rectangles are required, get the 2nd largest contour
			if (flag_second_rect)
			{
                second_largest_area = 0;
                for(i = 0; i< (contours.size()); i++)
                {
                    a = contourArea(contours[i], false);  //  Find the area of contour
                    if ( (a > second_largest_area) && (a < largest_area) )
                    {
                        second_largest_area = a;
                        second_largest_contour_index = i;                 //Store the index of largest contour
                        bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
                        l_centerP_2        = Point( bounding_rect.x + bounding_rect.width/2, bounding_rect.y + bounding_rect.height/2 );  // Center point of the bounding rectangle
                        centerP_adjusted_2 = Point( bounding_rect.x + bounding_rect.width/2, 480-(bounding_rect.y + bounding_rect.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
                        second_rotated_bounding_rect = minAreaRect( Mat(contours[i]) );
                    }
                }
			}

// This part recognizes label switching during a tracking process   :: multi_Agent
/*
			if (centerP_adjust_flag ==0)
            {
                prev_centerP_adjusted = centerP_adjusted;
                prev_centerP_adjusted_2 = centerP_adjusted_2;
                centerP_adjust_flag = 1;
            }
            else
            {
                float distance;
                distance = sqrt(pow(prev_centerP_adjusted.x-centerP_adjusted.x,2)+pow(prev_centerP_adjusted.y-centerP_adjusted.y,2));
                if (distance > 40)
                {
                    Point temp_center_point;
                    temp_center_point = centerP_adjusted;
                    centerP_adjusted = centerP_adjusted_2;
                    centerP_adjusted_2 = temp_center_point;
                }
                prev_centerP_adjusted = centerP_adjusted;
                prev_centerP_adjusted_2 = centerP_adjusted_2;
            }
*/
			//printf("Center Point is %d and %d.\n", centerP.x, centerP.y);

			cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color
			Scalar color = Scalar( (int)i*254/(contours.size()+1),255- (int)i*254/(contours.size()+1), (int)i*254/(contours.size()+1) );
			//rectangle( img_m_color, bounding_rect.tl(), bounding_rect.br(), color, 1, 8, 0 ); //draw largest rectangle (x-y, not rotated)
			circle( img_m_color, centerP, 2, Scalar(0,0,255), 3, 8, 0 ); 			  //draw centerpoint of largest rectangle

            //
            swimmer_angle = rotated_bounding_rect.angle;   // Get the swimmer's orientation angle.
            Point2f rect_points[4], second_rect_points[4];												// (07-19) rect. point of (1st & 2nd) rect; Point2f is defined as Point_<float>
			rotated_bounding_rect.points( rect_points );    											// (07-19) get rect. points of 1st rect.

            //for( int j = 0; j < 4; j++ )
                //line( img_m_color, rect_points[j], rect_points[(j+1)%4], Scalar(200, 50, 0), 1, 8 ); //Feb 27

			if (flag_second_rect) {																		// IF 2 rectangles are required, plot the 2nd largest rectangle
				//Point2f second_rect_points[4];
				second_rotated_bounding_rect.points( second_rect_points );    // Point2f is defined as Point_<float>
				//for( int j = 0; j < 4; j++ )
					//line( img_m_color, second_rect_points[j], second_rect_points[(j+1)%4], Scalar(0, 0, 255), 1, 8 ); //Feb 27
				circle( img_m_color, l_centerP_2, 2, Scalar(255,0,0), 3, 8, 0 );
			}
            // Get the center point of 2 short sides
            float rect_line[4], rect_line_2[4];
            //uint temp_index = 0;
            //uint shortest_line = 0;
			for ( index = 0; index < 4; index ++ ) {
				rect_line[index] = pow(rect_points[index].x - rect_points[(index+1)%4].x, 2) + pow(rect_points[index].y - rect_points[(index+1)%4].y, 2);
																										// (07-19) calc. length of sides
				if (flag_second_rect)																	// (07-19) IF 2nd rect. is required ...
					rect_line_2[index] = pow(second_rect_points[index].x - second_rect_points[(index+1)%4].x, 2) + pow(second_rect_points[index].y - second_rect_points[(index+1)%4].y, 2);
			}
            //Point2f interested_P[2];

            if (rect_line[0] > rect_line[1])   // Long-short-long-short
            {
                centerP_rect_short_side[0] = Point2f( (rect_points[2].x + rect_points[1].x)/2.0, (rect_points[2].y + rect_points[1].y)/2.0 );
                centerP_rect_short_side[1] = Point2f( (rect_points[0].x + rect_points[3].x)/2.0, (rect_points[0].y + rect_points[3].y)/2.0 );
            }
            else                     // Short-long-short-long
            {
                centerP_rect_short_side[0] = Point2f( (rect_points[0].x + rect_points[1].x)/2.0, (rect_points[0].y + rect_points[1].y)/2.0 );
                centerP_rect_short_side[1] = Point2f( (rect_points[2].x + rect_points[3].x)/2.0, (rect_points[2].y + rect_points[3].y)/2.0 );
            }

			if (flag_second_rect)																		// (07-19) IF 2nd rect. is required ...
			{
				if (rect_line_2[0] > rect_line_2[1])   // Long-short-long-short
				{
					centerP_rect_short_side[2] = Point2f( (second_rect_points[2].x + second_rect_points[1].x)/2.0, (second_rect_points[2].y + second_rect_points[1].y)/2.0 );
					centerP_rect_short_side[3] = Point2f( (second_rect_points[0].x + second_rect_points[3].x)/2.0, (second_rect_points[0].y + second_rect_points[3].y)/2.0 );
				}
				else                     // Short-long-short-long
				{
					centerP_rect_short_side[2] = Point2f( (second_rect_points[0].x + second_rect_points[1].x)/2.0, (second_rect_points[0].y + second_rect_points[1].y)/2.0 );
					centerP_rect_short_side[3] = Point2f( (second_rect_points[2].x + second_rect_points[3].x)/2.0, (second_rect_points[2].y + second_rect_points[3].y)/2.0 );
				}
			}

            //circle( img_m_color, centerP_rect_short_side[0], 2, Scalar(255,0,0), 3, 8, 0 ); //Commented out by Patrick Feb 27
            //circle( img_m_color, centerP_rect_short_side[1], 2, Scalar(255,0,0), 3, 8, 0 ); //Commented out by Patrick Feb 27

			/// Draw all polygonal contour + bonding rects + circles
			for(i = 0; i< (contours.size()); i++ )
			{
				continue; //don't draw any rects
				Scalar color = Scalar( (int)i*254/(contours.size()+1),255- (int)i*254/(contours.size()+1), (int)i*254/(contours.size()+1) );
               // drawContours( img_m_color, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                //rectangle( img_m_color, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 ); //Commented out by Patrick Feb 27

                //
                //Point2f rect_points[4]; minRect[i].points( rect_points );
                //for( int j = 0; j < 4; j++ )
                //    line( img_m_color, rect_points[j], rect_points[(j+1)%4], Scalar(200, 50, 0), 1, 8 );
                //
			}
		}

		else if(detect ==0 && flag_orinet_circ ==0) //if general detection and multi-agent detection are both off
			cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color anyways

		//
		cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color anyways
		//

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

		//
		//img_m_color_for_display2 = img_m_color_xz;
		//

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

		//g_print("  %.1f fps\n", fpsReceive); //we now do this in the gui using a separate thread
        //

        //Plot Lines for Rotating Magnet Control
        if (plot_line_flag)
        {
            Point pt1, pt2;

            for (int count=0; count<12; count=count+2)
            {

                if ( (line_x[count]==line_x[count+1]) && (line_y[count]==line_y[count+1]) )
                    count=12; //skip to end
                else
                {
                     pt1.x = line_x[count];
                     pt1.y = line_y[count];
                     pt2.x = line_x[count+1];
                     pt2.y = line_y[count+1];
                    line( img_m_color, pt1, pt2, Scalar(line_R[count/2], line_G[count/2], line_B[count/2]), 1, 8 );
                }
            }
        }

		/// (07-19) Plot Lines representing DC & AC Angle
		if (flag_DC_AC_angle)
		{
			line( img_m_color, DC_line_P_1, DC_line_P_2, Scalar(255, 0, 0), 2, 8 );
			line( img_m_color, AC_line_P_1, AC_line_P_2, Scalar(0, 255, 0), 2, 8 );
			circle( img_m_color, DC_line_P_2, 4, Scalar( 255, 0, 0 ), 2, 8, 0 );					// draw circle to denote arrow
		}

	} //end vision loop due to killVisionthread==1

	cam.stopGrabbingVideo();
	usleep ((int)1e5); //make sure that ImageProc_xz has closed
	cam.deinitialize();

	/*
	cam_xz.stopGrabbingVideo_xz();
	usleep ((int)1e5); //make sure that ImageProc_xz has closed
	cam_xz.deinitialize();
	*/

	//cvReleaseImage(&img);
	//cvReleaseImage(&img_color);
	//free(image);
    fclose(fp);   // Close file
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
	Mat img_m_xz, img_m_color_xz;
	vector<Vec3f> circles_m_xz; //for hough circle detectio

	Mat threshold_output_xz; //for threshold and rectangle detection
	vector<vector<Point> > contours_xz; //for threshold and rectangle detection
	vector<Point> largest_contour_xz;
	vector<Vec4i> hierarchy_xz; //for threshold and rectangle detection

	int largest_area_xz=0;
	int largest_contour_index_xz = 0;
	Rect bounding_rect_xz;
	RotatedRect rotated_bounding_rect_xz, second_rotated_bounding_rect_xz; //P
	double a_xz;
    int second_largest_area_xz = 0;
	int second_largest_contour_index_xz = 0;

	Point centerP_xz;
	Point l_centerP_2_xz;

	int i;
	int frame_xz = 0;
	char str_xz[100];
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

    double time_current_xz, time_elapsed_xz, time_init_xz;
	struct timeval start_xz;
	gettimeofday(&start_xz, NULL);
	time_init_xz = (double) start_xz.tv_sec + start_xz.tv_usec*1e-6 ; // Start time

    FILE *fp_xz;
    fp_xz = fopen("pointFollowRecord.txt","w");

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

		if (magnetdetection)
        {
            detect_xz = 0;
            img_m_xz_ori = img_m_xz.clone(); // store the original image
            //blur(img_m_xz, img_m_xz, Size(3,3)); // blur

            if (mouse_xz.x>0) // designate ROI by clicking: top-left corner
            {
                img_m_xz(Range(0, mouse_xz.y),Range(0,640)) = Scalar::all(255);
                img_m_xz(Range(0,480),Range(0, mouse_xz.x)) = Scalar::all(255);
            }
            if (mouseC_xz.x>0) // designate ROI by clicking: bottom-right corner
            {
                img_m_xz(Range(mouseC_xz.y, 480),Range(0,640)) = Scalar::all(255);
                img_m_xz(Range(0,480),Range(mouseC_xz.x, 640)) = Scalar::all(255);
            }
            if (mouseR_xz.x>0) // designate ROI by clicking: needle area
            {
                img_m_xz(Range(0, mouseR_xz.y),Range(mouseR_xz.x-15, mouseR_xz.x+15)) = Scalar::all(255);
            }

            threshold( img_m_xz, img_m_xz, 32, 255, THRESH_BINARY); // binarize

            img_m_xz_bi = img_m_xz.clone(); // store the binary image

            Mat strel = getStructuringElement( MORPH_ELLIPSE, Size(closediameter, closediameter) ); // generate a disk structure element
            dilate( img_m_xz, img_m_xz, strel); // do the close operation using disk kernel
            erode ( img_m_xz, img_m_xz, strel);

            vector<vector<Point> > magnetcontours;
            vector<Vec4i> hierarchy;
            img_m_xz_temp =  img_m_xz.clone();
            findContours( img_m_xz, magnetcontours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // find magnet contours

            RotatedRect minRect;
            if (magnetcontours.size()>=2) //contour.0 is the image
            {
                double magnetcontours_area[magnetcontours.size()];
                magnet_maxarea = 0.0;
                for ( int i=1; i<magnetcontours.size(); i++ ) // find the largest contour as the magnet contour candidate
                {
                    magnetcontours_area[i] =  contourArea(magnetcontours[i]);
                    if ( magnetcontours_area[i] > magnet_maxarea )
                    {
                        magnet_maxarea = magnetcontours_area[i];
                        ind_maxarea = i;
                    }
                }
                contour_number = true;
            }
            else
            {
                continue; // skip this loop
            }

            if ( contour_number & (magnet_maxarea > 0.75*pre_area) & (magnet_maxarea < 1.25*pre_area) ) // draw box and calculate only if the candidate magnet area is in a certain range
            {
                minRect = minAreaRect( Mat(magnetcontours[ind_maxarea]) ); // fit using a rectangle

                img_m_xz = img_m_xz_temp.clone();

                if (!showprocess) // are we showing the already processed image?
                {
                    img_m_xz = img_m_xz_ori;
                }

                if (showbox) // are we displaying the boundary box?
                {
                    Point2f rect_points[4]; minRect.points( rect_points ); //draw the magnet contour
                    for( int j = 0; j < 4; j++ )
                    {
                        line( img_m_xz, rect_points[j], rect_points[(j+1)%4], 72, 2, 8 );
                    }
                    orientation_display(rect_points[0], rect_points[1], rect_points[2], rect_points[3]); // calculating box parameters
                    drawMagetization=1;
                }
                contour_number = false;
            }
            else
            {
                img_m_xz =  img_m_xz_temp.clone();
                if (!showprocess) // are we showing the already processed image?
                {
                    img_m_xz = img_m_xz_ori;
                }
            }
            pre_area = magnet_maxarea;
        }

		if(edgemap_xz==1)
		{
			Canny(img_m_xz, img_m_xz, cannyLow_xz, cannyHigh_xz, 3 ); //edge detect

			if(dilater_xz>0) //if dilater = 0, just use original edgemap
			{
				dilate( img_m_xz, img_m_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
				//smooth( img_m, img_m, CV_MEDIAN, 5, 5);
				erode( img_m_xz, img_m_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
			}
		}

		if(detect_xz ==2) //if the detect? button is toggled for circle detection
		{
			HoughCircles( img_m_xz, circles_m_xz, CV_HOUGH_GRADIENT, 1, (int)width/1, visionParam1_xz, visionParam2_xz,   5, 100 ); //find circles on image

			cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color

			for( i = 0; i < circles_m_xz.size(); i++ ) //process and print all circles from HoughCircles()
			{
			      	Point center_m_xz(cvRound(circles_m_xz[i][0]), cvRound(circles_m_xz[i][1]));
			      	int radius_m_xz = cvRound(circles_m_xz[i][2]);
			      	// circle center
                    circle( img_m_color_xz, center_m_xz, 3, Scalar(0,255,0), -1, 8, 0 );
			        // circle outline
			      	circle( img_m_color_xz, center_m_xz, radius_m_xz, Scalar(0,0,255), 1, 8, 0 );
				    //g_print("m:   x: %d y: %d r: %d\n",center_m.x,center_m.y, radius_m);

			}
		}
		if(detect_xz == 1) //for threshold and bounding box detection
		{
			blur( img_m_xz, threshold_output_xz, Size(4,4) ); //blur image to remove small blips etc
			threshold( threshold_output_xz, threshold_output_xz, visionParam1_xz, 255, THRESH_BINARY_INV );
			//adaptiveThreshold(img_m, threshold_output, 255,	ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,91,0);
			if(dilater_xz>0) //if dilater = 0, just use original edgemap
			{
				dilate( threshold_output_xz, threshold_output_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
				//smooth( img_m, img_m, CV_MEDIAN, 5, 5);
				erode( threshold_output_xz, threshold_output_xz, Mat(), Point(-1, -1), 2*dilater_xz, 1, 1);
				dilate( threshold_output_xz, threshold_output_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
			}
			if(binary_xz==1) //show binary image, don't do any more processing
			{
				cvtColor(threshold_output_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color
				gdk_threads_enter();	//display video image in program window
				gtk_image_set_from_pixbuf(videoWindow2, gdk_pixbuf_new_from_data(img_m_color_xz.data, GDK_COLORSPACE_RGB, false, 8, img_m_color_xz.cols, img_m_color_xz.rows, img_m_color_xz.step, NULL, NULL));
				gdk_threads_leave();
				continue; //don't do any more processing
			}

			//cvtColor(threshold_output, img_m_color, CV_GRAY2BGR); //convert to color
			findContours( threshold_output_xz, contours_xz, hierarchy_xz, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); //find contours

			vector<vector<Point> > contours_poly_xz( contours_xz.size() );
			vector<Rect> boundRect_xz( contours_xz.size() );
			vector<RotatedRect> minRect_xz( contours_xz.size() ); //P
			vector<Point2f>center_xz( contours_xz.size() );
			vector<float>radius_xz( contours_xz.size() );

            if (contours_xz.size() >= 1)   // if the camera detects rectangle ... sometimes the view is all white, no rectangles are detected
            {

                for( i = 0; i < contours_xz.size(); i++ )
                   {
                        minRect_xz[i] = minAreaRect( Mat(contours_xz[i]) );
                        approxPolyDP( Mat(contours_xz[i]), contours_poly_xz[i], 3, true );
                        boundRect_xz[i] = boundingRect( Mat(contours_poly_xz[i]) );
                   }

                //first, find the largest contour
                largest_area_xz=0;
                for(i = 0; i< (contours_xz.size()); i++ )
                {
                    a_xz = contourArea( contours_xz[i],false);  //  Find the area of contour
                        if(a_xz > largest_area_xz)
                        {
                            largest_area_xz = a_xz;
                            largest_contour_index_xz = i;                //Store the index of largest contour
                        }
                }
                bounding_rect_xz = boundingRect(contours_xz[largest_contour_index_xz]); // Find the bounding rectangle for biggest contour
                centerP_xz = Point(  bounding_rect_xz.x + bounding_rect_xz.width/2, bounding_rect_xz.y + bounding_rect_xz.height/2  );


                while (centerP_dataSafeLock);   // wait until reading is done
                centerP_dataSafeLock = 1;
                centerP_adjusted_xz = Point( bounding_rect_xz.x + bounding_rect_xz.width/2, 480-(bounding_rect_xz.y + bounding_rect_xz.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
                centerP_dataSafeLock = 0;

                rotated_bounding_rect_xz = minAreaRect( Mat( contours_xz[largest_contour_index_xz] ) ); //P
            }

            /// Recording to File
            gettimeofday(&start_xz, NULL);
            time_current_xz = (double) start_xz.tv_sec + start_xz.tv_usec*1e-6;
            //if (flag_centerP_record)
            //    fprintf(fp_xz, "%.6f %d %d\n", time_current_xz, centerP_adjusted_xz.x, centerP_adjusted_xz.y);   // record current time (sec), and center point coor.
            ///

            // If 2 rectangles are required, get the 2nd largest contour
			if (flag_second_rect)
			{
                second_largest_area_xz = 0;
                for(i = 0; i< (contours_xz.size()); i++)
                {
                    a_xz = contourArea(contours_xz[i], false);  //  Find the area of contour
                    if ( (a_xz > second_largest_area_xz) && (a_xz < largest_area_xz) )
                    {
                        second_largest_area_xz = a_xz;
                        second_largest_contour_index_xz = i;                 //Store the index of largest contour
                        bounding_rect_xz = boundingRect(contours_xz[i]); // Find the bounding rectangle for biggest contour
                        l_centerP_2_xz        = Point( bounding_rect_xz.x + bounding_rect_xz.width/2, bounding_rect_xz.y + bounding_rect_xz.height/2 );  // Center point of the bounding rectangle
                        centerP_adjusted_2_xz = Point( bounding_rect_xz.x + bounding_rect_xz.width/2, 480-(bounding_rect_xz.y + bounding_rect_xz.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
                        second_rotated_bounding_rect_xz = minAreaRect( Mat(contours_xz[i]) );
                    }
                }
			}

			cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color
			Scalar color_xz = Scalar( (int)i*254/(contours_xz.size()+1),255- (int)i*254/(contours_xz.size()+1), (int)i*254/(contours_xz.size()+1) );
			//rectangle( img_m_color_xz, bounding_rect_xz.tl(), bounding_rect_xz.br(), color_xz, 1, 8, 0 ); //draw largest rectangle // Commented out by Patrick Feb 27
			circle( img_m_color_xz, centerP_xz, 2, Scalar(0,0,255), 3, 8, 0 ); //draw centerpoint of largest rectangle

            //swimmer_angle = rotated_bounding_rect.angle;   // Get the swimmer's orientation angle.
            Point2f rect_points_xz[4]; rotated_bounding_rect_xz.points( rect_points_xz );    // Point2f is defined as Point_<float>
            //for( int j = 0; j < 4; j++ )
                //line( img_m_color_xz, rect_points_xz[j], rect_points_xz[(j+1)%4], Scalar(200, 50, 0), 1, 8 ); //Feb 27

            // If 2 rectangles are required, plot the 2nd largest rectangle
            if (flag_second_rect)
			{
                Point2f second_rect_points_xz[4];
                second_rotated_bounding_rect_xz.points( second_rect_points_xz );    // Point2f is defined as Point_<float>
                //for( int j = 0; j < 4; j++ )
                    //line( img_m_color, second_rect_points_xz[j], second_rect_points_xz[(j+1)%4], Scalar(0, 0, 255), 1, 8 ); //Feb 27

                circle( img_m_color_xz, l_centerP_2_xz, 2, Scalar(255,0,0), 3, 8, 0 );
            }

            // Get the center point of 2 short sides
            float rect_line_xz[4];
            uint temp_index = 0;
            //uint shortest_line = 0;
            for (temp_index = 0; temp_index<4; temp_index ++)
            {
                rect_line_xz[temp_index] = pow(rect_points_xz[temp_index].x - rect_points_xz[(temp_index+1)%4].x, 2) + pow(rect_points_xz[temp_index].y - rect_points_xz[(temp_index+1)%4].y, 2);
            }
            //Point2f interested_P[2];

            if (rect_line_xz[0] > rect_line_xz[1])   // Long-short-long-short
            {
                centerP_rect_short_side_xz[0] = Point2f( (rect_points_xz[2].x + rect_points_xz[1].x)/2.0, (rect_points_xz[2].y + rect_points_xz[1].y)/2.0 );
                centerP_rect_short_side_xz[1] = Point2f( (rect_points_xz[0].x + rect_points_xz[3].x)/2.0, (rect_points_xz[0].y + rect_points_xz[3].y)/2.0 );
            }
            else                     // Short-long-short-long
            {
                centerP_rect_short_side_xz[0] = Point2f( (rect_points_xz[0].x + rect_points_xz[1].x)/2.0, (rect_points_xz[0].y + rect_points_xz[1].y)/2.0 );
                centerP_rect_short_side_xz[1] = Point2f( (rect_points_xz[2].x + rect_points_xz[3].x)/2.0, (rect_points_xz[2].y + rect_points_xz[3].y)/2.0 );
            }

			/// Draw all polygonal contour + bonding rects + circles
			for(i = 0; i< (contours_xz.size()); i++ )
			{
				continue; //don't draw any rects
				Scalar color_xz = Scalar( (int)i*254/(contours_xz.size()+1),255- (int)i*254/(contours_xz.size()+1), (int)i*254/(contours_xz.size()+1) );
			       	//drawContours( img_m_color, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			       	//rectangle( img_m_color_xz, boundRect_xz[i].tl(), boundRect_xz[i].br(), color_xz, 2, 8, 0 ); //Commented out by Patrick Feb 27
			}
		}
		else if(detect_xz ==0) //if detection is off
			cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color anyways

        //draw mouse clicks:
		if(mouse_xz.x>0)
			circle( img_m_color_xz, mouse_xz, 4, Scalar(  200, 50, 0), 2, 8, 0 );
		if(mouseR_xz.x>0)
			circle( img_m_color_xz, mouseR_xz,4, Scalar( 20, 250, 300 ), 2, 8, 0 );
		if(mouseC_xz.x>0)
			circle( img_m_color_xz, mouseC_xz, 4, Scalar( 220, 130, 100 ), 2, 8, 0 );

        // draw lines and and put texts
        Point textpoint;
		double fab_fontscale = 0.6;
		Scalar fab_color = Scalar(255, 0, 0); // red
		int fab_thickness = 1;
        //show angle
        char angle_text[51];
        sprintf(angle_text, "Angle:%6.1f", m_a);
        textpoint.x=5; textpoint.y=475;
        putText( img_m_color_xz, angle_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
        //show temperature
        sprintf(angle_text, "T:%5.1f", current_temp);
        textpoint.x=160; textpoint.y=475;
        putText( img_m_color_xz, angle_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
        // show time
        textpoint.x=5; textpoint.y=455;
        putText( img_m_color_xz, fab_time, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
        // show status
        textpoint.x=80; textpoint.y=455;
        putText( img_m_color_xz, fab_status, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
        // draw field at bottom right
        Point startP; Point endP;
        startP.x = 580;
        startP.y = 420;
        endP.x = 580 + 40*cos(field_angle*PI/180.0);
        endP.y = 420 - 40*sin(field_angle*PI/180.0);
        line( img_m_color_xz, startP, endP, Scalar(255,0,0), 2, 8, 0);
		endP.x = 580 + 40*field_x/14.0;
        endP.y = 420 - 40*field_z/14.0;
        circle( img_m_color_xz, endP, 2, Scalar(255,0,0), 2, 8, 0 );
        circle( img_m_color_xz, startP, 40, Scalar(255,0,0), 1, 8, 0 );


        if (drawMagetization)
        {
            //draw magnet direction
            startP.x = m_x; startP.y = m_y;
            endP.x = m_x + Mlength/1.5 * sin(m_a*PI/180.0);
            endP.y = m_y + Mlength/1.5 * cos(m_a*PI/180.0);
            line( img_m_color_xz, startP, endP, Scalar(255,0,0), 1, 8, 0);
            circle( img_m_color_xz, endP, 2, Scalar(255,0,0), 2, 8, 0 );
            //draw destination angle
            if (showdestination)
            {
                startP.x = m_x; startP.y = m_y;
                endP.x = m_x + Mlength/1.5 * sin(destination_angle*PI/180.0);
                endP.y = m_y + Mlength/1.5 * cos(destination_angle*PI/180.0);
                line( img_m_color_xz, startP, endP, Scalar(0,255,0), 1, 8, 0);
                circle( img_m_color_xz, endP, 2, Scalar(0,255,0), 2, 8, 0 );
            }
            //draw field angle & magnitude
            if (showfielddirection)
            {
                startP.x = m_x; startP.y = m_y;
                endP.x = m_x + Mlength/1.5 * cos(field_angle*PI/180.0);
                endP.y = m_y - Mlength/1.5 * sin(field_angle*PI/180.0);
                line( img_m_color_xz, startP, endP, Scalar(255,99,0), 1, 8, 0);

                endP.x = m_x + Mlength/1.5 * field_x/14.0;
				endP.y = m_y - Mlength/1.5 * field_z/14.0;
                circle( img_m_color_xz, endP, 2, Scalar(255,99,0), 2, 8, 0 );
            }
            drawMagetization=0;
        }


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

	//cvReleaseImage(&img);
	//cvReleaseImage(&img_color);
	//free(image);

    fclose(fp_xz);   // Close file
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


void img_test ()
{
	IplImage *img, *img_grey, *img_color;
 	int height,width,step,channels;
  	int i,j,k;


  	// load an image
 	img=cvLoadImage("testimage.jpg",CV_LOAD_IMAGE_UNCHANGED);
 	if(!img)
	{
		printf("Could not load image file:");
		exit(0);
	}

  	// get the image data
  	height    = img->height;
	width     = img->width;
	step      = img->widthStep;
	channels  = img->nChannels;
	printf("Processing a %dx%d image with %d channels\n",height,width,channels);

	  // create a window
	  //cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
	  //cvMoveWindow("mainWin", 100, 100);




  // show the image
//  cvShowImage("mainWin", img );
//	cvShowImage(videoWindow, img );
	img_grey = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1); //create empty image

	cvCvtColor(img, img_grey, CV_BGR2GRAY); //convert to grey
	cvCanny(img_grey, img_grey, cannyLow, cannyHigh, 3 ); //edge detect


	cvCvtColor(img_grey, img, CV_GRAY2BGR); //convert to color

	/*
	char str[10] = "123456789";
	///

	gdk_threads_enter();
	//gtk_image_set_from_pixbuf(videoWindow,convertOpenCv2Gtk(img) );
	///
	gtk_label_set_text (testLabel, str);
	gtk_label_set_text (centerPointCoor, str);
	///
	gdk_threads_leave();
*/
  // release the image
//	cvReleaseImage(&img ); /I think this crashes our code sometimes. Not sure if we really need it or not...


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
	detect = d; //for image processing
}

 void setvision_HoughMinRadius_vision(int d)
{
	Hough_MinRAdius_var = d;                  //for image processing to get min radius of Hough circle
}


 void setvision_HoughMaxRadius_vision(int d)
{
	Hough_MaxRAdius_var = d;                  //for image processing to get max radius of Hough circle
}

void set_agentshowcirc(int d)
{
	showagentcirc = d; //for showing magnet boundary box
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
    //whether to accept this m_a_temp based on the previous several values????????????
    ///////////////////////

    // update the historical data for m_x, m_y and m_a
    for ( int i = 0; i < 5; i++ )
    {
        m_x_history[i] = m_x_history[i+1];
        m_x_sum = m_x_sum + m_x_history[i];

        m_y_history[i] = m_y_history[i+1];
        m_y_sum = m_y_sum + m_y_history[i];

        m_a_history[i] = m_a_history[i+1];

        if ( i == 0)
            m_a_ave = m_a_history[0];
        else
            m_a_ave = anglePlus_v(m_a_ave, angleMinus_v(m_a_history[i], m_a_ave));
    }
    m_x_history[5] = m_x_temp;
    m_x_sum = m_x_sum + m_x_history[5];
    m_y_history[5] = m_y_temp;
    m_y_sum = m_y_sum + m_y_history[5];
    m_a_history[5] = m_a_temp;
    m_a_ave = anglePlus_v(m_a_ave, angleMinus_v(m_a_history[5], m_a_ave));

    // output m_x, m_y and m_a;
    m_x = m_x_sum/6.0;
    m_y = m_y_sum/6.0;
    m_a = m_a_ave;
    //printf(" (%.2f  %.2f)  %.2f\n", m_x_sum/6.0, m_y_sum/6.0, m_a_sum/6.0);
}

float anglePlus_v( float a, float b) // rotate angle by angle b => angle c, wrap c in (-pi, pi]. "v" means vision.c
{
    float c = a + b;
    while ( c > 180 )   { c = c - 360.0; }
    while ( c <= -180 ) { c = c + 360.0; }
    return c;
}

float angleMinus_v( float c, float a) // rotate angle by angle b => angle c, wrap b in (-pi, pi]. "v" means vision.c
{
    float b = c - a;
    while ( b > 180 )   { b = b - 360.0; }
    while ( b <= -180 ) { b = b + 360.0; }
    return b;
}
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

        if (count <6)																			// (07-19) 6 lines in total?
        {
            line_R[count]=lR[count];
            line_G[count]=lG[count];
            line_B[count]=lB[count];
        }

    }
}

/// (07-19) Plot DC & AC Angles
int plot_DC_AC_angle (double DC_angle, double AC_angle)
{
	flag_DC_AC_angle = 1;
	//DC_line_x[0] = 600;
	//DC_line_x[1] = 600 + (int) (10*cos(DC_angle));
	DC_line_P_1.x = 600;
	DC_line_P_1.y = 480 - 40;

	DC_line_P_2.x = 600 + (int) (20*cos(DC_angle));
	DC_line_P_2.y = 480 - (40 + (int) (20*sin(DC_angle)));

	//DC_line_y[0] = 480 - 50;
	//DC_line_y[1] = 480 - (50 + (int) (10*sin(DC_angle)));

	//AC_line_x[0] = 600;
	//AC_line_x[1] = 600 + (int) (20*cos(AC_angle));

	//AC_line_y[0] = 480 - 50;
	//AC_line_y[1] = 480 - (50 + (int) (20*sin(AC_angle)));

	AC_line_P_1.x = 600;
	AC_line_P_1.y = 480 - 40;

	AC_line_P_2.x = 600 + (int) (40*cos(AC_angle));
	AC_line_P_2.y = 480 - (40 + (int) (40*sin(AC_angle)));

	return 1;
}
