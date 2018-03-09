////////////////////////////////////////////////////////////////////////////////////////
// File      : vision.c
// Function  : Image processing and drawing. initialize camera, receive and display frames, process images
// Edited by : Jiachen, Zhe
////////////////////////////////////////////////////////////////////////////////////////
#include "vision.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FWcamera cam, cam_xz; //see FWcamera.cpp

int width = 640;   //image width, pixels
int height = 480;  //image height, pixels
int depth = 1;     //depth of image
//unsigned char * image = NULL;
int killVisionThread = 1; //this stops our vision thread

GtkLabel *labelFPSreceive, *labelFPSreceive_xz;

static Point mouse, mouseC, mouseR;
static Point mouse_xz, mouseC_xz, mouseR_xz;

// York's parameters

// Chris's global vision parameters
static Image_JZ presFrame;                                      // present obtained frame
static Point_JZ cargoOne(0,0);                  				 // cargo detected from frames
static Point_JZ cargoOneMemo(0,0);           					// cargo memories
static Point agentOne(0,0);             						 // first agent center recoreded variable
static Point agentOneMemo(0,0);                      	        // memory of 2 agents, used to avoid switching
static float r_x = 10;
static float r_y = 10;
static float centerPointCoorArray[3]  = {320, 240, 0}; // Current pose of robot
static float centerPointCoorArray_2[3]  = {320, 140, 0}; // Current pose of cargo
static float goalPointCoorArray[2]   = {320, 240};
Point centerP_adjusted, centerP_adjusted_xz;
Point centerP_adjusted_2;

int visionParam1 = 65; //for processing. Used in threshold() and houghCircle().
int visionParam2 = 35; //for processing
static int centerP_dataSafeLock = 0;  // 1: centerP is being changed, wait until it is done
static float angle1;
static float angle2;

int binaryThreshold = 90;                   // indicating the level of binary conversion.
int Hough_Erosion_adj = 0;                  //for processing. Used in houghCircle() indicating the level of erosion conversion.
int Hough_dilation_adj = 6;                 //for processing. Used in houghCircle() indicating the level of dilation conversion.
int setBlursizeHough = 10;          // set Blur size in Hough transform                put 7 if you are using Median filter otherwise 16 to 25 is good

// X-Y Camera
int cannyLow = 100, cannyHigh = 150; //thresholds for image processing filter
static int dilater = 1;
static int edgemap = 0, binary = 0; //are we performing edgemap calculations?
static int flag3dIndicator = 0, flag2dIndicator = 1;

static float fpsReceive; //frames per second of video
Mat img_m_color_for_display;

// X-Z Camera
int cannyLow_xz=100, cannyHigh_xz=150;
static int dilater_xz = 1;
static int edgemap_xz = 0, binary_xz = 0;
static int binaryThreshold_xz = 65;
int visionParam2_xz = 35;
static int flag3dIndicatorXZ = 1, flag2dIndicatorXZ = 1;
static int topcam_on = 1; //is the sidecam capturing? Default: YES
static float fpsReceive_xz;
Mat img_m_color_for_display2;

// Digital arena variables
static float scaleX = 210;
static float scaleY = 190;
static float wall_width = 2.5*scaleY;
static float wall_length = 3*scaleX;
static float bound_length = 0.4*scaleY;
static float bound_gap = 0.4*scaleY;

// magnet detection variables -- Zhe
float m_x, m_y, m_a = 0.0, m_x_history[6] = {0,0,0,0,0,0}, m_y_history[6] = {0,0,0,0,0,0}, m_a_history[6] = {0,0,0,0,0,0}; // historical value of magnet centre (m_x, m_y) and angle m_a
// float m_x_temp, m_y_temp, m_a_temp;
float magnet_area = 0;
float trust_area = 0;
bool flag_magnet_sampled = false;
float Mwidth, Mlength;
// extern float fangle, destination_angle, v1, v2;
// extern float current_temp;
// extern char fab_status[];
// extern char fab_time[];
// extern float field_x, field_y, field_z, field_mag, field_angle;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat getImage(void) {
	return img_m_color_for_display;
}

Mat getImage2(void) {
	return img_m_color_for_display2;
}

// York's contour function
bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
    double i = fabs( contourArea(Mat(contour1)) );
    double j = fabs( contourArea(Mat(contour2)) );
    return ( i < j );
}

float *get_robot_pose(void){
	//	int *centerPointCoorArray  = (int*) malloc(2*sizeof(int));
	//	printf("TEST1\n");
	while(centerP_dataSafeLock);                   // wait until change is done
		centerP_dataSafeLock = 1;
	    centerPointCoorArray[0] =  centerP_adjusted.x; // Set to global variables
	    centerPointCoorArray[1] =  480 - centerP_adjusted.y;   // do not forget 480 offset
		centerPointCoorArray[2] = angle1; // Set to global angle variable
		centerP_dataSafeLock = 0;
		// printf("x= %f, y= %f", centerPointCoorArray[0], centerPointCoorArray[1]);

	    return centerPointCoorArray;
}

float *get_cargo_pose(void){
	while(centerP_dataSafeLock);                   // wait until change is done
		centerP_dataSafeLock = 1;
		centerPointCoorArray_2[0] =  centerP_adjusted_2.x; // Set to global variables
		centerPointCoorArray_2[1] =  480 - centerP_adjusted_2.y;   // do not forget 480 offset
		centerPointCoorArray_2[2] = angle2; // Set to global angle variable
		centerP_dataSafeLock = 0;
		// printf("x= %f, y= %f", centerPointCoorArray[0], centerPointCoorArray[1]);

		return centerPointCoorArray_2;
}

float* getGoalPointCoor(void){
    if(mouse.x>0){
        goalPointCoorArray[0] = (float)mouse.x;
        goalPointCoorArray[1] = (float)(480 - mouse.y);   // Note the positive y dir.
    }
    return goalPointCoorArray;
}

void initVision(void) {
		pthread_t vthread, fthread, vthread_xz, fthread_xz;
		if(!cam_xz.initialize_xz()) {//cam is instance of FWCamera, found in FWcamera.cpp
				g_print("FW camera xz could not be found in initVision!!!\n");
				return;
		}
		usleep(1e5);
		if(!cam_xz.startGrabbingVideo_xz()) {
				g_print("FW cam xz could not grab in initVision!!!\n");
				return;
		}
		if(killVisionThread == 0) {
				g_print("Vision already running in initVision!!!\n");
		}
		usleep(1e5);
		if(killVisionThread == 1)	{
		  	killVisionThread = 0;
				pthread_create(&vthread_xz, NULL, visionThread_xz, NULL);  //start vision thread
		}

		// X-Z Camera
		if(topcam_on == 1) {//if we are also using the top cam
				usleep(2e5);
				printf("Before cam_xy.initialize_xz().\n");
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
		return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Frame Print Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This thread is not used? UI still displays images after this is commented out.
// void* FPSprint(void*)
// {
// 	char strReceive[50];
// 	char strReceive_xz[50];
//
// 	while(!killVisionThread)   //while the image processing is running //repeat vision loop until we set killVisionthread=1 using stopVision()
// 	{
// 		//  int sprintf(char *str, const char *format, ...) sends formatted output to a string pointed to by str.
// 		sprintf(strReceive, "%.1f", fpsReceive); //writes into strRecieve the frames per second
// 		sprintf(strReceive_xz, "%.1f", fpsReceive_xz);
// 		gdk_threads_enter();
// 		gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive); //draw on the gui
// 		gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
// 		gdk_threads_leave();
// 		usleep((int)1e6); //sets frame rate display frequency
// 	}
// 	//printf("In the test zone!\n");
// 	sprintf(strReceive, "N/A"); //when we turn off the vision, write N/A into the gui
// 	sprintf(strReceive_xz, "N/A");
// 	gdk_threads_enter();
// 	gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive);
// 	gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
// 	gdk_threads_leave();
// }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// visionThread --- Camera 1
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void* visionThread(void*) {
	printf("@ the Beginning of visionThread().\n");

	int index = 0;
	unsigned char *inImage;																											// = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	Mat img_m, img_m_color, img_m_gray;
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

	RotatedRect rotated_bounding_rect;
	Point centerP;
	Point l_centerP_2;
																			// for threshold and rectangle detection
	// Time
	int i;
	timeval tStart, tEnd;
	float time;
	double current_time;
	float fpsVec[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex = 0;
	char x_str[5]; char y_str[5]; char ang_str[5];

	while(!killVisionThread) {																						//repeat vision loop until we set killVisionthread=1 using stopVision()
		gettimeofday(&tStart, NULL);
		//usleep(6e4); 																													//slow down vision thread; this function watis for a new frame, it takes a long time, so we can do some image processing in this thread

		inImage = cam.grabAframe(); 																						//unsigned char *inImage;
		if(inImage == NULL)	{
			g_print("Error in firewire stream! Reattempting...\n");
			usleep((int)1e3); 																										// I don't know what the wait delay should be
		}
		img_m = Mat(height, width, CV_8UC1, inImage); 													//convert to Mat format

/*
			Mat hsv;
			Mat mask;
			vector<vector<Point> > cnts;

			cvtColor(img_m_color,hsv,COLOR_BGR2HSV);
			inRange(hsv,Scalar(0,0,0),Scalar(60,60,60),mask);
			findContours(mask,cnts,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point(0,0));

			if ((int)cnts.size()>0){
				sort(cnts.begin(), cnts.end(),compareContourAreas);
				//printf("size: %d\n", (int)cnts.size());
				vector<RotatedRect> output(cnts.size());

				for( int i = 0; i < cnts.size(); i++ ){
					output[i] = minAreaRect(cnts[i]);
				}

				drawContours(img_m_color,cnts,0,Scalar(0,0,255),2);

				float r_angle=-output[0].angle;
				r_x=output[0].center.x;
				r_y=output[0].center.y;
				float r_w=output[0].size.width;
				float r_h=output[0].size.height;

				if(r_w<r_h){
					r_angle=r_angle+90;
				} else {
					r_angle=r_angle;
				}

				// printf("Angle: %.2f\n",r_angle);
				// printf("X Position: %.2f\n",r_x);
				// printf("Y Position: %.2f\n",r_y);


			//	circle( img_m_color, Point((int)r_x,(int)r_y), 4, Scalar(  200, 50, 0), 2, 8, 0 );
				sprintf(x_str, "%f", r_x); sprintf(y_str, "%f", r_y); sprintf(x_str, "%f", r_angle);

				putText(img_m_color,"Cargo",Point((int)r_x,(int)r_y),FONT_HERSHEY_SIMPLEX,0.5,255,2);
				putText(img_m_color,"X:",Point(100,120),FONT_HERSHEY_SIMPLEX,0.5,255,2);
				putText(img_m_color,"Y:",Point(100,150),FONT_HERSHEY_SIMPLEX,0.5,255,2);
				putText(img_m_color,"Angle:",Point(100,180),FONT_HERSHEY_SIMPLEX,0.5,255,2);

				putText(img_m_color,x_str,Point(150,120),FONT_HERSHEY_SIMPLEX,0.5,255,2);
				putText(img_m_color,y_str,Point(150,150),FONT_HERSHEY_SIMPLEX,0.5,255,2);
				putText(img_m_color,ang_str,Point(150,180),FONT_HERSHEY_SIMPLEX,0.5,255,2);
			}else {
				break;
			}



*/
	blur( img_m, threshold_output, Size(4,4) ); //blur image to remove small blips etc
	threshold( threshold_output, threshold_output, visionParam1, 255, THRESH_BINARY_INV );


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
		centerP_adjusted = Point( bounding_rect.x + bounding_rect.width/2, (bounding_rect.y + bounding_rect.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
		centerP_dataSafeLock = 0;

		rotated_bounding_rect = minAreaRect( Mat( contours[largest_contour_index] ) );
		angle1=rotated_bounding_rect.angle;//angle 1 is the angle of the robot
		/*if (bounding_rect.width<bounding_rect.height){
			angle1=0-angle1+M_PI_2;
		} else {
			angle1=0-angle1;
		}
		*/
		// printf("Robot Pose: (%d, %d, %f): \n",centerP_adjusted.x, centerP_adjusted.y, angle1);
	}

	cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color anyways
	circle(img_m_color, centerP_adjusted, 40, Scalar(0, 50, 200), 2, 8, 0 );

	/////   this is for the case there are two separate objects
	if (contours.size() >= 2){
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
				centerP_adjusted_2 = Point( bounding_rect.x + bounding_rect.width/2, (bounding_rect.y + bounding_rect.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
				second_rotated_bounding_rect = minAreaRect( Mat(contours[i]) );
				angle2=second_rotated_bounding_rect.angle;//angle2 is the angle of the cargo
				// printf("Cargo Pose: (%d, %d, %f): \n",centerP_adjusted_2.x, centerP_adjusted_2.y, angle2);
			}
		}
		circle( img_m_color, centerP_adjusted_2, 40, Scalar(0, 200, 50), 2, 8, 0 );
	}

/////////////////////////////////////////////////////////
/*
for(i = 0; i< (contours.size()); i++ )
{
//	continue; //don't draw any rects
	Scalar color = Scalar( (int)i*254/(contours.size()+1),255- (int)i*254/(contours.size()+1), (int)i*254/(contours.size()+1) );
   drawContours( img_m_color, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
	//rectangle( img_m_color, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 ); //Commented out by Patrick Feb 27
}

*/
		if(mouse.x>0)
			circle( img_m_color, mouse, 4, Scalar(  200, 50, 0), 2, 8, 0 );

		img_m_color_for_display = img_m_color;
		draw_digital_arena(&img_m_color_for_display); // Draw arena


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
		}																																						//end vision loop due to killVisionthread==1
		cam.stopGrabbingVideo();
		usleep ((int)1e5); //make sure that ImageProc_xz has closed
		cam.deinitialize();
		printf("@ the End of visionThread().\n");
		return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// visionThread_xz --- Camera 2   * refer to openCV *
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void* visionThread_xz(void*) {
		printf("In vision thread x-z.\n");

		unsigned char *inImage_xz;
		Mat img_m_xz, img_m_color_xz;
		Mat threshold_output_xz; //for threshold and rectangle detection

		// Time
		int i;
		timeval tStart_xz, tEnd_xz;
		float time_xz;
		double current_time_xz;
		float fpsVec_xz[10] = {10,10,10,10,10,10,10,10,10,10};
		int fpsIndex_xz = 0;

		// Local variables
		Mat img_m_xz_ori; // storing the original image of xz
		Mat img_m_xz_bi;  // storing the binary image of xz
		Mat img_m_xz_temp;
		float magnet_maxarea, pre_area = 100.0;
		int ind_maxarea;
		bool contour_number = false; // whether the number of contours is greater than 2 ?

		while(!killVisionThread) {																											    //repeat vision loop until we set killVisionthread = 1 using stopVision()
			gettimeofday(&tStart_xz, NULL);
			//usleep(6e4); 																																	//slow down vision thread

			inImage_xz = cam_xz.grabAframe_xz(); 																						//unsigned char *inImage;
			if(inImage_xz == NULL) {
					g_print("Error in firewire stream xz! Reattempting...\n");
					usleep((int)1e3); 																													// I don't know what the wait delay should be
			}

			img_m_xz = Mat(height, width, CV_8UC1, inImage_xz); 														//convert to Mat format

			// opencv image processing
			if(edgemap_xz == 1) {img_m_xz = opencv_edgemap (img_m_xz.clone(), cannyLow_xz, cannyHigh_xz, dilater_xz);}
			if(binary_xz == 1) {img_m_xz = opencv_binary (img_m_xz.clone(), binaryThreshold_xz);}
			cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); 				//convert to color

			// draw field indicator
			if (flag2dIndicatorXZ == 1) {draw_xz_magnetic_field(img_m_color_xz,580,400);}
			if (flag3dIndicatorXZ == 1) {
				if (currentActiveTabIndex == 1) {
					draw_3d_magnetic_field_twisted(img_m_color_xz,480,400);
				}else{
					draw_3d_magnetic_field(img_m_color_xz,480,400);
				}
			}

			img_m_color_for_display2 = img_m_color_xz;

			gettimeofday(&tEnd_xz, NULL);
			current_time_xz = ((double)tEnd_xz.tv_sec + (double)tEnd_xz.tv_usec*1e-6) ;
			time_xz = (int)round( (((double)tEnd_xz.tv_sec + (double)tEnd_xz.tv_usec*1e-6) - ((double)tStart_xz.tv_sec + (double)tStart_xz.tv_usec*1e-6) )*1000.0);
			fpsVec_xz[fpsIndex_xz++] = 1000.0/time_xz;
			if(fpsIndex_xz > 9) fpsIndex_xz = 0;
			fpsReceive_xz = 0;
			for(int i = 0; i < 10; i++)
			fpsReceive_xz += fpsVec_xz[i];
			fpsReceive_xz /= 10.0;
		}																																												//end vision loop due to killVisionthread==1
		cam_xz.stopGrabbingVideo_xz();
		usleep ((int)1e5); 																																			//make sure that ImageProc_xz has closed
		cam.deinitialize(); 																																		//taken care of by top camera deinitialize call
		printf("@ the End of visionThread_xz().\n");
		return NULL;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Camera   ------------------> callback.c
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
void set_binaryThreshold_vision (int d)
{
	binaryThreshold = d; //for image processing
}
void setvisionParam2_vision(int d)
{
	visionParam2 = d; //for image processing
}
void set_3d_indicator_flag (int d)
{
	flag3dIndicator = d;
}
void set_2d_indicator_flag (int d)
{
	flag2dIndicator = d;
}
void setcannyLow_vision(int d)
{
	cannyLow = d; //for image processing
}
void setcannyHigh_vision(int d)
{
	cannyHigh = d; //for image processing
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera   ------------------> callback.c
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setTopCam_vision(int d)
{
	topcam_on = d; //is the topcam on?
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
	cam_xz.setGain_xz(d);
}
void setShutter_xz_vision(int d)
{
	cam_xz.setShutter_xz(d);
}
void setDilate_xz_vision(int d)
{
	dilater_xz = d; //for image processing on edgemap
}
void set_binaryThreshold_xz_vision(int d)
{
	binaryThreshold_xz = d; //for image processing
}
void setvisionParam2_xz_vision(int d)
{
	visionParam2 = d; //for image processing
}
void set_3d_indicator_xz_flag (int d)
{
	flag3dIndicatorXZ = d;
}
void set_2d_indicator_xz_flag (int d)
{
	flag2dIndicatorXZ = d;
}
void setcannyLow_xz_vision(int d)
{
	cannyLow_xz = d; //for image processing
}
void setcannyHigh_xz_vision(int d)
{
	cannyHigh_xz = d; //for image processing
}
void settopcam_xz_vision(int d)
{
	topcam_on = d; //is the sidecam on?
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Mouse --- Zhe
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
				case 2: //centre mouse
					mouseC_xz.x 		= mouseClick[0];
					mouseC_xz.y 		= mouseClick[1];
					break;
				case 3: //right mouse
					mouseR_xz.x 		= mouseClick[0];
					mouseR_xz.y		= mouseClick[1];
					break;
			}
			break;
	}
}

// Chris's function to add digital arena
// static void draw_digital_arena(Mat img, float wall_width, float wall_length, float bound_length, float bound_gap){
// 	line(img, Point(0,0), Point(wall_length,0), Scalar(255,0,0), 5); // Top wall
// 	line(img, Point(0,wall_width), Point(wall_length,wall_width), Scalar(255,0,0), 5); // Bottom wall
// 	line(img, Point(0,0), Point(0,wall_width), Scalar(255,0,0), 5); // Left wall
// 	line(img, Point(wall_length,0), Point(wall_length,wall_width), Scalar(255,0,0), 5); // Right wall
//
// 	line(img, Point(wall_length/2,0), Point(wall_length/2,bound_length), Scalar(255,0,0), 5);
// 	line(img, Point(wall_length/2,bound_length+bound_gap), Point(wall_length/2,2*bound_length+bound_gap), Scalar(255,0,0), 5);
// }

// Jiachen's function to add digital arena
static void draw_digital_arena (Mat* data ) {
    /* inner top-left corner @ (7, 61), 5.6 um/pixel */
    rectangle ( *data, Point(  7, 61), Point(632,418), Scalar(255,0,0) );     // length: 3.5 mm (625 pixels), height: 2 mm (357 pixel)
    rectangle ( *data, Point(311, 61), Point(320,106), Scalar(255,0,0) );     // length: 50 um; height: 250 um
    rectangle ( *data, Point(311,177), Point(320,240), Scalar(255,0,0) );     // length: 50 um; height: 350 um
    line      ( *data, Point(  0, 52), Point(639, 52), Scalar(255,0,0) );
    line      ( *data, Point(  0,427), Point(639,427), Scalar(255,0,0) );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OpenCV Static Functions   --- Tianqi
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void draw_xy_magnetic_field (Mat img, float oX, float oY){
		float r = 40;
		float angleXY = atan2(field_y, field_x);
		float fullMag = 14.0; //when field magnitude equals to this value, the end point of the vector is on the sphere

		//draw figure
		Point startP(oX,oY);
		Point endP(oX + r * cos(angleXY), oY - r * sin(angleXY));
		line( img, startP, endP, Scalar(255,0,0),2);
		endP.x = oX + r * field_x / fullMag;
		endP.y = oY - r * field_y / fullMag;
		circle( img, endP, 2, Scalar(255,0,0),2);
		circle( img, startP, r, Scalar(255,0,0));

		//put text
		Point textpoint;
		textpoint.x = oX - r / 4;
		textpoint.y = oY + 1.5 * r;
		char text[] = "XY";
		putText( img, text, textpoint, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0));
}

static void draw_xz_magnetic_field (Mat img, float oX, float oY){
		float r = 40;
		float angleXZ = atan2(field_z, field_x);
		float fullMag = 14.0; //when field magnitude equals to this value, the end point of the vector is on the sphere

		//draw figure
		Point startP(oX,oY);
		Point endP(oX + r * cos(angleXZ), oY - r * sin(angleXZ));
		line( img, startP, endP, Scalar(255,0,0),2);
		endP.x = oX + r * field_x / fullMag;
		endP.y = oY - r * field_z / fullMag;
		circle( img, endP, 2, Scalar(255,0,0),2);
		circle( img, startP, r, Scalar(255,0,0));

		//put text
		Point textpoint;
		textpoint.x = oX - r / 4;
		textpoint.y = oY + 1.5 * r;
		char text[] = "XZ";
		putText( img, text, textpoint, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0));
}

static void draw_3d_magnetic_field (Mat img, float oX, float oY){
		float r = 40;
		float axisMajor = 2 * r;
		float axisMinor = 0.5 * r;
		float angleXY = atan2(field_y, field_x);
		float mag = field_mag ? field_mag : 14.0;
		float magXY = sqrt(pow(field_x,2) + pow(field_y,2));
		float p = magXY / mag;
		float fullMag = 14.0; //when field magnitude equals to this value, the end point of the vector is on the sphere

		//draw figure
		Point startP(oX,oY);
		Point endP;
		endP.x = oX + (axisMajor * 0.5) * p * cos(angleXY);
		endP.y = oY - (axisMinor * 0.5) * p * sin(angleXY) - r * field_z / mag;
		RotatedRect rRect = RotatedRect(Point2f(oX,oY), Size2f(axisMajor,axisMinor), 0);																	// ellipse minimum bounding rect
		RotatedRect rRectXY = RotatedRect(Point2f(oX,oY - r * field_z / mag), Size2f(axisMajor * p, axisMinor * p), 0);
		line( img, startP, endP, Scalar(255,0,0), 2);
		ellipse( img, rRect, Scalar(255,0,0));
		ellipse( img, rRectXY, Scalar(128,128,0));
		endP.x = oX + (axisMajor * 0.5) * p * cos(angleXY) * (mag / fullMag);
		endP.y = oY - (axisMinor * 0.5) * p * sin(angleXY) * (mag / fullMag) - r * field_z / fullMag;
		circle( img, endP, 2, Scalar(255,0,0), 2);
		circle( img, startP, r, Scalar(255,0,0));

		//put text
		Point textpoint;
		textpoint.x = oX - r / 3;
		textpoint.y = oY + 1.5 * r;
		char text[] = "XYZ";
		putText( img, text, textpoint, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0));
}

static void draw_3d_magnetic_field_twisted (Mat img, float oX, float oY){
		float r = 40;
		float axisMajor = 2 * r;
		float axisMinor = 0.5 * r;
		float angleXY = atan2(field_y, field_x);
		float mag = field_mag ? field_mag : 14.0;
		float magXY = sqrt(pow(field_x,2) + pow(field_y,2));
		float p = magXY / mag;
		float fullMag = 14.0; //when field magnitude equals to this value, the end point of the vector is on the sphere

		//draw figure
		Point startP(oX,oY);
		Point endP;
		endP.x = oX + (axisMajor * 0.5) * p * cos(angleXY);
		endP.y = oY - (axisMinor * 0.5) * p * sin(angleXY) - r * field_z / mag;
		RotatedRect rRect = RotatedRect(Point2f(oX,oY), Size2f(axisMajor,axisMinor), 0);
		RotatedRect rRectXY = RotatedRect(Point2f(oX+r*cosd(phi/2)*sind(beta),	oY-r*cosd(phi/2)*cosd(beta)),
																			Size2f(axisMajor*sind(phi/2), axisMinor*sind(phi/2)), beta);
		// RotatedRect rRectXY = RotatedRect(Point2f(oX+r*cosd(phi/2)*sind(beta)*cosd(theta),
		// 																					oY-r*cosd(phi/2)*cosd(beta)-(axisMinor * 0.5) * p *sind(theta)),
		// 																	Size2f(axisMajor*sind(phi/2), axisMinor*sind(phi/2)),
		// 																	beta*cosd(theta));
		line( img, startP, endP, Scalar(255,0,0), 2);
		ellipse( img, rRect, Scalar(255,0,0));
		ellipse( img, rRectXY, Scalar(128,128,0));
		endP.x = oX + (axisMajor * 0.5) * p * cos(angleXY) * (mag / fullMag);
		endP.y = oY - (axisMinor * 0.5) * p * sin(angleXY) * (mag / fullMag) - r * field_z / fullMag;
		circle( img, endP, 2, Scalar(255,0,0), 2);
		circle( img, startP, r, Scalar(255,0,0));

		//put text
		Point textpoint;
		textpoint.x = oX - r / 3;
		textpoint.y = oY + 1.5 * r;
		char text[] = "XYZ";
		putText( img, text, textpoint, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0));
}

static Mat opencv_edgemap (Mat img, int cannyLow, int cannyHigh, int dilater) {
	Canny(img, img, cannyLow, cannyHigh, 3 ); //edge detect
	if(dilater > 0) {																										//if dilater = 0, just use original edgemap
		dilate( img, img, Mat(), Point(-1, -1), dilater, 1, 1);
		//smooth( img, img, CV_MEDIAN, 5, 5);
		erode( img, img, Mat(), Point(-1, -1), dilater, 1, 1);
	}
	//circle( img, MM, 10, Scalar(20,100,255) , -1, 8, 0 );	          // Test Hough circle detection mode
	return img;
}

static Mat opencv_binary (Mat img, int binaryThreshold) {
	blur( img, img, Size(4,4) ); 												//blur image to remove small blips etc
	threshold( img, img, binaryThreshold, 255, THRESH_BINARY );
	return img;
}
