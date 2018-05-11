#include "vision.hpp"

/* file-wide variable */
static int width = 640;   //image width, pixels
static int height = 480;  //image height, pixels

static int fThread = 0;             // flag of whether or not thread is running
static Mat raw_frame = Mat(1024,1280,CV_8UC3);
static int raw_frame_lock = 0;

static int centerP_dataSafeLock = 0;                    // 1: centerP is being changed, wait until it is done
static float centerPointCoorArray[3]  = {320, 240, 0};  // Current pose of robot
static float centerPointCoorArray_2[3]  = {320, 140, 0}; // Current pose of cargo
static float angle1, angle2;                                    // robot angle ?

static float* click_vision_pos[1];
char buffer[64];
static Point centerP_adjusted;
static Point centerP_adjusted_2;

static Point mouse, mouseC, mouseR;
static float goalPointCoorArray[2]   = {320, 240};

static int visionParam1 = 65; //for processing. Used in threshold() and houghCircle().
static int visionParam2 = 35; //for processing

//static FWcamera cam; // create a new instance of FWcamera
static Mat presentFrame = Mat(480,640,CV_8UC3);            // present captured frame
static Mat frameForDisplay = Mat(480,640,CV_8UC3);         // different with presentFrame by annotation/drawing
static int frame_for_display_lock = 0;
static int copyLock = 0;

// York's contour function
static bool compareContourAreas ( std::vector<Point> contour1, std::vector<Point> contour2 ) {
    double i = fabs( contourArea(Mat(contour1)) );
    double j = fabs( contourArea(Mat(contour2)) );
    return ( i < j );
}

float *get_robot_pose(void){
	//	int *centerPointCoorArray  = (int*) malloc(2*sizeof(int));
	//	printf("TEST1\n");
	while (centerP_dataSafeLock);                   // wait until change is done
		centerP_dataSafeLock = 1;
	    centerPointCoorArray[0] =  centerP_adjusted.x; // Set to global variables  UNCOMMENT
	    // centerPointCoorArray[1] =  480 - centerP_adjusted.y;   // do not forget 480 offset UNCOMMENT
		centerPointCoorArray[1] =  centerP_adjusted.y;
		// centerPointCoorArray[0] = 520; // Test x coordinate
		// centerPointCoorArray[1] = 360; // Test y coordinate

		centerPointCoorArray[2] = angle1; // Set to global angle variable
		centerP_dataSafeLock = 0;
		// printf("x= %f, y= %f", centerPointCoorArray[0], centerPointCoorArray[1]);
	    return centerPointCoorArray;
}


float *get_cargo_pose(void){
	while(centerP_dataSafeLock);                   // wait until change is done
		centerP_dataSafeLock = 1;
		centerPointCoorArray_2[0] =  centerP_adjusted_2.x; // Set to global variables
		// centerPointCoorArray_2[1] =  480 - centerP_adjusted_2.y;   // do not forget 480 offset
		centerPointCoorArray_2[1] = centerP_adjusted_2.y;   // do not forget 480 offset

		// centerPointCoorArray_2[0] = 120; // Test x coordinate
		// centerPointCoorArray_2[1] = 100; // Test y coordinate

		// centerPointCoorArray_2[2] = 0;
		// centerPointCoorArray_2[2] = angle2; // Eventually use for CV
		centerP_dataSafeLock = 0;
		// printf("x= %f, y= %f", centerPointCoorArray[0], centerPointCoorArray[1]);
		return centerPointCoorArray_2;
}

float * getGoalPointCoor(void){
    if(mouse.x>0){
        goalPointCoorArray[0] = (float)mouse.x*1280/640;
        goalPointCoorArray[1] = (float)mouse.y*1024/480;
        // goalPointCoorArray[0] = (float)mouse.x;
        // goalPointCoorArray[1] = (float)mouse.y;
        // goalPointCoorArray[1] = (float)(480 - mouse.y);   // Note the positive y dir. Note resolution difference between cameras
    }
    return goalPointCoorArray;
}


int GetImage(void *buffer, unsigned int buffersize, void *context){
    if(buffersize > 0){ // [Tianqi] <= 0 indicates a bad frame is coming. e.g. the first few frames after initializing the camera
        Mat temp = Mat(1024,1280, CV_8UC3, buffer);
        while (raw_frame_lock);
        raw_frame_lock = 1;
        temp.copyTo(raw_frame);
        raw_frame_lock = 0;
    }
}

int isFrameValid (void) {
    while (raw_frame_lock);
    raw_frame_lock = 1;
    Size s = raw_frame.size();
    int rows = s.height;
    int cols = s.width;
    raw_frame_lock = 0;

    if (rows > 0 && cols > 0){
        return 1;
    } else{
        //printf("frame not valid.\n");
        return 0;
    }
}

int initCamera(int CAM_ID){
    int result;
    result = NETUSBCAM_Init();		// look for ICubes
	if(result==0){
		printf("No device\n");
		return 0;
    }
	result = NETUSBCAM_Open(CAM_ID);	// open camera
	if(result!=0){
		printf("Error: Open; Result = %d\n", result);
		return 0;
    }
    // set the camera clock lower, if a lot of bad frames arriving
	result = NETUSBCAM_SetCamParameter(CAM_ID,REG_PLL,20);
	if(result!=0){
		printf("Error: REG_PLL; Result = %d\n", result);
		return 0;
    }
	// if active, badframes are sent to the callback with buffersize = 0
	result = NETUSBCAM_SetCamParameter(CAM_ID,REG_CALLBACK_BR_FRAMES,1);
	if(result!=0){
		printf("Error: REG_CALLBACK_BR_FRAMES; Result = %d\n", result);
		return 0;
    }
    // set the callback to get the frame buffer
  	result = NETUSBCAM_SetCallback(CAM_ID,CALLBACK_RGB,&GetImage,NULL);
  	if(result!=0){
  		printf("Error: SetCallback; Result = %d\n", result);
  		return 0;
    }
  	 // start streaming of camera
  	result = NETUSBCAM_Start(CAM_ID);
  	if(result!=0){
  		printf("Error: Start; Result = %d\n", result);
  		return 0;
    }
    return 1;
}

int closeCamera(int CAM_ID){
    int result;
    // stop streaming of camera
    result = NETUSBCAM_Stop(CAM_ID);
    if(result!=0){
      printf("Error: Stop; Result = %d\n", result);
      return 0;
    }
    // close camera
    result = NETUSBCAM_Close(CAM_ID);
    if(result!=0){
      printf("Error: Close; Result = %d\n", result);
      return 0;
    }
    return 1;
}

static void draw_circle (Mat *data, int x, int y){
	circle(*data, Point(x,y), 3, Scalar(0,255,0), -1);
}

void draw_occ_grid(Mat *data, int** &occ_grid){ // Bad function, do not use
	for(int i = 1; i <= width; i++){
		for(int j = 1; j <= height; j++){
			if(occ_grid[j-1][i-1] == 0){
				// printf("reached?\n");
				circle(*data, Point(i,j), 1, Scalar(255,0,0), -1);
			}
		}
	}
}

static void draw_path (Mat *data, stack<Pair> Path){
	Pair p; // Iterator of pairs
	while(!Path.empty()){
		p = Path.top(); Path.pop();
		circle(*data, Point(p.second,p.first), 1, Scalar(255,0,0), -1);
	}
}

static void* video_stream_THREAD ( void *threadid ) {
    printf("at the start of video_stream_THREAD.\n");
    // Vision_Master myVision;
    //int i = 0;
    if(!initCamera(0)){
        printf("error in init. camera\n");
    }
    usleep(1e6);
    /* variable */
    timeval tStart, tEnd;

    /* Initiate occupancy grid with 1s (unoccupied) */
	for ( int i = 0; i < ROW; ++i ) {
	    occ_grid[i] = new int[COL];
	}
	for(int i = 0; i < ROW; ++i){
	    for(int j = 0; j < COL; ++j){
	        occ_grid[i][j] = 1;
	    }
	}

	printf("Initialized occupancy grid\n");

    while ( fThread ) {
        //printf("start of loop\n");
        if ( !isFrameValid() ) continue; // continue if receiving an empty frame
        //gettimeofday(&tStart, NULL);
        //imshow("camera",raw_frame);
        //char c = (char)waitKey(15); // duration of the frames (ms)
        //if (c == 27) break; // exit when pressing ESC key

        /*

        */

        /*  */
        Mat gray_frame;
        while (raw_frame_lock);
        raw_frame_lock = 1;
        cvtColor(raw_frame,gray_frame,CV_RGB2GRAY);     // ? not sure the src is RGB or other
        raw_frame_lock = 0;

        Mat threshold_output;
        blur( gray_frame, threshold_output, Size(4,4) );
        threshold( threshold_output, threshold_output, visionParam1, 255, THRESH_BINARY_INV );

        std::vector<std::vector<Point> > contours; //for threshold and rectangle detection
        std::vector<Vec4i> hierarchy; //for threshold and rectangle detection

        findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); //find contours

        std::vector<std::vector<Point> > contours_poly( contours.size() );
        std::vector<Rect> boundRect( contours.size() );
        std::vector<RotatedRect> minRect( contours.size() );

        int largest_area = 0;

        if (contours.size() >= 1) {   // if the camera detects rectangle ... sometimes the view is all white, no rectangles are detected
            for ( int i = 0; i < contours.size(); i++ ) {
                //
                minRect[i] = minAreaRect( Mat(contours[i]) );
                //
                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            }

            //first, find the largest contour    ...   Contour classification!
    		largest_area = 0;
            int largest_contour_index = 0;

    		for(int i = 0; i< (contours.size()); i++ ) {
    			double a = contourArea(contours[i], false);           //  Find the area of contour
    			if(a > largest_area) {                          // if current contour is bigger ...
    				largest_area = a;
    				largest_contour_index = i;                 //Store the index of largest contour
    			}
    		}

            Rect bounding_rect;
            bounding_rect = boundingRect( contours[largest_contour_index] );                                       // Find the bounding rectangle for biggest contour
    		//printf("Marker 5.\n");

    		Point centerP = Point( bounding_rect.x + bounding_rect.width/2, bounding_rect.y + bounding_rect.height/2 );  // Center point of the bounding rectangle

            while (centerP_dataSafeLock);   // wait until reading is done
    		centerP_dataSafeLock = 1;
    		centerP_adjusted = Point( bounding_rect.x + bounding_rect.width/2, (bounding_rect.y + bounding_rect.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
    		centerP_dataSafeLock = 0;

            RotatedRect rotated_bounding_rect = minAreaRect( Mat( contours[largest_contour_index] ) );
    		angle1=rotated_bounding_rect.angle;//angle 1 is the angle of the robot
        }

        Mat color_frame;
        cvtColor ( gray_frame, color_frame, CV_GRAY2BGR); //convert to color anyways
        circle (color_frame, centerP_adjusted, 40, Scalar(0, 50, 200), 2, 8, 0 ); // Draw blue circle where robot is

        /////   this is for the case there are two separate objects
    	if (contours.size() >= 2){
    		int second_largest_area = 0;
            int second_largest_contour_index = 0;
    		for(int i = 0; i< (contours.size()); i++) {
    			double a = contourArea(contours[i], false);  //  Find the area of contour
    			if ( (a > second_largest_area) && (a < largest_area) )
    			{
    				second_largest_area = a;
    				second_largest_contour_index = i;                 //Store the index of largest contour
    				Rect bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
    				//l_centerP_2        = Point( bounding_rect.x + bounding_rect.width/2, bounding_rect.y + bounding_rect.height/2 );  // Center point of the bounding rectangle
    				centerP_adjusted_2 = Point( bounding_rect.x + bounding_rect.width/2, (bounding_rect.y + bounding_rect.height/2) );  // Center point of the bounding rectangle. note the positive y dir.
    				RotatedRect second_rotated_bounding_rect = minAreaRect( Mat(contours[i]) );
    				angle2=second_rotated_bounding_rect.angle;//angle2 is the angle of the cargo
    				// printf("Cargo Pose: (%d, %d, %f): \n",centerP_adjusted_2.x, centerP_adjusted_2.y, angle2);
    			}
    		}
    		circle( color_frame, centerP_adjusted_2, 40, Scalar(0, 200, 50), 2, 8, 0 ); // Draw green circle where cargo is
    	}

        // Display useful position information
        click_vision_pos[0] = getGoalPointCoor();
        int ret = snprintf(buffer, sizeof buffer, "(%f, %f)", click_vision_pos[0][0], click_vision_pos[0][1]);
        putText(color_frame, buffer, Point(800,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0));
        draw_circle(&color_frame, click_vision_pos[0][0], click_vision_pos[0][1]);

        draw_circle (&color_frame, draw_x, draw_y); // Calibration point
		// draw_occ_grid(&img_m_color_for_display, occ_grid);
		draw_path (&color_frame, Path_vision);
		draw_circle (&color_frame, click_x, click_y); // Draw where next click is

        while (frame_for_display_lock);
        frame_for_display_lock = 1;
        //raw_frame.copyTo(frameForDisplay);
        //printf("before copy\n");
        //if (raw_frame.data == NULL)
        //    printf("raw frame is null.\n");
        //resize(raw_frame, frameForDisplay, Size(640,480) );
        resize(color_frame, frameForDisplay, Size(640,480) );
        //printf("after copy\n");
        frame_for_display_lock = 0;

        my_sleep(30);           // correspond to 30 Hz camera rate
        //printf("later in loop %d\n", i++);
    }

    closeCamera(0);
    //return 0;

    //if (!fSim) {
        // cam.stopGrabbingVideo();
    	// my_sleep(100);
    	// cam.deinitialize();
        // cam.deinitialize();
    //}

    printf("at the end of video_stream_THREAD.\n");
}

/* activate the video capture and processing thread */
void camera_activate (void) {
    fThread = 1;
    pthread_t videoStreamThread;
    pthread_create( &videoStreamThread, NULL, video_stream_THREAD, NULL);  //start vision thread
}

void camera_deactivate (void) {
    fThread = 0;
}

void get_present_image ( Mat * container ) {
    while (frame_for_display_lock);
    frame_for_display_lock = 1;
    frameForDisplay.copyTo(*container);
    frame_for_display_lock = 0;
}

void stop_video_stream(void) {
    fThread = 0;
}

gboolean key_event (GtkWidget *widget, GdkEventKey *event) {
    //printf ("key pressed %s\n", gdk_keyval_name( event->keyval ) );
    int keycode = -1;

    switch (event->keyval) {
        case 65363: keycode = 0; break;
        case 65362: keycode = 1; break;
        case 65361: keycode = 2; break;
        case 65364: keycode = 3; break;
        case    32: keycode = -1; break;
    }

    //set_directionCode (keycode);
    return TRUE;
}

gboolean key_event_release (GtkWidget *widget, GdkEventKey *event) {
    return TRUE;
}

void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) {
    switch(whichMouse) {
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

}

gboolean on_eventbox1_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data) {
    int click[2];
	click[0] = (int)event->x; //x position from top left in pixels
	click[1] = (int)event->y; //y position from top left in pixels
	int button_click = event->button; //which mouse button was clicked
	//g_print("Top video window %d click at location [%d %d].\n", button_click, click[0], click[1]);
	setMouse (0, button_click, click );      //void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
}

// Jiachen's function to add digital arena
static void draw_digital_arena (Mat* data ) {
    /* inner top-left corner @ (7, 61), 5.6 um/pixel */
    rectangle ( *data, Point(  7, 61), Point(632,418), Scalar(255,0,0) );     // length: 3.5 mm (625 pixels), height: 2 mm (357 pixel)
    rectangle ( *data, Point(311, 61), Point(320,106), Scalar(255,0,0) );     // length: 50 um; height: 250 um
    rectangle ( *data, Point(311,177), Point(320,240), Scalar(255,0,0) );     // length: 50 um; height: 350 um
    line      ( *data, Point(  0, 52), Point(639, 52), Scalar(255,0,0) );
    line      ( *data, Point(  0,427), Point(639,427), Scalar(255,0,0) );
}

void draw_goal(Mat *data){
	circle(*data, Point(245,120), 18, Scalar(200, 0, 50), 2, 8, 0 );
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
