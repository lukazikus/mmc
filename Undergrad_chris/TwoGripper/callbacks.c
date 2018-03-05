#include "callbacks.h"

//magnet control

int control_running = 0;     //is the control loop running?
bool visionStarted = false;  //is the vision running?
int newSpeed;                //speed to send to motor controller. int for pololu

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer Related Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint swimmer_actuationThreadControl = 0;  // Control flag for the swimmer actuation thread
static float AO_sinFreq = 0;   // Frequency for the AO testing sine wave.
int   AO_sinFreq_dir = 1;   // Direction of the rotating field.
uint  dirIndex = 0;         // 0: +x; 1: +y; 2: -x; 3: -y

uint swimmer_controlLoop = 0;  // Control loop flag
//float swimmer_A_h = 3.0;         // Amplitude of swimmer actuation voltage - horizontal
//float swimmer_A_v = 3.0;         // Amplitude of swimmer actuation voltage - vertical

float swimmer_dir_bias = 0.0;  // Directional bias of swimmer actuation field
float swimmer_vertical_bias = 0.0;  // Vertical bias of swimmer actuation field

//float swimmer_dirAdjust_angle = 0;  // Compensation angle for net magnetization
float dirAdjust_amp = 0;    // Field direction adjustment - amplitude
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Characterization Experiment
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float angle_difference=0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Walker Related VariableS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static uint GUI_refresh_flag_vidWin1 = 0;        // 1: refresh the vidWin1 @ a certain rate
static uint GUI_refresh_flag_vidWin2 = 1;        // 1: refresh the vidWin2 @ a certain rate
static uint GUI_refresh_running = 0;             // 1: GUI refresh thread is running

static uint flag_draw_field = 0;

//micro_fab related variables -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static float glo_theta = 180.00;
static float glo_phi = 0.00;
//GtkEntry *display_fab_status;
float microrobot_shape[51];
int line_number = 0; // line numbers of microrobot_shape.txt
//bool flag_auto_fab = false;
//extern bool flag_test90, flag_temp_control, flag_auto_feeding;
extern char fab_status[];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//static float freq  = 1.0;   //field waveform frequency
static float amp   = 400;   //field waveform amplitude
//calibrate
static float zout  = 2.50;  	//zero output of the oscilloscope
static float gout  = 30.0;  	//output of the gaussmeter
static float osout = 3.00 ; 	//output of the oscilloscope
static float k     = 3.00;  	//sensitivity of the sensors
static float osout_value = 3.00 ; //output of the oscilloscope
static float value = 3.00;  	//the final value

//////////////////////////////////////////////////////////////////////////////////////////////
//coil setup selection variables
static int flag_3coil_setup = 1;     // default:: active mode
static int flag_2coil_setup = 0;     // default:: disable mode
//////////////////////////////////////////////////////////////////////////////////////////////




//Permanent Magnet Initialization
int  step_size = 0.0;
int  fileinput_frequency = 10.0;
static float Bo[3]  = {0.0, 0.0, 0.0};         //B vector
static float Fo[3]  = {0.0, 0.0, 0.0};         //F vector

/*
int motor_parm[8][3] = {  // {step, dir, enable}
			{18,19,20},
			{9, 10, 11},
			{17, 16, 15},
			{5, 4, 3},
			{14, 13, 12},
			{22, 23, 20}, //shares enable pin with 1!
			{2,1,0},
			{6, 7, 8}};

//AccelStepper parameters

float max_speed = 100; //500;
float max_accel = 1300; //4500;


AccelStepper stepper1(1, motor_parm[1-1][0], motor_parm[1-1][1]);
AccelStepper stepper2(1, motor_parm[2-1][0], motor_parm[2-1][1]);
AccelStepper stepper3(1, motor_parm[3-1][0], motor_parm[3-1][1]);
AccelStepper stepper4(1, motor_parm[4-1][0], motor_parm[4-1][1]);
AccelStepper stepper5(1, motor_parm[5-1][0], motor_parm[5-1][1]);
AccelStepper stepper6(1, motor_parm[6-1][0], motor_parm[6-1][1]);
AccelStepper stepper7(1, motor_parm[7-1][0], motor_parm[7-1][1]);
AccelStepper stepper8(1, motor_parm[8-1][0], [8-1][1]);
AccelStepper stepper8(1, motor_parm[8-1][0], motor_parm[8-1][1]);

bool end_runmotorsthread = false;
float motor_steps2go[8]={0,0,0,0,0,0,0,0};
*/

float max_v = 80;//200;
float max_a = 1300;//1300;

bool motorgo = false;

int nmagnets = 0;
int use_magnets[8]={0,0,0,0,0,0,0,0};

float magnet_rotD[8]={0,0,0,0,0,0,0,0}; //magnet rotation in degrees
float magnet_rotR[8]={0,0,0,0,0,0,0,0}; //magnet rotation in radians
float motor_position[8]={0,0,0,0,0,0,0,0};

int magnet_flag=0;
bool update_magnetangles = false;


bool record_on = 0;
 FILE *fp;

static cairo_surface_t *surface = NULL;

// ??? C does not support "vector" - JZ
static vector <bool> keyPressed(0xffff); //vector of bools containing the key pressed states of all keys indexed by this: https://git.gnome.org/browse/gtk+/plain/gdk/gdkkeysyms.h

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI Variable
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GtkWidget	*coildraw, *field_drawingArea;
GtkImage 	*videoWindow, *videoWindow2;
GtkLabel    *label_centerP_x, *label_centerP_y;
GtkLabel    *label_centerP_x1, *label_centerP_y1;                        //  appeared in multi_agent vbox!
GtkLabel    *label_centerPx_mm, *label_centerPy_mm;

GtkLabel   *label_info_type, *label_info_plane, *label_info_angle;   // Label near camera image, showing info. about the image

float GUI_field_angle = 0;   // Storing the angle of current field for GUI display

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////





static int flag_second_rect_required = 0;   // Flag for 2 rectangles detection in Camera - 1
static int flag_orinet_circ_required = 0;   // Flag for 1st orient circles detection in multi-agent project


uint swimmer_heading_test = 0;



///////////////////////////////
// Multi-agent variables
//////////////////////////////

static float adj_angle_map;




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI Refresh Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////





/// Update the info. label with field driving dir.
static gboolean update_driving_dir (gpointer userdata)
{
    char l_str[20];

    sprintf(l_str, "Driving Field Dir.");
    gtk_label_set_text (label_info_type, l_str);

    int l_driving_dir = get_directional_index();   // "l" in variable's name stands for "local"

    switch (l_driving_dir)
    {
        case 0: sprintf(l_str, "+x"); break;
        case 1: sprintf(l_str, "+y"); break;
        case 2: sprintf(l_str, "-x"); break;
        case 3: sprintf(l_str, "-y"); break;
        default: sprintf(l_str, "Er"); printf("!!! Error in update_driving_dir().\n"); break;
    }

    gtk_label_set_text (label_info_plane, l_str);
    return G_SOURCE_REMOVE;
}


static gboolean update_GUI_time (gpointer userdata)
{
    time_t rawtime;
    struct tm *info;
    char buffer[80];

    time( &rawtime );

    info = localtime( &rawtime );
    //printf("Current local time and date: %s", asctime(info));

    double l_time_ms;
	struct timeval l_time_start;
	gettimeofday(&l_time_start, NULL);
    l_time_ms = (double) l_time_start.tv_usec*1e-3 ; // Current time

    //printf("ms is %d.\n", (int)l_time_ms);
    sprintf(buffer, "%sms: %d", asctime(info), (int)l_time_ms);

    gtk_label_set_text (label_info_angle, buffer);
    return G_SOURCE_REMOVE;
}

static gboolean update_centerPointLabel (gpointer userdata)
// gboolean: A standard boolean type. Variables of this type should only contain the value TRUE or FALSE.
{
    //printf("Inside update_text2().\n");
    int *centerP_coor;                     // Pointer to the array storing coordinates of center point.
    centerP_coor = getCenterPointCoor();   // in vision.c

    char str_centerPointCoor[50];
    //strcpy (str_centerPointCoor, "Center Point Coor. x: ");
    char str_temp[4];
    sprintf(str_temp, "%d ", centerP_coor[0]);
    gtk_label_set_text (label_centerP_x, str_temp);
    //strcat (str_centerPointCoor, str_temp);
    //strcat (str_centerPointCoor, "y: ");
    sprintf(str_temp, "%d", centerP_coor[1]);
    //strcat (str_centerPointCoor, str_temp);
    //
    //double swimmer_angle = get_swimmer_angle();
    //char str_temp2[6];
    //sprintf(str_temp2, "%.1f", swimmer_angle);
    //strcat()
    //

    gtk_label_set_text (label_centerP_y, str_temp);
    //printf("Leaving update_text2().\n");
    return G_SOURCE_REMOVE;
}


    static gboolean update_centerPointLabel_MA (gpointer userdata)
// gboolean: A standard boolean type. Variables of this type should only contain the value TRUE or FALSE.
{
    //printf("Inside update_text2().\n");
    int *centerP_coor_MA;                     // Pointer to the array storing coordinates of center point.
    centerP_coor_MA = getCenterPointCoor();   // in vision.c


    char str_temp_MA[4];
    sprintf(str_temp_MA, "%d ", centerP_coor_MA[0]);
    gtk_label_set_text (label_centerP_x1, str_temp_MA);

    sprintf(str_temp_MA, "%d", centerP_coor_MA[1]);
    gtk_label_set_text (label_centerP_y1, str_temp_MA);

    return G_SOURCE_REMOVE;
}



static gboolean update_centerPointmmLabel (gpointer userdata)
// gboolean: A standard boolean type. Variables of this type should only contain the value TRUE or FALSE.
{
    //printf("Inside update_text2().\n");
    float *centerP_mm;                     // Pointer to the array storing coordinates of center point.
    centerP_mm = get_centerP_mm();   // in vision.c

    printf("%.2f %.2f\n",centerP_mm[0],centerP_mm[1]);

    char str_centerPointmm[50];
    char str_temp[4];
    sprintf(str_temp, "%.2f ", centerP_mm[0]);
    gtk_label_set_text (label_centerPx_mm, str_temp);

    sprintf(str_temp, "%.2f", centerP_mm[1]);
    gtk_label_set_text (label_centerPy_mm, str_temp);
    //printf("Leaving update_text2().\n");
    return G_SOURCE_REMOVE;
}

static gboolean update_vidImage1 (gpointer userdata)
{
    //printf("Inside update_vidImage1().\n");
    Mat img_m_color = getImage();
    gtk_image_set_from_pixbuf(videoWindow, gdk_pixbuf_new_from_data(img_m_color.data, GDK_COLORSPACE_RGB, false, 8,
                              img_m_color.cols, img_m_color.rows, img_m_color.step, NULL, NULL));
    //printf("Leaving update_vidImage1().\n");
    return G_SOURCE_REMOVE;
}

static gboolean update_vidImage2 (gpointer userdata)
{
    //printf("Inside update_vidImage1().\n");
    Mat img_m_color2 = getImage2();
    gtk_image_set_from_pixbuf(videoWindow2, gdk_pixbuf_new_from_data(img_m_color2.data, GDK_COLORSPACE_RGB, false, 8,
                              img_m_color2.cols, img_m_color2.rows, img_m_color2.step, NULL, NULL));
    //printf("Leaving update_vidImage1().\n");
    return G_SOURCE_REMOVE;
}

static gboolean update_cameraImageInfoLabel (gpointer userdata)
{
    char temp[6];
    sprintf(temp, "%.1f", GUI_field_angle);
    gtk_label_set_text (label_info_angle, temp);
    return G_SOURCE_REMOVE;
}

static gboolean draw_field (gpointer userdata)
{
    //printf("Inside draw_field.\n");
    gtk_widget_queue_draw (field_drawingArea); //request a redraw
    return G_SOURCE_REMOVE;
}



void* GUI_refresh_thread(void*threadid)
{
	printf("@ the Beginning of GUI_refresh_thread().\n");
	usleep(1e6);
	Mat img_m_color,img_m_color2;
	// gdk_threads_enter and gdk_threads_leave has been deprecated since version 3.6. We use GTK2.0.

	//float refresh_rate = 30;                 //Frames to refresh per second. May not actually achieve this! Cannot be lower than 1.
	float refresh_rate = 60;
	float refresh_period = 1.0/refresh_rate; //seconds per refresh frame
	double time_current, time_elapsed, time_init;
	struct timeval start;
	gettimeofday(&start, NULL);
	time_init = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time

    /// Open a file to record data
    //FILE *fp;
    //fp = fopen("pointFollowRecord.txt","w");

    int *centerP_coor;
    int *centerP2_coor;

    while(GUI_refresh_running)
    {
        if (flag_draw_field)
            g_main_context_invoke (NULL, draw_field, NULL);   // Draw field control voltage value

        if (GUI_refresh_flag_vidWin1)
        {
            img_m_color = getImage();

            //gdk_threads_enter();	//display video image in program window

            if (img_m_color.data != NULL)
            {
                g_main_context_invoke (NULL, update_vidImage1, NULL);
            }
            if (GUI_refresh_flag_vidWin2 == 1)   // If the side camera is on
            {
                img_m_color2 = getImage2();

                if (img_m_color2.data != NULL)
                {
                    g_main_context_invoke (NULL, update_vidImage2, NULL);
                }

            }
            g_main_context_invoke (NULL, update_centerPointLabel, NULL);
             g_main_context_invoke (NULL, update_centerPointLabel_MA, NULL);              // for multi-agent project to read the center point
            //g_main_context_invoke (NULL, update_centerPointmmLabel, NULL);

            /* Commented on 2015-03-29 by JZ. Because not sure what this part is for.
            // If the walker is being actuated, update the camera image info label
            if (walker_actuationThreadControl)
                g_main_context_invoke (NULL, update_cameraImageInfoLabel, NULL);
            */
        }

        gettimeofday(&start, NULL);

        g_main_context_invoke (NULL, update_GUI_time, NULL);

        //g_main_context_invoke (NULL, update_GUI_time_ms, NULL);

        /// Record center point
        centerP_coor = getCenterPointCoor();


       // If orientation circle is detected in Camera-1, record its center point ...............commented by Mohammad for multi-agent
        if (flag_orinet_circ_required)
        {
         //   centerP2_coor = get2ndCenterPointCoor();
      //   printf("[Agent 1 center]: (%d , %d)  ........   [Agent 2 center]: (%d, %d) \n", centerP_coor[0], centerP_coor[1], centerP2_coor[0], centerP2_coor[1]);
        }
        else
        //printf("[Agent 1 center]: %d %d \n", centerP_coor[0], centerP_coor[1]);




        // If 2 rectangles are detected in Camera-1, record both center points
        if (flag_second_rect_required)
        {
            centerP2_coor = get2ndCenterPointCoor();
            //fprintf(fp, "%d %d %d %d\n", centerP_coor[0], centerP_coor[1], centerP2_coor[0], centerP2_coor[1]);
        }
        else
            //fprintf(fp, "%d %d\n", centerP_coor[0], centerP_coor[1]);


        time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; // Current time
        time_elapsed = time_current - time_init;                    // Time elapsed since last refresh
        if(time_elapsed < refresh_period)
	{
		//printf("Wait time is %.5f.\n", (refresh_period - time_elapsed) * 1e6);
            usleep( (refresh_period - time_elapsed) * 1e6 );        // Wait until refresh period has been reached
	}
        gettimeofday(&start, NULL);
        time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; // Current time
        time_init = time_current  ; //reset last time

        // If no GUI_refresh job is needed, stop this thread
        if ( (!flag_draw_field) && (!GUI_refresh_flag_vidWin1) )
            GUI_refresh_running = 0;
	}
	//fclose(fp);   // Close file
	printf("@ the End of GUI_refresh_thread().\n");

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// control
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void* controlThread(void*threadid) {
	printf("@ the Beginning of controlThread().\n");

	// WHY?
	const char * device = "/dev/ttyACM0";  // Linux open usb port

	// int   open(const char *path, int oflags       );
	int fd = open(device          , O_RDWR | O_NOCTTY);   // O_RDWR: Open the file so that it can be read from and written to.
							      // O_NOCTTY: Prevent the OS from assigning the opened file as the process's
							      //           controlling terminal when opening a TTY device file
	if (fd == -1)
	{
		perror(device);   //  write error messages to standard error
		printf("Error in controlthread openidaqThreadng device!!.\n");
	}
  	//smcExitSafeStart(fd); //Pololu usb error

  	//printf("Error status: %i\n", smcGetErrorStatus(fd)); //Pololu usb error
	struct timeval start;

	int speed = 0;//smcGetTargetSpeed(fd);

	//printf("Current Target Speed is %d.\n", speed);
	int ii = 0; //counter
	double time; //time in seconds
	gettimeofday(&start, NULL);
	double time_init = (double) start.tv_sec + start.tv_usec*1e-6 ; //elapsed

	float freq = 1;

	while(control_running == 1)
	{
		//printf("%i ",ii); //print out counter
		usleep(1e4); //throttle back loop
		ii++; //increment counter

///////////////////////////////// Pololu USB control
		gettimeofday(&start, NULL);
		time = (double) start.tv_sec + start.tv_usec*1e-6 - time_init; //elapsed time in seconds
		newSpeed = amp* (  sin( time*freq*2*M_PI )  );
		printf("Setting Target Speed to %d. Time is %f sec.\n", newSpeed, time);
		//smcSetTargetSpeed(fd, newSpeed);
/////////////////////////////////
	}
	newSpeed = 0;
	printf("Setting Target Speed to %d.\n", newSpeed); //turn off motors
	//smcSetTargetSpeed(fd, newSpeed); //turn off motors

	close(fd);
	printf("@ the End of controlThread().\n");
}

void* daqThread(void*threadid)
{
	printf("@ the Beginning of daqThread().\n");
	//usleep(1e4);
	//s826_init();
        //usleep(1e4);
        // printf("data write to daq here %i\n", newSpeed);
        //s826_close();
	//usleep(1e3);
	/*while(control_running == 1)
	{ ;
		int s826_init(uint board);
                usleep(1e4);
                printf("data write to daq here %i\n", newSpeed);
                int s826_close(void);
		usleep(1e3);
	}

	printf("\nDAQ thread done.\n");
	*/
	printf("@ the End of daqThread().\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUbroutine: Coil Current Control
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int currentControlSubroutine(uint aoChan, float outputV)
{
	//printf("In Current Control Subroutine.\n");
	//int errcode     = S826_ERR_OK;

	uint aoRange = 2;   // 2: -5 ~ +5 V.

	float outputV_local = 0.0, outputV_adjust = 0;   // Local variable storing the output voltage.

	/*
	switch(aoChan)
	{
		case 0: outputV_local = outputV1; break;
		case 1: outputV_local = outputV2; break;
		case 2: outputV_local = outputV3; break;
	}
	*/

	switch(aoChan)
	{
		case 0: outputV_adjust =      dirAdjust_amp * outputV; break;
		case 2: outputV_adjust = -1 * dirAdjust_amp * outputV; break;
	}

	//printf("outputV_adjust is %f \n",outputV_adjust);

	s826_aoPin( aoChan , aoRange, outputV);

	// Output horizontal adjust field.
	if (aoChan != 1)
		s826_aoPin( 2-aoChan , aoRange, outputV_adjust);
}

/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUbroutine: Coil Current Clear Subroutine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int coilCurrentClear(void)
{
	uint aoRange = 2;   // 2: -5 ~ +5 V.
	s826_aoPin( 0 , aoRange, 0);
	s826_aoPin( 1 , aoRange, 0);
	s826_aoPin( 2 , aoRange, 0);
}
*/

/* Commented on 2015-03-29. This function is moved to walker.c
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Walker Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* Walker_Actuation_Thread(void*threadid)
{
	printf("@ the Beginning of Walker_Actuation_Thread.\n");

	if (walker_actuationFreq == 0)
		printf("Error in Walker_Actuation_Thread: period = 0. Thread ends.\n");
	else
	{
		//s826_init();
		uint temp_dirIndex = dirIndex;
		float temp_actuationFreq = walker_actuationFreq;
		float period = 1 / temp_actuationFreq;          // Period of the sawtooth wave in seconds

		//float amplitude_actuation = 3.5;     // Amplitude in volts
		float amplitude_actuation = 3.0;
		//float amplitude_actuation = 2.0;
		//float amplitude_actuation  = 0.0;
		//float bias_actuation = 0;		   // Bias of the signal: output = sawtooth + bias

		//float directional_bias = 0.5;
		//float directional_bias = 1.5;
		float directional_bias = 2.0;
		//float directional_bias = 2.5;

		float slope_actuation = amplitude_actuation * 2 * temp_actuationFreq;;  // Slope of Sawtooth wave, half of the slop of Triangle wave

		struct timeval start;

		double time_now, time_variable = 0;                                 //time in seconds
		gettimeofday(&start, NULL);
		double time_init = (double) start.tv_sec + start.tv_usec*1e-6 ;         // Initial time in seconds.

		float outputV_h = 0, outputV_v = 0;   // Output voltage of the AO channel

        float field_angle_radian = 0;   // Field anagle in radians
		while(walker_actuationThreadControl == 1)
		{
			if (temp_actuationFreq != walker_actuationFreq)   // If actuation freq. is changed...
			{
				temp_actuationFreq = walker_actuationFreq;   // Store current actuation freq.
				slope_actuation    = amplitude_actuation * 2 * temp_actuationFreq;
				//printf("temp actuation freq. is %.3f.\n", temp_actuationFreq);
				if (temp_actuationFreq == 0)
					period = -1;
				else
					period = 1 / temp_actuationFreq;
			}

			if (temp_dirIndex != dirIndex)         // If dirIndex is changed...
			{
				temp_dirIndex = dirIndex;      // Store current dirIndex
				coilCurrentClear();            // Clear any directional offset
				switch(temp_dirIndex)     	       // Set   new directional offset
				{
					case 0: s826_aoPin( 0 , 2,  directional_bias);//currentControlSubroutine(0,  directional_bias);
                            break;
					case 1: s826_aoPin( 2 , 2,  directional_bias);//currentControlSubroutine(2,  directional_bias);
                            break;
					case 2: s826_aoPin( 0 , 2, -directional_bias);//currentControlSubroutine(0, -directional_bias);
                            break;
					case 3: s826_aoPin( 2 , 2, -directional_bias);//currentControlSubroutine(2, -directional_bias);
                            break;
				}
			}
			time_now = (double) start.tv_sec + start.tv_usec*1e-6;

			if (period == -1)
			{
				outputV_v = 0;
				outputV_h = 0;
			}
			else
			{
				time_variable = time_variable + walker_actuationDir * (time_now - time_init);

				while (time_variable > period) time_variable = time_variable - period;
				while (time_variable < 0     ) time_variable = period + time_variable;

				switch (walker_actuation_method)
				{
                    case -1:
                            printf("Error: walker_actuation_method is -1. Thread Ends.\n");
                            walker_actuationThreadControl=0;
                            break;
                    case 0: // Sinusoidal Actuation
                            field_angle_radian = 2 * M_PI * temp_actuationFreq * time_variable;


                            outputV_h = amplitude_actuation * cos(field_angle_radian);
                            outputV_v = amplitude_actuation * sin(field_angle_radian);
                            break;

                    case 1:
                            outputV_h = 0;
                            if (time_variable < period*0.5)   // Going up
                                outputV_v = -1 * amplitude_actuation + 2 * slope_actuation * time_variable                  + walker_actuationBias;
                            else                              // Going down
                                outputV_v =      amplitude_actuation - 2 * slope_actuation * (time_variable - period * 0.5) + walker_actuationBias;
                            break;

                    case 2: // Sawtooth Actuation
                            outputV_h = 0;
                            outputV_v = -1 * amplitude_actuation + slope_actuation * time_variable + walker_actuationBias;
                            break;
				}

				if (!walker_actuation_method)
                    GUI_field_angle = field_angle_radian * 180 / M_PI;
                else
                {
                    if (outputV_v > 0)
                        GUI_field_angle = 90;
                    else if (outputV_v < 0)
                        GUI_field_angle = 270;
                    else
                        GUI_field_angle = 0;   // TODO: Needs improvements. The angle is not 0, there is no field at this point
                }
				//outputV = -1 * amplitude_actuation + slope_actuation * time_variable + walker_actuationBias;

				//printf("Slope actuation is %.3f.\n", slope_actuation);
            }
			time_init = time_now;
			//printf("Output voltage is %.3f.\n", outputV);
			//currentControlSubroutine(0, outputV_h);
			//currentControlSubroutine(2, outputV_v);
			s826_aoPin( 0 , 2, outputV_h);
			s826_aoPin( 1 , 2, outputV_v);

			waitUsePeriodicTimer(1e3);
		}

		coilCurrentClear();
		//s826_close();
	}
	printf("@ the End of Walker_Actuation_Thread.\n");
}
*/


/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Walker Sawtooth Z-Field Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* walker_sawtoothThread (void*threadid)
{
	printf("@ the Beginning of walker_sawtoothThread.\n");

	float period = 0;
	if (walker_actuationFreq == 0)
		printf("Error in walker_sawtoothThread: period = 0. Thread ends.\n");
	else
	{
		s826_init();
		uint temp_dirIndex = dirIndex;
		float temp_actuationFreq = walker_actuationFreq;
		period = 1 / temp_actuationFreq;          // Period of the sawtooth wave in seconds

		float amplitude_actuation = 3.5;     // Amplitude in volts
		//float amplitude_actuation = 2.0;
		//float amplitude_actuation  = 0.0;
		//float bias_actuation = 0;		   // Bias of the signal: output = sawtooth + bias

		//float directional_bias = 0.5;
		//float directional_bias = 1.5;
		float directional_bias = 2.0;
		//float directional_bias = 2.5;

		float slope_actuation = amplitude_actuation * 2 * temp_actuationFreq;;  // Slope of the sawtooth wave


		struct timeval start;

		double time_now, time_variable = 0;                                 //time in seconds
		gettimeofday(&start, NULL);
		double time_init = (double) start.tv_sec + start.tv_usec*1e-6 ;         // Initial time in seconds.

		float outputV = 0;   // Output voltage of the AO channel

		while(walker_actuationThreadControl == 1)
		{
			if (temp_actuationFreq != walker_actuationFreq)   // If actuation freq. is changed...
			{
				temp_actuationFreq = walker_actuationFreq;   // Store current actuation freq.
				slope_actuation    = amplitude_actuation * 2 * temp_actuationFreq;
				//printf("temp actuation freq. is %.3f.\n", temp_actuationFreq);
				if (temp_actuationFreq == 0)
					period = -1;
				else
					period = 1 / temp_actuationFreq;
			}

			if (temp_dirIndex != dirIndex)         // If dirIndex is changed...
			{
				temp_dirIndex = dirIndex;      // Store current dirIndex
				coilCurrentClear();            // Clear any directional offset
				switch(temp_dirIndex)     	       // Set   new directional offset
				{
					case 0: currentControlSubroutine(0,  directional_bias); break;
					case 1: currentControlSubroutine(2,  directional_bias); break;
					case 2: currentControlSubroutine(0, -directional_bias); break;
					case 3: currentControlSubroutine(2, -directional_bias); break;
				}
			}

			gettimeofday(&start, NULL);
			time_now = (double) start.tv_sec + start.tv_usec*1e-6;

			if (period == -1)
			{
				outputV = 0;
			}
			else
			{
				time_variable = time_variable + walker_actuationDir * (time_now - time_init);

				while (time_variable > period) time_variable = time_variable - period;
				while (time_variable < 0     ) time_variable = period + time_variable;
				outputV = -1 * amplitude_actuation + slope_actuation * time_variable + walker_actuationBias;
				//printf("Slope actuation is %.3f.\n", slope_actuation);
		        }
			time_init = time_now;
			//printf("Output voltage is %.3f.\n", outputV);
			currentControlSubroutine(1, outputV);
			waitUsePeriodicTimer(1e3);
		}

		coilCurrentClear();
		s826_close();
	}
	printf("@ the End of walker_sawtoothThread.\n");
}
*/

/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AO Sine Wave Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* AO_sineWaveThread(void*threadid)
{
	//printf("In AO_testThread.\n");
	//int errcode     = S826_ERR_OK;

	int temp = 0;

	char str_centerPointCoor[50];

	//usleep(1e4);

	s826_init();
        usleep(1e4);

	uint aoChan_horizontal = 0;   // Horizontal Channel
	uint aoChan_adjust     = 2;   // Field adjustment channel

	uint aoRange = 2;   // 2: -5 ~ +5 V.

	struct timeval start;

	double time_elapsed, time_now, time_variable = 0;                                 //time in seconds
	gettimeofday(&start, NULL);
	double time_init = (double) start.tv_sec + start.tv_usec*1e-6 ;         // Initial time in seconds.

	temp = dirIndex;

	float outputV_horizontal = 0, outputV_vertical = 0;

	while(swimmer_actuationThreadControl == 1)
	{
	*/
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
		if (dirIndex != temp)
		{
			coilCurrentClear();

			switch(dirIndex)
			{
				case 0: aoChan_horizontal = 0; aoChan_adjust = 2; AO_sinFreq_dir =  1;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
					break;
				case 1: aoChan_horizontal = 2; aoChan_adjust = 0; AO_sinFreq_dir =  1;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
					break;
				case 2: aoChan_horizontal = 0; aoChan_adjust = 2; AO_sinFreq_dir = -1;
					//currentControlSubroutine(0, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(0, aoRange, 0);
					break;
				case 3: aoChan_horizontal = 2; aoChan_adjust = 0; AO_sinFreq_dir = -1;
					//currentControlSubroutine(2, 3.5);
					//waitUsePeriodicTimer(1e6);
					//s826_aoPin(2, aoRange, 0);
					break;
			}
			temp = dirIndex;
		}

		gettimeofday(&start, NULL);
		time_now = (double) start.tv_sec + start.tv_usec*1e-6;
		time_elapsed = time_now - time_init;                         //elapsed time in seconds
		time_variable = time_variable + AO_sinFreq_dir * time_elapsed;
		//time_variable = time_variable + time_elapsed;
		time_init = time_now;

		//outputV_horizontal = 3.0 * sin( 2*M_PI * AO_sinFreq * time_variable) + 0.5;  // Weak Offset
		//outputV_horizontal = 2.5 * sin( 2*M_PI * AO_sinFreq * time_variable) + 1.0;   // Strong Offset
		//outputV_horizontal = 3.5 * sin( 2*M_PI * AO_sinFreq * time_variable);
		//outputV_vertical   = 3.5 * cos( 2*M_PI * AO_sinFreq * time_variable);

        outputV_horizontal = swimmer_A_h * sin( 2*M_PI * AO_sinFreq * time_variable) + swimmer_dir_bias;  // For under surface swimmer
        outputV_vertical   = swimmer_A_v * cos( 2*M_PI * AO_sinFreq * time_variable) + swimmer_vertical_bias;

		currentControlSubroutine(aoChan_horizontal, outputV_horizontal);
		currentControlSubroutine(1                , outputV_vertical  );

		//s826_aoPin( aoChan_horizontal, aoRange, outputV_horizontal                );
		//s826_aoPin( 1                , aoRange, outputV_vertical                  );
		//s826_aoPin( aoChan_adjust    , aoRange, dirAdjust_amp * outputV_horizontal);

		waitUsePeriodicTimer(1e3);//1e5:10 Hz; 5e4:20 Hz; 1e4: 100 Hz; 1e3: 1000 Hz
	}

	// Reset all 3 coil currents to 0.
	//s826_aoPin( 0, aoRange, 0); s826_aoPin( 1, aoRange, 0); s826_aoPin( 2, aoRange, 0);
	coilCurrentClear();

	// Close DAQ board.
	s826_close();
	usleep(1e3);
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Current Control Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void* currentControlThread(void*threadid)
{
	printf("In currentControlThread.\n");


	int errcode     = S826_ERR_OK;

	usleep(1e4);
	s826_init();
        usleep(1e4);

	//uint aoChan  = 0;
	uint aoRange = 2;   // 2: -5 ~ +5 V.

	int i = 0;
	//while(control_running == 1)
	//{
		//i = i+1;
		//s826_aoPin( aoChan  , aoRange, 2*sin(M_PI*i/25.0) );
		//s826_aoPin( aoChan+1, aoRange, 2*cos(M_PI*i/25.0) );
		//waitUsePeriodicTimer(1e6);//1e5:10hz  5e4:20hz
	//}

	//s826_aoPin( 0, aoRange, outputV1 );
	//s826_aoPin( 1, aoRange, outputV2 );
	//s826_aoPin( 2, aoRange, outputV3 );
	//printf("OutputV1 is %.1f \n",outputV1);
	//printf("OutputV2 is %.1f \n",outputV2);
	//printf("OutputV3 is %.1f \n",outputV3);

	s826_close();
	usleep(1e3);
}
*/


void* waveformThread(void*threadid)
{
	printf("In waveform thread.\n");
}

/* Toggle Button: Draw field control voltage value */
void on_tButton_draw_field_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if (d==1)//if button is toggled up
	{
        flag_draw_field = 1;
		//GUI_refresh_running = 1; //turn on control loops
		if (!GUI_refresh_running)  // If the GUI_refresh thread is NOT running...
		{
            GUI_refresh_running = 1;
            pthread_t   GUI_refresh;
            pthread_create(&GUI_refresh, NULL, GUI_refresh_thread, NULL);  //start control loop thread
		}
	}else
	{
		flag_draw_field = 0; //turn off control loops
	}
}

// Undergradate demonstration
void on_b_undergradTest_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active(togglebutton);
    undergrad_start_stop_demo(d);                               // undergrad.cc
}


/* Draw field control voltage value */
void on_field_drawingArea_expose_event (GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
    cairo_t *cr;
    cr = gdk_cairo_create(widget->window);

	char str[10];//string for printing text. reused.
   	/* Set color for background */
   	cairo_set_source_rgb(cr, 1, 1, 1);
   	/* fill in the background color*/
   	cairo_paint(cr);

   	/* x field */
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 30, 150);
   	cairo_line_to(cr, 0, 150);
   	cairo_stroke(cr);

   	cairo_set_source_rgb(cr, 255,0,0); //set color to Red
   	cairo_set_line_width(cr, 30);
   	float coil_current_x = get_coil_current(0);

   	//printf("current x is %.1f.\n",coil_current_x);

   	cairo_move_to(cr, 15, 150);
   	cairo_line_to(cr, 15, 150 - coil_current_x * 30);
   	cairo_stroke(cr);

   	/* y field */
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 80, 150);
   	cairo_line_to(cr, 50, 150);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0,255,0); //set color to Green
   	cairo_set_line_width(cr, 30);
   	float coil_current_y = get_coil_current(1);

   	cairo_move_to(cr, 65, 150);
   	cairo_line_to(cr, 65, 150 - coil_current_y * 30);
   	cairo_stroke(cr);

   	/* z field */
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 130, 150);
   	cairo_line_to(cr, 100, 150);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0,0,255); //set color to Red
   	float coil_current_z = get_coil_current(2);

   	cairo_move_to(cr, 115, 150);
   	cairo_line_to(cr, 115, 150 - coil_current_z * 30);
   	cairo_stroke(cr);

   	cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_move_to (cr, 0, 275);
	sprintf(str, "%.1f", coil_current_x);
	cairo_show_text (cr, str);
	cairo_move_to (cr, 50, 275);
	sprintf(str, "%.1f", coil_current_y);
	cairo_show_text (cr, str);
    cairo_move_to (cr, 100, 275);
	sprintf(str, "%.1f", coil_current_z);
    cairo_show_text (cr, str);

    /////////////////////////////////////////////////////////
    // Orientation
    /////////////////////////////////////////////////////////

    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 1);
   	cairo_move_to(cr, 30, 350);
   	cairo_line_to(cr, 110, 350);
   	cairo_move_to(cr, 70, 310);
   	cairo_line_to(cr, 70, 390);

   	cairo_move_to(cr,  30, 470);
   	cairo_line_to(cr, 110, 470);
   	cairo_move_to(cr,  70, 430);
   	cairo_line_to(cr,  70, 510);
   	cairo_stroke(cr);

    /// y-z circle
    cairo_move_to(cr,  30, 590);
   	cairo_line_to(cr, 110, 590);
   	cairo_move_to(cr,  70, 550);
   	cairo_line_to(cr,  70, 630);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0.17, 0.63, 0.12);
   	cairo_set_line_width(cr,1);
   	cairo_arc(cr, 70, 350, 40, 0, 2*G_PI);
   	cairo_move_to(cr, 110, 470);
   	cairo_arc(cr, 70, 470, 40, 0, 2*G_PI);
   	cairo_stroke(cr);
   	/// y-z circle
   	cairo_arc(cr, 70, 590, 40, 0, 2*G_PI);
   	cairo_stroke(cr);
    cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 200, 0, 0);
    cairo_move_to (cr, 120, 350);
	sprintf(str, "x");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 70, 300);
	sprintf(str, "y");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 120, 470);
	sprintf(str, "x");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 70, 420);
	sprintf(str, "z");
	cairo_show_text (cr, str);
	/// y-z circle
	cairo_move_to (cr, 120, 590);
	sprintf(str, "y");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 70, 540);
	sprintf(str, "z");
	cairo_show_text (cr, str);

    float orientation_xy = atan2(coil_current_y, coil_current_x);
    float orientation_xz = atan2(coil_current_z, coil_current_x);
    float orientation_yz = atan2(coil_current_z, coil_current_y);

    cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 200);
    if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_y > 0.01) || (coil_current_y < -0.01) )
    {
        cairo_move_to (cr, 100, 390);
        sprintf(str, "%.1f", orientation_xy * 180 / M_PI);
        cairo_show_text (cr, str);
    }
    if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to (cr, 100, 510);
        sprintf(str, "%.1f", orientation_xz * 180 / M_PI);
        cairo_show_text (cr, str);
    }
    /// y-z circle orientation value
    if ( (coil_current_y > 0.01) || (coil_current_y < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to (cr, 100, 630);
        sprintf(str, "%.1f", orientation_yz * 180 / M_PI);
        cairo_show_text (cr, str);
    }

	cairo_set_source_rgb(cr, 255,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_y > 0.01) || (coil_current_y < -0.01) )
    {
        cairo_move_to(cr, 70, 350);        // Center of x-y circle
        cairo_line_to(cr, 70+40 * cos(orientation_xy), 350-40 * sin(orientation_xy));
    }
    if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to(cr, 70, 470);        // Center of x-y circle
        cairo_line_to(cr, 70+40 * cos(orientation_xz), 470-40 * sin(orientation_xz));
    }
    /// y-z circle orientation line
    if ( (coil_current_y > 0.01) || (coil_current_y < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to(cr, 70, 590);        // Center of y-z circle
        cairo_line_to(cr, 70+40 * cos(orientation_yz), 590-40 * sin(orientation_yz));
    }

    // Display current directional adjust angle
    float l_dirAdjust_angle = get_dirAdjust_angle();
    cairo_move_to (cr, 70, 700);
    sprintf(str, "%.1f", l_dirAdjust_angle * 180 / M_PI);
    cairo_show_text (cr, str);

    // Display current swimmer heading
    float l_swimmer_heading = get_swimmer_heading();
    cairo_move_to (cr, 70, 800);
    sprintf(str, "%.1f", l_swimmer_heading * 180 / M_PI);
    cairo_show_text (cr, str);

    int l_swimming_dir = get_swimming_dir();
    cairo_move_to (cr, 70, 900);
    sprintf(str, "%d", l_swimming_dir);
    cairo_show_text (cr, str);

    cairo_stroke(cr);

	cairo_destroy(cr);
}



void on_coildraw_draw (GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
	cairo_t *cr;
    cr = gdk_cairo_create(widget->window);

	char str[25];//string for printing text. reused.
   	/* Set color for background */
   	cairo_set_source_rgb(cr, 1, 1, 1);
   	/* fill in the background color*/
   	cairo_paint(cr);

   	/* set color for rectangle */
   	//cairo_set_source_rgb(cr, 0.42, 0.65, 0.80);
   	/* set the line width */
   	//cairo_set_line_width(cr,6);
   	/* draw the rectangle's path beginning at 3,3 */
   	//cairo_rectangle (cr, 3, 3, 100, 100);
   	/* stroke the rectangle's path with the chosen color so it's actually visible */
   	//cairo_stroke(cr);

   	/* draw circle *//*
   	cairo_set_source_rgb(cr, 0.17, 0.63, 0.12);
   	cairo_set_line_width(cr,3);
   	cairo_arc(cr, 150, 260, 10, 0, 2*G_PI);
   	cairo_stroke(cr);*/

   	/* draw line *//*
   	cairo_set_source_rgb(cr, fabs(newSpeed)/500.+0.3,0,0); //set color
   	cairo_set_line_width(cr, 15);
   	cairo_move_to(cr, newSpeed*0.1+150,260);
   	cairo_line_to(cr, 150, 260);
   	cairo_stroke(cr);*/

   	/* x field */ /*
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 180, 200);
   	cairo_line_to(cr, 150, 200);
   	cairo_stroke(cr);

   	cairo_set_source_rgb(cr, 255,0,0); //set color to Red
   	cairo_set_line_width(cr, 30);
   	float coil_current_x = get_coil_current(0);
   	cairo_move_to(cr, 165, 200);
   	cairo_line_to(cr, 165, 200 - coil_current_x * 30);
   	cairo_stroke(cr);

    cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_move_to (cr, 150, 350);
	sprintf(str, "%.1f", coil_current_x);
    cairo_show_text (cr, str); //print time
    */

   	/* y field */ /*
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 230, 200);
   	cairo_line_to(cr, 200, 200);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0,255,0); //set color to Green
   	cairo_set_line_width(cr, 30);
   	float coil_current_y = get_coil_current(1);

   	cairo_move_to(cr, 215, 200);
   	cairo_line_to(cr, 215, 200 - coil_current_y * 30);
   	cairo_stroke(cr);

   	cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_move_to (cr, 200, 350);
	sprintf(str, "%.1f", coil_current_y);
    cairo_show_text (cr, str); //print time
    */

   	/* z field */ /*
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 280, 200);
   	cairo_line_to(cr, 250, 200);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0,0,255); //set color to Red
   	cairo_set_line_width(cr, 30);
   	float coil_current_z = get_coil_current(2);

   	cairo_move_to(cr, 265, 200);
   	cairo_line_to(cr, 265, 200 - coil_current_z * 30);
   	cairo_stroke(cr);

   	cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_move_to (cr, 250, 350);
	sprintf(str, "%.1f", coil_current_z);
    cairo_show_text (cr, str); //print time
    */

   	/* draw text *//*
	struct timeval start;
	gettimeofday(&start, NULL);
	double time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; //elapsed
	cairo_set_font_size (cr, 14);
    	cairo_set_source_rgb (cr, 1, 0.5, 0.5);
    	cairo_move_to (cr, 50, 50);
	sprintf(str, "%f", time_current);
    	cairo_show_text (cr, str); //print time
    	*/

    /*
	cairo_move_to (cr, 135, 285);
	sprintf(str, "%i", newSpeed);
    cairo_show_text (cr, str); //print speed
    */

	cairo_destroy(cr);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// drawThread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void* drawThread(void*threadid)
{
	printf("@ the Beginning of drawThread.\n");

	gdk_threads_enter();
	gtk_widget_queue_draw (coildraw); //request a redraw
	gdk_threads_leave();

	float draw_rate = 45; //Frames to draw per second. May not actually achieve this!
	float draw_period = 1.0/draw_rate; //seconds per draw frame
	double time_current, time_elapsed, time_last;
	struct timeval start;
	gettimeofday(&start, NULL);
	time_last = (double) start.tv_sec + start.tv_usec*1e-6 ; //elapsed
	while(control_running == 1) //repeat as long as our control loop is running
	{
		gdk_threads_enter();
		gtk_widget_queue_draw (coildraw); //request a redraw
		gdk_threads_leave();

		gettimeofday(&start, NULL);
		time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; //current time
		time_elapsed = time_current - time_last; //time elapsed since last draw
		if(time_elapsed < draw_period)
			usleep(   (draw_period-time_elapsed)*1e6  ); //wait until time elapsed has been reached
		gettimeofday(&start, NULL);
		time_last = time_current  ; //reset last time
	}

	printf("Leaving the drawThread\n");
}


void on_window_destroy (GtkWidget *widget, gpointer data) {
	stopVision(); //turn of visionloop
	control_running = 0; //turn off control loops
    GUI_refresh_running = 0;

    coilCurrentClear();                // Reset coil current to 0
    usleep(1e5);
    s826_close();                      // Close s826 board

    stop_swimmer_actuation_thread();   // Stop swimmer_actuation_thread if it is running
	motors_unlockFunction();
	usleep(3e5); //wait for all loops to detect shutdown and end themselves
	printf("Exiting program.\n");
        gtk_main_quit();
}

/*
void on_button1_clicked (GtkWidget *widget, gpointer data)
{       //printf("hello");
	img_test(); //test opencv
	pthread_t dthread;
	pthread_create(&dthread, NULL, drawThread, NULL);  //start draw loop thread. should only run once here
}
*/

/*void on_current_clicked(GtkWidget *widget, gpointer data)
{
	printf("hello\n");
}
*/

void on_enableFields_toggled (GtkToggleButton *togglebutton, gpointer data) //control loop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	printf("enableFields toggled to %i\n", d );

	pthread_t daqthread, cthread, dthread;
	if(d==1) //if button is toggled up
	{
		control_running = 1; //turn on control loops
		pthread_create(&cthread, 	NULL, controlThread, NULL);  //start control loop thread
		pthread_create(&daqthread, 	NULL, daqThread, NULL);      //start daq thread
		pthread_create(&dthread, 	NULL, drawThread, NULL);     //start draw loop thread
	}else
	{
		control_running = 0; //turn off control loops
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Camera Enable: Video toggle button
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void on_videoButton_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		if(visionStarted) return;
		g_print("Starting vision...\n");
		visionStarted = true;
		initVision(); //in vision.c

        GUI_refresh_flag_vidWin1 = 1;   // Begin to refresh vidWin
        if (!GUI_refresh_running)       // If the GUI_refresh_thread is not running ...
        {
            pthread_t GUI_refresh_thread_instance;
            GUI_refresh_running = 1;
            pthread_create(&GUI_refresh_thread_instance, NULL, GUI_refresh_thread, NULL);  //start control loop thread
        }
	}
	else
	{
		g_print("Stopping vision...\n");
		stopVision(); //sets killThread to 1
		visionStarted = false;
        //GUI_refresh_running = 0;
		GUI_refresh_flag_vidWin1 = 0;   // Stop refreshing GUI
	}
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// One AO sine wave test button
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_AO_sine_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	//printf("Inside the toggle button program!!!\n");

	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("togglebutton2 toggled to %i\n", d );
	//pthread_t ao_sinewavethread;

	if(d==1) //if button is toggled up
	{
        init_rotate_field_thread();
		//printf("Button has been pressed!!!\n");
		//swimmer_actuationThreadControl = 1;
		//pthread_create(&ao_sinewavethread, NULL, AO_sineWaveThread, NULL);
	}
	else
	{
		//printf("Button has been released!!!\n");
		//swimmer_actuationThreadControl = 0;
		//stop_rotate_field_thread();
		stop_swimmer_actuation_thread();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer: Tentative Control Loop Switch
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_swimmer_controlLoop_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	swimmer_controlLoop = gtk_toggle_button_get_active(togglebutton);
	/*
	while (swimmer_actuationThreadControl == 1)   // While the swimmer's actuation field is working...
	{
		if (centerP.x > 400)
		{
			dirIndex = 2;       // -x
			printf("change to -x direction.\n");
		}
		else if (centerP.x < 150)
		{
			dirIndex = 0;  // +x
			printf("change to +x direction.\n");
		}
	}
*/
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Toggle Button to Reverse the Direction of Rotating Field
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void on_Direction_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);

	if (d == 1)   AO_sinFreq_dir = -1;
	else	      AO_sinFreq_dir =  1;

	//printf("togglebutton2 toggled to %i\n", d );

	//pthread_t ao_testthread;


	//if(d==1) //if button is toggled up
	//{
	//	printf("Button has been pressed!!!\n");
	//	control_running = 1;
	//	pthread_create(&ao_testthread, NULL, AO_testThread, NULL);
	//}
	//else
	//{
	//	printf("Button has been released!!!\n");
	//	control_running = 0;
	//
	//}
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer Control
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void on_swimmer_A_h_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    //swimmer_A_h = d;
    set_swimmer_A_h(d);
}

void on_swimmer_A_v_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    //swimmer_A_v = d;
    set_swimmer_A_v(d);
}

void on_swimmer_dir_bias_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    //swimmer_dir_bias = d;
    set_swimmer_h_bias(d);
}

void on_swimmer_vertical_bias_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    //swimmer_vertical_bias = d;
    set_swimmer_v_bias(d);
}

void on_swimmer_actuationFreq_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_swimmerActuationFreq(d);   // coilFieldControl.c
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer: Feedback Control
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_swimmer_pointFollow_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_swimmer_thread();
	}else
	{
		stop_swimmer_actuation_thread();
	}
}

void on_swimmer_pathFollow_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		//init_swimmer_thread();
		init_swimmer_pathFollow();   // coilFieldControl.c
	}else
	{
		//stop_swimmer_thread();
		stop_swimmer_pathFollow();
	}
}

void on_tB_UT_follow_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_UT_follow();   // coilFieldControl.c
	}else
	{
		//stop_swimmer_thread();
		stop_UT_follow();
	}
}

void on_swimmer_proportional_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_swimmer_proportional(d);
}

void on_swimmer_integral_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_swimmer_integral(d);
}

// Actuation Amplitude Feedback Control
void on_cB_actuation_A_control_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);
    set_flag_actuation_A_control(d);
}

// Compensation angle for net magnetization
void on_swimmer_dirAdjust_angle_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    //swimmer_dirAdjust_angle = d;
    set_swimmer_dirAdjust_angle(d);
}

// Field Direction Adjustment - Amplitude
void on_dirAdjust_amp_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//dirAdjust_amp = d;
	set_swimmer_dirAdjust_amp(d);
}

void on_swimmer_swim_dir_reverse_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);
    set_swimmer_swim_dir_reverse(d);
}

void on_twoGripper_drop_clicked (GtkWidget *widget, gpointer data) {
    twoGripper_drop_gripper ();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Button to Start/Stop Swimmer Heading Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_button_swimmerHeadingTest_clicked (GtkWidget *widget, gpointer data)
{
        if (swimmer_heading_test)
        {
            swimmer_heading_test = 0;
            stop_swimmerHeadingTest();
        }
        else
        {
            swimmer_heading_test = 1;
            init_swimmerHeadingTest();
        }
}

void on_b_magnetizationHeading_clicked (GtkWidget *widget, gpointer data)
{
    click_swimmer_magnetizationHeading();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer Speed Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_b_swimmerSpeed_clicked (GtkWidget *widget, gpointer data)
{
    initiate_swimmingSpeedTest();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer: Series Test Button Clicked
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_b_seriesTest_clicked (GtkWidget *widget, gpointer data)
{
    swimmer_series_test();   // coilCurrentControl.c
}

void on_sB_swimmer_actuationAcontrol_l_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_swimmer_actuationAcontrol_low(d);
}

void on_sB_swimmer_actuationAcontrol_h_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_swimmer_actuationAcontrol_high(d);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// tB: Multiple Swimmer Test
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_tB_swimmer_multiSwimmer_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);
    set_swimmer_multiSwimmer(d);
}

void on_b_swimmer_1stQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(4);   // Move to +x +y 45 degree
}

void on_b_swimmer_2ndQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(5);
}

void on_b_swimmer_3rdQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(6);
}

void on_b_swimmer_4thQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(7);
}

void on_b_swimmer_stop_clicked (GtkWidget *widget, gpointer data)
{
    stop_actuation_field();
}

void on_b_swimmer_pX_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(0);
}
void on_b_swimmer_pY_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(1);
}
void on_b_swimmer_nX_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(2);
}
void on_b_swimmer_nY_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index(3);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_edgemap_toggled (GtkToggleButton *togglebutton, gpointer data) //edgemap toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_edgemap(d); //set edgemap variable in vision.c
}
void on_binary_toggled (GtkToggleButton *togglebutton, gpointer data) //edgemap toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	set_binary(d); //set edgemap variable in vision.c
}
/*
void on_spinbutton1_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("Freq changed to %.2f Hz\n", d );
	freq = d; //change frequency

}
*/
void on_spinbutton2_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("Amp changed to %.1f units\n", d );
	amp = d; //change frequency
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set 2nd Rect. Requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_cB_2ndRect_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);
    if (d == 1)
        flag_second_rect_required = 1;
    else
        flag_second_rect_required = 0;
    set_2nd_rect (d);                                   // vision.c
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AO sine Freq Change
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_AO_sineFreq_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("AO_sinFreq has been changed to %.1f.\n", d);
	//printf("Freq changed to %d \n", d );
	AO_sinFreq = d;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Coil Current Change
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_current1_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("Current - 1 changed to %.1f \n", d );
	//AO_sinFreq = d;
	//printf("AO_sinFreq has been changed to %d.\n", AO_sinFreq);
	//outputV1 = d;

	//s826_init();
	//int temp = currentControlSubroutine(0, d);
	//pthread_t currentcontrolthread;
	//pthread_create(&currentcontrolthread, NULL, currentControlThread, NULL);
	//s826_close();

        if (flag_3coil_setup == 1)
        {

            s826_aoPin(0, 2, d/5.003);    // coil 1.0    x-left
           s826_aoPin(3, 2, d/4.879);         // coil 1.1   x-right
        }
        else
            coilCurrentClear();                        // make current zero for safety!
            //printf("3-coil system is disabled");
}

void on_current2_changed (GtkEditable *editable, gpointer user_data)   // y-field
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("Current - 2 changed to %.1f \n", d );
	//outputV2 = d;

	//s826_init();
	//int temp = currentControlSubroutine(1, d);
	//pthread_t currentcontrolthread;
	//pthread_create(&currentcontrolthread, NULL, currentControlThread, NULL);
	//s826_close();

        if (flag_3coil_setup == 1)
        {


s826_aoPin(1, 2, d/5.024);    // coil 2.0        y-left
s826_aoPin(5, 2, d/4.433);    //  coil 3           y-right

        }

        else
            coilCurrentClear();                             // make current zero for safety!
           // printf("3-coil system is disabled");
}

void on_current3_changed (GtkEditable *editable, gpointer user_data)           // z-direction
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("Current - 3 changed to %.1f \n", d );
	//outputV3 = d;

	//s826_init();
	//int temp = currentControlSubroutine(2, d);
	//pthread_t currentcontrolthread;
	//pthread_create(&currentcontrolthread, NULL, currentControlThread, NULL);
	//s826_close();

        if (flag_3coil_setup == 1)
        {
      s826_aoPin(4, 2, d/5.143);    // coil 2.1  == Z direction field

        }
        else
            coilCurrentClear();                              // make current zero for safety!
            //printf("3-coil system is disabled");
}




//  changes current on 2-coil system  created by Mohammad on July 11, 2015

void on_2coil_current1_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

        if (flag_2coil_setup == 1)
            set_coil_current_to (0, d/(5.2264));                  // Large coil slope factor is considered here to convert desired field to the input voltage generated by DAQ:: July 10, 2015
        else
            coilCurrentClear();                              // make current zero for safety!

}




void on_2coil_current2_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

        if (flag_2coil_setup == 1)
            set_coil_current_to (1, d/(5.8538));                        // Small coil slope factor is considered here to convert desired field to the input voltage generated by DAQ:: July 10, 2015
        else
            coilCurrentClear();                             // make current zero for safety!
           // printf("3-coil system is disabled");
}






///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Field Angle Change (theta and phi in a spherical coordinate)
///// /////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_field_angle_theta_changed (GtkEditable *editable, gpointer user_data)
{
	float theta = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    glo_theta = theta;
    float phi = glo_phi;

    float d1 = 3.5*sin(theta*3.1416/180)*cos(phi*3.1416/180);
    float d2 = 3.5*sin(theta*3.1416/180)*sin(phi*3.1416/180);
    float d3 = 3.5*cos(theta*3.1416/180);
	printf("Angle theta changed to %.1f \n", theta );
	//AO_sinFreq = d;
	//printf("AO_sinFreq has been changed to %d.\n", AO_sinFreq);
	//outputV1 = d;
	set_coil_current_to (0, d1);
	set_coil_current_to (1, d2);
    set_coil_current_to (2, d3);
	//pthread_t currentcontrolthread;
	//pthread_create(&currentcontrolthread, NULL, currentControlThread, NULL);
}
void on_field_angle_phi_changed (GtkEditable *editable, gpointer user_data)
{
	float phi = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
        glo_phi = phi;
        float theta = glo_theta;
    float d1 = 3.5*sin(theta*3.1416/180)*cos(phi*3.1416/180);
    float d2 = 3.5*sin(theta*3.1416/180)*sin(phi*3.1416/180);
    float d3 = 3.5*cos(theta*3.1416/180);
	printf("Angle phi changed to %.1f \n", phi );
	//AO_sinFreq = d;
	//printf("AO_sinFreq has been changed to %d.\n", AO_sinFreq);
	//outputV1 = d;

	set_coil_current_to (0, d1);
	set_coil_current_to (1, d2);
        set_coil_current_to (2, d3);
	//pthread_t currentcontrolthread;
	//pthread_create(&currentcontrolthread, NULL, currentControlThread, NULL);
}
void on_reset_field_button_clicked (GtkWidget *widget, gpointer data)
{
   	set_coil_current_to (0, 0);
	set_coil_current_to (1, 0);
    set_coil_current_to (2, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Magnet Detection Callbacks -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// magnet vision detection functions
void on_magnet_detection_toggled (GtkToggleButton *togglebutton, gpointer data) //magnet_detection toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_magnetdetection(d); //set magnetdetection variable in vision.c
}


void on_show_box_toggled (GtkToggleButton *togglebutton, gpointer data) //show_box toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_showbox(d); //set showbox variable in vision.c
}

void on_show_process_toggled (GtkToggleButton *togglebutton, gpointer data) //show_process toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_showprocess(d); //set showprocess variable in vision.c
}

void on_close_diameter_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("close diameter changed to %.1f units\n", d );
	set_closediameter(d);
}

// Magnet Feedback control thread functions -- Zhe
void on_test90_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		//init_test90_thread();
		init_test90();   // coilFieldControl.c
	}else
	{
		//stop_test90_thread();
		stop_test90();
	}
}

void on_kp_changed (GtkEditable *editable, gpointer user_data) //change Kp in controller
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("kp changed to %.1f units\n", d );
	set_kp(d);
}

void on_ki_changed (GtkEditable *editable, gpointer user_data) //change Ki in controller
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("ki changed to %.1f units\n", d );
	set_ki(d);
}

void on_destination_angle_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("destination angle set to %.1f units\n", d );
	set_destination_angle(d);
}

void on_show_destination_toggled (GtkToggleButton *togglebutton, gpointer data) //show_destination toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("show_destination toggled to %i\n", d );
	set_showdestination(d); //set showprocess variable in vision.c
}

void on_show_field_direction_toggled (GtkToggleButton *togglebutton, gpointer data) //show_field_direction toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("show_field_direction toggled to %i\n", d );
	set_showfielddirection(d); //set showfielddirection variable in vision.c
}

void on_reverse_field_toggled (GtkToggleButton *togglebutton, gpointer data) //reverse_field toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("reverse_field toggled to %i\n", d );
	set_reversefield(d); //set reversefield variable in coilFieldControl.c
}

void on_bending_go_toggled (GtkToggleButton *togglebutton, gpointer data) //bending_go toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("bending_go toggled to %i\n", d );
	set_bending_go(d); //set bending_go variable in coilFieldControl.c
}

void on_torque_angle_changed (GtkEditable *editable, gpointer user_data) // change field-magnetization angle
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("destination angle set to %.1f units\n", d );
	set_torque_angle(d);
}

// temperature control thread callbacks
void on_temp_control_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to perform temperature control
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_temp_control();   // TemperatureFeeding.c
	}
	else
	{
		stop_temp_control();
	}
}

void on_destination_temp_changed (GtkEditable *editable, gpointer user_data) // change destination temperature
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("destination temperature set to %.1f units\n", d );
	set_destination_temp(d);
}

void on_temp_go_toggled (GtkToggleButton *togglebutton, gpointer data) // temperature go/stop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("bending_go toggled to %i\n", d );
	set_temp_go(d); //set temp_go variable in coilFieldControl.c
}

// rotational field thread functions
void on_rotational_field_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to generate a rotational field
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_rotational_field();   // coilFieldControl.c
	}
	else
	{
		stop_rotational_field();
	}
}

void on_fab_amp_changed (GtkEditable *editable, gpointer user_data) // change micro-fabrication amplitude
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("amplitude set to %.1f units\n", d );
	set_fab_amp(d);
}

void on_fab_fre_changed (GtkEditable *editable, gpointer user_data) // change micro-fabrication frequency
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("frequency set to %.1f units\n", d );
	set_fab_fre(d);
}

void on_rotationalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data) // rotational field go/stop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("rotational field_go toggled to %i\n", d );
	set_rotationalfield_go(d); //set rotational field_go variable in coilFieldControl.c
}

// automatic feeding thread callbacks
void on_auto_feeding_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to perform automatic feeding
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d) //if button is toggled up
	{
		init_auto_feeding();   // TemperatureFeeding.c
	}
	else
	{
		stop_auto_feeding();
	}
}

void on_feeding_distance_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("feeding distance set to %.1f units\n", d );
	set_feeding_distance(d);
}

void on_feeding_speed_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("feeding speed set to %.1f units\n", d );
	set_feeding_speed(d);
}

void on_feeding_go_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("feeding go toggle button set to %.1f units\n", d );
	set_feeding_go(d);
}

void on_feeding_increments_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("feeding increments is set to %.1f units\n", d );
	set_feeding_increments(d);
}

void on_manual_override_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to perform manual
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d) //if button is toggled up
	{
		init_manual_feeding();   // TemperatureFeeding.c
		printf("manual override set.\n");
	}
	else
	{
		stop_manual_feeding();
		printf("manual override stopped.\n");
	}
}

void on_feeder_extend_button_clicked (GtkWidget *widget, gpointer data)
{
    feederIncrement();
    //printf("increment clicked\n");
}

void on_feeder_retract_button_clicked (GtkWidget *widget, gpointer data)
{
    feederDecrement();
    //printf("decrement clicked\n");
}

//automatic fabrication callbacks
void on_shape_read_clicked (GtkWidget *widget, gpointer data) // read shape data from text file
{
    line_number = 0;
    char filename[] = "microrobot_shape.txt";
	FILE *file = fopen(filename, "r");
	char *line = NULL;
	size_t len = 0;
	if (file == NULL)
		printf("Cannot open file: %s\n", filename);
	while ((getline(&line, &len, file)) != -1)
	{
		microrobot_shape[line_number] = atof(line);
		line_number++;
	}

	printf("number of lines: %d\n", line_number);
	for (int i=0; i<line_number; i++)
    {
        printf("%.2f ", microrobot_shape[i]);
    }
    printf("\n");

	free(line);
	fclose(file);
    //gtk_entry_set_text(display_fab_status, "Data read.");
    printf("Date read.\n");
    sprintf(fab_status, "Shape read.");
}

void on_shape_fab_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d)
	{
		init_auto_fab(); // AutoFabrication.c
	}
	else
	{
		stop_auto_fab(); // AutoFabrication.c
	}
}

// make an "S" callbacks
void on_s_go_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d)
	{
		init_make_s(); // AutoFabrication.c
	}
	else
	{
		stop_make_s(); // AutoFabrication.c
	}
}

void on_radius_s_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("radius of s set to %.1f units\n", d );
	set_radius_s(d);
}

void on_time_s_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("time of s set to %.1f units\n", d );
	set_time_s(d);
}

// field control callbacks
void on_coil_selection_toggled (GtkToggleButton *togglebutton, gpointer data) // which coil system are we using
{
    int d = gtk_toggle_button_get_active(togglebutton);
	set_factor(d); // d=1, 3D coil system; d=0 2D coil system
}

void on_field1_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_field_xyz (0, d);
}

void on_field2_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_field_xyz (1, d);
}

void on_field3_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_field_xyz (2, d);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void on_AO_sineFreq_activate (GtkEntry *entry, gpointer  user_data)
{
	uint d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(entry));
	printf("Freq changed to %d \n", d );
}


void on_AO_sineFreq_editing_done (GtkCellEditable *editable, gpointer user_data)
{
	uint d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("Freq changed to %d \n", d );
}
*/

void on_cannyLow_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyLow changed to %.1f units\n", d );
	setcannyHigh_vision((int)d);
	//cannyLow = d; //change frequency
	//img_test(); //test opencv. remove this later?
}
void on_cannyHigh_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyHigh changed to %.1f units\n", d );
	setcannyLow_vision((int)d);
	//cannyHigh = d; //change frequency
	//img_test(); //test opencv. remove this later?
}
void on_gain_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("gain changed to %.1f units\n", d );
	setGain_vision(d);
}
void on_shutter_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("shutter changed to %.1f units\n", d );
	setShutter_vision(d);
}
void on_dilate_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("dilate changed to %.1f units\n", d );
	setDilate_vision(d);
}
void on_visionParam1_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam1_vision(d);
}
void on_visionParam2_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam2_vision(d);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// edgemap toggle button
void on_edgemap_xz_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_edgemap_xz(d); //set edgemap variable in vision.c
}
void on_binary_xz_toggled (GtkToggleButton *togglebutton, gpointer data) //edgemap toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	set_binary_xz(d); //set edgemap variable in vision.c
}

void on_gain_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("gain changed to %.1f units\n", d );
	setGain_xz_vision(d);
}
void on_shutter_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("shutter changed to %.1f units\n", d );
	setShutter_xz_vision(d);
}

void on_visionParam1_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam1_xz_vision(d);
}
void on_visionParam2_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam2_xz_vision(d);
}

void on_sidecam_on_toggled (GtkToggleButton *togglebutton, gpointer data) {
	int d = gtk_toggle_button_get_active(togglebutton);
	setSideCam_vision(d);

	if(d==1) //if button is toggled up
	{
		GUI_refresh_flag_vidWin2 = 1;
		//if (GUI_refresh_flag_vidWin1 != 1)   // If
		//	pthread_create(&GUI_refresh_thread_instance, NULL, GUI_refresh_thread, NULL);  //start control loop thread
	} else
	{
		GUI_refresh_flag_vidWin2 = 0;
	}

	if(visionStarted) //if the vision is already started, restart it with new sidecam choice
	{
		usleep(1e5);
		stopVision();
		visionStarted = FALSE;
		usleep(20e5);
		g_print("Restarting vision thread.\n");
		initVision();
		visionStarted = TRUE;
		usleep(20e5);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//calibrate the sensors
void on_zero_output_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("zero output changed to %.1f V\n", d );
	zout = d;
}
void on_gaussmeter_output1_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("gaussmeter changed to %.1f mT\n", d );
	gout = d;
}
void on_oscilloscope_output1_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("gaussmeter changed to %.1f V\n", d );
	osout = d;
}
void on_calibrate_clicked (GtkWidget *widget, gpointer data)
{

	k=(osout-zout)*100.00/gout;
	printf("sensitivity of the sensor is %.1f mV/G\n", k );

}
void on_oscilloscope_output2_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("zero output changed to %.1f V\n", d );
	osout_value = d;
}
void on_measure_clicked (GtkWidget *widget, gpointer data)
{

	value=osout_value*1000/k;
	printf("the value is %.1f G\n", value );

}

void on_b_ms_trial_clicked (GtkWidget *widget, gpointer data)
{
    initiate_multiswimmer_trial();
}

void on_swimmer_ac_angle_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_angle_difference(d);
}
void on_b_ms_stop_clicked (GtkWidget *widget, gpointer data)
{
    stop_everything();
}
void on_ad_net_magnet_dir_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_netmagnetizationdir(d);
}

void on_ad_net_magnet_dir2_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_netmagnetization2nddir(d);
}

// Get Net Magnetization Dir.
void on_b_netMDir_clicked (GtkWidget *widget, gpointer data)
{
    mS_getNetMagnetizationDir();                                    // in multiSwimmertrial.c
}

////// Two swimmer UT follow ///////////////
void on_b_ms_UT_follow_clicked (GtkWidget *widget, gpointer data)
{
    initiate_UT_follow();
}


/// Jiachen Zhang
void on_tB_cilia_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);
    cilia_buttonToggled(d);                                      // in coilFieldControl.c
}

///


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAGNET CONTROL
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void on_max_speed_button_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the Bz
{
    /*
    setup_constants();

    float f;
    float grad[8];
    float hess[8][8];

    float theta[8]={0,0,0,0,0,0,0,0};
    float K = 5;
    float m_pos[3]={0,0,0};
    float m[3]={0,1,1};
    float Bo[3]={1e-2,0,1e-2};
    float Fo[3]={0.5e-6,0.5e-6,0};
    float parameters[10]={2,0.007,5,0,0,0,0,0,0,3};


    struct timeval start;
    unsigned long time1;
    unsigned long time2;
    float time_dif;
    */

    ///Calculate Gradient and Hessian
    /*
    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;

    for(int i=0; i<10; i++)
    {
        f = f_grad_hess__norm2 (grad, hess,theta,2,K,m_pos,m,Bo,Fo);
    }

    gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1)*1e-6;
    printf("Time to calculate grad,hess: %.4f \n",time_dif );
    */

    ///Calculate Newton Direction
    /*
    float direction[M_NUM];
    Newtondirection(direction, grad, hess);

    cout << "dir: " << endl;
    for(int i=0;i<M_NUM;i++)
    {
        cout << direction[i] << endl;
    }
    */

    ///Conduct test to determine average freq of minimum finding
    /*
    float theta_end[M_NUM];

    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;


    for (int m_1=-1; m_1<2; m_1++)
    {
        m[0]=m_1; m[1]=1; m[2]=1;
        unitv(m,3);
        m[0] = m[0]*m_mag; m[1] = m[1]*m_mag; m[2] = m[2]*m_mag;

        for (int B_2=-1; B_2<2; B_2++)
        {
            Bo[1]=B_2*1e-2;

            for (int F_3=-1; F_3<2; F_3++)
            {
                Fo[2]=F_3*(0.5)*1e-6;

                for (int t_1=-1; t_1<2; t_1++)
                {
                    theta[0]=t_1;

                    for (int t_2=-1; t_2<2; t_2++)
                    {
                        theta[1]=t_2;

                        for (int t_3=-1; t_3<2; t_3++)
                        {
                            theta[2]=t_3;

                            localminNewton_norm2(theta_end,parameters,theta,m_pos,m,Bo,Fo);


                            cout << endl << "theta: ";
                            for(int i=0;i<M_NUM;i++)
                            {
                                cout << theta_end[i]*180/PI << " ";
                            }
                            cout << endl;


                        }
                    }
                }
            }
        }
    }

    gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1);
    printf("Time to find min: %.4f \n",time_dif*1e-6 );
    */




    ///Eigen Matrix Initialization
    /*
    Matrix8f H;
    Vector8f G;

    for(int i=0; i<M_NUM; i++)
    {
        G(i) = grad[i];
        for(int j=0; j<M_NUM;j++)
        {
            H(i,j)=hess[i][j];
        }
    }

    cout << "Here is the matrix H:\n" << H << endl;
    cout << "Here is the vector G:\n" << G << endl;
    cout << endl;
    */


    /// Try the Ax=b solving methods
    /*
    Vector8f x; //Hx=G;

    x = H.fullPivLu().solve(G);
    cout << "The solution is:\n" << x << endl;
    double relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;

    x = H.householderQr().solve(G);
    cout << "The solution is:\n" << x << endl;
    relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;

    x = H.colPivHouseholderQr().solve(G);
    cout << "The solution is:\n" << x << endl;
    relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;

    x = H.fullPivHouseholderQr().solve(G);
    cout << "The solution is:\n" << x << endl;
    relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;
    */

    ///Test speed and accuracy of Ax=b solving methods
    /*
    Vector8f x; //Hx=G;
    double relative_error;
    ///////////////////////////////////////////////////////////////////////////////////
    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;

    for(int i=0; i<5000; i++)
    {
        x = H.householderQr().solve(G);
    }

    gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1)*1e-6;
    printf("householderQr Time: %.4f \n",time_dif );
    cout << "The solution is:\n" << x << endl;
    relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;



    ///////////////////////////////////////////////////////////////////////////////////
    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;

    for(int i=0; i<5000; i++)
    {
        x = H.fullPivLu().solve(G);
    }

    gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1)*1e-6;
    printf("fullPivLu Time: %.4f \n",time_dif );
    cout << "The solution is:\n" << x << endl;
    relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;

    ////////////////////////////////////////////////////////////////////////////////////////
    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;

    for(int i=0; i<5000; i++)
    {
        x = H.fullPivHouseholderQr().solve(G);
    }

    gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1)*1e-6;
    printf("fullPivHouseholderQr Time:%.4f \n",time_dif );
    cout << "The solution is:\n" << x << endl;
    relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;

    //////////////////////////////////////////////////////////////////////////////////////
    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;

    for(int i=0; i<5000; i++)
    {
        x = H.colPivHouseholderQr().solve(G);
    }

    gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1)*1e-6;
    printf("colPivHouseholderQr Time: %.4f \n",time_dif );
    cout << "The solution is:\n" << x << endl;
    relative_error = (H*x - G).norm() / G.norm(); // norm() is L2 norm
    cout << "The relative error is:\n" << relative_error << endl;
    cout << endl;
    */


    ///Try Eigenvalue methods
    /*
    Matrix8f H2;
    for(int i=0; i<M_NUM; i++)
    {
        G(i) = grad[i];
        for(int j=0; j<M_NUM;j++)
        {
            H2(i,j)=H(i,j);
        }
    }

    SelfAdjointEigenSolver<Matrix8f> eigensolver(H);
    cout << eigensolver.eigenvalues() << endl;

    EigenSolver<Matrix8f> eigensolver2(H2);
    cout << eigensolver2.eigenvalues() << endl;

    // - How to get eigenvalues for updated Hessian matrix???
    H(3,3) = 6.0;
    //eigensolver(H);
    cout << eigensolver.eigenvalues() << endl;
    */

    /// Test the speed of SelfAdjoint EigenSolver vs. EigenSolver
    /*
    Matrix8f H2;
    for(int i=0; i<M_NUM; i++)
    {
        G(i) = grad[i];
        for(int j=0; j<M_NUM;j++)
        {
            H2(i,j)=H(i,j);
        }
    }
    ////////////////////////////////////////////////////////////////////////////
    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;

    SelfAdjointEigenSolver<Matrix8f> eigensolver(H);
    cout << eigensolver.eigenvalues() << endl;

        gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1);
    printf("SelfAdjointEigenSolver Time: %.4f \n",time_dif );

    ///////////////////////////////////////////////////////////////////////////
    gettimeofday(&start, NULL);
    time1  = (double) start.tv_sec*1e6+ start.tv_usec;

    EigenSolver<Matrix8f> eigensolver2(H2);
    cout << eigensolver2.eigenvalues() << endl;

        gettimeofday(&start, NULL);
    time2  = (double) start.tv_sec*1e6+ start.tv_usec;
    time_dif = (time2-time1);
    printf("EigenSolver Time: %.4f \n",time_dif );
    */

    //float d[8]={1, 2, 3, 4, 5, 6, 7, 8};
    //test_arraypassing(d);

    ////////// Actual Button Function ///////////////////////////////

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	max_v = d;
	printf("Speed: %f \n",max_v);

}


void on_max_accel_button_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the Bz
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	max_a= d;
	printf("Accel: %f \n",max_a);
}

void* recorddata_thread (void*threadid)
{

    uint aiChan = 1 | (1<<11);   // Set AI0 and AI-11.
	printf("aiChan is 0x%x.\n",aiChan);
	double aiV[16];             // Buffer to receive the AI reading.

	uint aiSlot = 0;
	uint tsettle = 500;   // settling time in microseconds
	uint aiRangeCode = 1; // 1: -5 ~ +5 V.

	fp=fopen("zHallTest0.txt","w");
	s826_aiInit(aiChan,aiRangeCode);

    while (record_on == true)
    {
        s826_aiRead(aiChan,aiV);

		//printf("The channel 3 value is %.2f.\n",aiV[11]);
		fprintf(fp,"%f\n",aiV[11]);

		waitUsePeriodicTimer(1e3);//1e5:10hz    5e4:20hz  2e4:50hz 1e4:100hz    5e3:200Hz  2e3:500Hz   1e3:1000Hz

    }

	fclose(fp);
}

void on_magent_b1_clicked (GtkWidget *widget, gpointer data)
{

    float d = 40.0; //number of steps to rotate
    float d_array[8];
    for(int i=0; i<8; i++)
            d_array[i]=d;

    motors_moveFunction(d_array);
}

void on_magnetdiagnose_clicked (GtkWidget *widget, gpointer data)               // diagnose button
{
    float d = 4*400;
    //for(int i=0; i<8; i++)
            //motor_position[i]=d;


    motor_setSpeedAcceleration(200,4500);

    int i=6;
    motor_position[i]=d;


    pthread_t recordthread;
    pthread_create(&recordthread, NULL, recorddata_thread, NULL);  //start swimmer thread
    record_on = true;


    motors_moveToFunction (motor_position);

    int num_turning = 1;

    while(num_turning>0)
        num_turning = check_motorsdonemoving();

    record_on = false;
    //motors_unlockFunction();




}

void on_reset_clicked (GtkWidget *widget, gpointer data) //reset button
{
    float d = 0;
    for(int i=0; i<8; i++)
            motor_position[i]=d;

    motors_moveToFunction (motor_position);

}

void on_fileinput_clicked(GtkWidget *widget, gpointer data)                    //stop button
{
    const int num_jukes = 8;
	float e[num_jukes] = {20, -20, 20, -20, 20,-20,20,-20};
	int position_int=0;
	int end_reached;
	float distance=0;
	while (position_int < num_jukes) {
        printf("Setting new positions %d \n",position_int);
        for(int i=0; i<8; i++)
        {
            motor_position[i]=e[position_int];
        }

        motors_moveToFunction(motor_position);


		end_reached=1;


		while (end_reached!=0)
		{
		    end_reached = check_motorsdonemoving();
		    //printf("Num motors moving %d \n",end_reached);
		}
		printf("Reached position %d \n",position_int);

		position_int++;
	}
}

void on_togglebutton_feedback_toggled (GtkToggleButton *togglebutton, gpointer data) //feedback toggle button
{

	int d = gtk_toggle_button_get_active(togglebutton);

    int control_type = 2;
    // 0: horizontal click waypoint following
    // 1: horizontal pre-set waypoint following
    // 2: horizontal force heading
    // 3: vertical pre-set waypoint following

	if(d==1) //if button is toggled up
	{
		init_magnetcontrol_thread(control_type);
	}
	else
	{
		stop_magnetcontrol_thread();
	}
}

void on_magnet_lock_clicked (GtkWidget *widget, gpointer data)
{
    motors_lockFunction(use_magnets);
}

void on_magnet_unlock_clicked (GtkWidget *widget, gpointer data)
{
    motors_unlockFunction();
}

void on_fileinput_frequency_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag4
{
	//float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("fileinput_frequency changed to %.1f \n", d );
	//fileinput_frequency = d;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_magnet_go_clicked (GtkWidget *widget, gpointer data)
{
	printf("go\n");

	motorgo=false;
	usleep(1e3);
	motorgo=true;

    temp_gobutton_input();

	setup_constants(); // ideally this should go in same spot as s826_init (i.e. the window opening function)

	if (magnet_flag ==1) //magnet angles entered
	{
		for (int i=0; i<8; i++)
		{
			magnet_rotR[i] = magnet_rotD[i]*PI/180;
			motor_position[i]=magnet_rotD[i]/360*400;
		}

		motors_moveToFunction(motor_position);

		float B[3], F[3];

		float m_pos[3] = {-0.00, 0.00, 0.0}; // microrobot position [m] - this will have to be provided by the control system
		float Bo_dir[3] = {0,0,0};
        float m[3];

        m[0]=m_mag*Bo_dir[0]; m[1]=m_mag*Bo_dir[1]; m[2]=m_mag*Bo_dir[2];
        field_force(B,F,magnet_rotR, m_pos, m);

		printf("\nActual  Field      Force\n");
		for (int i=0; i<3; i++)
		{
			printf("      %f    %f\n", B[i]*1000, F[i]*1e6);
		}

	}

	else if (magnet_flag ==2) //magnet B and F entered
	{

		printf("Desired Field Force\n");
		float Bo_dir[3];
		for (int i=0; i<3; i++)
		{
			printf(" %f   %f\n",Bo[i]*1000,Fo[i]*1e6);

			Bo_dir[i]=Bo[i];
		}
		unitv(Bo_dir,3); //get a unit vector in the direction of Bo - to define microrobot direction

		float m_pos[3] = {0.0, 0.0, 0.0}; // microrobot position [m] - this will have to be provided by the control system

	    float m[3];
	    //m[0]=m_mag*Bo_dir[0]; m[1]=m_mag*Bo_dir[1]; m[2]=m_mag*Bo_dir[2];

		//float M_theta[M_NUM];
	    int iter_max = 400;
	    //float BFaccept[6] = {15, 1e-3, 50e-3, 15, 0.5e-7, 3e-6}; //[Bangle_err Bmag_min Bmag_max Fangle_err Fmag_min Fmag_max ]
		float BFaccept[6] = {20, 5e-3, 32e-3, 20, 0.1e-6, 3e-6}; //[Bangle_err Bmag_min Bmag_max Fangle_err Fmag_min Fmag_max ]
        float parameters[] = {0.2, 0.1,15}; // step size and cutoff
	    float M_theta0[] ={0,0,0,0,0,0,0,0};
	    sufficientlocalminGrad_norm(magnet_rotR, M_theta0, iter_max, BFaccept, parameters,m_pos, m, Bo, Fo);

        ///Or use preset angles
        //float temp_angles[8] = {2.7137,   -0.7815  , -0.3728	,0.5066	,1.6702	,0.7268	,3.4936  , -0.9307};
        //float temp_angles[8] = {2.6328,   -3.0189,   -0.1024,   -0.1392,	0.6519,   -1.2305	,0.0828 ,  -1.5239};
        //for (int jj=0; jj<8; jj++)
        //{
        //magnet_rotR[jj] = temp_angles[jj];
        //}

        float B[3], F[3];

	    field_force(B,F,magnet_rotR, m_pos, m);

		for (int i=0; i<8; i++)
			motor_position[i]=magnet_rotR[i]*(180.0/PI)*(400.0/360.0);

        motors_moveToFunction(motor_position);

		printf("\nActual  Field      Force\n");
		for (int i=0; i<3; i++)
		{
			printf("      %f    %f\n", B[i]*1000.0, F[i]*1.0e6);
		}
		printf("\n\n");

		printf("Angles:\n");
		for (int i=0; i<8; i++)
		{
			printf(" %.1f ", magnet_rotR[i]*180.0/PI);
			//printf(" %.1f \n", motor_position[i]);
		}
		printf("\n");

	}

	else if(magnet_flag ==3)
	{

	}
}



void on_magnet_stop_clicked (GtkWidget *widget, gpointer data)
{
	printf("stop\n");
	motorgo=false;
	record_on = false;
}



void on_allmagnets_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled on
	{

		nmagnets=8;

		for (int i=0; i<8; i++)
			use_magnets[i] = i+1;

		printf("Using all magnets\n");

    motor_setSpeedAcceleration(max_v,max_a);

	}


	else
	{
		nmagnets=0;
		for (int i=0; i<8; i++)
			use_magnets[i] = 0;
	}
}



void on_magnet1_toggled (GtkToggleButton *togglebutton, gpointer data) //lock toggle button
{
	int magnet_id = 1;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{
		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}

				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}

void on_magnet2_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int magnet_id = 2;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{
		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}
				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}
void on_magnet3_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int magnet_id = 3;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{
		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}

				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}

// toggle two-gripper path following
void on_twoGripper_pathFollow_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active(togglebutton);
    twoGripper_start_or_stop_path_follow (d);
}



void on_magnet4_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int magnet_id = 4;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{
		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}

				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}
void on_magnet5_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int magnet_id = 5;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{
		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}

				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}
void on_magnet6_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int magnet_id = 6;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{
		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}

				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}
void on_magnet7_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int magnet_id = 7;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{

		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}

				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}
void on_magnet8_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int magnet_id = 8;////////////////////////////////////////////////////////////////////////////////////////////////

	int d = gtk_toggle_button_get_active(togglebutton);

	int temp[8];

	if(d==1) //if button is toggled on
	{

		nmagnets++;

		int ind=0;


	 	while (ind<8)
		{
			if (magnet_id < use_magnets[ind])
			{

				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				use_magnets[ind] = magnet_id;


				for(int j=ind+1; j<8; j++)
				{
					use_magnets[j]=temp[j-1];
				}

				ind=9;

			}
			else if(use_magnets[ind]==0)
			{
				use_magnets[ind] = magnet_id;
				ind = 9;
			}
			else
			ind++;

		}

	}
	else
	{
		nmagnets--;
		for (int ind=0; ind<8; ind++)
		{
			if (use_magnets[ind]==magnet_id)
			{
				for(int j=0; j<8; j++)
				{
					temp[j]=use_magnets[j];
				}

				for(int j=ind; j<8-1; j++)
				{
					use_magnets[j] = temp[j+1];
				}
				use_magnets[8]=0;
			ind=9;
			}
		}
	}

	//for(int i=0; i<8; i++)
	//{
	//	printf("Work: %d \n",use_magnets[i]);
	//}
}

void on_angle_mag1_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag1
{
	int magnet_id = 1;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag = 1;

	printf("angle_mag1 changed to %.1f \n", d );

	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor1.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor1.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor1.set_enable_pin(motor_parm[magnet_id-1][2]);

}

void on_angle_mag2_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag2
{
	int magnet_id = 2;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag = 1;

	printf("angle_mag2 changed to %.1f \n", d );

	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor2.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor2.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor2.set_enable_pin(motor_parm[magnet_id-1][2]);



}

void on_angle_mag3_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag3
{
	int magnet_id = 3;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag=1;

	printf("angle_mag3 changed to %.1f \n", d );


	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor3.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor3.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor3.set_enable_pin(motor_parm[magnet_id-1][2]);

}

void on_angle_mag4_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag4
{
	int magnet_id = 4;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag=1;

	printf("angle_mag4 changed to %.1f \n", d );


	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor4.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor4.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor4.set_enable_pin(motor_parm[magnet_id-1][2]);

}
void on_angle_mag5_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag1
{
	int magnet_id = 5;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag=1;

	printf("angle_mag5 changed to %.1f \n", d );

	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor5.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor5.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor5.set_enable_pin(motor_parm[magnet_id-1][2]);
}

void on_angle_mag6_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag2
{
	int magnet_id = 6;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag=1;

	printf("angle_mag6 changed to %.1f \n", d );


	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor6.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor6.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor6.set_enable_pin(motor_parm[magnet_id-1][2]);
}

void on_angle_mag7_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag3
{
	int magnet_id = 7;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag=1;

	printf("angle_mag7 changed to %.1f \n", d );

	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor7.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor7.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor7.set_enable_pin(motor_parm[magnet_id-1][2]);
}
void on_angle_mag8_changed (GtkEditable *editable, gpointer user_data)    //set angle for the mag4
{
	int magnet_id = 8;

	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	magnet_rotD[magnet_id-1] = d;

	magnet_flag=1;

	printf("angle_mag8 changed to %.1f \n", d );


	//Quadstep Initialization
	//(These commands result in program failure if s826_init() not called))
	//motor8.set_step_pin(motor_parm[magnet_id-1][0]);
	//motor8.set_direction_pin(motor_parm[magnet_id-1][1]);
	//motor8.set_enable_pin(motor_parm[magnet_id-1][2]);
}


void on_stepsize_changed (GtkEditable *editable, gpointer user_data)    //set stepsize
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("stepsize changed to %.1f \n", d );
	step_size = d;
}


////
void on_Bx_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the Bx
{
    /*
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("Bx changed to %.1f \n", d );
	Bo[0] = d/1000; //convert to T
	magnet_flag=2;
	*/


    pthread_t recordthread;
    pthread_create(&recordthread, NULL, recorddata_thread, NULL);  //start swimmer thread
    record_on = true;



}

void on_By_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the By
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("By changed to %.1f \n", d );
	Bo[1] = d/1000; //convert to T
	magnet_flag=2;
}

void on_Bz_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the Bz
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("Bz changed to %.1f \n", d );
	Bo[2] = d/1000; //convert to T
	magnet_flag=2;
}
void on_Fx_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the Fx
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("Fx changed to %.2f \n", d );
	Fo[0] = d/1e6; //convert to N
	magnet_flag=2;
}
void on_Fy_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the Fy
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("Fy changed to %.2f \n", d );
	Fo[1] = d/1e6; //convert to N
	magnet_flag=2;
}

void on_Fz_changed (GtkEditable *editable, gpointer user_data)    //set magnitude for the Fz
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("Fz changed to %.2f \n", d );
	Fo[2] = d/1e6; //convert to N
	magnet_flag=2;
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////



gboolean on_videoWindow_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data)
{
	int click[2];
	click[0] = (int)event->x; //x position from top left in pixels
	click[1] = (int)event->y; //y position from top left in pixels
	int button_click = event->button; //which mouse button was clicked
	//g_print("Top video window %d click at location [%d %d].\n", button_click, click[0], click[1]);
	setMouse(0, button_click, click );      //void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
}
gboolean on_videoWindow2_button_press_event(GtkWidget *widget, GdkEventButton *event, gpointer data)
{
	int click[2];
	click[0] = (int)event->x; //x position from top left in pixels
	click[1] = (int)event->y; //y position from top left in pixels
	int button_click = event->button; //which mouse button was clicked
	//g_print("Side video window %d click at location [%d %d].\n", button_click, click[0], click[1]);
	setMouse(1, button_click, click );      //void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
}


// A gboolean is an int that should only hold two values, TRUE and FALSE (uppercase). FALSE is defined as 0 and TRUE is defined as !FALSE.
gboolean key_event (GtkWidget *widget, GdkEventKey *event)
{
	printf("%s\n", gdk_keyval_name (event->keyval)  );
	keyPressed[ event->keyval ] = TRUE; 			//set the appropriate key value true

	if(
  //  ((event->keyval >= GDK_KEY_0 ) && (event->keyval <= GDK_KEY_9)
//	    || (event->keyval == GDK_KEY_period)  )
        (event->keyval == GDK_KEY_period)
	    || (event->keyval == GDK_KEY_BackSpace)
	    || (event->keyval == GDK_KEY_Delete)
	    || (event->keyval == GDK_KEY_Return) )
		return FALSE; 					//register keystroke to gui normally for numbers

	/////////////////////////////////////////////////////////
	// For rotating field direction control - JZ
	/////////////////////////////////////////////////////////

	switch (event->keyval)
	{
		case GDK_KEY_Right: set_directional_index(0); break;   // +x
		case GDK_KEY_Up:    set_directional_index(1); break;   // +y
		case GDK_KEY_Left:  set_directional_index(2); break;   // -x
		case GDK_KEY_Down:  set_directional_index(3); break;   // -y

		case GDK_KEY_KP_6: set_directional_index_MA(0); break;   // +x
        case GDK_KEY_KP_8: set_directional_index_MA(1); break;   // +y
        case GDK_KEY_KP_4: set_directional_index_MA(2); break;   // -x
        case GDK_KEY_KP_2: set_directional_index_MA(3); break;   // -y
        case GDK_KEY_KP_9: set_directional_index_MA(4); break;   // 1st Quad
        case GDK_KEY_KP_7: set_directional_index_MA(5); break;   // 2nd Quad
        case GDK_KEY_KP_3: set_directional_index_MA(6); break;   // 3rd Quad
        case GDK_KEY_KP_1: set_directional_index_MA(7); break;   // 4th Quad
        case GDK_KEY_KP_5: stop_twoagentctrl_field();   break;   // stop field

	}

	g_main_context_invoke (NULL, update_driving_dir, NULL);

	/////////////////////////////////////////////////////////

	return TRUE; 						//returning true means that the keypress event wont be processed normally
}

gboolean key_event_release(GtkWidget *widget, GdkEventKey *event)
{
	//printf("_%s\n", gdk_keyval_name (event->keyval)  );
	keyPressed[ event->keyval ] = FALSE; 			//set the appropriate key value FALSE
	//printf("Got You In The Release !!!\n");
	return TRUE;
}

int sumt(int a, int b)
{
    return a+b;
}

/////////////////////////////////////////////////////////
// Magnetic Gripper
/////////////////////////////////////////////////////////

/// Rotation Demo
void on_b_rotationDemo_clicked (GtkWidget *widget, gpointer data)
{
    gripper_initRotationDemo();                                                         // initialize the gripper's rotation demo thread
}

/// Gripping Motion
void on_sb_gripping_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	printf("Gripping Motion changed to %.2f \n", d );
	gripper_setGrippingMotion(d);                                                       // change the level of gripping
}

/// Locomotion along X-Axis
void on_sb_gripperX_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    printf("Locomotion X changed to %.2f \n", d );
    gripper_setLX(d);                                                                   //  change level of locomotion along x-axis
}

/// Locomotion along Y-Axis
void on_sp_gripperY_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    printf("Locomotion Y changed to %.2f \n", d );
    gripper_setLY(d);                                                                   //  change level of locomotion along y-axis
}

/////////////////////////////////////////////////////////
// Multi_agent control
/////////////////////////////////////////////////////////
// Image processing
void on_imageprocessing_multiagent_clicked (GtkWidget *widget, gpointer data)
{

    printf("Welcome to Multi_agent project \n");

// printf("angle:  %f /n", (180/M_PI)*adjust_angle_local_MA(220));

// ****printf("angle:  %f /n", (180/M_PI)*adjust_angle_local_MA(adj_angle_map*M_PI/180) );

}

// set orientation circle detection toggle button

void on_cB_3rdcirc_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active(togglebutton);
    if (d == 1)
        flag_orinet_circ_required = 1;
    else
        flag_orinet_circ_required = 0;
    set_1st_orient_circ (d);                                   // vision.c    ---> to activate Hough circle detections
}

void on_show_agent_circle_toggled (GtkToggleButton *togglebutton, gpointer data) //show_box toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	set_agentshowcirc(d); //set showagent variable in vision.c
}

void on_preProcessMA_toggled (GtkToggleButton *togglebutton, gpointer data) { //pre-process toggle button
	int d = gtk_toggle_button_get_active(togglebutton);
	set_agentpreprocess(d); //set preprocessVar in vision.c
}

// set orientation circle detection min radius spin button

void on_visionHough_minRadius_changed (GtkEditable *editable, gpointer user_data) {
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_HoughMinRadius_vision(d);
}

// set orientation circle detection max radius spin button

void on_visionHough_maxRadius_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_HoughMaxRadius_vision(d);
}


void on_Houghcanny1_changed(GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_Houghcanny1_vision(d);
}

void on_Houghcanny2_changed(GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_Houghcanny2_vision(d);
}


void on_HoughminDis_changed(GtkEditable *editable, gpointer user_data) {
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_HoughminDis_vision(d);
}

// set the level of binary adjustment
void on_HoughBinary_changed (GtkEditable *editable, gpointer user_data) {
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_HoughBinary(d);
}

// set the level of erosion adjustment

void on_Hougherosion_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_HoughErosion(d);
}



// set the level of dilation adjustment

void on_Houghdilation_changed (GtkEditable *editable, gpointer user_data) {
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvision_HoughDilation(d);
}


// Microrobot fabrication using coil system                                           // To fabricate multi-agent robots

void on_robot_fabrication_clicked (GtkWidget *widget, gpointer data)
{

printf("Set the value of external field in voltage (1V :: 5mT) \n");

//mS_getNetMagnetizationDir_MA();

}

void on_moldfieldmag_changed (GtkEditable *editable, gpointer user_data)            // To fabricate multi-agent robots
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//set_alignFieldMag(d);
}

// Separate 3-coil field selection from 2-coil* setup       written by Mohammad on July 10, 2015
void on_3coil_toggled (GtkToggleButton *togglebutton, gpointer data) {         //3-coil setup selection
	int d = gtk_toggle_button_get_active(togglebutton);
    if (d == 1)
        flag_3coil_setup = 1;
    else
        flag_3coil_setup = 0;
}

// Separate 2-coil field selection from 2-coil* setup       written by Mohammad on July 10, 2015
void on_2coil_toggled (GtkToggleButton *togglebutton, gpointer data)         //3-coil setup selection
{
	int d = gtk_toggle_button_get_active(togglebutton);

    if (d == 1)
        flag_2coil_setup = 1;
    else
        flag_2coil_setup = 0;
}

// toggle start/stop 2 gripper control (high level & low level)
void on_twoGripper_start_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active(togglebutton);
    twoGripper_start_or_stop(d);                        // twoGripper.c
}

// define cargo positions by mouse clicking
void on_twoGripper_specifyCargo_clicked (GtkWidget *widget, gpointer data) {
    twoGripper_define_cargo_pos();                      // twoGripper.c
}

// Two Agent Control Test ::: Multi-agent created by Mohammad
void on_2Agent_MA_toggled (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active(togglebutton);
    set_2Agentctrl(d);
}

void on_2Agents_stop_clicked (GtkWidget *widget, gpointer data)
{
    stop_twoagentctrl_field();
}

void on_MultiAgent_pX_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(0);
}
void on_MultiAgent_pY_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(1);
}
void on_MultiAgent_nX_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(2);
}
void on_MultiAgent_nY_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(3);
}

void on_MA_1stQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(4);   // Move to +x +y 45 degree
}

void on_MA_2ndQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(5);
}

void on_MA_3rdQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(6);
}

void on_MA_4thQuad_clicked (GtkWidget *widget, gpointer data)
{
    set_directional_index_MA(7);
}

void on_HoughBlur_changed (GtkEditable *editable, gpointer user_data) {            // To set Blur Marker Size in Hough transform
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_HoughBlur(d);
}

void on_HoughMindistance_changed (GtkEditable *editable, gpointer user_data)            // To change Hough Binary Threshold
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_HoughMindistance(d);
}


void on_HoughSwapRatio_changed (GtkEditable *editable, gpointer user_data)            // To change Hough Swap Ratio between true and false states of swapping
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_HoughSwapratio(d);
}




// Binary control law  implementation in multi-agent project by Mohammad
void on_Binarycontrollaw_clicked (GtkWidget *widget, gpointer data) {
    printf("start multi-agent Binary controller \n");
    Binary_control_law();           // multiagent.c
}

// constrain to water 2 control inputs  implementation in multi-agent project      by Mohammad

void on_ConstrainControllaw_clicked (GtkWidget *widget, gpointer data)
{

printf("start multi-agent constrain to water surface controller for two agents \n");

//Constrain_control_law();

}



// Robust Adaptive control law  implementation in multi-agent project      by Mohammad

void on_Adaptivecontrollaw_clicked (GtkWidget *widget, gpointer data)
{

printf("start multi-agent Robust Adaptive Controller \n");

//Adaptive_control_law();

}





// 2-agent optimization implementation in multi-agent project      by Mohammad
void on_opt2agent_clicked (GtkWidget *widget, gpointer data) {
    printf("start two-agent optimization \n");
    //opt2agent_law_start();
}

void on_Pcontrol_toggled (GtkToggleButton *togglebutton, gpointer data) { //set up the P-controller
	int d = gtk_toggle_button_get_active(togglebutton);
    set_Pctrl_flag_on(d);
}

void on_Pcontrol_straight_toggled (GtkToggleButton *togglebutton, gpointer data) {      //set up the P-controller Straight-Line
	int d = gtk_toggle_button_get_active(togglebutton);
    set_Pctrl_Straight_flag_on(d);
}



void on_P_controlMA_changed (GtkEditable *editable, gpointer user_data) {            // To set K parameter in P-controller design
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_K_Pctrl_to(d);                                                   // assign a value to K-parameter in P_control design
}

void on_constrainK_changed (GtkEditable *editable, gpointer user_data)            // To set K parameter in constrained control law
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_K_Constrainedctrl_to(d);                                                   // assign a value to K-parameter in constrained control law design
}



void on_constrainFactor_changed(GtkEditable *editable, gpointer user_data)            // To set Factor parameter in constrained control law
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_Factor_Constrainedctrl_to(d);                                                   // assign a value to FActor-parameter in constrained control law design
}




void on_ROTperiod_MA_changed (GtkEditable *editable, gpointer user_data)            // To set the period of zero-rotation in P-controller design  :: how fast to correction the rotation
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_ROTperiod_to(d);                                                   // assign a value to the period of zero-rotationin P_control design
}








void on_desDis_MA_changed (GtkEditable *editable, gpointer user_data)            // To set desired distance in multi-agent
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_des_goalMA_to(d);                                                 // To set desired distance in multi-agent
}


void on_desAng_MA_changed (GtkEditable *editable, gpointer user_data)            // To set desired angle in multi-agent
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_des_angMA_to(d);                                                 // To set desired distance in multi-agent
           adj_angle_map = d;
}


void on_des_pull_ang_changed(GtkEditable *editable, gpointer user_data)            // To set desired pulling angle in multi-agent
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_des_PullangMA_to(d);                                                 // To set desired pulling angle in multi-agent
}



void on_Coil_ratio_changed(GtkEditable *editable, gpointer user_data)            // To set the ratio between gradients in Y and X pair of coils in multi-agent
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_pair_ratios_to(d);                                                 // To set desired pulling angle in multi-agent
}


void on_satAdj_changed(GtkEditable *editable, gpointer user_data) {            // To set saturation field in Y and X pair of coils in multi-agent
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_SATadj_to(d);                                                 // To set desired coil saturation in multi-agent
}





void on_pix2mm_MA_changed(GtkEditable *editable, gpointer user_data)            // To change the scale
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_pix2mm_to(d);                                                   // convert pix 2 mm
}

void on_Bmag_MA_changed(GtkEditable *editable, gpointer user_data) {           // To set external magentic field mag parameter in P-controller design
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_Bmag_MA_to(d);                                                   // assign a value to external magentic field mag-parameter in P_control design
    set_Bmag_MA_to_opt(d);                                             // assign a value to external magentic field mag-parameter in Optimization
}

void on_AdaptiveLambda_MA_changed(GtkEditable *editable, gpointer user_data)            // To set Lambda in Adaptive-controller design
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
          // set_AdaptiveLambda_MA_to(d);                                                   // assign a value to Lambda in Adaptive-controller design
}

void on_AdaptiveK_MA_changed(GtkEditable *editable, gpointer user_data)            // To set K in Adaptive-controller design
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

          // set_AdaptiveK_MA_to(d);                                                   // assign a value to K in Adaptive-controller design
}

void on_AdaptGammaF_MA_changed(GtkEditable *editable, gpointer user_data)            // To set Fluid Gamma in Adaptive-controller design
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

          // set_AdaptiveGammaF_MA_to(d);                                                   // assign a value to Fluid Gamma  in Adaptive-controller design
}

void on_AdaptGammaM_MA_changed(GtkEditable *editable, gpointer user_data)            // To set Mass Gamma in Adaptive-controller design
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

         //  set_AdaptiveGammaM_MA_to(d);                                                   // assign a value to Fluid Mass  in Adaptive-controller design
}




void on_AdaptivePhi_MA_changed(GtkEditable *editable, gpointer user_data)            // To set Boundary thickness Phi in Adaptive-controller design
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

       //    set_AdaptivePhi_MA_to(d);                                                   // assign a value to Phi  in Adaptive-controller design
}

void on_moment_MA_changed(GtkEditable *editable, gpointer user_data)            // To set m in optimization 2-agent
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

   //        set_Optm_MA_to(d);                                                   // assign a value to m in 2-agent optimization
     //      set_Optm_MA_to_CO(d);                                                   // assign a value to m in 2-agent constrained to water surface

}


void on_ZfieldAdj_changed(GtkEditable *editable, gpointer user_data)            // To adjust z-field in constrained
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_zfieldAdj_MA_to(d);                                                    // To adjust z-field in constrained

}




void on_Reg1_MA_changed(GtkEditable *editable, gpointer user_data)            // To set regulator 1 in optimization 2-agent logistic function
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
}


void on_Reg2_MA_changed(GtkEditable *editable, gpointer user_data)            // To set regulator 2 in optimization 2-agent logistic function
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
}



void on_Reg3_MA_changed(GtkEditable *editable, gpointer user_data)            // To set regulator 2 in optimization 2-agent logistic function
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
}



void on_Stepsize_multiple_changed(GtkEditable *editable, gpointer user_data)            // To  decrease step-size
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
}



void on_der_t_changed(GtkEditable *editable, gpointer user_data)            // To  adjust step-time
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

}


void on_stopthreshold_MA_changed(GtkEditable *editable, gpointer user_data)            // To  adjust threshold in stop criterion
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
}

// Turn on pulling control
void on_pullingMA_clicked (GtkWidget *widget, gpointer data) {
    printf("start pulling control \n");
   set_pullingCtrl_to();                                        // assign a value to Phi in Adaptive-controller design
}

void on_pullingGAin_MA_changed(GtkEditable *editable, gpointer user_data) {           // To  adjust the pulling gain
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_pullinconstant_MA_to(d);                                                  // To  adjust threshold in stop criterion
}

void on_integral_changed(GtkEditable *editable, gpointer user_data)            // To  adjust the integral gain
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));

           set_integralGain_MA_to(d);                                                  // To  adjust integral gain
}

/////////////////// Windowing on Side camera  //////////////////////////
void on_record_button_clicked (GtkWidget *widget, gpointer data)
{
  //  valve_define_detect_region();
}
