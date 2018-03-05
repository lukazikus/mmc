////////////////////////////////////////////////////////////////////////////////////////
// File      : callbacks.c
// Function  :
// Edited by :
////////////////////////////////////////////////////////////////////////////////////////
#include "callbacks.h"

bool visionStarted = false;  //is the vision running?

static uint GUI_refresh_flag_vidWin1 = 1;        // 1: refresh the vidWin1 @ a certain rate
static uint GUI_refresh_flag_vidWin2 = 0;        // 1: refresh the vidWin2 @ a certain rate
static uint GUI_refresh_running = 0;             // 1: GUI refresh thread is running

static uint flag_draw_field = 0;

static vector <bool> keyPressed(0xffff); //vector of bools containing the key pressed states of all keys indexed by this: https://git.gnome.org/browse/gtk+/plain/gdk/gdkkeysyms.h


//micro_fab related variables -- Zhe
extern float field_mag_fab;
extern bool flag_temp_control;
extern float magnet_area, trust_area;
extern bool flag_magnet_sampled;
extern float factor_x, factor_y, factor_z;

////////////////////////////////////////////////  /////////////////////// ////////////////////////////////////////////
// GUI Variable
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GtkWidget *field_drawingArea;
GtkImage  *videoWindow, *videoWindow2;
GtkLabel  *label_centerPx_mm, *label_centerPy_mm;

GtkLabel  *label_info_type, *label_info_plane, *label_info_angle;   // Label near camera image, showing info. about the image
GtkLabel  *label_current_temp; // label showing current temperature
GtkLabel  *label_sampled_area, *label_current_area; // label showing magnet areas

GtkWidget *field1_fab, *field2_fab, *field3_fab, *g_field_mag_fab, *xy_3d_fab, *xz_3d_fab;

float GUI_field_angle = 0;   // Storing the angle of current field for GUI display


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI Refresh Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

static gboolean update_vidImage1 (gpointer userdata)
{
    //printf("Inside update_vidImage1().\n");
    //if ( !flag_image_pro )
		Mat img_m_color = getImage();
	//else
	//	Mat img_m_color = get_processed_image();														// (07-19) in image_processing_subroutine.c

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

static gboolean update_magnet_areas (gpointer userdata)
{
    char temp[8];
    float d = trust_area;
    sprintf(temp, "%6.1f", d);
    gtk_label_set_text (label_sampled_area, temp);
    d = magnet_area;
    sprintf(temp, "%6.1f", d);
    gtk_label_set_text (label_current_area, temp);
    return G_SOURCE_REMOVE;
}

static gboolean update_cameraImageInfoLabel (gpointer userdata)
{
    char temp[6];
    sprintf(temp, "%.1f", GUI_field_angle);
    gtk_label_set_text (label_info_angle, temp);
    return G_SOURCE_REMOVE;
}

// static gboolean update_current_temp (gpointer userdata)
// {
//     char temp[8];
//     float d = get_current_temp();
//     sprintf(temp, "%6.1f", d);
//     gtk_label_set_text (label_current_temp, temp);
//     return G_SOURCE_REMOVE;
// }

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
	float refresh_rate = 60.0;
	float refresh_period = 1.0/refresh_rate; //seconds per refresh frame
	double time_current, time_elapsed, time_init;
	struct timeval start;
	gettimeofday(&start, NULL);
	time_init = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time

    while(GUI_refresh_running)
    {
        if (flag_draw_field)
            g_main_context_invoke (NULL, draw_field, NULL);   // Draw field control voltage value

        if (GUI_refresh_flag_vidWin2)
        {
            img_m_color2 = getImage2();

            //gdk_threads_enter();	//display video image in program window

            if (img_m_color2.data != NULL)
            {
                g_main_context_invoke (NULL, update_vidImage2, NULL);
            }
            if (GUI_refresh_flag_vidWin1 == 1)   // If the side camera is on
            {
                img_m_color = getImage();

                if (img_m_color.data != NULL)
                {
                    g_main_context_invoke (NULL, update_vidImage1, NULL);
                }

            }
        }
        g_main_context_invoke (NULL, update_GUI_time, NULL);

        //g_main_context_invoke (NULL, update_GUI_time_ms, NULL);
        gettimeofday(&start, NULL);
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
        if ( (!flag_draw_field) && (!GUI_refresh_flag_vidWin2))
            GUI_refresh_running = 0;
	}
	//fclose(fp);   // Close file
	printf("@ the End of GUI_refresh_thread().\n");

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

// Draw field control voltage value
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

    cairo_set_source_rgb(cr, 0,0,255); //set color to Blue
    cairo_set_line_width(cr, 30);
   	float coil_current_z = get_coil_current(2);

   	cairo_move_to(cr, 115, 150);
   	cairo_line_to(cr, 115, 150 - coil_current_z * 30);
   	cairo_stroke(cr);

   	cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_move_to (cr, 0, 275);
	sprintf(str, "%.1f", coil_current_x * factor_x);
	cairo_show_text (cr, str);
	cairo_move_to (cr, 50, 275);
	sprintf(str, "%.1f", coil_current_y * factor_y);
	cairo_show_text (cr, str);
    cairo_move_to (cr, 100, 275);
	sprintf(str, "%.1f", coil_current_z * factor_z);
    cairo_show_text (cr, str);

    /// Orientation:
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

    cairo_stroke(cr);

	cairo_destroy(cr);
}

void on_window_destroy (GtkWidget *widget, gpointer data)
{
	stopVision(); //turn of visionloop
    GUI_refresh_running = 0;

    coilCurrentClear();                // Reset coil current to 0
    usleep(5e4);

    // stop_auto_feeding();
    // stop_temp_control();
    s826_close();                      // Close s826 board

	usleep(2e5); //wait for all loops to detect shutdown and end themselves

	printf("Exiting program.\n");
    gtk_main_quit();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

        GUI_refresh_flag_vidWin2 = 1;   // Begin to refresh vidWin
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
		GUI_refresh_flag_vidWin2 = 0;   // Stop refreshing GUI
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Twisting Field Walking Callbacks -- Omid
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// twisting field related functions -- Omid
void on_twisting_walking_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_twist_field(d);
  }else
  {
    stop_twist_field(d);
  }
}

void on_twisted_walking_enabler_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_twisted_walking(d);
  }else
  {
    stop_twisted_walking(d);
  }
}


void on_manual_field_toggled (GtkToggleButton *togglebutton, gpointer data)
{
        int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_manual_field(d);
  }else
  {
    stop_manual_field(d);
  }
}
void on_manual_field_enabler_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_manual_field_go(d);
  }else
  {
    stop_manual_field_go(d);
  }
}

void on_theta_heading_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_theta_heading(d);
}

void on_beta_tilt_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_beta_tild(d);
}

void on_ang_freq_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_ang_freq(d);
}

void on_ang_span_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_ang_span(d);
}

void on_tfield_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_tfield_mag(d);
}

void on_bx_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_bx_mag(d);
}

void on_by_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_by_mag(d);
}

void on_bz_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_bz_mag(d);
}

void on_dbx_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_dbx_mag(d);
}

void on_dby_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_dby_mag(d);
}

void on_dbz_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_dbz_mag(d);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Page General Control --- Tianqi
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void gui_clear_field_mode_xyz (void) {
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(field1_fab), 0.0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(field2_fab), 0.0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(field3_fab), 0.0);
}

static void gui_clear_field_mode_angle (void) {
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(g_field_mag_fab), 0.0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(xy_3d_fab), 0.0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(xz_3d_fab), 0.0);
}

void on_field1_fab_changed (GtkEditable *editable, gpointer user_data) {
  gui_clear_field_mode_angle();
  float x = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field1_fab));
  float y = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field2_fab));
  float z = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field3_fab));
  set_field_mode_xyz(x,y,z);
}

void on_field2_fab_changed (GtkEditable *editable, gpointer user_data) {
  gui_clear_field_mode_angle();
  float x = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field1_fab));
  float y = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field2_fab));
  float z = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field3_fab));
  set_field_mode_xyz(x,y,z);
}

void on_field3_fab_changed (GtkEditable *editable, gpointer user_data) {
  gui_clear_field_mode_angle();
  float x = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field1_fab));
  float y = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field2_fab));
  float z = gtk_spin_button_get_value(GTK_SPIN_BUTTON(field3_fab));
  set_field_mode_xyz(x,y,z);
}

void on_field_mag_fab_changed (GtkEditable *editable, gpointer user_data) {
  gui_clear_field_mode_xyz();
  float mag = gtk_spin_button_get_value(GTK_SPIN_BUTTON(g_field_mag_fab));
  float xy = gtk_spin_button_get_value(GTK_SPIN_BUTTON(xy_3d_fab));
  float xz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(xz_3d_fab));
  set_field_mode_angle(mag,xy,xz);
}

void on_xy_3d_fab_changed (GtkEditable *editable, gpointer user_data) {
  gui_clear_field_mode_xyz();
  float mag = gtk_spin_button_get_value(GTK_SPIN_BUTTON(g_field_mag_fab));
  float xy = gtk_spin_button_get_value(GTK_SPIN_BUTTON(xy_3d_fab));
  float xz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(xz_3d_fab));
  set_field_mode_angle(mag,xy,xz);
}

void on_xz_3d_fab_changed (GtkEditable *editable, gpointer user_data) {
  gui_clear_field_mode_xyz();
  float mag = gtk_spin_button_get_value(GTK_SPIN_BUTTON(g_field_mag_fab));
  float xy = gtk_spin_button_get_value(GTK_SPIN_BUTTON(xy_3d_fab));
  float xz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(xz_3d_fab));
  set_field_mode_angle(mag,xy,xz);
}

void on_reset_field_button_clicked (GtkWidget *widget, gpointer data) {
  gui_clear_field_mode_xyz();
  gui_clear_field_mode_angle();
  set_field_mode_xyz(0,0,0);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Y Camera
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
void on_binaryThreshold_changed (GtkEditable *editable, gpointer user_data)
{
	int d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_binaryThreshold_vision(d);
}
void on_visionParam2_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam2_vision(d);
}

void on_3d_indicator_toggled (GtkToggleButton *togglebutton, gpointer data) {
	int d = gtk_toggle_button_get_active(togglebutton);
	set_3d_indicator_flag (d);
}
void on_2d_indicator_toggled (GtkToggleButton *togglebutton, gpointer data) {
	int d = gtk_toggle_button_get_active(togglebutton);
	set_2d_indicator_flag (d);
}
void on_topcam_on_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	setTopCam_vision(d);

	if(d==1) //if button is toggled up
	{
		GUI_refresh_flag_vidWin1 = 1;
	} else
	{
		GUI_refresh_flag_vidWin1 = 0;
	}

	if(visionStarted) //if the vision is already started, restart it with new topcam choice
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
void on_cannyLow_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyLow changed to %.1f units\n", d );
	setcannyHigh_xz_vision((int)d);
}
void on_cannyHigh_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyHigh changed to %.1f units\n", d );
	setcannyLow_xz_vision((int)d);
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
void on_dilate_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("dilate changed to %.1f units\n", d );
	setDilate_xz_vision(d);
}
void on_binaryThreshold_xz_changed (GtkEditable *editable, gpointer user_data)
{
	int d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_binaryThreshold_xz_vision(d);
}
void on_visionParam2_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam2_xz_vision(d);
}
void on_3d_indicator_xz_toggled (GtkToggleButton *togglebutton, gpointer data) {
	int d = gtk_toggle_button_get_active(togglebutton);
	set_3d_indicator_xz_flag (d);
}
void on_2d_indicator_xz_toggled (GtkToggleButton *togglebutton, gpointer data) {
	int d = gtk_toggle_button_get_active(togglebutton);
	set_2d_indicator_xz_flag (d);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Catch Mouse Click Event
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void on_current_tab_changed (GtkNotebook *notebook,
GtkNotebookPage *notebook_page, int page, gpointer data)
{
	set_current_tab_page (page);
}

// undergradate projection
// toggle the keyboard detection thread
void on_undergrad_keyboard_toggled (GtkToggleButton *togglebutton, gpointer data) {
	int d = gtk_toggle_button_get_active(togglebutton);
	undergrad_keyboard_init_stop (d);
}

// toggle walking thread
void on_undergrad_walk_toggled (GtkToggleButton *togglebutton, gpointer data) {
	int d = gtk_toggle_button_get_active(togglebutton);
	undergrad_walk_init_stop (d);
}

void on_undergrad_xGradientSpin_changed (GtkEditable *editable, gpointer user_data) {
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    undergrad_set_x_gradient(d);
}

void on_undergrad_yGradientSpin_changed (GtkEditable *editable, gpointer user_data) {
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    undergrad_set_y_gradient(d);
}

// keystrokes
gboolean key_event (GtkWidget *widget, GdkEventKey *event) {
	printf("%s\n", gdk_keyval_name (event->keyval)  );
	keyPressed[ event->keyval ] = TRUE; 			//set the appropriate key value true

	if(    (event->keyval == GDK_KEY_period)
	    || (event->keyval == GDK_KEY_BackSpace)
	    || (event->keyval == GDK_KEY_Delete)
	    || (event->keyval == GDK_KEY_Return) )
		return FALSE; 					//register keystroke to gui normally for numbers

	switch (event->keyval) {
		case GDK_KEY_Right: undergrad_set_dir(0); break;   // +x
		case GDK_KEY_Up:    undergrad_set_dir(1); break;   // +y
		case GDK_KEY_Left:  undergrad_set_dir(2); break;   // -x
		case GDK_KEY_Down:  undergrad_set_dir(3); break;   // -y

		case GDK_KEY_KP_6: undergrad_set_dir(0); break;   // +x
        case GDK_KEY_KP_8: undergrad_set_dir(1); break;   // +y
        case GDK_KEY_KP_4: undergrad_set_dir(2); break;   // -x
        case GDK_KEY_KP_2: undergrad_set_dir(3); break;   // -y
        case GDK_KEY_KP_9: undergrad_set_dir(4); break;   // 1st Quad
        case GDK_KEY_KP_7: undergrad_set_dir(5); break;   // 2nd Quad
        case GDK_KEY_KP_3: undergrad_set_dir(6); break;   // 3rd Quad
        case GDK_KEY_KP_1: undergrad_set_dir(7); break;   // 4th Quad
        case GDK_KEY_KP_5: undergrad_set_dir(-1);   break;   // stop field

	}
	return TRUE; 						//returning true means that the keypress event wont be processed normally
}

gboolean key_event_release(GtkWidget *widget, GdkEventKey *event) {
	keyPressed[ event->keyval ] = FALSE; 			//set the appropriate key value FALSE
	return TRUE;
}

////   MMC project functions  ////
void on_Pgain_changed (GtkEditable *editable, gpointer user_data){
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setPgain_MMC((int)d);  // This inilializes your internal variable
    printf("P changed\n");
}

void on_startMMC_clicked (GtkWidget *widget, gpointer data) {
    on_startMMC_Thread();
    printf("MMC path planning started!\n");
}

void on_stopMMC_clicked(GtkWidget *widget, gpointer data){
    on_stopMMC_Thread();
    printf("MMC stopped, coils turned off\n");
}

void on_actuation_clicked(GtkWidget *widget, gpointer data){
    on_tB_actuation_Thread();
    printf("Actuation started\n");
}
