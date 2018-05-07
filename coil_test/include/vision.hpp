#ifndef VISION
#define VISION

#include "general_header.hpp"

#include <gdk/gdkkeysyms.h>

#include "astar.hpp"
#include "undergrad.hpp"

#include "NETUSBCAM_API.h"
#include "ICubeDefines.h"

/* for OpenCV 2 */
//#include <opencv/cv.h>
//#include <opencv/highgui.h>

/* for OpenCV 3 */
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//#include "FWcamera.hpp"

using namespace cv;

/* GUI callback functions */
extern "C" {
    //void on_toggle_arena_toggled (GtkToggleButton *togglebutton, gpointer data);    // show/hide digital arena
    //void on_spin_binaryThreshold_changed (GtkEditable *editable, gpointer user_data);
    //void on_cB_simulation_toggled (GtkToggleButton *togglebutton, gpointer data);   // toggle between simulation / real mode
    //void on_rB_1_toggled (GtkToggleButton *togglebutton, gpointer data);
    //void on_rB_2_toggled (GtkToggleButton *togglebutton, gpointer data);
    //void on_rB_3_toggled (GtkToggleButton *togglebutton, gpointer data);
    gboolean on_eventbox1_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data);
}

/* Functions */
void camera_activate (void);
void camera_deactivate (void);
void get_present_image ( Mat * container );

//void return_center_pt_info ( Point *robot, Point *cargo, float *angle );  // return current position information of robot and cargo to controller

//void return_size_info ( float *size1, float *size2 );           // return the size in pixels of robot and cargo
//int get_cargo_type (void);

void stop_video_stream(void);

float * get_robot_pose(void);
float * get_cargo_pose(void);
float * getGoalPointCoor(void);



gboolean key_event (GtkWidget *widget, GdkEventKey *event);
gboolean key_event_release (GtkWidget *widget, GdkEventKey *event);

#endif
