#ifndef RCVIS
#define RCVIS

#include <gtk/gtk.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


#include <opencv/HungarianAlg.h>


//#include <math.h>
//#include <vector>
#include <sys/time.h>
//#include <time.h>
//#include <string>

//#include "callbacks.h"
#include "FWcamera.h"
//#include "ImageProcessor.h"
#include "constantValue.h"

using namespace cv;

extern GtkImage *videoWindow, *videoWindow2;

///
extern GtkLabel         *testLabel, *centerPointCoor;
///

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Edited by JZ
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern Point centerP;

Mat getImage(void);                 // Get the image to display on the vidWin1
Mat getImage2(void);                // Get the image to display on the vidWin2
int * getCenterPointCoor(void);     // Get the coordinates of the center point.
int * getGoalPointCoor(void);       // Get the coordinates of the destination point.
int * getGoalPointCoorC(void);      // Get the coordinates of the center-clicked destination point
int * getGoalPointCoorR(void);      // Get the coordinates of the right-clicked destination point
int * get2ndCenterPointCoor(void);  // Get the coord. of the center point of the 2nd rect.
int * getGoalPointCoor_xz(void);
int * getGoalPointCoorR_xz(void);
int * getGoalPointCoorC_xz(void);

float * get_CenterP_rect_short_side_coor_array     ( void ); 			// Get the coor. of the 2 center point of the rect.'s 2 short sides
float * get_CenterP_2nd_rect_short_side_coor_array ( void ); 			// Get the center points of the short sides of the 2nd rect.
int JZ_get_heading_point ( float heading_point[] );						// Get center points of rect. for determining headings

double get_swimmer_angle();      // Get the orientation angle of the swimmer from the camera image.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initVision();
void stopVision(void);
void* FPSprint(void*);
void* FPSprint_xz(void*);
void* visionThread(void*);
void* visionThread_xz(void*);

GdkPixbuf *convertOpenCv2Gtk(IplImage *image);
void img_test();
void set_edgemap(int);
void set_binary(int);

void setGain_vision(int);
void setShutter_vision(int);
void setDilate_vision(int);
void setvisionParam1_vision(int);
void setvisionParam2_vision(int);
void setvision_HoughMinRadius_vision(int d);
void setvision_HoughMaxRadius_vision(int d);
void setdetect_vision(int);
void setcannyLow_vision(int);
void setcannyHigh_vision(int);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setSideCam_vision(int d);
void set_edgemap_xz(int);
void set_binary_xz(int);
void setGain_xz_vision(int);
void setShutter_xz_vision(int);
void setDilate_xz_vision(int);
void setvisionParam1_xz_vision(int);
void setvisionParam2_xz_vision(int);
void setdetect_xz_vision(int);
void setsidecam_xz_vision(int);
void setcannyLow_xz_vision(int);
void setcannyHigh_xz_vision(int);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// magnet detection functions declarations __ Edited by Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_magnetdetection(int d);
void set_showbox(int d);
void set_showprocess(int d);
void set_closediameter(int d);
void orientation_display(Point2f, Point2f, Point2f, Point2f);
void set_showdestination(int d);
void set_showfielddirection(int d);
void reverse_magent(void);
float anglePlus_v( float a, float b);
float angleMinus_v( float c, float a);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ); //click in pixels

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Set 2nd Rect. Requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_2nd_rect (int d);

int switch_record_centerP(int argu);

void plot_line(bool plot_flag, int line_x[], int line_y[], int line_R[], int line_G[], int line_B[]);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Hough circle detection requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int set_1st_orient_circ (int d);
void set_agentshowcirc (int d);

int check_newFrame( void );																	// (07-15) See if new frame has arrived

int plot_DC_AC_angle (double DC_angle, double AC_angle);									// (07-19) Plot DC & AC Angles


#endif
