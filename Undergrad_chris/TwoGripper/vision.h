////////////////////////////////////////////////////////////////////////////////////////
// File      : vision.h
// Function  : Camera-related subroutines
// Edited by : Jiachen, Zhe
////////////////////////////////////////////////////////////////////////////////////////
#ifndef RCVIS
#define RCVIS

#include "classDefJZ.h"
#include <gtk/gtk.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

//#include "Kalman.h"
//#include "HungarianAlg.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "Ctracker.h"
#include <deque>
#include "math_subroutine.h"

// essential libraries for Kalman multi-object tracking
//#include <mex.h>

// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>

#include "opencv2/video/tracking.hpp"

#include <math.h>
#include <sys/time.h>
//#include <time.h>
#include <string>
#include <sys/time.h>

//#include "callbacks.h"
#include "FWcamera.h"
//#include "ImageProcessor.h"
#include "constantValue.h"
#include "multiagent.h"
#include "optimizationagents.h"
#include <opencv2/video/background_segm.hpp>

using namespace cv;
using namespace std;

extern GtkImage *videoWindow, *videoWindow2;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Edited by Jiachen
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// magnet detection functions declarations -- Edited by Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_draw_annotation(int d);
void set_draw_roi(int d);
void set_draw_points(int d);
void set_closediameter(int d);
void set_needle_thick(int d);
void set_magnet_threshold(double d);
void set_showfielddirection(int d);
void reverse_magent(void);
void set_magnet_trust(void);
float anglePlus_v( float a, float b);
float angleMinus_v( float c, float a);
float angleMiddle_v( float a, float b);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Edited by JZ
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int vision_set_next_com (Point_JZ &input);          // set next center-of-mass of agent pair

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


float * get_CenterP_rect_short_side_coor_array(void); // Get the coor. of the 2 center point of the rect.'s 2 short sides

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initVision();
void stopVision(void);
void* FPSprint(void*);
void* FPSprint_xz(void*);

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
void setvision_Houghcanny1_vision(int d);
void setvision_Houghcanny2_vision(int d);
void setvision_HoughminDis_vision(int d);

void setvision_HoughBinary(int d);
void setvision_HoughErosion(int d);
void setvision_HoughDilation(int d);

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
void setvisionParam1_xz_vision(int);
void setvisionParam2_xz_vision(int);
void setsidecam_xz_vision(int);

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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ); //click in pixels

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Set 2nd Rect. Requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int set_2nd_rect (int d);

int switch_record_centerP(int argu);

int vision_plot_line (bool flag, Point_JZ &input0, Point_JZ &input1, Point_JZ &input2, Point_JZ &input3);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Hough circle detection requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int set_1st_orient_circ (int);
void set_agentshowcirc (int);
void set_agentpreprocess (int);
void set_agentSingle (int);

void drawCross(Point centerK, CvScalar colorK, int dK);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Protocol: Blob circle detection requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_HoughBlur (int);                  // To set Blur Marker Size in Hough transform
void set_HoughMindistance (int);           // To set Min distance in Hough transform
void set_HoughSwapratio (int);             // To set Swapratio in Hough transform
void set_3agentSwap (int);                // To set up swap threshold in 3-agent configuration

int * getCenterPointCoor_MA (void);        // To get center coordinate of agents and orientation circles
void  set_des_goalMA_to (float);           // set desired distance
void  set_des_PullangMA_to(float);         // To set desired pulling angle in multi-agent
float* getDistanceCoor(void);                // To get distance vector coordinate
float* getCTRL_MA(void);                    // returns desired distance and pixel2mm conversion
void  set_pix2mm_to (float);                 // convert mm to pix
void  set_des_goalMA_to (float);           // set desired distance
void  set_des_PullangMA_to(float);         // To set desired pulling angle in multi-agent

// Multi-Agent
int return_multi_agent_pos (Point_JZ &pt1, Point_JZ &pt2); // Jiachen: return multi-agent pos in frame
int vision_draw_cargo (int xCoor[2], int yCoor[2]);       // draw specified cargo positions on top-view
int vision_return_cargo_pos (Point_JZ &pt1, Point_JZ &pt2);
int vision_set_show_cargo_flag (int input);                 // set the flag for whether or not show the detected cargoes
int set_agentControlDisplay ( float * input);               // get values to display from multiagent.c

int return_single_agent_pos(Point_JZ &pt);

#endif
