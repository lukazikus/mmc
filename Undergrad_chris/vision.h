////////////////////////////////////////////////////////////////////////////////////////
// File      : vision.h
// Function  : Camera-related subroutines
// Edited by : Jiachen, Zhe
////////////////////////////////////////////////////////////////////////////////////////
#ifndef RCVIS
#define RCVIS

#include <gtk/gtk.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
// #include <bits/stdc++.h>
// #include <vector>
#include "astar.h"

#include <sys/time.h>
#include "FWcamera.h"
#include "math_subroutine.h"

#include "classDefJZ.h"

#include <math.h>

using namespace cv;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// defined in GeneralControl.c
extern float factor_x, factor_y, factor_z;
extern float field_x, field_y, field_z, field_mag, field_angle;
extern int currentActiveTabIndex; //used to change the behavior of the 3d indicator
// defined in PageTwistField.c
extern float theta, beta, omega, phi;
// defined in callbacks.c
extern GtkImage  *videoWindow, *videoWindow2;

extern Point centerP;
extern Mat img_m_color_for_display;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat getImage(void);                 // Get the image to display on the vidWin1
Mat getImage2(void);                // Get the image to display on the vidWin2

void initVision();
void stopVision(void);
void* FPSprint(void*);
void* FPSprint_xz(void*);
void* visionThread(void*);
void* visionThread_xz(void*);

// MMC Functions
float* get_robot_pose(void); // Get the pose of the robot
float* get_cargo_pose(void); // Get the pose of the cargo
float * getGoalPointCoor(void);       // Get the coordinates of the destination point.


// X-Y Camera1
GdkPixbuf *convertOpenCv2Gtk(IplImage *image);
void set_edgemap(int);
void set_binary(int);
void setGain_vision(int);
void setShutter_vision(int);
void setDilate_vision(int);
void set_binaryThreshold_vision (int);
void setvisionParam2_vision(int);
void set_3d_indicator_flag(int);
void set_2d_indicator_flag(int);
void setcannyLow_vision(int);
void setcannyHigh_vision(int);


// X-Z Camera2
void setTopCam_vision(int d);
void set_edgemap_xz(int);
void set_binary_xz(int);
void setGain_xz_vision(int);
void setShutter_xz_vision(int);
void setDilate_xz_vision(int);
void set_binaryThreshold_xz_vision (int);
void setvisionParam2_xz_vision(int);
void set_3d_indicator_xz_flag(int);
void set_2d_indicator_xz_flag(int);
void setdetect_xz_vision(int);
void settopcam_xz_vision(int);
void setcannyLow_xz_vision(int);
void setcannyHigh_xz_vision(int);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ); //click in pixels

static Mat opencv_edgemap (Mat, int, int, int);
static Mat opencv_binary (Mat, int);
static void draw_xy_magnetic_field (Mat,float,float);
static void draw_xz_magnetic_field (Mat,float,float);
static void draw_3d_magnetic_field (Mat,float,float);
static void draw_3d_magnetic_field_twisted (Mat,float,float);
// static void draw_digital_arena (Mat, float, float, float, float);
static void draw_digital_arena (Mat*);
void draw_goal(Mat*);
void draw_circle(Mat*, int, int);
void draw_occ_grid(Mat*, int** &);
void draw_path(Mat*, stack<Pair>);

#endif
