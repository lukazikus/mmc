#ifndef UNDERGRAD
#define UNDERGRAD
#define WIDTH 480 // Width of occupancy grid
#define LENGTH 640 // Length of occupancy grid

#include "math_subroutine.h"
#include <gtk/gtk.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include "Amplifier.h"
#include "vision.h"
// #include "astar.h"
#include <math.h>
#include <bits/stdc++.h>

#define tand(x) (tan(fmod((x),360) * M_PI / 180)) // From actuation.hpp
extern int draw_x;
extern int draw_y;
extern int** occ_grid;
extern stack<Pair> Path_vision;
extern int click_x;
extern int click_y;

int undergrad_keyboard_init_stop(int d);            // toggle the keyboard detection thread
int undergrad_walk_init_stop (int d);               // toggle the walk thread
int undergrad_set_dir(int d);                       // set gradient dir using keyboard

int undergrad_set_x_gradient(float d);
int undergrad_set_y_gradient(float d);

int setPgain_MMC(float d);                          // Pgain assignment function
int setIgain_MMC(float d);                          // Igain assignment function
int setDgain_MMC(float d);                          // Dgain assignment function
int on_startMMC_Thread (void);                // starts MMC main function
int on_stopMMC_Thread (void);                  // stops MMC main function
float *get_output_signals(void);                // Function for giving output signals to actuation

double get_present_time (void);                 // Get present time function
void create_og(int** &, Point [], int, float**, float);           // Create occupancy grid
void calc_clickPos(float*, Pair, float, float*); // Calculate next click position
void calc_dest(float**, float, Pair &); // Calculate destination of robot based on cargo

#endif
