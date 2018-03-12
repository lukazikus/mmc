#ifndef UNDERGRAD
#define UNDERGRAD

#include "math_subroutine.h"
#include <gtk/gtk.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include "Amplifier.h"
#include "vision.h"
#include <math.h>

#define tand(x) (tan(fmod((x),360) * M_PI / 180)) // From actuation.hpp

int undergrad_keyboard_init_stop(int d);            // toggle the keyboard detection thread
int undergrad_walk_init_stop (int d);               // toggle the walk thread
int undergrad_set_dir(int d);                       // set gradient dir using keyboard

int undergrad_set_x_gradient(float d);
int undergrad_set_y_gradient(float d);

int setPgain_MMC(float d);                          // Pgain assignment function
int on_startMMC_Thread (void);                // starts MMC main function
int on_stopMMC_Thread (void);                  // stops MMC main function
float *get_output_signals(void);                // Function for giving output signals to actuation

double get_present_time (void);                 // Get present time function

#endif
