#ifndef GENERALHEADER
#define GENERALHEADER

#include <pthread.h>                    // for using thread
#include <stdio.h>                      // for printf
#include <time.h>
#include <gtk/gtk.h>
#include <errno.h>
#include <sys/time.h>
#include <iostream>
#include <math.h>

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))
#define tand(x) (tan(fmod((x),360) * M_PI / 180))

void my_sleep (unsigned msec);          // sleep
double get_present_time (void);
#endif
