#ifndef TEMPFEEDING
#define TEMPFEEDING

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Header Files
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <pthread.h>
#include <gtk/gtk.h>
#include "constantValue.h"     // Values of the constants used
#include "s826_subroutine.h"   // s826 board subroutines
#include "vision.h"            // Camera related subroutines
#include "coilFieldControl.h"  //
#include "AccelStepper.h"
#include <stdio.h>
#include <math.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function Declaration
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Magnet feedback control
int init_test90(void);
int stop_test90(void);
double magnet2field_angle( double );
void set_kp( float );
void set_ki( float );
void set_destination_angle( float );
float magnetAngleChange( float, float );
void set_reversefield(int d);
void set_bending_go(int d);
void set_torque_angle(float d);
// rotational field thread
int init_rotational_field(void);
int stop_rotational_field(void);
void set_fab_amp(float d);
void set_fab_fre(float d);
void set_rotationalfield_go(int d);
// temperature control
int init_temp_control ( void );
int stop_temp_control ( void );
void set_destination_temp ( float );
void set_temp_go ( int );

// automatic feeding
int init_auto_feeding(void);
int stop_auto_feeding(void);
void set_feeding_distance ( float );
void set_feeding_speed ( float );
void set_feeding_go ( int );

// manual feeding
void set_feeding_increments( float );
int init_manual_feeding( void );
int stop_manual_feeding( void );
void feederIncrement (void);
void feederDecrement (void);

// automatic fabrication
int init_auto_fab (void);
int stop_auto_fab (void);

// make an "S"
int init_make_s( void );
int stop_make_s( void );
void set_time_s ( float );
void set_radius_s ( float );

// field control
void set_factor(int);
void set_field_xyz (int, float);
void set_field_polar (float, float);

#endif // TEMPFEEDING
