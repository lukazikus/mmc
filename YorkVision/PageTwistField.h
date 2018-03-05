////////////////////////////////////////////////////////////////////////////////////////
// File      : PageTwistField.h
// Function  : Tab: twisted walking
// Edited by : Omid
////////////////////////////////////////////////////////////////////////////////////////
// #ifndef TEMPFEEDINGTWIST
// #define TEMPFEEDINGTWIST

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Header Files
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <pthread.h>
#include <gtk/gtk.h>
#include "s826_subroutine.h"
#include "vision.h"
#include "PageGeneralControl.h"
// #include "AccelStepper.h"
#include <stdio.h>
#include <math.h>

// defined in GeneralControl.c
extern float factor_x, factor_y, factor_z;
extern float field_x, field_y, field_z, field_mag, field_angle;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function Declaration
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // discrete bending control
// void init_discrete_bending(void);
// void stop_discrete_bending(void);
// double magnet2field_angle( double );
// double field2magnet_angle( double );
// void set_kp( float );
// void set_ki( float );
// void set_destination_angle( float );
// float anglePlus( float, float );
// float angleMinus( float, float );
// void set_discrete_go(int d);
//
// // continuous bending control
// void init_continuous_bending(void);
// void stop_continuous_bending(void);
// void set_continuous_radius(float d);
// void set_continuous_central(float d);
// void set_continuous_start(float d);
// void set_continuous_time(float d);
// void set_continuous_feedback(int d);
// void set_continuous_go(int);

// sinusoidal field (2D) thread related functions
void init_sinusoidal_field(void);
void stop_sinusoidal_field(void);
void set_fab_heading_s(float d);
void set_fab_amp_s(float d);
void set_fab_fre_s(float d);
void set_sinusoidalfield_go(int d);

// rotational field thread (vertical)
void init_rotational_field(void);
void stop_rotational_field(void);
void set_fab_heading(float d);
void set_fab_amp(float d);
void set_fab_fre(float d);
void set_rotationalfield_go(int d);

// rotational field (horizontal) thread related functions
void init_rotational_field_h (void);
void stop_rotational_field_h (void);
void set_fab_heading_h (int d);
void set_rotationalfield_go_h (int d);


// sawtooth mode thread related functions
void init_sawtooth_mode(void);
void stop_sawtooth_mode(void);
void set_sawtooth_go(int);
void set_peak_angle_st(float);

// // temperature control
// void init_temp_control ( void );
// void stop_temp_control ( void );
// void set_destination_temp ( float );
// void set_temp_go ( int );
// float get_current_temp(void);
//
// // automatic feeding
// void init_auto_feeding(void);
// void stop_auto_feeding(void);
// void set_feeding_distance ( float );
// void set_feeding_speed ( float );
// void set_feeding_go ( int );
//
// // manual feeding
// void set_feeding_increments(float);
// void init_manual_feeding(void);
// void stop_manual_feeding(void);
// void feederIncrement (void);
// void feederDecrement (void);
//
// // automatic fabrication
// void set_left_constraint (int);
// void init_auto_fab (void);
// void stop_auto_fab (void);
//
// // timing thread
// void init_timing(void);
// void stop_timing(void);
//
// // cut off thread
// void set_neutral_cut(float);
// void set_mag_cut(float);
// void set_fre_cut(float);
// void init_cut_off (void);
// void stop_cut_off (void);
//

// twisting field walking
void init_twist_field (int);
void stop_twist_field (int);
void init_twisted_walking (int);
void stop_twisted_walking (int);
void set_theta_heading (float);
void set_beta_tild (float);
void set_ang_freq (float);
void set_ang_span (float);
void set_tfield_mag (float);

// manual field
void init_manual_field(int);
void stop_manual_field(int);
void init_manual_field_go(int);
void stop_manual_field_go(int);
void set_bx_mag(float);
void set_by_mag(float);
void set_bz_mag(float);
void set_dbx_mag(float);
void set_dby_mag(float);
void set_dbz_mag(float);
void resetCoils(void);

// #endif // TEMPFEEDING
