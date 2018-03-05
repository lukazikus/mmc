#ifndef MATH_SUBROUTINE
#define MATH_SUBROUTINE

#include "math.h"
#include <stdio.h>

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))
#define PI 3.14159265

int cross_product_of_2_vec ( float vec_1[3], float vec_2[3], float res[3] );	// Cross Product of Two Vectors
float abs_f (float value);                                                  // Absolute Value for Float Type

float adjust_angle_range (float a_angle_radian);
float adjust_angle_local_MA (float a_angle_radian);
float adjust_angle_heading_MA (float a_angle_radian);

float get_abs_angle_diff ( float angle_1, float angle_2 );		// (07-15) Get Abs. Angle Diff. Within Range [0, pi]
double calc_dis_from_point_to_line ( int point[2], int line[4] );		// (08-14) Get dis from a point to a line segment

int min_array (const int *arr, int length);					// Return Minimum of 1D Array
int max_array (const int *arr, int length);					// Return Maximum of 1D Array

float get_max_abs_of_array ( int length, float * arr );		// Get maximum absoluate value of an array

float calc_dis_between_two_pt (int input[4]);                 // calc. dis. between 2 pt. (pt1_x, pt1_y, pt2_x, pt2_y)
int get_sign (float input);

#endif
