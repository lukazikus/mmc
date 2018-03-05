# include "math_subroutine.h"

float l_angle_radian;
float Quotient_MA;
float remainder_angle_MA;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutine: Change Angle to (-PI, PI]
//      Input: a_angle_radian: angle requires adjustment. Unit: radian
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float adjust_angle_range (float a_angle_radian) {
    l_angle_radian = a_angle_radian;
    l_angle_radian = fmodf(l_angle_radian, 2*M_PI);   // (-2*PI, 2*PI)
    if (l_angle_radian > M_PI)
        l_angle_radian = l_angle_radian - 2*M_PI;     // (-2*PI,   PI]
    else if (l_angle_radian <= (-M_PI) )
        l_angle_radian = l_angle_radian + 2*M_PI;     // (  -PI,   PI]
    return l_angle_radian;
}

// Cross Product of Two Vectors
int cross_product_of_2_vec ( float vec_1[3], float vec_2[3], float res[3] ) {
	res[0] = vec_1[1] * vec_2[2] - vec_1[2] * vec_2[1];
	res[1] = vec_1[2] * vec_2[0] - vec_1[0] * vec_2[2];
	res[2] = vec_1[0] * vec_2[1] - vec_1[1] * vec_2[0];
	return 1;
}
// Absolute Value for Float Type
float abs_f (float value) {
  if (value < 0)
    return -value;
  else
    return value;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutine: Change Angle to [0, PI/2]  created for multi-agent project
//      Input: a_angle_radian: angle requires adjustment. Unit: radian
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float adjust_angle_local_MA (float a_angle_radian) {    // [-PI/2, PI/2]
  l_angle_radian = a_angle_radian;

    l_angle_radian = fmodf(fabs(l_angle_radian), M_PI);
    if (l_angle_radian <= M_PI_2  && a_angle_radian>=0)
        l_angle_radian = l_angle_radian;
    else if (l_angle_radian <= M_PI_2  && a_angle_radian<0)
        l_angle_radian = -l_angle_radian;
    else if (l_angle_radian > M_PI_2 && a_angle_radian>=0)
        l_angle_radian = (l_angle_radian - M_PI);
    else if (l_angle_radian > M_PI_2 && a_angle_radian<0)
        l_angle_radian = -(l_angle_radian - M_PI);
    return l_angle_radian;

}

float adjust_angle_heading_MA (float a_angle_radian) {    // [-PI/2, PI/2]
  l_angle_radian = a_angle_radian;

    l_angle_radian = fmodf(fabs(l_angle_radian), M_PI);
    if (l_angle_radian <= 1.1*M_PI_2  && a_angle_radian>=0)
        l_angle_radian = l_angle_radian;
    else if (l_angle_radian <= 1.1*M_PI_2  && a_angle_radian<0)
        l_angle_radian = -l_angle_radian;
    else if (l_angle_radian > 1.1*M_PI_2 && a_angle_radian>=0)
        l_angle_radian = (l_angle_radian - M_PI);
    else if (l_angle_radian > 1.1*M_PI_2 && a_angle_radian<0)
        l_angle_radian = -(l_angle_radian - M_PI);
    return l_angle_radian;
}

/// Return Minimum of 1D Array
int min_array (const int *arr, int length) {
    int i = 0;
    double minimum = arr[0];
    for (i = 1; i < length; ++i) {
		//printf("current element %d.\n", arr[i]);
        if (minimum > arr[i]) {
            minimum = arr[i];
        }
    }
    return minimum;
}

/// Return Maximum of 1D Array
int max_array (const int *arr, int length) {
    int i = 0;
    double maximum = arr[0];
    for (i = 1; i < length; ++i) {
        if (maximum < arr[i]) {
            maximum = arr[i];
        }
    }
    return maximum;
}

// Get Abs. Angle Diff. Within Range [0, pi]
float get_abs_angle_diff (float angle_1, float angle_2) {
	float diff = adjust_angle_range (angle_1 - angle_2);
	/*
	while ( diff < 0 )
	{
		diff = diff + 2 * M_PI;
	}
	while (diff > 2 * M_PI)
	{
		diff = diff - 2 * M_PI;
	}
	if (diff > M_PI)
		diff = 2 * M_PI - diff;
	*/
	diff = abs_f (diff);
	return diff;
}

// Get dis from a point to a line segment
double calc_dis_from_point_to_line ( int point[2], int line[4] ) {
	int x0 = point[ 0 ];
	int y0 = point[ 1 ];
	int	x1 = line[ 0 ];
	int	y1 = line[ 1 ];
	int x2 = line[ 2 ];
	int y2 = line[ 3 ];
	double dis = abs_f( ( y2-y1 ) * x0 - ( x2-x1 ) * y0 + x2*y1 - x1*y2 ) / sqrt( pow( (y2-y1), 2) + pow( (x2-x1), 2) );
	return dis;
}

/// Get maximum absoluate value of an array
float get_max_abs_of_array ( int length, float * arr ) {
	float max = 0.0;
	float loc = 0.0;
	for ( int i = 0; i < length; i++ ) {
		loc = abs_f(arr[i]);
		if ( max < loc )	max = loc;
	}
	return max;
}

// calc. dis. between 2 pt. (pt1_x, pt1_y, pt2_x, pt2_y)
float calc_dis_between_two_pt (int input[4]) {
	float dis = (float)sqrt(pow(input[0]-input[2],2)+pow(input[1]-input[3],2));
	return dis;
}

int get_sign (float input) {
    if (input > 0)
        return 1;
    else if (input < 0)
        return -1;
    else
        return 0;
}
