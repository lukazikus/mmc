#pragma once
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
using namespace cv;
using namespace std;
// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
class TKalmanFilter               // class
{
public:
	KalmanFilter* kalman;                       // make an object of what KalmanFilter in openCV returns
	double deltatime;                           // time increment
	Point2f LastResult;                         // estimated points:: positions
	TKalmanFilter(Point2f p,float dt=0.2,float Accel_noise_mag=0.5);     //  constructor
	~TKalmanFilter();                                                    // destructor
	Point2f GetPrediction();                          // function: returns predicted point
	Point2f Update(Point2f p, bool DataCorrect);      // function: returns updated point
};

