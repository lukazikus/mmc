//#pragma once
#include "Kalman.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
TKalmanFilter::TKalmanFilter(Point2f pt,float dt,float Accel_noise_mag)        // class
{
	//time increment (lower values makes target more "massive")
	deltatime = dt; //0.2

	// We don't know acceleration, so, assume it to process noise.
	// But we can guess, the range of acceleration values that can be achieved by tracked object.
    // Process noise. (standard deviation of acceleration: m/s^2)
	// shows, how much target can accelerate.
	//track_t Accel_noise_mag = 0.5;

	//4 state variables, 2 measurements
	kalman = new KalmanFilter( 4, 2, 0 );
	// Transition matrix
	kalman->transitionMatrix = (Mat_<float>(4, 4) << 1,0,deltatime,0,   0,1,0,deltatime,  0,0,1,0,  0,0,0,1);

	// initialization
	LastResult = pt;
	kalman->statePre.at<float>(0) = pt.x; // x
	kalman->statePre.at<float>(1) = pt.y; // y

	kalman->statePre.at<float>(2) = 0;
	kalman->statePre.at<float>(3) = 0;

	kalman->statePost.at<float>(0)=pt.x;   //
	kalman->statePost.at<float>(1)=pt.y;   //

	setIdentity(kalman->measurementMatrix);   // 2*4 measurement matrix

	kalman->processNoiseCov=(Mat_<float>(4, 4) <<                         // covariance  Q, P matrix
		pow(deltatime,4.0)/4.0	,0						,pow(deltatime,3.0)/2.0		,0,
		0						,pow(deltatime,4.0)/4.0	,0							,pow(deltatime,3.0)/2.0,
		pow(deltatime,3.0)/2.0	,0						,pow(deltatime,2.0)			,0,
		0						,pow(deltatime,3.0)/2.0	,0							,pow(deltatime,2.0));


	kalman->processNoiseCov*=Accel_noise_mag;            // scaled covariance Q, P matrix by Accel_noise_mag   4*4

	setIdentity(kalman->measurementNoiseCov, Scalar::all(0.1));    // R: 2*2

	setIdentity(kalman->errorCovPost, Scalar::all(.1));        // post P

}
//---------------------------------------------------------------------------
TKalmanFilter::~TKalmanFilter()
{
	delete kalman;    // delete Kalman object
}

//---------------------------------------------------------------------------
Point2f TKalmanFilter::GetPrediction()            // prediction function
{
	Mat prediction = kalman->predict();
	LastResult=Point2f(prediction.at<float>(0),prediction.at<float>(1));
	return LastResult;                              // returns estimated position for each track
}
//---------------------------------------------------------------------------
Point2f TKalmanFilter::Update(Point2f p, bool DataCorrect)
{
	Mat measurement(2,1,CV_32FC1);
	if(!DataCorrect)
	{
		measurement.at<float>(0) = LastResult.x;  //update using prediction :: failure has occurred
		measurement.at<float>(1) = LastResult.y;
	}
	else
	{
		measurement.at<float>(0) = p.x;  //update using measurements
		measurement.at<float>(1) = p.y;
	}
	// Correction
		Mat estimated = kalman->correct(measurement);
		LastResult.x=estimated.at<float>(0);   //update using measurements
		LastResult.y=estimated.at<float>(1);
	return LastResult;
}
//---------------------------------------------------------------------------
