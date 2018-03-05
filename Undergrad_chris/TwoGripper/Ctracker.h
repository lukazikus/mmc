// ---------------------------------------------------------------------------
// Tracking developed by Mohammad
// ---------------------------------------------------------------------------
//#pragma once
#include "Kalman.h"
#include "HungarianAlg.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

class CTrack {             // CTrack class
	public:
		vector<Point2d> trace;
		static size_t NextTrackID;
		size_t track_id;
		size_t skipped_frames;
		Point2d prediction;
	    Point2d exitP;
	    bool ExitF;
	    bool EnterF;
		TKalmanFilter* KF;             // make an object of what TKalmanFilter returns
		CTrack(Point2f p, float dt, float Accel_noise_mag);           // CTrack constructor
		~CTrack();                                                        // CTrack destructor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       r
};

class CTracker {           // CTracker class
	public:

		// sampling time
		float dt;

		float Accel_noise_mag;

		// distance threshold. If the points are separated by a distance,
		// Exceeds this threshold, then this pair is not considered in the assignment problem.
		double dist_thres;
		// The maximum number of frames that the track is maintained without getting on the measurement data.
		int maximum_allowed_skipped_frames;
		// The maximum trace length
		int max_trace_length;

		vector<CTrack*> tracks;    // track constructor
		void Update(vector<Point2d>& detections);             // main update loop function
		CTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames,int _max_trace_length);    // CTracker constructor
		~CTracker(void);              // CTracker destructor
};
