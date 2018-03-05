// ---------------------------------------------------------------------------
// Tracking developed by Mohammad
// ---------------------------------------------------------------------------
#include "Ctracker.h"
using namespace cv;
using namespace std;

size_t CTrack::NextTrackID=0;

// ---------------------------------------------------------------------------
// Track designer
// When you create, track begins with a certain point,
// this pointer is passed to the constructor as an argument.
// ---------------------------------------------------------------------------
CTrack::CTrack(Point2f pt, float dt, float Accel_noise_mag) {
	track_id=NextTrackID;
	NextTrackID++;
	// Each track has its own Kalman filter,
	// by which the prediction is made, where the next point should be.
	KF = new TKalmanFilter(pt,dt,Accel_noise_mag);
	// coordinates of the point where the track predicts the following observation (Detection) are stored here.
	prediction=pt;
	exitP = pt;
	ExitF = false;
	EnterF = false;
	skipped_frames=0;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTrack::~CTrack() {
	// destruct  Kalman filter
	delete KF;
}

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames,int _max_trace_length)
{
	dt=_dt;
	Accel_noise_mag=_Accel_noise_mag;
	dist_thres=_dist_thres;
	maximum_allowed_skipped_frames=_maximum_allowed_skipped_frames;
	max_trace_length=_max_trace_length;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(vector<Point2d>& detections) {
    int width = 640;   //image width, pixels
    int height = 480;  //image height, pixels
   // vector<Point2d> exitP;        // array that captures detected point before exiting the window
	// -----------------------------------
	//  If there is no tracks yet, then every cv::Point begins its own track.
	// -----------------------------------
	if(tracks.size()==0)
	{
		// If there is no track build a track object
		for(int i=0;i<detections.size();i++)
		{
			CTrack* tr=new CTrack(detections[i],dt,Accel_noise_mag);
			tracks.push_back(tr);
		}
	}

	// -----------------------------------
	// If tracks already there anyway
	// -----------------------------------
	int N=tracks.size();		// tracks
	int M=detections.size();	// detections

	// Distance Matrix of N-dimension tracks to detecting the M-dimension
	vector< vector<double> > Cost(N,vector<double>(M));   // define the COST matrix
	vector<int> assignment; // destination => assignment vector

	// -----------------------------------
	// Tracks already there, create the matrix of distances
	// -----------------------------------
	double dist;
	for(int i=0;i<tracks.size();i++)
	{
		// Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for(int j=0;j<detections.size();j++)
		{
			Point2d diff=(tracks[i]->prediction-detections[j]);
			dist=sqrtf(diff.x*diff.x+diff.y*diff.y);
			Cost[i][j]=dist;
		}
	}
	// -----------------------------------
	// Solving assignment problem (tracks and predictions of Kalman filter)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);

	// -----------------------------------
	// clean assignment from pairs with large distance
	// -----------------------------------
	// Not assigned tracks
	vector<int> not_assigned_tracks;

	for(int i=0;i<assignment.size();i++)
	{
		if(assignment[i]!=-1)
		{
			if(Cost[i][assignment[i]]>dist_thres)
			{
				assignment[i]=-1;
				// Note the unassigned tracks, and increment the missed shots,
				// when the number of dropped frames exceeds a threshold, the track is erased.
				not_assigned_tracks.push_back(i);     // stores list of unassigned tracks in this vector
			}
		}
		else
		{
				// If track have no assigned detection, then increment skipped frames counter.
			tracks[i]->skipped_frames++;
		}

	}


	// -----------------------------------
		// If agents are about to exit the window, active flag  ExitF
	// -----------------------------------


	// -----------------------------------
		// If track didn't get detections over long time, remove it.
	// -----------------------------------
	for(int i=0;i<tracks.size();i++)
	{
	    if(tracks[i]->exitP.x>width-40  ||  tracks[i]->exitP.y>height-40 || tracks[i]->exitP.x<40  || tracks[i]->exitP.y<40)    // this if loop is just for the case where agents are about to leave the window
    {
        tracks[i]->ExitF = true;
    }

		if( (tracks[i]->ExitF == true)  &&  (tracks[i]->EnterF == true) )
		{
		  //  tracks[i]->ExitF = false;
           // tracks[i]->EnterF = false;
			delete tracks[i];
			tracks.erase(tracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
          //  cout << "OOOOOOOOOOOOOOOOOOOOO" << endl;
		}

		if(tracks[i]->skipped_frames>maximum_allowed_skipped_frames     &&   (tracks[i]->ExitF == false))
		{
			delete tracks[i];
			tracks.erase(tracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
		}
	}
	// -----------------------------------
    // Search for unassigned detects and start new tracks for them.
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(int i=0;i<detections.size();i++)
	{
		it=find(assignment.begin(), assignment.end(), i);
		if(it==assignment.end())
		{
			not_assigned_detections.push_back(i);
				for(int k=0;k<tracks.size();k++)
                {
                    if(tracks[k]->ExitF == true)
                    {
                      tracks[k]->EnterF = true;
                  //  cout << "XXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
                    }
                }
                /*
                for(int k=0;k<tracks.size();k++)
                {
             if(tracks[k]->ExitF == true)
        {
         //   tracks[k]->ExitF == true
			delete tracks[k];
			tracks.erase(tracks.begin()+k);
			assignment.erase(assignment.begin()+k);
			k--;
                }
                }
                */
		}
	}

	// -----------------------------------
	// and start new tracks for them
	// -----------------------------------
	if(not_assigned_detections.size()!=0)
	{
		for(int i=0;i<not_assigned_detections.size();i++)
		{
			CTrack* tr=new CTrack(detections[not_assigned_detections[i]],dt,Accel_noise_mag);
			tracks.push_back(tr);
		}
	}

	// Update Kalman Filters state

	for(int i=0;i<assignment.size();i++)
	{
		// If track updated less than one time, than filter state is not correct.

		tracks[i]->KF->GetPrediction();

		if(assignment[i]!=-1) // // If we have assigned detection, then update using its coordinates,
		{
			tracks[i]->skipped_frames=0;
			tracks[i]->prediction=tracks[i]->KF->Update(detections[assignment[i]],1);
			tracks[i]->exitP =  detections[assignment[i]];    /// store detections before exiting the window!

		}else	     // if not continue using predictions
		{
                if(tracks[i]->ExitF == true)    // this if loop is just for the case where agents are about to leave the window
            {
                        if(tracks[i]->skipped_frames >= 4 )
                            {
                        //  tracks[i]->prediction=tracks[i]->KF->Update(tracks[i]->exitP,1);
                        tracks[i]->prediction=tracks[i]->KF->Update(tracks[i]->exitP,0);

                            }
            } else
            			tracks[i]->prediction=tracks[i]->KF->Update(Point2f(0,0),0);
		}

		if(tracks[i]->trace.size()>max_trace_length)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(),tracks[i]->trace.end()-max_trace_length);
		}

		tracks[i]->trace.push_back(tracks[i]->prediction);
		tracks[i]->KF->LastResult=tracks[i]->prediction;
	}

}
// ---------------------------------------------------------------------------
//  	// destruct the track
// ---------------------------------------------------------------------------
CTracker::~CTracker(void)
{
	for(int i=0;i<tracks.size();i++)
	{
	delete tracks[i];
	}
	tracks.clear();
}
