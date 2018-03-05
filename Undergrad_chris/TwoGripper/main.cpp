#include <opencv2/opencv.hpp>
#include "Ctracker.h"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

float X=0,Y=0;
float Xmeasured=0,Ymeasured=0;
RNG rng;
//-----------------------------------------------------------------------------------------------------
// Mouse callbacks
//-----------------------------------------------------------------------------------------------------
void mv_MouseCallback(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if(event == CV_EVENT_MOUSEMOVE)
	{
		X=(float)x;
		Y=(float)y;
	}
}

int main(int ac, char** av)
{

	int k=0;
	Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,255,255)};
	namedWindow("Video");
	Mat frame=Mat(800,800,CV_8UC3);

	VideoWriter vw=VideoWriter("output.mpeg", CV_FOURCC('P','I','M','1'), 20, frame.size());
	// Bind the mouse to window
	setMouseCallback("Video",mv_MouseCallback,0);

	CTracker tracker(0.2,0.5,60.0,25,25);
	float alpha=0;
	while(k!=27)
	{
		frame=Scalar::all(0);

		// Add Noise (simulate actual measurements (detections))
		Xmeasured=X+rng.gaussian(2.0);
		Ymeasured=Y+rng.gaussian(2.0);

		// We write rotating around the mouse points (often overlapping)
		vector<Point2d> pts;
		pts.push_back(Point2d(Xmeasured+100.0*sin(-alpha),Ymeasured+100.0*cos(-alpha)));
		pts.push_back(Point2d(Xmeasured+100.0*sin(alpha),Ymeasured+100.0*cos(alpha)));
		pts.push_back(Point2d(Xmeasured+100.0*sin(alpha/2.0),Ymeasured+100.0*cos(alpha/2.0)));
		pts.push_back(Point2d(Xmeasured+100.0*sin(alpha/3.0),Ymeasured+100.0*cos(alpha/1.0)));
		alpha+=0.05;


	for(int i=0; i<pts.size(); i++)
	{
	circle(frame,pts[i],3,Scalar(0,255,0),1,CV_AA);
	}

		tracker.Update(pts);

		cout << tracker.tracks.size()  << endl;

		for(int i=0;i<tracker.tracks.size();i++)
		{
			if(tracker.tracks[i]->trace.size()>1)
			{
				for(int j=0;j<tracker.tracks[i]->trace.size()-1;j++)
				{
					line(frame,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[i%6],2,CV_AA);
				}
			}
		}

		imshow("Video",frame);
		vw << frame;

		k=waitKey(10);
	}

	vw.release();
	destroyAllWindows();
	return 0;

}


