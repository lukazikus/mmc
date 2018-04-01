#ifndef FWCAMERA
#define FWCAMERA

#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include <pthread.h>
#include <stdint.h>
#include <inttypes.h>
#include <dc1394/dc1394.h>
//#include "callbacksTS.h"

//#define FOCULUS_TC 0x00097eff42802240
//#define FOCULUS_TB 0x00097eff42909500
/// Take side camera as top camera since top camera is used by Patrick. 2015-09-02
#define FOCULUS_TB 0x00097eff42802240
#define FOCULUS_TC  0x00097eff42909500

// #define FOCULUS_TC 0x00097eff42802240
// #define FOCULUS_TB 0x00097eff42909500

//#define FOCULUS 0x00097eff40300024
////#define PIXELINK 396320271505170
//#define PIXELINK 0x0001687381001712
//#define FOCULUS2 0x00097eff40300046 //I know, crazy right, years in between getting foculus cameras and the ID numbers are only 22 cameras apart
//#define PIKE 0x000a470110071013

class FWcamera
{
public:
	FWcamera();
	~FWcamera();
	bool initialize();
	bool initialize_xz();
	bool isInitialized();
	bool startGrabbingVideo();
	bool startGrabbingVideo_xz();
	unsigned char * grabAframe(void);
	unsigned char * grabAframe_xz(void);
	void stopGrabbingVideo();
	void stopGrabbingVideo_xz();
	void startThreads();
	bool isGrabbingVideo();
	void deinitialize();
	bool setShutter(int s);
	bool setGain(int s);
	bool setBrightness(int g);
	bool setFPS(int f);
	bool setShutter_xz(int s);
	bool setGain_xz(int s);
	bool setBrightness_xz(int g);
	bool setFPS_xz(int f);


private:

	bool mIsGrabbingVideo;
	bool mIsInitialized;
	bool mIsInitialized_xz;

	// dc1394 stuff
	dc1394camera_t * mCamera, * mCamera_xz;
	dc1394_t * md;
	dc1394video_frame_t * mFrame, *mFrame_xz,   *mFrame2, *mFrame2_xz;

	// camera settings
	int format;
	int mode;
	int frameRate;
	int bytesPerPixel;
	const char* device;
	int nCard;
	int cameraMode;
	bool showCap;
	bool stopCapture;
	char * sname;

	int mShutter;
	int mGain;
	int mBrightness;
	int mFPS;

	int mShutter_xz;
	int mGain_xz;
	int mBrightness_xz;
	int mFPS_xz;

	pthread_t t;

};


#endif
