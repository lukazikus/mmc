#include "FWcamera.h"

static char * filename;

FWcamera :: FWcamera(void)
{
	stopCapture = true;
	mIsGrabbingVideo = false; //note - it is never set to true

	nCard		= 0; // this might change from 0 to 1
   	//device	= "/dev/video1394/0"; // this might change from 0 to 1 //commented by Eric cause we don't use it
   	mIsInitialized = false;
   	mIsInitialized_xz = false;

   	mShutter_xz = 1085;
   	mGain_xz = 0;
   	mBrightness_xz = 490;

   	mShutter = 1085;
   	mGain = 0;
   	mBrightness = 490;

   	//ioctl(open(device,O_RDONLY),VIDEO1394_IOC_UNLISTEN_CHANNEL,&nCard);
}


bool FWcamera :: isInitialized(void)
{
	return mIsInitialized;
}

bool FWcamera :: initialize() // this function is used for the xy camera
{
	if(mIsInitialized)
		return true;

	dc1394error_t err;
	dc1394camera_list_t * list;

	md = dc1394_new(); /* Initialize libdc1394, Creates a new context in which cameras can be searched and used. This should be called before using any other libdc1394 function. */
	if (!md)
		return false;

	err=dc1394_camera_enumerate (md, &list);                                /* Find cameras */
	DC1394_ERR_RTN(err,"Failed to enumerate cameras");

	if (list->num == 0)
	{                                                  /* Verify that we have at least one camera */
		dc1394_log_error("No cameras found");
		return false;
	}

    	//mCamera = dc1394_camera_new (md, list->ids[0].guid);
	mCamera = dc1394_camera_new (md, FOCULUS_TC);
    	if (!mCamera) {
        	//dc1394_log_error("Failed to initialize camera with guid ??\n", list->ids[0].guid);
		dc1394_log_error("Failed to initialize camera with guid ??\n", FOCULUS_TC);
        	return false;
    	}
    dc1394_camera_free_list (list);
/* Maybe we can do this later. Instead, we just use the first camera available
	uint64_t choice;// = getCamera_choice_xy(); //commented by Eric to compile
	switch (choice) //note the device IDs are assumed to be foculus first, then pixelink.
	{
		case 1: //foculus
		g_print("The top camera is using foculus.\n");
		choice = FOCULUS;
		mCamera = dc1394_camera_new (md,FOCULUS);
		break;
		case 2: //pixelink
		g_print("The top camera is using pixelink.\n");
		choice = PIXELINK;
		mCamera = dc1394_camera_new (md, PIXELINK);
		break;
		case 3: //foculus 2
		g_print("The top camera is using the second foculus.\n");
		choice = FOCULUS2;
		mCamera = dc1394_camera_new (md,FOCULUS2);
		break;
		case 4: //pike
		g_print("The top camera is using pike.\n");
		choice = PIKE;
		mCamera = dc1394_camera_new (md, PIKE);
		break;
		default:
		g_print("ERROR: Incorrect input to top camera selected.\n");
		break; //will create an error
	} */

	//dc1394_video_set_iso_channel(mCamera,10); //eric commented this
	//dc1394_video_set_iso_channel(mCamera,0 ); //eric commented this

	err = dc1394_video_set_iso_speed(mCamera, DC1394_ISO_SPEED_400);
	if (err != 0)
	{
		g_print("Error in setting video ISO speed. Code : %i.\n",err);
		return false;
	}

	//err = dc1394_video_set_mode(mCamera, DC1394_VIDEO_MODE_640x480_RGB8);//dc1394_video_set_mode(mCamera, DC1394_VIDEO_MODE_FORMAT7_0);
	err = dc1394_video_set_mode(mCamera, DC1394_VIDEO_MODE_640x480_MONO8);
	if (err != 0)
	{
		g_print("Error in setting video mode. Code : %i.\n",err);
		return false;
	}

/*	if(choice == PIXELINK) //commented by Eric
	{
		err = dc1394_format7_set_image_size(mCamera,DC1394_VIDEO_MODE_FORMAT7_0, 648, 480);
		if (err != 0)
		{
			g_print("Error in setting video image size. Code : %i.\n",err);
			return false;
		}
		//err = dc1394_format7_set_image_position(mCamera,DC1394_VIDEO_MODE_FORMAT7_0, 552, 1200);
		err = dc1394_format7_set_image_position(mCamera,DC1394_VIDEO_MODE_FORMAT7_0, 720, 1272);
		if (err != 0)
		{
			g_print("Error in setting video position. Code : %i.\n",err);
			return false;
		}

	}

	if(choice == PIKE)
	{
		err = dc1394_format7_set_image_size(mCamera,DC1394_VIDEO_MODE_FORMAT7_0, 640, 480);
		if (err != 0)
		{
			g_print("Error in setting video image size. Code : %i.\n",err);
			return false;
		}
		err = dc1394_format7_set_image_position(mCamera,DC1394_VIDEO_MODE_FORMAT7_0, 600, 1150);
		if (err != 0)
		{
			g_print("Error in setting video position. Code : %i.\n",err);
			return false;
		}
	}*/


	DC1394_ERR_CLN_RTN(err,dc1394_camera_free (mCamera),"cannot choose camera format");
	printf ("I: video mode is set\n");

	uint32_t channel;
	dc1394_video_get_iso_channel(mCamera,&channel);
	printf ("ISO channel: %i\n", channel);

	mIsInitialized = true;

	if(!setGain(mGain))
	{
		mIsInitialized = false;
		return false;
	}

	if(!setShutter(mShutter))
	{
		mIsInitialized = false;
		return false;
	}

	if(!setBrightness(mBrightness))
	{
		mIsInitialized = false;
		return false;
	}

	uint32_t band = 0;
	err = dc1394_video_get_bandwidth_usage(mCamera,&band);
	g_print("Top Bandwidth usage: %i\n", band);

	/*

	err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_GAIN, mGain);
	DC1394_ERR_CLN_RTN(err,dc1394_camera_free (mCamera),"cannot set gain");
	printf ("I: gain is 100\n");

	err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_SHUTTER, mShutter);
	DC1394_ERR_CLN_RTN(err,dc1394_camera_free (mCamera),"cannot set shutter");
	g_print("I: shutter is %d\n", mShutter);
	*/



	return true;
}

bool FWcamera::initialize_xz () //ERIC: still need to modify like the first initializer function!!!
{
	if(mIsInitialized_xz)
		return true;
	md = dc1394_new();
	if (!md)
	{
		printf ("Error, library not initialized.\n");
		return false;
	}
	dc1394error_t err;
	dc1394camera_list_t * list;
	err=dc1394_camera_enumerate (md, &list);                                /* Find cameras */
	DC1394_ERR_RTN(err,"Failed to enumerate cameras");

	if (list->num == 0)
	{                                                  /* Verify that we have at least one camera */
		dc1394_log_error("No cameras found");
		return false;
	}

	if (list->num == 1 && mIsInitialized)
	{
		g_print("Error: There is only one camera and its already taken!\n");
		return false;
	}

	//mCamera = dc1394_camera_new (md, list->ids[0].guid);
	mCamera_xz = dc1394_camera_new (md, FOCULUS_TB);
    	if (!mCamera_xz) {
        	//dc1394_log_error("Failed to initialize camera with ??\n", list->ids[0].guid);
		dc1394_log_error("Failed to initialize camera with ??\n", FOCULUS_TB);
        	return false;
    	}
    dc1394_camera_free_list (list);

/*	uint64_t choice;// = getCamera_choice_xz(); //commented by eric to compile
	switch (choice) //note the device IDs are assumed to be foculus first, then pixelink.
	{
		case 1: //foculus
		g_print("The side camera is using foculus.\n");
		choice = FOCULUS;
		mCamera_xz = dc1394_camera_new (md,FOCULUS);
		break;
		case 2: //pixelink
		g_print("The side camera is using pixelink.\n");
		choice = PIXELINK;
		mCamera_xz = dc1394_camera_new (md,PIXELINK);
		break;
		case 3: //foculus 2
		g_print("The side camera is using the second foculus.\n");
		choice = FOCULUS2;
		mCamera_xz = dc1394_camera_new (md,FOCULUS2);
		break;
		case 4: //pike
		g_print("The top camera is using pike.\n");
		choice = PIKE;
		mCamera_xz = dc1394_camera_new (md, PIKE);
		break;
		default:
		g_print("ERROR: Incorrect input to side camera selected.\n");
		break; //will create an error
	}

	if (!mCamera_xz)
	{
		dc1394_log_error("Failed to initialize side camera with guid %llx", choice);
		dc1394_camera_free (mCamera_xz);
		//stopGrabbingVideo();
		//deinitialize();
		return false;
	}

	dc1394_camera_free_list (list); */

	dc1394_video_set_iso_channel(mCamera_xz,11);

	err = dc1394_video_set_iso_speed(mCamera_xz, DC1394_ISO_SPEED_400);
	if (err != 0)
	{
		g_print("Error in setting video ISO speed. Code : %i.\n",err);
		return false;
	}

	err = dc1394_video_set_mode(mCamera_xz, DC1394_VIDEO_MODE_640x480_MONO8); //Might to change camera mode
	if (err != 0)
	{
		g_print("Error in setting video mode. Code : %i.\n",err);
		return false;
	}
/*	might need to do this later
	if(choice == PIXELINK)
	{
		err = dc1394_format7_set_image_size(mCamera_xz,DC1394_VIDEO_MODE_FORMAT7_0, 648, 480);
		if (err != 0)
		{
			g_print("Error in setting video size. Code : %i.\n",err);
			return false;
		}
		err = dc1394_format7_set_image_position(mCamera_xz,DC1394_VIDEO_MODE_FORMAT7_0, 624, 1272);
		if (err != 0)
		{
			g_print("Error in setting video position. Code : %i.\n",err);
			return false;
		}
	}
	else if(choice == PIKE)
	{
		err = dc1394_format7_set_image_size(mCamera_xz,DC1394_VIDEO_MODE_FORMAT7_0, 640, 480);
		if (err != 0)
		{
			g_print("Error in setting video image size. Code : %i.\n",err);
			return false;
		}
		err = dc1394_format7_set_image_position(mCamera_xz,DC1394_VIDEO_MODE_FORMAT7_0, 600, 1150);
		if (err != 0)
		{
			g_print("Error in setting video position. Code : %i.\n",err);
			return false;
		}
	}
	else if(choice == FOCULUS || choice == FOCULUS2)
	{
		err = dc1394_format7_set_image_size(mCamera_xz,DC1394_VIDEO_MODE_FORMAT7_0, 640, 480);
		if (err != 0)
		{
			g_print("Error in setting video size. Code : %i.\n",err);
			return false;
		}
	}*/

	DC1394_ERR_CLN_RTN(err,dc1394_camera_free (mCamera_xz),"cannot choose format7_0");
	printf ("I: video mode is format7_0\n");

	uint32_t channel;
	dc1394_video_get_iso_channel(mCamera_xz,&channel);
	printf ("ISO channel: %i\n", channel);

	mIsInitialized_xz = true;

	if (!setGain_xz(mGain_xz))
	{
		mIsInitialized_xz = false;
		return false;
	}

	if(!setShutter_xz(mShutter_xz))
	{
		mIsInitialized_xz = false;
		return false;
	}

	if(!setBrightness_xz(mBrightness_xz))
	{
		mIsInitialized_xz = false;
		return false;
	}

	uint32_t band = 0;
	err = dc1394_video_get_bandwidth_usage(mCamera_xz,&band);
	g_print("Side Bandwidth usage: %i\n", band);

	return true;
}

bool FWcamera::setShutter(int s)
{
	mShutter = s;
	dc1394error_t err;
	if(mIsInitialized)
	{
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_SHUTTER, mShutter);
		if (err != 0)
		{
			g_print("Error in setting top shutter. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setGain(int g)
{
	mGain = g;
	dc1394error_t err;
	if(mIsInitialized)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_GAIN, mGain);
		if (err != 0)
		{
			g_print("Error in setting top gain. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setBrightness(int g)
{
	mBrightness = g;
	dc1394error_t err;
	if(mIsInitialized)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_BRIGHTNESS, mBrightness);
		if (err != 0)
		{
			g_print("Error in setting top brightness. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setFPS(int g)
{
	mFPS = g;
	dc1394error_t err;
	if(mIsInitialized)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera, DC1394_FEATURE_FRAME_RATE, mFPS);
		if (err != 0)
		{
			g_print("Error in setting top FPS. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}


bool FWcamera::setShutter_xz(int s)
{
	mShutter_xz = s;
	dc1394error_t err;
	if(mIsInitialized_xz)
	{
		err = dc1394_feature_set_value (mCamera_xz, DC1394_FEATURE_SHUTTER, mShutter_xz);
		if (err != 0)
		{
			g_print("Error in setting side shutter. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setGain_xz(int g)
{
	mGain_xz = g;
	dc1394error_t err;
	if(mIsInitialized_xz)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera_xz, DC1394_FEATURE_GAIN, mGain_xz);
		if (err != 0)
		{
			g_print("Error in setting side gain. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setBrightness_xz(int g)
{
	mBrightness_xz = g;
	dc1394error_t err;
	if(mIsInitialized_xz)
	{
		dc1394error_t err;
		err = dc1394_feature_set_value (mCamera_xz, DC1394_FEATURE_BRIGHTNESS, mBrightness_xz);
		if (err != 0)
		{
			g_print("Error in setting side brightness. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}

bool FWcamera::setFPS_xz(int g)
{
	mFPS_xz = g;
	if(mIsInitialized_xz)
	{
		dc1394error_t err;
		err = dc1394_feature_set_absolute_value (mCamera_xz, DC1394_FEATURE_FRAME_RATE, mFPS_xz);
		if (err != 0)
		{
			g_print("Error in setting side FPS. Code: %i.\n",err);
			return false;
		}
		return true;
	}
	return false;
}



FWcamera::~FWcamera(void)
{
	deinitialize();
}

void FWcamera::deinitialize( void )
{
	if(!mIsInitialized)
		return;

	g_print("Camera deinitializing\n");
	mIsInitialized = false;
	stopCapture = true;
	mIsGrabbingVideo = false;

	dc1394_camera_free (mCamera);                                           /* cleanup and exit */
	if(false)//	if (getCameraNum () == 2 && mIsInitialized_xz) //changed by eric to compile
	{
		dc1394_camera_free (mCamera_xz);
		mIsInitialized_xz = false;
	}
	dc1394_free(md);
	g_print("Camera deinitialized\n");
}

bool FWcamera::startGrabbingVideo()
{
	dc1394error_t err;
	err=dc1394_capture_setup(mCamera, 8, DC1394_CAPTURE_FLAGS_DEFAULT);     /* Setup capture (camera, #images in buffer, capture flags) */
	if (err != 0)
	{
		g_print("Error in starting capture for top camera. Code: %i.\n",err);//-1 is DC1394_FAILURE
		return false;
	}
	err=dc1394_video_set_transmission(mCamera, DC1394_ON);                  /* Start transmission (camera, power) */
	if (err != 0)
	{
		g_print("Error in setting top camera transmission. Code: %i.\n",err);
		return false;
	}

	return true;
}

bool FWcamera::startGrabbingVideo_xz()
{
	printf("@ the Beginning of startGrabbingVideo_xz.\n");
	dc1394error_t err;
	err=dc1394_capture_setup(mCamera_xz, 8, DC1394_CAPTURE_FLAGS_DEFAULT);    /* Setup capture (camera, #images in buffer, capture flags) */
	if (err != 0)
	{
		g_print("Error in starting capture for side camera. Code: %i.\n",err);
		return false;
	}
	err=dc1394_video_set_transmission(mCamera_xz, DC1394_ON);                  /* Start transmission (camera, power) */
	if (err != 0)
	{
		g_print("Error in setting side camera transmission. Code: %i.\n",err);
		return false;
	}

	printf("@ the end of startGrabbingVideo_xz.\n");
	return true;
}

unsigned char * FWcamera::grabAframe(void)
{
	int i=0;
	int whichFrame = 0; //which frame to return, 1 or 2.
	//unsigned char* src = (unsigned char*)malloc(sizeof(unsigned int)*640*480*1);
	dc1394error_t err;

	i = 0;
	do 	//this loop clears out old frames from the buffer to ensure we are getting the most recent frame.
	{	//It solves a problem where sections of the image are old and some are new when the buffer is not empty.
		err = dc1394_capture_dequeue(mCamera, DC1394_CAPTURE_POLICY_POLL, &mFrame);/* Capture */
		if (err != 0)
		{
			g_print("Error in top camera dequeue. Code: %i.\n",err);
			//return NULL;
		}
		if(mFrame != NULL)
			dc1394_capture_enqueue(mCamera, mFrame); //enqueue if successfully grabbed
		if( (mFrame != NULL) ) //if we got mFrame successfully, try again with mFrame2
		{

			err = dc1394_capture_dequeue(mCamera, DC1394_CAPTURE_POLICY_POLL, &mFrame2);/* Capture */
			if (err != 0)
			{
				g_print("Error in top camera dequeue. Code: %i.\n",err);
				//return NULL;
			}
			if(mFrame2 != NULL)
				dc1394_capture_enqueue(mCamera, mFrame2); //enqueue if successfully grabbed
			else
			{
				whichFrame = 1;
				break; //if mFrame success but mFrame2 fail
			}
		}
		i++;
	}while(   (mFrame == NULL)  ); //repeat while the first capture was unsuccessful

	if(whichFrame ==1)
		return mFrame->image;
	else
		return mFrame2->image;
}

unsigned char * FWcamera::grabAframe_xz(void)
{
	//unsigned char* src = (unsigned char*)malloc(sizeof(unsigned int)*640*480*1);
	dc1394error_t err;
	int i = 0;
	int whichFrame = 0; //which frame to return, 1 or 2.
	do
	{
		usleep(500); //adjustable sleep function
		//err=dc1394_capture_dequeue(mCamera_xz, DC1394_CAPTURE_POLICY_WAIT, &mFrame_xz);/* Capture */
		err=dc1394_capture_dequeue(mCamera_xz, DC1394_CAPTURE_POLICY_POLL, &mFrame_xz);/* Capture */
		if (err != 0)
		{
			g_print("Error in side camera dequeue. Code: %i.\n",err);
			//return NULL;
		}
		//
		if(mFrame_xz != NULL)
			dc1394_capture_enqueue(mCamera_xz, mFrame_xz); //enqueue if successfully grabbed
		if( (mFrame_xz != NULL) ) //if we got mFrame successfully, try again with mFrame2
		{

			err = dc1394_capture_dequeue(mCamera_xz, DC1394_CAPTURE_POLICY_POLL, &mFrame2_xz);/* Capture */
			if (err != 0)
			{
				g_print("Error in top camera dequeue. Code: %i.\n",err);
				//return NULL;
			}
			if(mFrame2_xz != NULL)
				dc1394_capture_enqueue(mCamera_xz, mFrame2_xz); //enqueue if successfully grabbed
			else
			{
				whichFrame = 1;
				break; //if mFrame success but mFrame2 fail
			}
		}
		//
		if(i == 500000)
		{
			g_print("SIDE! This may be a while...\n");
		}
		if (i == 1000000)
		{
			g_print("SIDE! You've been waiting a long time...\n");
		}
		if (i == 100000000)
		{
			g_print("SEGFAULT TIME! SIDE!\n");
			break;
		}
		i++;
	}while(mFrame_xz == NULL);
	//DC1394_ERR_RTN(err,"Problem getting an image");
	//memcpy something here

	if(whichFrame ==1)
		return mFrame_xz->image;
	else
		return mFrame2_xz->image;

	//dc1394_capture_enqueue(mCamera_xz,mFrame_xz);

	//return src;
	//return mFrame_xz->image;
}

bool FWcamera::isGrabbingVideo(void)
{
	return mIsGrabbingVideo;
}

void FWcamera::stopGrabbingVideo()
{
	printf("Stopping iso transmission...\n");
	dc1394error_t err;

	dc1394_capture_enqueue(mCamera,mFrame);
	usleep(1e3);
	err=dc1394_video_set_transmission(mCamera, DC1394_OFF);
	if (err != 0)
	{
		g_print("Error in top transmission set to OFF. Code: %i.\n",err);
		//return;
	}
	usleep(1e3);
	err=dc1394_capture_stop(mCamera);
	if (err != 0)
	{
		g_print("Error in stopping top capture. Code: %i.\n",err);
		//return;
	}

	mIsGrabbingVideo = false;
	printf("Iso transmission stopped...\n");
}

void FWcamera::stopGrabbingVideo_xz()
{
	printf("Stopping iso transmission...\n");
	dc1394error_t err;

	dc1394_capture_enqueue(mCamera_xz,mFrame_xz); //we know this function is throwing an error (libdc1394 error: VIDEO1394_IOC_LISTEN_QUEUE_BUFFER ioctl failed!), but that error does not seem to affect the program
	usleep(1e3);
	err=dc1394_video_set_transmission(mCamera_xz, DC1394_OFF);
	if (err != 0)
	{
		g_print("Error in side transmission set to OFF. Code: %i.\n",err);
		//return;
	}
	usleep(1e3);
	err=dc1394_capture_stop(mCamera_xz);
	if (err != 0)
	{
		g_print("Error in stopping side capture. Code: %i.\n",err);
		//return;
	}

	mIsGrabbingVideo = false;
	printf("Iso transmission stopped...\n");
}
