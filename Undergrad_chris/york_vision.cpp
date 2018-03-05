#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string.h>
#include "FWcamera.h"


//~/Desktop/Undergraduate_Thesis/York$ g++ `pkg-config --cflags opencv` -o test test.cpp `pkg-config --libs opencv`
//~/Desktop/Undergraduate_Thesis/York$ ./test

using namespace std;
using namespace cv;

bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
    double i = fabs( contourArea(Mat(contour1)) );
    double j = fabs( contourArea(Mat(contour2)) );
    return ( i < j );
}

void* visionThread(void*) {
		printf("@ the Beginning of visionThread().\n");

            // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name

    // float *coords = new float[2]; // Will hold center of mass of robot

    FWcamera cam;
    int width = 640;   //image width, pixels
    int height = 480;  //image height, pixels
    int depth = 1;     //depth of image

    // Time variables
    int i;
    timeval tStart, tEnd;
    float time;
    double current_time;
    float fpsVec[10] = {10,10,10,10,10,10,10,10,10,10};
    int fpsIndex = 0;

    gettimeofday(&tStart, NULL);
    //usleep(6e4); 	//slow down void get_coords(float *coords){
vision thread; this function waits for a new frame, it takes a long time, so we can do some image processing in this thread

    unsigned char *inImage = cam.grabAframe();	//unsigned char *inImage;
    if(inImage == NULL)	{
        g_print("Error in firewire stream! Reattempting...\n");
        usleep((int)1e3); // I don't know what the wait delay should be
    }
    Mat img_m = Mat(height, width, CV_8UC1, inImage);	//convert to Mat format

    Mat hsv;
    Mat mask;
    vector<vector<Point> > cnts;
    if (img_m.empty())
        return;

    cvtColor(img_m,hsv,COLOR_BGR2HSV);
    inRange(hsv,Scalar(0,0,0),Scalar(60,60,60),mask);
    findContours(mask,cnts,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point(0,0));
    sort(cnts.begin(), cnts.end(),compareContourAreas);
    //printf("size: %d\n", (int)cnts.size());
    vector<RotatedRect> output(cnts.size());
void get_coords(float *coords){

    for( int i = 0; i < cnts.size(); i++ ){
        output[i] = minAreaRect(cnts[i]);
    }

    drawContours(img_m,cnts,0,Scalar(0,0,255),2);
    float r_angle=-output[0].angle;
    float r_x=output[0].center.x;
    float r_y=output[0].center.y;
    float r_w=output[0].size.width;
    float r_h=output[0].size.height;

    coords[0] = r_x; coords[1] = r_y; // Extract x and y coordinates of robot

    if(r_w<r_h){
        r_angle=r_angle+90;
    } else {
        r_angle=r_angle;
    }

    printf("Angle: %.2f\n",r_angle);
    printf("X Position: %.2f\n",r_x);
    printf("Y Position: %.2f\n",r_y);

    // putText(img_m,"Cargo",Point((int)r_x,(int)r_y),FONT_HERSHEY_SIMPLEX,0.5,255,2);
    // putText(img_m,"X:",Point(100,120),FONT_HERSHEY_SIMPLEX,0.5,255,2);
    // putText(img_m,"Y:",Point(100,150),FONT_HERSHEY_SIMPLEX,0.5,255,2);
    // putText(img_m,"Angle:",Point(100,180),FONT_HERSHEY_SIMPLEX,0.5,255,2);

    // imshow( "Frame", img_m);
    // char c=(char)waitKey(25);
    // if(c==27)
    //     break;
    // }

    // return coords;
}
