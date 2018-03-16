#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string.h>
#include "general_header.hpp"


//~/Desktop/Undergraduate_Thesis/York$ g++ `pkg-config --cflags gtk+-3.0` `pkg-config --cflags opencv` -o test test.cpp `pkg-config --libs opencv` `pkg-config --libs gtk+-3.0`
//~/Desktop/Undergraduate_Thesis/York$ ./test

using namespace std;
using namespace cv;

bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
    double i = fabs( contourArea(Mat(contour1)) );
    double j = fabs( contourArea(Mat(contour2)) );
    return ( i < j );
}


int main(){

  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap("test2.mp4");
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  while(1){
    Mat frame;
    Mat hsv;
    Mat ir;
    Mat er;
    Mat mask;
    Mat binaryImg;
    vector<vector<Point> > cnts;
    cap >> frame;
    if (frame.empty())
      break;
    frame=frame(Rect(0,50,600,400)); //crop the frame
    cvtColor(frame,hsv,COLOR_BGR2HSV);
    inRange(hsv,Scalar(0,0,0),Scalar(60,60,60),mask);
    //erode(ir,er,0,point(-1,-1),2,1,1);
    //dilate(er,mask,0,point(-1,-1),2,1,1);
    findContours(mask,cnts,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point(0,0));
    if ((int)cnts.size()>0){
        sort(cnts.begin(), cnts.end(),compareContourAreas);
        //printf("size: %d\n", (int)cnts.size());
        vector<RotatedRect> output(cnts.size());

        for( int i = 0; i < cnts.size(); i++ ){
            output[i] = minAreaRect(cnts[i]);
        }

        float r_angle;
        float r_x=output[0].center.x;
        float r_y=output[0].center.y;
        float r_w=output[0].size.width;
        float r_h=output[0].size.height;

        //get robot angle___________________________________________________________________________
        threshold ( frame, binaryImg, 65, 255, THRESH_BINARY_INV );
        float tempAngle = output[0].angle;          // get angle of rotated rect
        int checkPt[8];
        //printf("angle: %.3f, width: %.3f, height: %.3f\n", tempAngle, width, height);
        checkPt[0] = r_x - 0.4 * r_w  * cosd(tempAngle);
        checkPt[1] = r_y - 0.4 * r_h * sind(tempAngle);
        checkPt[2] = r_x + 0.4 * r_h * sind(tempAngle);
        checkPt[3] = r_y - 0.4 * r_h * cosd(tempAngle);
        checkPt[4] = r_x + 0.4 * r_w  * cosd(tempAngle);
        checkPt[5] = r_y + 0.4 * r_h * sind(tempAngle);
        checkPt[6] = r_x - 0.4 * r_w  * sind(tempAngle);
        checkPt[7] = r_y + 0.4 * r_h * cosd(tempAngle);
        //printf("binary image channel %d depth %d\n", binaryImg.channels(), binaryImg.depth());
        //printf("pt1 %d pt2 %d pt3 %d pt4 %d\n", binaryImg.at<unsigned char>(checkPt[1],checkPt[0]),
        //                                        binaryImg.at<unsigned char>(checkPt[3],checkPt[2]),
        //                                        binaryImg.at<unsigned char>(checkPt[5],checkPt[4]),
        //                                        binaryImg.at<unsigned char>(checkPt[7],checkPt[6]));

        int iDir = -1;
        if (binaryImg.at<unsigned char>(checkPt[1],checkPt[0]) == 0){
            iDir = 0;
            r_angle=output[0].angle+180;}
        else if (binaryImg.at<unsigned char>(checkPt[3],checkPt[2]) == 0){
            iDir = 1;
            r_angle=output[0].angle+270;}
        else if (binaryImg.at<unsigned char>(checkPt[5],checkPt[4]) == 0){
            iDir = 2;
            r_angle=output[0].angle;}
        else if (binaryImg.at<unsigned char>(checkPt[7],checkPt[6]) == 0){
            iDir = 3;
            r_angle=output[0].angle+90;}
        printf("Fake Angle: %.2f\n",output[0].angle);
        printf("Robot Angle: %.2f\n",r_angle);
        //___________________________________________________________________________________
        // printf("Angle: %.2f\n",r_angle);
        // printf("X Position: %.2f\n",r_x);
        // printf("Y Position: %.2f\n",r_y);

    }
    else {
        break;
    }
    imshow( "Frame", frame);
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
  cap.release();
  destroyAllWindows();

  return 0;
}
