#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string.h>


//~/Desktop/Undergraduate_Thesis/York$ g++ `pkg-config --cflags opencv` -o test test.cpp `pkg-config --libs opencv`
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
  VideoCapture cap("test.mp4");
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
        float r_angle=-output[0].angle;
        float r_x=output[0].center.x;
        float r_y=output[0].center.y;
        float r_w=output[0].size.width;
        float r_h=output[0].size.height;

        if (r_w*r_h<3500){
            //Robot Detection
            drawContours(frame,cnts,0,Scalar(0,0,255),2);

            if(r_w<r_h){
                r_angle=r_angle+90;
            } else {
                r_angle=r_angle;
            }

            printf("Robot Angle: %.2f\n",r_angle);
            printf("Robot X Position: %.2f\n",r_x);
            printf("Robot Y Position: %.2f\n",r_y);


            putText(frame,"Robot",Point((int)r_x,(int)r_y),FONT_HERSHEY_SIMPLEX,0.5,255,2);
            putText(frame,"X:",Point(100,120),FONT_HERSHEY_SIMPLEX,0.5,255,2);
            putText(frame,"Y:",Point(100,150),FONT_HERSHEY_SIMPLEX,0.5,255,2);
            putText(frame,"Angle:",Point(100,180),FONT_HERSHEY_SIMPLEX,0.5,255,2);

            //Cargo Detection
            drawContours(frame,cnts,1,Scalar(0,0,255),2);
            float c_angle=-output[1].angle;
            float c_x=output[1].center.x;
            float c_y=output[1].center.y;
            float c_w=output[1].size.width;
            float c_h=output[1].size.height;

            if(c_w<c_h){
                c_angle=c_angle+90;
            } else {
                c_angle=c_angle;
            }

            printf("Cargo Angle: %.2f\n",c_angle);
            printf("Cargo X Position: %.2f\n",c_x);
            printf("Cargo Y Position: %.2f\n",c_y);

        }
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
