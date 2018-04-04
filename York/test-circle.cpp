#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string.h>
#include "general_header.hpp"
#include "NETUSBCAM_API.h"
#include "ICubeDefines.h"
//Read: usable variables: Robot: r_x,r_y,r_angle; Cargo: c_x,c_y

//~/Desktop/Undergraduate_Thesis/York$ g++ `pkg-config --cflags gtk+-3.0` `pkg-config --cflags opencv` -o test test.cpp `pkg-config --libs opencv` `pkg-config --libs gtk+-3.0` -lNETUSBCAM
//~/Desktop/Undergraduate_Thesis/York$ ./test

using namespace std;
using namespace cv;

bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
    double i = fabs( contourArea(Mat(contour1)) );
    double j = fabs( contourArea(Mat(contour2)) );
    return ( i < j );
}

// unsigned int nGoodCnt=0;
// unsigned int nBadCnt=0;
// int GetImage(void *buffer, unsigned int buffersize, void *context)
// {
//   if(buffersize==0){		// badframe arrived (this happens here, when (REG_CALLBACK_BR_FRAMES==1)
// 	nBadCnt++;
//   }
//   else				// good frame arrived
//   {
// 	nGoodCnt++;
//   char filename[24];
//   sprintf(filename,"Output%d.jpg",nGoodCnt);
//   printf("%p",buffer);
//
// 	#ifdef SAVE_RAW
//   printf("%p\n", buffer);
// 	char buf[24];
// 	sprintf(buf,"Nr%d.raw",nGoodCnt);
// 	SaveRaw((unsigned char*)buffer,buffersize,buf);
// 	#endif
//   }
// }

int main(){

  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name

  // NETUSBCAM_Init();
  // NETUSBCAM_Open(0);
  // NETUSBCAM_SetCamParameter(0,REG_PLL,20);
  // NETUSBCAM_SetCamParameter(0,REG_CALLBACK_BR_FRAMES,1);
  // NETUSBCAM_SetCallback(0,CALLBACK_RGB,&GetImage,NULL);
  // NETUSBCAM_Start(0);
  VideoCapture cap("test2.mp4");
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
  int mouse_x_1;
  int mouse_y_1;
  int mouse_x_2;
  int mouse_y_2;
  int arena_w=400;//abs(mouse_x_1-mouse_x_2);
  int arena_h=300;//abs(mouse_y_1-mouse_y_2);
  int occupy[arena_h][arena_w];//the occupy matrix has the size of the arena
  Mat frame;
  Mat hsv;
  Mat mask;
  Mat mask2;
  Mat binaryImg;
  while(1){
    vector<vector<Point> > cnts;
    vector<vector<Point> > cnts2;
    cap >> frame;
    if (frame.empty())
      break;
    frame=frame(Rect(50,40,580,350)); //crop the frame
    cvtColor(frame,hsv,COLOR_BGR2GRAY);
    inRange(hsv,0,60,mask);
    Mat maskfororientation = mask;
    erode(mask,mask,Mat(),Point(-1,-1),2);
    dilate(mask,mask,Mat(),Point(-1,-1),2);
    imshow("hsv", mask);
	inRange(hsv,80,230,mask2);
    erode(mask2,mask2,Mat(),Point(-1,-1),2);
    dilate(mask2,mask2,Mat(),Point(-1,-1),2);
    for (int w=0; w<arena_w; w++){
        for (int h=0; h<arena_h; h++){
            if (mask.at<int>(h,w)==255){
                occupy[h][w]=1;
            }
            if (mask2.at<int>(h,w)==255){
                occupy[h][w]=2;
            }
        }
    }


    findContours(mask,cnts,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point(0,0));
	findContours(mask2,cnts2,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point(0,0));

    if ((int)cnts.size()>0){

        sort(cnts.begin(), cnts.end(),compareContourAreas);

    	sort(cnts2.begin(), cnts2.end(),compareContourAreas);

        //printf("size: %d\n", (int)cnts.size());
        vector<RotatedRect> output(cnts.size());

        for( int i = 0; i < cnts.size(); i++ ){
            output[i] = minAreaRect(cnts[i]);
        }
        float r_x=output[0].center.x;
        float r_y=output[0].center.y;
        float r_w=output[0].size.width;
        float r_h=output[0].size.height;

		drawContours(frame,cnts,0,Scalar(0,0,255),2);
        putText(frame,"Robot",Point((int)r_x,(int)r_y),FONT_HERSHEY_SIMPLEX,0.5,255,2);
        Point2f center;
        float radius;
        //for( int i = 0; i < cnts2.size(); i++ ){
        minEnclosingCircle(cnts2[0],center,radius);
        //}
        float c_x=center.x;
        float c_y=center.y;
        float c_r=radius;
        circle(frame, Point((int)c_x,(int)c_y), (int)radius, Scalar(0, 255, 0), 2, 8, 0 );
        putText(frame,"Cargo",Point((int)c_x,(int)c_y),FONT_HERSHEY_SIMPLEX,0.5,255,2);


        //get robot angle___________________________________________________________________________
        threshold (frame, binaryImg, 65, 255, THRESH_BINARY_INV );
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

        float r_angle;
        int iDir = -1;
        if (maskfororientation.at<unsigned char>(checkPt[1],checkPt[0]) == 0){
            iDir = 0;
            r_angle=output[0].angle+180;}
        else if (maskfororientation.at<unsigned char>(checkPt[3],checkPt[2]) == 0){
            iDir = 1;
            r_angle=output[0].angle+270;}
        else if (maskfororientation.at<unsigned char>(checkPt[5],checkPt[4]) == 0){
            iDir = 2;
            r_angle=output[0].angle+360;}
        else if (maskfororientation.at<unsigned char>(checkPt[7],checkPt[6]) == 0){
            iDir = 3;
            r_angle=output[0].angle+90;}
        circle(frame, Point(checkPt[iDir*2], checkPt[iDir*2+1]), 5, Scalar(255,0,0));
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
  //NETUSBCAM_Stop(0);
  cap.release();
  destroyAllWindows();

  return 0;
}
