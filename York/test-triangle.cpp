#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string.h>
#include "general_header.hpp"
#include "NETUSBCAM_API.h"
#include "ICubeDefines.h"
#include <unistd.h>
#include <signal.h>

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//Read: usable variables: Robot: r_x,r_y,r_angle; Cargo: c_x,c_y

//~/Desktop/Undergraduate_Thesis/York$ g++ `pkg-config --cflags gtk+-3.0` `pkg-config --cflags opencv` -o test test.cpp `pkg-config --libs opencv` `pkg-config --libs gtk+-3.0` -lNETUSBCAM -std=c++11
//~/Desktop/Undergraduate_Thesis/York$ ./test

using namespace std;
using namespace cv;

bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
    double i = fabs( contourArea(Mat(contour1)) );
    double j = fabs( contourArea(Mat(contour2)) );
    return ( i < j );
}

void SignalHandler(int i)
{
   signal(SIGINT, SignalHandler);
}

Mat camera;
int GetImage(void *buffer, unsigned int buffersize, void *context)
{
  camera = Mat(1024,1280, CV_8UC3, buffer);
}

int ifframe(Mat camera){
    Size s = camera.size();
    int rows = s.height;
    int cols = s.width;
    if (rows > 0 && cols > 0){
        return 1;
    } else{
        return 0;
    }
}

int main(){
    int result=0;
    result = NETUSBCAM_Init();		// look for ICubes
	if(result==0){
		printf("No device\n");
		return 0; }

  	signal(SIGINT, SignalHandler);		// register signal handler for Ctrl+C

	result = NETUSBCAM_Open(0);	// open camera
	if(result!=0){
		printf("Error: Open; Result = %d\n", result);
		return 0; }

    // set the camera clock lower, if a lot of bad frames arriving
	result = NETUSBCAM_SetCamParameter(0,REG_PLL,20);
	if(result!=0){
		printf("Error: REG_PLL; Result = %d\n", result);
		return 0; }

	// if active, badframes are sent to the callback with buffersize = 0
	result = NETUSBCAM_SetCamParameter(0,REG_CALLBACK_BR_FRAMES,1);
	if(result!=0){
		printf("Error: REG_CALLBACK_BR_FRAMES; Result = %d\n", result);
		return 0; }

    // set the callback to get the frame buffer
  	result = NETUSBCAM_SetCallback(0,CALLBACK_RGB,&GetImage,NULL);
  	if(result!=0){
  		printf("Error: SetCallback; Result = %d\n", result);
  		return 0; }

  	 // start streaming of camera
  	result = NETUSBCAM_Start(0);
  	if(result!=0){
  		printf("Error: Start; Result = %d\n", result);
  		return 0; }

  int mouse_x_1;
  int mouse_y_1;
  int mouse_x_2;
  int mouse_y_2;
  int mouse_x_3;
  int mouse_y_3;
  int mouse_x_4;
  int mouse_y_4;
  int mouse_x_5;
  int mouse_y_5;
  int mouse_x_6;
  int mouse_y_6;
  int arena_w=400;//abs(mouse_x_1-mouse_x_2);
  int arena_h=300;//abs(mouse_y_1-mouse_y_2);
  int occupy[arena_h][arena_w];//the occupy matrix has the size of the arena
  //Mat frame;
  Mat hsv;
  Mat mask;
  Mat mask2;
  Mat binaryImg;
  while(1){

    if(ifframe(camera)){

        vector<vector<Point> > cnts;
        vector<vector<Point> > cnts2;
        //cap >> frame;
        // if (camera.empty())
        //   break;
        //frame=frame(Rect(50,40,580,350)); //crop the frame

//draw arena--------------
        line(camera,Point(mouse_x_1,mouse_y_1),Point(mouse_x_1+arena_w,mouse_y_1),Scalar(0,255,0),3);
        line(camera,Point(mouse_x_1,mouse_y_1),Point(mouse_x_1,mouse_y_1-arena_h),Scalar(0,255,0),3);
        line(camera,Point(mouse_x_2,mouse_y_2),Point(mouse_x_1+arena_w,mouse_y_1),Scalar(0,255,0),3);
        line(camera,Point(mouse_x_2,mouse_y_2),Point(mouse_x_1,mouse_y_1-arena_h),Scalar(0,255,0),3);
        line(camera,Point(mouse_x_3,mouse_y_3),Point(mouse_x_4,mouse_y_4),Scalar(0,255,0),3);
        line(camera,Point(mouse_x_5,mouse_y_5),Point(mouse_x_6,mouse_y_6),Scalar(0,255,0),3);
//---------
        cvtColor(camera,hsv,COLOR_BGR2GRAY);
        inRange(hsv,0,80,mask);
        Mat maskfororientation = mask;
        erode(mask,mask,Mat(),Point(-1,-1),2);
        dilate(mask,mask,Mat(),Point(-1,-1),2);
    	inRange(hsv,80,230,mask2);
        erode(mask2,mask2,Mat(),Point(-1,-1),2);
        dilate(mask2,mask2,Mat(),Point(-1,-1),2);
        for (int w=0; w<arena_w; w++){
            occupy[0][w]=3;
            occupy[arena_h-1][w]=3;
            for (int j=mouse_y_3; j<mouse_y_4;j++){
                occupy[j][w]=3;
            }
            for (int j=mouse_y_5; j<mouse_y_6;j++){
                occupy[j][w]=3;
            }
            for (int h=0; h<arena_h; h++){
                occupy[h][0]=3;
                occupy[h][arena_w-1]=3;
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
            vector<RotatedRect> output2(cnts2.size());

            for( int i = 0; i < cnts.size(); i++ ){
                output[i] = minAreaRect(cnts[i]);
            }
            float r_x=output[0].center.x;
            float r_y=output[0].center.y;
            float r_w=output[0].size.width;
            float r_h=output[0].size.height;

            drawContours(camera,cnts,0,Scalar(0,0,255),2);
            putText(camera,"Robot",Point((int)r_x,(int)r_y),FONT_HERSHEY_SIMPLEX,0.5,255,2);

            //for( int i = 0; i < cnts2.size(); i++ ){


            float c_x=moments(cnts2[0]).m10/moments(cnts2[0]).m00; //use moments function to calculate center for triangle
            float c_y=moments(cnts2[0]).m01/moments(cnts2[0]).m00;

            vector<Point2f> triangle;
            minEnclosingTriangle(cnts2[0],triangle);

            float x1=triangle[0].x;
            float y1=triangle[0].y;
            float x2=triangle[1].x;
            float y2=triangle[1].y;
            float x3=triangle[2].x;
            float y3=triangle[2].y;
            float s1[5]={sqrt(pow((x1-x2),2)+pow((y1-y2),2)),x1,y1,x2,y2};
            float s2[5]={sqrt(pow((x1-x3),2)+pow((y1-y3),2)),x1,y1,x3,y3};
            float s3[5]={sqrt(pow((x2-x3),2)+pow((y2-y3),2)),x2,y2,x3,y3};
            float vertice[6]; //first one is C-hypotenus, second is B-longer side, third is A-short side

            if (min({s1[0],s2[0],s3[0]})==s1[0]){
                if (min(s2[0],s3[0])==s2[0]){
                    vertice[0]=x3;vertice[1]=y3;vertice[2]=x1;vertice[3]=y1;vertice[4]=x2;vertice[5]=y2;
                }
                else {
                    vertice[0]=x3;vertice[1]=y3;vertice[2]=x2;vertice[3]=y2;vertice[4]=x1;vertice[5]=y1;
                }
            }
            if (min({s1[0],s2[0],s3[0]})==s2[0]){
                if (min(s1[0],s3[0])==s1[0]){
                    vertice[0]=x2;vertice[1]=y2;vertice[2]=x1;vertice[3]=y1;vertice[4]=x3;vertice[5]=y3;
                }
                else {
                    vertice[0]=x2;vertice[1]=y2;vertice[2]=x3;vertice[3]=y3;vertice[4]=x1;vertice[5]=y1;
                }
            }
            if (min({s1[0],s2[0],s3[0]})==s3[0]){
                if (min(s1[0],s2[0])==s1[0]){
                    vertice[0]=x1;vertice[1]=y1;vertice[2]=x2;vertice[3]=y2;vertice[4]=x3;vertice[5]=y3;
                }
                else {
                    vertice[0]=x1;vertice[1]=y1;vertice[2]=x3;vertice[3]=y3;vertice[4]=x2;vertice[5]=y2;
                }
            }

            float c_angle=asin(abs(vertice[1]-vertice[3])/sqrt(pow((vertice[1]-vertice[3]),2)+pow(abs(vertice[0]-vertice[2]),2)));



            printf("Cargo Angle: %.2f\n",c_angle);
            drawContours(camera,cnts2,0,Scalar(0,255,0),2);

            putText(camera,"Cargo",Point((int)c_x,(int)c_y),FONT_HERSHEY_SIMPLEX,0.5,255,2);


            //get robot angle___________________________________________________________________________
            threshold (camera, binaryImg, 65, 255, THRESH_BINARY_INV );
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
            circle(camera, Point(checkPt[iDir*2], checkPt[iDir*2+1]), 5, Scalar(255,0,0));
            printf("Robot Angle: %.2f\n",r_angle);

// draw path---------------------------
            // while(!Path.empty()){
            //     Pair p = Path.top();
            //     Path.pop();
            //     circle(camera, Point(p.second,p.first), 1, Scalar(255,0,0), -1);
            //     }
//-----------------------------------------
        }
        else {
            break;
        }
        imshow("camera",camera);

        char c=(char)waitKey(1);
        if(c==27) break;
      }
  }

  //pause();
  // cap.release();
  // destroyAllWindows();

  // stop streaming of camera
  result = NETUSBCAM_Stop(0);
  if(result!=0){
      printf("Error: Stop; Result = %d\n", result);
      return 0; }

  // close camera
  result = NETUSBCAM_Close(0);
  if(result!=0){
      printf("Error: Close; Result = %d\n", result);
      return 0; }

  return 0;

}
