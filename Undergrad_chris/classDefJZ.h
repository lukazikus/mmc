#ifndef CLASSDEFJZ
#define CLASSDEFJZ

#include "math_subroutine.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "s826_subroutine.h"

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class Point_JZ {
    public:
        Point_JZ (void);
        Point_JZ (int inputX, int inputY);
        Point_JZ (Point_JZ &input);
        //~Point_JZ();

        int dis_to_another_pt (Point_JZ &input);
        int update (Point_JZ &input);
        int offset_x (int input);
        int offset_y (int input);

        int x;
        int y;
};

class Vector_JZ {
    public:
        Vector_JZ (void);
        Vector_JZ (Point_JZ &pt1, Point_JZ &pt2);
        int update (Point_JZ &pt1, Point_JZ &pt2);
        int delX;
        int delY;
        Point_JZ centerPt;
        float length;
        float angle;            // pointing orientation w.r.t. +x in radians
};

class Pair_JZ {
    public:
        Pair_JZ (void);
        int update_info (Point_JZ &pt1, Point_JZ &pt2);

        Point_JZ comPt;         // center-of-mass point
        float sep;              // pair sep. in pixels
        float ori;              // pair orientation w.r.t. +x
};

class Image_JZ {
    public:
        Image_JZ (void);
        Image_JZ (cv::Mat &input);
        int update (cv::Mat &input);
        int blurImg (int para);
        int binarizeImg (int para);
        int dilateImg (int para);
        int getContours (void);
        int sortContour (void);
        cv::Mat img;
        cv::Mat processedImg;
        cv::Mat contourImg;
        std::vector<std::vector<cv::Point> > 	contours;
        int agentIndex[4];                                  // index of detected agent in contours (maximum 4)
};

// controlling hardware signal
class Coil_JZ {
    public:
        Coil_JZ (void);
        int output_signal (void);                       // output signal to amplifiers
        int clear_output (void);                        // clear all output
        int set_px_signal (float input);
        int set_nx_signal (float input);
        int set_py_signal (float input);
        int set_ny_signal (float input);
        int set_pz_signal (float input);
        int set_nz_signal (float input);
    private:
        int update_signal (int pin, float input);
        float volt[7];                        // signal to 6 amplifiers
        int range[7];                         // range code for each amplifier
};

#endif
