#include "classDefJZ.h"

Point_JZ :: Point_JZ (void) {
    x = 0;
    y = 0;
}
Point_JZ :: Point_JZ (int inputX, int inputY) {
    x = inputX;
    y = inputY;
}

Point_JZ :: Point_JZ (Point_JZ &input) {
    x = input.x;
    y = input.y;
}

int Point_JZ :: dis_to_another_pt (Point_JZ &input) {
    int dis = (int) sqrt(pow(input.x - x, 2) + pow(input.y - y, 2));
    return dis;
}

int Point_JZ :: update (Point_JZ &input) {
    x = input.x;
    y = input.y;
    return 1;
}

int Point_JZ :: offset_x (int input) {
    x = x + input;
    return 1;
}

int Point_JZ :: offset_y (int input) {
    y = y + input;
    return 1;
}

Vector_JZ :: Vector_JZ (void) {

}

Vector_JZ :: Vector_JZ (Point_JZ &pt1, Point_JZ &pt2) {
    delX = pt2.x - pt1.x;
    delY = pt2.y - pt1.y;
    centerPt.x = (int) (0.5 * (pt1.x + pt2.x));
    centerPt.y = (int) (0.5 * (pt1.y + pt2.y));
    length = (float) sqrt (pow (delX, 2) + pow (delY, 2));
    angle = (float) atan2 (delY, delX);
}

int Vector_JZ :: update (Point_JZ &pt1, Point_JZ &pt2) {
    delX = pt2.x - pt1.x;
    delY = pt2.y - pt1.y;
    centerPt.x = (int) (0.5 * (pt1.x + pt2.x));
    centerPt.y = (int) (0.5 * (pt1.y + pt2.y));
    length = (float) sqrt (pow (delX, 2) + pow (delY, 2));
    angle = (float) atan2 (delY, delX);
    return 1;
}

Pair_JZ :: Pair_JZ (void) {

}

// update pair information
int Pair_JZ :: update_info (Point_JZ &pt1, Point_JZ &pt2) {
    comPt.x = (int) (0.5 * (pt1.x + pt2.x));
    comPt.y = (int) (0.5 * (pt1.y + pt2.y));

    sep = (float) sqrt (pow(pt1.x-pt2.x, 2) + pow(pt1.y-pt2.y, 2));
    ori = (float) atan2 (pt2.y-pt1.y, pt2.x-pt1.x);
    return 1;
}

Image_JZ :: Image_JZ (void) {

}

Image_JZ :: Image_JZ (cv::Mat &input) {
    input.copyTo (img);
}

int Image_JZ :: update (cv::Mat &input) {
    input.copyTo (img);
    return 1;
}

int Image_JZ :: blurImg (int para) {
    cv::blur (img, processedImg, cv::Size(para,para));          // run blur before any other operations
    return 1;
}

int Image_JZ :: binarizeImg (int para) {
    cv::threshold (processedImg, processedImg, para, 255, cv::THRESH_BINARY_INV);
    return 1;
}

int Image_JZ :: dilateImg (int para) {
    cv::dilate (processedImg, processedImg, cv::Mat(), cv::Point(-1, -1), para, 1, 1);
    return 1;
}

int Image_JZ :: getContours (void) {
    processedImg.copyTo (contourImg);
    cv::findContours (contourImg, contours, 3, 2, cv::Point(0, 0)); //find contours, will destroy input image
    return 1;
}

// sort the extracted contours from largest to smallest and put results into agentIndex
int Image_JZ :: sortContour (void) {
    int nContour = contours.size();                         // # of contours found
    if (nContour == 0)                                      // if no contours are detected
        return 0;
    float area[nContour];                                   // area of each entry in contours
    int index[nContour];                                    // index of each area in contours
    float tempArea = 0.0;
    int tempIndex = 0;
    for (int i = 0; i < nContour; i ++) {
        area [i] = cv::contourArea (contours[i], false);
        index[i] = i;
    }
    for (int i = 0; i < nContour-1; i ++) {
        for (int j = i+1; j < nContour; j ++) {
            if (area[i] < area[j]) {
                tempArea  = area[i];
                tempIndex = index[i];
                area[i]   = area[j];
                index[i]  = index[j];
                area[j]   = tempArea;
                index[j]  = tempIndex;
            }
        }
    }
    //printf("%d agents are detected.\n", nContour);
    if (nContour <= 4) {
        for (int i = 0; i < nContour; i ++)
            agentIndex[i] = index[i];
        return nContour;
    } else {
        for (int i = 0; i < 4; i ++)
            agentIndex[i] = index[i];
        return nContour;
    }
    /*
    if (nContour < 2) {
        printf("Only %d agents are detected.\n", nContour);
        return 0;
    } else if (nContour < 4) {
        printf("Only %d agents are detected.\n", nContour);
        for (int i = 0; i < nContour; i ++)
            cl[i] = index[i];                           // ??? what is cl
        return 2;
    } else {
        for (int i = 0; i < 4; i ++)
            agentIndex[i] = index[i];
        return 1;
    }
    */
}

// controlling hardware signal
// constructor
Coil_JZ :: Coil_JZ (void) {
    for (int i = 0; i < 7; i ++) {
        volt[i] = 0;
        range[i] = 0;
    }
}

int Coil_JZ :: output_signal (void) {
    for (int i = 0; i < 7; i ++) {
        s826_aoPin (i, range[i], volt[i]);
    }
    printf("output signal %.2f, %.2f, %.2f, %.2f %.2f %.2f\n", volt[0], volt[1],volt[2],volt[3],volt[4],volt[5]);
    return 1;
}

int Coil_JZ :: clear_output (void) {
    for (int i = 0; i < 7; i ++) {
        volt[i] = 0.0;
        range[i] = 2;
    }
    output_signal ();
    return 1;
}

int Coil_JZ :: set_px_signal (float input) {
    update_signal (0, input);
    return 1;
}
int Coil_JZ :: set_nx_signal (float input) {
    update_signal (3, input);
    return 1;
}
int Coil_JZ :: set_py_signal (float input) {
    update_signal (4, input);
    return 1;
}
int Coil_JZ :: set_ny_signal (float input) {
    update_signal (1, input);
    return 1;
}
int Coil_JZ :: set_pz_signal (float input) {
    update_signal (2, input);
    //update_signal (6, input);                   // should be pin 2, but amp 1 is not working!!!
    return 1;
}
int Coil_JZ :: set_nz_signal (float input) {
    update_signal (5, input);
    return 1;
}
int Coil_JZ :: update_signal (int pin, float input) {
    volt[pin] = input;                // pin-0 - px coil
    /*
    if (input >= 0) {
        if (input <= 5)
            range[pin] = 0;           // 0 - +5 V
        else
            range[pin] = 1;           // 0 - +10 V
    } else {
        if (input >= -5)
            range[pin] = 2;           // -5 - +5 V
        else
            range[pin] = 3;           // -10 - +10 V
    }*/
    range[pin] = 2;
    return 1;
}
