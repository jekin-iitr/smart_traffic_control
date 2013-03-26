/************************************************************************/
/* Common includes for main.cpp and caliberate.cpp                      */
/************************************************************************/
// shared variables are declared in this file.
#ifndef CALIBERATE_H_
#define CALIBERATE_H_
//#include  <iostream>
#include  <cmath>
//using namespace std;
#include<cv.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#define NCHANNELS 3
#define WIDTH_SMALL 320
#define HEIGHT_SMALL 240
#define WIDTH_STEP_SMALL WIDTH_SMALL*NCHANNELS
#define MAX_CORNERS 2500
#define RED   cvScalar(0,0,255)
#define GREEN cvScalar(0,255,0)
#define BLUE  cvScalar(255,0,0)
#define MAXCARS 50
// common variables used in both cpp files
extern VideoCapture capture;
extern Mat frameImg;
extern Mat polygonImg;
extern char* frameData;
extern Point pts[4];
extern double polyArea;
#endif

Mat findRoadImage(void);
void calibPolygon(void);
