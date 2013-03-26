#include<iostream>
#include"caliberate.h"
using namespace std;
Mat avgImage; //Road Image
//char* avgImageData; //data of road image
float table[WIDTH_SMALL][HEIGHT_SMALL][NCHANNELS];
bool isFixed[WIDTH_SMALL][HEIGHT_SMALL];
Mat polygonImg;
//int polyPts[4][2]; //four points of polygon
Point pts[4]; //four points of polygon
int counter=0 ;
double polyArea = 0;
bool firstTime = true;
void my_mouse_callback(int event, int x, int y, int flags, void* param )
{
	if(counter==4)	counter=0;
	switch( event ) 
	{
		case CV_EVENT_LBUTTONDOWN: 
		{
			pts[counter].x = x; pts[counter].y = y;
			//polyPts[counter][0] = x ; polyPts[counter][1] = y;
			//cout<<polyPts[counter][0]<<"  "<<polyPts[counter][1]<<endl;
			cout<<pts[counter].x<<" "<<pts[counter].y<<endl;
			if(!firstTime)
			{
				int prevCounter = (counter==0?3:counter-1);
				//line(avgImage, Point(polyPts[prevCounter][0], polyPts[prevCounter][1]), Point(polyPts[counter][0], polyPts[counter][1]), CV_RGB(255,0,0),1,CV_AA);
				line(avgImage, pts[prevCounter], pts[counter], CV_RGB(255,0,0), 1, CV_AA);
				imshow("photo_road", avgImage);
			}
			firstTime = false;
		}
		break;
		case CV_EVENT_LBUTTONUP: 
		{
			++counter;
		}
		break; 
	}
}

void calibPolygon(void)
{
	//while caliberating polygon, click on four points to select polygon.
	//If any pixel is chosen wrong keep clicking circularly clockwise to update polygon points
	//polygonImg = cvCreateImage(size(WIDTH_SMALL,HEIGHT_SMALL),IPL_DEPTH_8U,1);	cvZero(polygonImg);	//blackout area out of polygon
	polygonImg = Mat(Size(WIDTH_SMALL, HEIGHT_SMALL), CV_8UC1);	
	polygonImg.setTo(0);
	imshow("photo_road", avgImage);
	setMouseCallback("photo_road",my_mouse_callback,(void*) 0);
	waitKey(0);

	/*pts[0] = Point( polyPts[0][0], polyPts[0][1] );
	pts[1] = Point( polyPts[1][0], polyPts[1][1] );
	pts[2] = Point( polyPts[2][0], polyPts[2][1] );
	pts[3] = Point( polyPts[3][0], polyPts[3][1] );*/
	
	fillConvexPoly( polygonImg, pts, 4, cvScalar( 255, 255, 255 ), 4);
	float s,a,b,c,d,e;
	a = (double)pow((double)(pts[0].x-pts[1].x)*(pts[0].x-pts[1].x)+(pts[0].y-pts[1].y)*(pts[0].y-pts[1].y),.5);
	b = (double)pow((double)(pts[1].x-pts[2].x)*(pts[1].x-pts[2].x)+(pts[1].y-pts[2].y)*(pts[1].y-pts[2].y),.5);
	c = (double)pow((double)(pts[0].x-pts[2].x)*(pts[0].x-pts[2].x)+(pts[0].y-pts[2].y)*(pts[0].y-pts[2].y),.5);
	d = (double)pow((double)(pts[2].x-pts[3].x)*(pts[2].x-pts[3].x)+(pts[2].y-pts[3].y)*(pts[2].y-pts[3].y),.5);
	e = (double)pow((double)(pts[3].x-pts[0].x)*(pts[3].x-pts[0].x)+(pts[3].y-pts[0].y)*(pts[3].y-pts[0].y),.5);
	s = (a+b+c)/2;
	polyArea+= fabs(sqrt(s*(s-a)*(s-b)*(s-c)));
	s = (c+d+e)/2;
	polyArea+= fabs(sqrt( s*(s-c)*(s-d)*(s-e) ));
	destroyWindow("photo_road");
}

Mat findRoadImage(void)
{	
	avgImage = Mat(Size(WIDTH_SMALL, HEIGHT_SMALL), CV_8UC3);
	//avgImage = cvCreateImage( size(WIDTH_SMALL, HEIGHT_SMALL), 8, 3);	//averaged over 100 gray image frames to get gray photo of road only
	//avgImageData = (char*)avgImage->imageData;	cvZero(avgImage);
	//IplImage* img1_origSize;
	//IplImage* img1 = cvCreateImage(size(WIDTH_SMALL,HEIGHT_SMALL),8,3);	cvZero(img1);
	//IplImage* img2 = cvCreateImage(size(WIDTH_SMALL,HEIGHT_SMALL),8,3);	//previous frame of img1
	//IplImage* img3 = cvCreateImage(size(WIDTH_SMALL,HEIGHT_SMALL),8,3);	//previous frame of img2
	//IplImage* img4 = cvCreateImage(size(WIDTH_SMALL,HEIGHT_SMALL),8,3);	//previous frame of img3
	Mat img1_origSize, img1, img2, img3, img4;
	for(int i=0; i<HEIGHT_SMALL; ++i)
	{
		for(int j=0; j<WIDTH_SMALL; ++j)
		{
			isFixed[j][i] = false;
		}
	}
	//img1_origSize = cvQueryFrame(capture);	cvResize(img1_origSize, img1); cvCopyImage(img1, img4);// imshow("4",img4); cvWaitKey(0);
	capture>>img1_origSize;	
	//imshow("test",img1_origSize);
	//waitKey(20);
	//try{
	resize(img1_origSize, img1, Size(WIDTH_SMALL, HEIGHT_SMALL), 0.0, 0.0, CV_INTER_AREA);	img1.copyTo(img4);
	//}
	//catch(exception& e) {cout<<e.what()<<endl;}
	//img1_origSize = cvQueryFrame(capture);	cvResize(img1_origSize, img1); cvCopyImage(img1, img3);
	//img1_origSize = cvQueryFrame(capture);	cvResize(img1_origSize, img1); cvCopyImage(img1, img2);// imshow("2",img2); imshow("4",img4); cvWaitKey(0);
	capture>>img1_origSize;	resize(img1_origSize, img1, Size(WIDTH_SMALL, HEIGHT_SMALL), 0, 0, CV_INTER_AREA);	img1.copyTo(img3);
	capture>>img1_origSize;	resize(img1_origSize, img1, Size(WIDTH_SMALL, HEIGHT_SMALL), 0, 0, CV_INTER_AREA);	img1.copyTo(img2);
	/*char* img1data = (char*)img1->imageData;
	char* img2data = (char*)img2->imageData;
	char* img3data = (char*)img3->imageData;
	char* img4data = (char*)img4->imageData;*/

	int xSamples = 100;
	int thresh = 3;
	createTrackbar("road_thresh", "trackbar", &thresh, 50, 0);
	createTrackbar("road_xSamples", "trackbar", &xSamples, 200, 0);
	for(int i=0; i<xSamples; ++i)
	{
		capture>>img1_origSize;
		//img1_origSize = cvQueryFrame(capture);
		resize(img1_origSize, img1, img1.size(), 0, 0, INTER_LINEAR);
		int index;
		for(int h=0; h<HEIGHT_SMALL; ++h)
		{
			for(int w=0; w<WIDTH_SMALL; ++w)
			{
				index = h*WIDTH_STEP_SMALL + w*NCHANNELS;

				if( isFixed[w][h] == false &&
					abs(img1.data[index+0]-img2.data[index+0])  < thresh &&
					abs(img1.data[index+1]-img2.data[index+1])  < thresh &&
					abs(img1.data[index+2]-img2.data[index+2])  < thresh &&
					abs(img2.data[index+0]-img3.data[index+0])  < thresh &&
					abs(img2.data[index+1]-img3.data[index+1])  < thresh &&
					abs(img2.data[index+2]-img3.data[index+2])  < thresh &&
					abs(img3.data[index+0]-img4.data[index+0])  < thresh &&
					abs(img3.data[index+1]-img4.data[index+1])  < thresh &&
					abs(img3.data[index+2]-img4.data[index+2])  < thresh )
				{
					isFixed[w][h] = true;
					avgImage.data[index] = img1.data[index];
					avgImage.data[index+1] = img1.data[index+1];
					avgImage.data[index+2] = img1.data[index+2];
				}
			}
		}
		img3.copyTo(img4);
		img2.copyTo(img3);
		img1.copyTo(img2);
		//cvCopyImage(img3, img4);
		//cvCopyImage(img2, img3);
		//cvCopyImage(img1, img2);
		imshow("road_image_formation", avgImage);
		cvWaitKey(33);
	}
	return avgImage;
}
