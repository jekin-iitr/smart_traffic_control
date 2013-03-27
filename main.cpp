#include<iostream>
#include<ctime>	//To calculate FPS using time(0) - returns current time
#include "caliberate.h"	//contains functions which are called only once - 1.average brightness 2.Polygon Caliberation 3.Road Image finding
using namespace std;
VideoCapture capture;	//The capture class captures video either from harddisk(.avi) or from camera
Mat frameImg;	//contains every frame of the video
//char* frameData;	//data of frameImg image
Mat g_image;		//gray image of frameImg, passed into cvFindContour function
//CvMemStorage* g_storage = cvCreateMemStorage(0);	//Memory storage for contours. Each contour corresponds to one vehicle.
double dist(Point2f x1, Point2f x2)	//distance between two points x1 and x2
{	return pow((double) (x1.x-x2.x)*(x1.x-x2.x) + (x1.y-x2.y)*(x1.y-x2.y) ,(double) .5); }
int main() 
{
	cout<<"hi\n";
	string fileName = "traffic.avi";
	capture.open(fileName);	//Video capture from harddisk(.avi) or from camera
	if( !capture.isOpened() ) 	{	cerr<<"video opening error\n"; waitKey(0); system("pause");  }

	Mat frameImg_origSize;	//image taken from camera feed in original size
	//frameImg = Mat(Size(WIDTH_SMALL, HEIGHT_SMALL),CV_8UC3);	//same image from camera feed but in smaller size for faster calculation
	namedWindow( "out"	  , CV_WINDOW_AUTOSIZE);	//window to show output
	namedWindow( "trackbar", CV_WINDOW_AUTOSIZE);	//Trackbars to change value of parameters
	resizeWindow( "trackbar", 300, 600);	//Resizing trackbar window for proper view of all the parameters
	
	
	//cout<<"hello\n";
	capture>>frameImg_origSize; //Just to know original size of video
		//imshow("video", frameImg);
		//waitKey(0);
	//cout<<"yuhoo\n";
	if( frameImg_origSize.empty() ) { cout<<"something wrong"; }
	/*while(true)
	{
		imshow("hey",frameImg_origSize);
		capture>>frameImg_origSize;
		waitKey(0);
	}*/

	resize(frameImg_origSize, frameImg, Size(WIDTH_SMALL, HEIGHT_SMALL), 0, 0, CV_INTER_AREA);	//Resize original frame into smaller frame for faster calculations
	//cout<<"resize\n";

	Size origSize = frameImg_origSize.size();	//original size
	cout<<"ORIG: size = "<<frameImg_origSize.cols
		<<" X "<<frameImg_origSize.rows
		<<" step "<<frameImg_origSize.step
		<<" nchannels "<<frameImg_origSize.channels()<<endl;	//print original size: width, height, widthStep, no of channels.

	g_image = Mat(Size(WIDTH_SMALL, HEIGHT_SMALL), CV_8UC1);	g_image.setTo(0);	//Gray image of frameImg
	//frameData  = (char*)frameImg ->imageData;	//Data of frameImg
	//calibIntensity();	//Average Intensity of all pixels in the image

	//cout<<"calibintensity\n";
	Mat roadImage = Mat(Size(WIDTH_SMALL,HEIGHT_SMALL), CV_8UC3);	//Image of the road (without vehicles)
	roadImage = findRoadImage();	//Image of the road
	
	cout<<"roadimage\n";
	//char* roadImageData = (char*)roadImage->imageData;	//Data of roadImage
	calibPolygon();	//Polygon caliberation: Select four points of polygon clockwise and press enter

	cout<<"polyArea = "<<polyArea;	//Area of selected polygon
	Mat binImage = Mat(Size(WIDTH_SMALL, HEIGHT_SMALL),CV_8UC1);	//white pixel = cars, black pixel = other than cars
	//char* binImageData = (char*)binImage->imageData;	//data of binImage
	Mat finalImage = Mat(Size(WIDTH_SMALL,HEIGHT_SMALL), CV_8UC3);	//final image to show output

	double T = time(0);	//Current time
	float fps = 0, lastCount = 0;	//frames per second
	int thresh_r = 43, thresh_g = 43, thresh_b = 49;	//Threshold parameters for Red, Green, Blue colors
	createTrackbar( "Red Threshold", "trackbar", &thresh_r, 255, 0 );	//Threshold for Red color
	createTrackbar( "Green Threshold", "trackbar", &thresh_g, 255, 0 );	//Threshold for Green color
	createTrackbar( "Blue Threshold", "trackbar", &thresh_b, 255, 0 );//Threshold for Blue color
	int dilate1=1, erode1=2, dilate2=5;	//Dilate and Erode parameters
	Mat imgA = Mat(Size(WIDTH_SMALL,HEIGHT_SMALL),CV_8SC3);//Used for opticalFlow
	//CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];	//Input points for opticalFlow
	//CvPoint2D32f* cornersB = new CvPoint2D32f[ MAX_CORNERS ];	//Output points from opticalFlow
	vector<Point2f> cornersA, cornersB;

	frameImg.copyTo(imgA);//cvCopyImage(frameImg,imgA);	//copy from frameImg to imgA
	
	int win_size = 20;	//parameter for opticalFlow
	int corner_count = MAX_CORNERS;	//no of points tracked in opticalFlow
	//Mat pyrA;// = cvCreateImage( size(WIDTH_SMALL,HEIGHT_SMALL), IPL_DEPTH_32F, 1 );	//Temp image (opticalFlow)
	//Mat pyrB;// = cvCreateImage( size(WIDTH_SMALL,HEIGHT_SMALL), IPL_DEPTH_32F, 1 );	//Temp image (opticalFlow)
	double distance;	//Length of lines tracked by opticalFlow
	int maxArrowLength = 100, minArrowLength = 0;	//div by 10 //Max and Min length of the tracked lines
	int arrowGap = 5;	//distance between consecutive tracking points (opticalFlow)
	createTrackbar("max arrow length", "trackbar", &maxArrowLength, 100, 0);	//Manually change max length of tracked lines
	createTrackbar("min arrow length", "trackbar", &minArrowLength, 100, 0);	//Manually change min length of tracked lines
	createTrackbar("dilate 1","trackbar", &dilate1, 15, 0);	//first dilate
	createTrackbar("erode 1","trackbar", &erode1, 15, 0);		//first erode
	createTrackbar("dilate 2","trackbar", &dilate2, 15, 0);	//second dilate
	char features_found[ MAX_CORNERS ];	//temp data (opticalFlow)
	float feature_errors[ MAX_CORNERS ];//temp data (opticalFlow)
	Mat dilate1_element = getStructuringElement(MORPH_ELLIPSE , Size(2 * dilate1 + 1, 2 * dilate1 + 1), Point(-1,-1) );
	Mat erode1_element = getStructuringElement(MORPH_ELLIPSE , Size(2 * erode1 + 1, 2 * erode1 + 1), Point(-1,-1) );
	Mat dilate2_element = getStructuringElement(MORPH_ELLIPSE , Size(2 * dilate2 + 1, 2 * dilate2 + 1), Point(-1,-1) );
	vector<Vec4i> hierarchy;
		vector< vector<Point> > contours;
		vector<uchar>vstatus; vector<float>verror;
	//////////////////////////////////////////////////////////////////////////
	while(true) //Loops till video buffers
	{
		cout<<endl;
		++fps;	//calculation of Frames Per Second
		capture>>frameImg_origSize; //Store image in original size
		if( frameImg_origSize.empty() ) break; //if there is no frame available (end of buffer); stop.
		resize(frameImg_origSize, frameImg, frameImg.size()); //resize original image into smaller image for fast calculation
		imshow("video", frameImg);
		
		register int X; //temp variable
		for( int i=0; i<HEIGHT_SMALL; ++i) //iter through whole frame and compare it with image of road; if greater than threshold, it must be a vehicle
		{
			for(int j=0; j<WIDTH_SMALL; ++j)
			{
				//X = i*WIDTH_STEP_SMALL+j*NCHANNELS;
				if(	abs(roadImage.at<Vec3b>(i,j)[0]-frameImg.at<Vec3b>(i,j)[0])<thresh_r &&
					abs(roadImage.at<Vec3b>(i,j)[1]-frameImg.at<Vec3b>(i,j)[1])<thresh_g &&
					abs(roadImage.at<Vec3b>(i,j)[2]-frameImg.at<Vec3b>(i,j)[2])<thresh_b ) //comparing frame image against road image using threshold of Red, Green and Blue
				{	binImage.at<uchar>(i,j) = 0;
					
				}	//other than vehicle (black)
				else
				{	binImage.at<uchar>(i,j) = 255;
					
				}	//vehicle (white)
		    }
		}
		
		frameImg.copyTo(finalImage);
		
		bitwise_and(binImage, polygonImg, binImage, noArray());	//Quadrilateral Cropping

		imshow("bin image", binImage);
		//int dilate1 = 4;
		
		dilate(binImage, binImage, dilate1_element);
		erode(binImage, binImage, erode1_element);
		dilate(binImage, binImage, dilate2_element);
		imshow("noise removed", binImage);

		//////////////////////////////////////////////////////////////////////////
		binImage.copyTo(g_image);
		
		
		//findContours( g_image, contours, hierarchy, CV_RETR_LIST  , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); 
		/// finds contours. g_image = imput image, g_storage = temp storage, &contours = location where contour info is saved
		/// CV_RETR_CCOMP = contours are stored as connected component, CV_CHAIN_APPROX_SIMPLE = contour finding method
		double  percentArea = 0; // % of area occupied by vehicles from the area of polygon
		double contoursArea = 0;

		cout<<"\ncontour size "<<contours.size()<<endl;

		//vector< vector<Point> >::iterator it;
		int idx=0;
		//imshow("gimage",g_image);

		//TODO Uncomment below contour finding code and debug it. First debug contourArea and then drawContours
		//for(it = contours.begin(); it!= contours.end(); it++)
		/*
		for(; idx<contours.size(); idx++)
		{
			cout<<"idx = "<<idx;	//idx++;
			//contoursArea +=   contourArea( Mat( *it )) ;
			//if( !contours[idx].empty() )	contoursArea += contourArea( Mat( contours[idx] ) );
			cout<<" area = "<<contoursArea<<endl;
			Scalar color( rand()&255, rand()&255, rand()&255 );
			//drawContours(finalImage, contours, idx, color, 1, 8, noArray());
		}
		*/
		
		//imshow("contour drawing", finalImage);

		//contours.clear();
		//hierarchy.clear();

		// ---------------------------------------------------------------------------------------------------------------------------
		int xCorners = 0; //No of points to be tracked by opticalFlow
		for(int i=0; i<HEIGHT_SMALL; i+=arrowGap) //preparing input points to be tracked
		{
			for(int j=0; j<WIDTH_SMALL; j+=arrowGap)
			{
				if( xCorners >= MAX_CORNERS-1 ) break; //no of points must not exceed MAX_CORNERS
				if( binImage.at<uchar>(i,j) == 255 )
				//if( binImageData[i*WIDTH_SMALL + j] == 255 ) //points must be chosen only on the vehicles (white pixels)
				{
					cornersA.push_back(Point2f(i,j));
					//cornersA[xCorners].x = j;
					//cornersA[xCorners].y = i;
					++xCorners;
				}
			}
		}
		cornersB.reserve(xCorners);
		//if( percentArea>80.0 || fps<=4 )	arrowGap=15; //reduce point density if processor is loaded
		//else if( percentArea>40.0 || fps<=7 )	arrowGap=10;
		//else	arrowGap=5;
		//x corner_count = xCorners; //no of points to be tracked
		
		
		calcOpticalFlowPyrLK(imgA,frameImg,cornersA,cornersB,vstatus, verror, Size( win_size,win_size ),5,cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),0); //calculates opticalFlow
		/// imgA = previous image; frameImg = current image; cornersA = input points; cornersB = output points; Rest is not important
			
		int xCornersInRange = 1; // No of points which satisfies Min and Max length criteria
		double avgDist = 0; //average length of tracked lines = average movement of vehicles.
		for( int i=0; i<xCorners; i++ ) //iterate through all tracking points
		{
			distance = dist(cornersA[i], cornersB[i]); //length of tracked lines = magnitude of movement of vehicle
			//if( distance < maxArrowLength/10 && distance > minArrowLength/10) //only accept points which lies in Min-Max range
			{
				++xCornersInRange;
				avgDist += distance; //add length of all lines
				line( finalImage, Point(cornersA[i].x,cornersA[i].y), Point(cornersB[i].x,cornersB[i].y) , CV_RGB(0,0,255),1 , CV_AA); //draw all tracking  lines
			}
		}
		avgDist /= xCornersInRange; //average length of lines
		cout<<avgDist;
		frameImg.copyTo(imgA);
		cornersA.clear();
		cornersB.clear();
		vstatus.clear();
		verror.clear();
		//cvCopyImage(frameImg,imgA); //current image frameImg will be previous image imgA for the next frame
		//////////////////////////////////////////////////////////////////////////
		line(finalImage, pts[0], pts[1], CV_RGB(0,255,0),1,CV_AA); //draw polygon in final image (Green)
		line(finalImage, pts[1], pts[2], CV_RGB(0,255,0),1,CV_AA);
		line(finalImage, pts[2], pts[3], CV_RGB(0,255,0),1,CV_AA);
		line(finalImage, pts[3], pts[0], CV_RGB(0,255,0),1,CV_AA);
		imshow( "out", finalImage); // show final output image
		waitKey(33);
	}
	cout<<"\nFINISH\n";
	return 0;
}
