//objectTrackingTutorial.cpp

//Written by  Kyle Hounslow 2015

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.
#include <sstream>
#include <string>
#include <iostream>
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "RaspiCamCV.h"
#include <stdio.h>
#include<time.h>
#ifdef __unix__
#include <signal.h>
volatile int quit_signal=0;
extern "C" void quit_signal_handler(int signum) {
 if (quit_signal!=0) exit(0); // just exit already
 quit_signal=1;
 printf("Will quit at next camera frame (repeat to kill now)\n");
}
#endif
using namespace cv;
using namespace std;
//initial min and max HSV filter values.
//these will be changed using trackbars
bool serialcomm=true;
int last_x=-1;
int last_y=0;
FILE *file;
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int R_H_MIN = 0;
int R_H_MAX = 256;
int R_S_MIN = 0;
int R_S_MAX = 256;
int R_V_MIN = 0;
int R_V_MAX = 256;
int G_H_MIN = 0;
int G_H_MAX = 256;
int G_S_MIN = 0;
int G_S_MAX = 256;
int G_V_MIN = 0;
int G_V_MAX = 256;
int RW_H_MIN = 0;
int RW_H_MAX = 256;
int RW_S_MIN = 0;
int RW_S_MAX = 256;
int RW_V_MIN = 0;
int RW_V_MAX = 0;
int GW_H_MIN = 0;
int GW_H_MAX = 256;
int GW_S_MIN = 0;
int GW_S_MAX = 256;
int GW_V_MIN = 0;
int GW_V_MAX = 256;
int detect_surface=1;
Mat roi1,roi11;
Mat roileft;
Mat roiright;
char surface;
int input=0;
char mode;
Mat cameraleft;
Mat cameraright;
//default capture width and height
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

bool calibrationMode;//used for showing debugging windows, trackbars etc.

bool mouseIsDragging;//used for showing a rectangle on screen as user clicks and drags mouse
bool mouseMove;
bool rectangleSelected;
cv::Point initialClickPoint, currentMousePoint; //keep track of initial point clicked and current position of mouse
cv::Rect rectangleROI; //this is the ROI that the user has selected
vector<int> H_ROI, S_ROI, V_ROI;// HSV values from the click/drag ROI region stored in separate vectors so that we can sort them easily

void on_trackbar(int, void*)
{//This function gets called whenever a
	// trackbar position is changed

	//for now, this does nothing.



}
void createTrackbars(){
	//create window for trackbars


	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, 255, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, 255, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, 255, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, 255, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, 255, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, 255, on_trackbar);


}
void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param){
	//only if calibration mode is true will we use the mouse to change HSV values
	if (calibrationMode == true){
		//get handle to video feed passed in as "param" and cast as Mat pointer
		Mat* videoFeed = (Mat*)param;

		if (event == CV_EVENT_LBUTTONDOWN && mouseIsDragging == false)
		{
			//keep track of initial point clicked
			initialClickPoint = cv::Point(x, y);
			//user has begun dragging the mouse
			mouseIsDragging = true;
		}
		/* user is dragging the mouse */
		if (event == CV_EVENT_MOUSEMOVE && mouseIsDragging == true)
		{
			//keep track of current mouse point
			currentMousePoint = cv::Point(x, y);
			//user has moved the mouse while clicking and dragging
			mouseMove = true;
		}
		/* user has released left button */
		if (event == CV_EVENT_LBUTTONUP && mouseIsDragging == true)
		{
			//set rectangle ROI to the rectangle that the user has selected
			rectangleROI = Rect(initialClickPoint, currentMousePoint);

			//reset boolean variables
			mouseIsDragging = false;
			mouseMove = false;
			rectangleSelected = true;
		}

		if (event == CV_EVENT_RBUTTONDOWN){
			//user has clicked right mouse button
			//Reset HSV Values
			H_MIN = 0;
			S_MIN = 0;
			V_MIN = 0;
			H_MAX = 255;
			S_MAX = 255;
			V_MAX = 255;

		}
		if (event == CV_EVENT_MBUTTONDOWN){

			//user has clicked middle mouse button
			//enter code here if needed.
		}
	}

}
void recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame){

	//save HSV values for ROI that user selected to a vector
	if (mouseMove == false && rectangleSelected == true){
		
		//clear previous vector values
		if (H_ROI.size()>0) H_ROI.clear();
		if (S_ROI.size()>0) S_ROI.clear();
		if (V_ROI.size()>0 )V_ROI.clear();
		//if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
		if (rectangleROI.width<1 || rectangleROI.height<1) cout << "Please drag a rectangle, not a line" << endl;
		else{
			for (int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++){
				//iterate through both x and y direction and save HSV values at each and every point
				for (int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++){
					//save HSV value at this point
					H_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[0]);
					S_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[1]);
					V_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[2]);
				}
			}
		}
		//reset rectangleSelected so user can select another region if necessary
		rectangleSelected = false;
		//set min and max HSV values from min and max elements of each array
					
		if (H_ROI.size()>0){
			//NOTE: min_element and max_element return iterators so we must dereference them with "*"
			H_MIN = *std::min_element(H_ROI.begin(), H_ROI.end());
			H_MAX = *std::max_element(H_ROI.begin(), H_ROI.end());
			cout<<"Setting values for ";
			cout<<mode;
			cout<<" surface ";
			cout << "MIN 'H' VALUE: " << H_MIN << endl;
			cout << "MAX 'H' VALUE: " << H_MAX << endl;
			
		   switch(mode)
		   {
			   
		   case 'r':
			   cout<<"values set for red color \n";
				R_H_MIN=H_MIN;
				R_H_MAX=H_MAX;
				break;
		   case 'g':
			   cout<<"values set for green color \n";
			   G_H_MIN=H_MIN;
				G_H_MAX=H_MAX;
				break;
		   case 'w':
			   cout<<"values set for white color on red color \n";
			   RW_H_MIN=H_MIN;
				RW_H_MAX=H_MAX;
				break;
		   case 'e':
			   cout<<"values set for white color on green color \n";
			   
			  GW_H_MIN=H_MIN;
				GW_H_MAX=H_MAX;
				break;

			}
		}
		if (S_ROI.size()>0){
			S_MIN = *std::min_element(S_ROI.begin(), S_ROI.end());
			S_MAX = *std::max_element(S_ROI.begin(), S_ROI.end());
			cout << "MIN 'S' VALUE: " << S_MIN << endl;
			cout << "MAX 'S' VALUE: " << S_MAX << endl;
			
		     switch(mode)
		   {
			   
		   case 'r':
				R_S_MIN=S_MIN;
				R_S_MAX=S_MAX;
				break;
		   case 'g':
			   G_S_MIN=S_MIN;
				G_S_MAX=S_MAX;
				break;
		   case 'w':
			   RW_S_MIN=S_MIN;
				RW_S_MAX=S_MAX;
				break;
		   case 'e':
			  GW_S_MIN=S_MIN;
				GW_S_MAX=S_MAX;
				break;

			}
		}
		if (V_ROI.size()>0){
			V_MIN = *std::min_element(V_ROI.begin(), V_ROI.end());
			V_MAX = *std::max_element(V_ROI.begin(), V_ROI.end());
			cout << "MIN 'V' VALUE: " << V_MIN << endl;
			cout << "MAX 'V' VALUE: " << V_MAX << endl;
			     switch(mode)
		   {
			   
		   case 'r':
				R_V_MIN=V_MIN;
				R_V_MAX=V_MAX;
				break;
		   case 'g':
			   G_V_MIN=V_MIN;
				G_V_MAX=V_MAX;
				break;
		   case 'w':
			   RW_V_MIN=V_MIN;
				RW_V_MAX=V_MAX;
				break;
		   case 'e':
			  GW_V_MIN=V_MIN;
				GW_V_MAX=V_MAX;
				break;

			}
		}

	}

	if (mouseMove == true){
		//if the mouse is held down, we will draw the click and dragged rectangle to the screen
		rectangle(frame, initialClickPoint, cv::Point(currentMousePoint.x, currentMousePoint.y), cv::Scalar(0, 255, 0), 1, 8, 0);
	}


}
string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
}
void drawObject(int x, int y, Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!


	//'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);


	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);



}
int trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed,int ser){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	int largestIndex = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
					//save index of largest contour to use with drawContours
					largestIndex = index;
if(ser==1){
 if(serialcomm){
                                        fprintf(file, "%d", x);

                        fprintf(file, "\n");
                    }
}
 
				}
				else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true){
				putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				drawObject(x, y, cameraFeed);
				return 1;
				//draw largest contour
				//drawContours(cameraFeed, contours, largestIndex, Scalar(0, 255, 255), 2);
			}

		}
		else{ putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
		return 0;
		}
	}
}
int main(int argc, char* argv[])
{
	//some boolean variables for different functionality within this
	//program
	bool trackObjects = true;
	bool useMorphOps = true;
	calibrationMode = true;
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;
	//x and y values for the location of the object
	int x = 0, y = 0;
	//video capture object to acquire webcam feed
	#ifdef __unix__
 signal(SIGINT,quit_signal_handler); // listen for ctrl-C
#endif
    //Matrix to store each frame of the webcam feed
   
RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
	
	config->width=320;
	config->height=240;
      config->framerate=30;
 RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture2(0,config); 
	//must create a window before setting mouse callback
	cv::namedWindow(windowName);
	//set mouse callback function to be active on "Webcam Feed" window
	//we pass the handle to our "frame" matrix so that we can draw a rectangle to it
	//as the user clicks and drags the mouse
	cv::setMouseCallback(windowName, clickAndDrag_Rectangle, &cameraFeed);
	//initiate mouse move and drag to false 
	mouseIsDragging = false;
	mouseMove = false;
	rectangleSelected = false;

	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while (1){
		//store image to matrix
 cameraFeed= raspiCamCvQueryFrame(capture);   
    
if (quit_signal) exit(0);
if(serialcomm){
file = fopen("/dev/ttyACM0", "w");

}		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
		//set HSV values from user selected region
		recordHSV_Values(cameraFeed, HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		
		if(mode=='s')
		{
			if(detect_surface==1)
			{
		 inRange(HSV, Scalar(R_H_MIN, R_S_MIN, R_V_MIN), Scalar(R_H_MAX, R_S_MAX, R_V_MAX), threshold);
		 if (useMorphOps)
			morphOps(threshold);
		  if(trackFilteredObject(x, y, threshold, cameraFeed,0)==1)
		  {
			  surface='r';
			  cout<<"surface ="<<surface;
		  }
			  inRange(HSV, Scalar(G_H_MIN, G_S_MIN, G_V_MIN), Scalar(G_H_MAX, G_S_MAX, G_V_MAX), threshold);
		 if (useMorphOps)
			morphOps(threshold);
		      if(trackFilteredObject(x, y, threshold, cameraFeed,0)==1)
			  {
			  surface='g';
			  cout<<"surface ="<<surface;
			  }
			 
			}
			  switch(surface)
			  {
			  case 'r' : inRange(HSV, Scalar(RW_H_MIN, RW_S_MIN, RW_V_MIN), Scalar(RW_H_MAX, RW_S_MAX, RW_V_MAX), threshold);
		                cout<<"tracking red white surface \n";  
						roileft=threshold(Rect(0,0,100,240));
						cameraleft=cameraFeed(Rect(0,0,100,240));
						      roi1=threshold(Rect(0,100,320,50));
	  roi11=cameraFeed(Rect(0,100,320,50));

						roiright=threshold(Rect(200,0,100,240));
						cameraright=cameraFeed(Rect(200,0,100,240));
						if (useMorphOps){
			            morphOps(threshold);
						morphOps(roileft);
						morphOps(roiright);
						}
						if((trackFilteredObject(x,y,roileft,cameraleft,0)==1)&&(trackFilteredObject(x,y,roiright,cameraright,0)==1))
							cout<<"T turn";

						if(  trackFilteredObject(x,y,roi1,roi11,1)==1)
							detect_surface=0;
						else 
							detect_surface=1;
						break;

				 case 'g' : inRange(HSV, Scalar(GW_H_MIN, GW_S_MIN, GW_V_MIN), Scalar(GW_H_MAX, GW_S_MAX, GW_V_MAX), threshold);
		               cout<<"tracking green white surface \n";  
				    
					 if (useMorphOps)
			            morphOps(threshold);
  roi1=threshold(Rect(0,100,320,50));
	  roi11=cameraFeed(Rect(0,100,320,50));
						if(  trackFilteredObject(x,y,roi1,roi11,1)==1)
							detect_surface=0;
						else 
							detect_surface=1;
						break;
			  
			  }

		}
		else
		{
			inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
			//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
			if (useMorphOps)
			morphOps(threshold);
			//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if (trackObjects)
		{
			
			trackFilteredObject(x, y, threshold, cameraFeed,0);
		   
		}

		}
		
		
		
		
		

		//show frames 
		if (calibrationMode == true){

			//create slider bars for HSV filtering
			createTrackbars();
			imshow(windowName1, HSV);
			imshow(windowName2, threshold);
		}
		else{

			destroyWindow(windowName1);
			destroyWindow(windowName2);
			destroyWindow(trackbarWindowName);
		}
		imshow(windowName, cameraFeed);
		


		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		//also use waitKey command to capture keyboard input
  if(serialcomm)
        {
               fclose(file);

        }		
input=waitKey(30);
	
		switch(input)
		{
		case 99 : calibrationMode = !calibrationMode;
				  break;
		case 114: mode='r';
			 	break;
		case 103: mode='g';
			    break;
		case 119: mode='w'; ///for white red color
			break;
		case 101: mode='e'; //for white green color
			break;
		case 's' : mode ='s';
			  cout<<"s pressed";
			    break;
		case 'x' : cout<<"\n";
			  cout<<"HSV values for red surface \n";
			  cout<<R_H_MIN<<"\t";
			  cout<<R_H_MAX<<"\t";
			  cout<<R_S_MIN<<"\t";
			  cout<<R_S_MAX<<"\t";
			  cout<<R_V_MIN<<"\t";
			  cout<<R_V_MAX<<"\t";
			  break;

		}
	
		
		
	}






	return 0;
}