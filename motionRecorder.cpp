//============================================================================
// Name        : motionRecorder.cpp
// Author      : Noah Brewer
// Version     : 0.1.0
// Copyright   : Your copyright notice
// Description : A simple motion recorder using C++ and opencv
//============================================================================

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <ctime>
#include <condition_variable>
#include <csignal>

using namespace std;
using namespace cv;

// Configuration
string filename = "motionOutput.avi";
int max_BINARY_value = 255;
int threshold_value = 15;
int threshold_slider = 50;
int threshold_slider_max = 255;
bool configureParameters = false;
int frameCaptures = 30;
Size ksize;
int kslider = 25;
bool running = true;

VideoCapture cap;
vector<cv::Mat> videoCaptureBuffer;

std::mutex mtx1, mtx2;
unique_lock<std::mutex> lck(mtx1), lck2(mtx2);
std::condition_variable condvar;

// Initialize video writer
cv::VideoWriter writer;
int codec = cv::VideoWriter::fourcc('M', 'J','P','G');

static void ksize_trackbar( int, void* )
{
	//kernel must be odd
	if (kslider % 2 == 1){
		ksize.height = kslider;
		ksize.width = kslider;
	}
}

static void threshold_trackbar( int, void* )
{
}

void getTimeDate(string& str){
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
	str = oss.str();
}

void writeDateTimeToCapture(Mat& capture){
	cv::Point point(10, 40);
	cv::Scalar color(255,255,255);
	int thickness = 1;
	int lineType = 8;
	string str;
	getTimeDate(str);

	cv::putText(capture, str , point,  FONT_HERSHEY_PLAIN, 1.0, color, thickness, lineType);
}

void clearVideoCaptureBuffer(int i){
	lck2.lock();
	videoCaptureBuffer.erase(videoCaptureBuffer.begin(),videoCaptureBuffer.begin()+i);
	lck2.unlock();
}

void writeCaptureToFile(){
	while( running ){
		condvar.wait(lck);
		int i = 0;
		for (; i < frameCaptures && i < (int)videoCaptureBuffer.size(); i++){
			writer.write(videoCaptureBuffer[i]);
		}
		clearVideoCaptureBuffer(i);
	}
}

//Store multiple modified frames and find the difference between them
void modifyVideo(){
    cv::Mat prevGrey, cleanImg;
    vector<cv::Mat> modFrames(4);

    ksize.height = kslider;
    ksize.width = kslider;

    const int k_slider_max = 100;

    if (configureParameters){
		namedWindow("configure", WINDOW_AUTOSIZE); // Create Window
		createTrackbar( "kernel size (smoothing)", "configure", &kslider, k_slider_max, ksize_trackbar );
		createTrackbar( "threshold_slider", "configure", &threshold_value, threshold_slider_max, threshold_trackbar );
    }

	// wait for a new frame from camera and store it into 'frame'
	cap.read(modFrames[0]);
	// check if we succeeded
	if (modFrames[0].empty()) {
		cerr << "ERROR! blank frame grabbed\n";
		return;
	}
	writer.open(filename, codec , 25.0 ,modFrames[0].size(), true);

	cvtColor(modFrames[0], modFrames[1], COLOR_BGR2GRAY, 0);
	//index 2 is the old frame
	GaussianBlur(modFrames[1], prevGrey, ksize, 0);
	int capsAfterMov = 0;
	bool afterCap = false;
    for (;;)
	{
		// wait for a new frame from camera and store it into 'frame'
		cap.read(modFrames[0]);
		cleanImg = modFrames[0].clone();
		// check if we succeeded
		if (modFrames[0].empty()) {
			cerr << "ERROR! blank frame grabbed\n";
			break;
		}
		cvtColor(modFrames[0], modFrames[1], COLOR_BGR2GRAY, 0);
		GaussianBlur(modFrames[1], modFrames[0], ksize, 0);
		imshow("Blurred image", modFrames[0]);
		modFrames[3] = modFrames[0].clone();
		absdiff(modFrames[0], prevGrey, modFrames[1]);

		if (configureParameters)
			imshow("diff between grey images", modFrames[1]);

		threshold(modFrames[1], modFrames[0],threshold_value, max_BINARY_value, THRESH_BINARY);
		prevGrey = modFrames[3].clone();
		if (configureParameters)
			imshow("diff and threshold image", modFrames[0]);

		//if movement and trigger less than 10, we want all movement, plus 10 lagging non movement
		if ( !(countNonZero(modFrames[0]) < 1) ){
			afterCap = true;
			//cout << "Movement detected" <<  endl;
			writeDateTimeToCapture(cleanImg);
			lck2.lock();
			videoCaptureBuffer.push_back(cleanImg);
			lck2.unlock();

			if ((int)videoCaptureBuffer.size() > frameCaptures){
				condvar.notify_one(); //signal we are ready to write
			}
			capsAfterMov = 0;
		}
		else if( afterCap ){
			capsAfterMov += 1;
			//cout << "after cap: capsAfterMov = " << capsAfterMov <<  endl;
			writeDateTimeToCapture(cleanImg);
			lck2.lock();
			videoCaptureBuffer.push_back(cleanImg);
			lck2.unlock();
			//signal to thread to write
			if (capsAfterMov >= frameCaptures){
				condvar.notify_one(); //signal we are ready to write
				afterCap = false;
			}
		}

		if (configureParameters){
			if (cv::waitKey(5) >= 0)
				break;
		}
	}
    running = false;
    condvar.notify_one();
}

//TODO: Read config file each time
//TODO: If settings change while running write to file
int main()
{
	char buff[FILENAME_MAX];
	string timedate, filepath;

	filepath = getcwd(buff, FILENAME_MAX);
	getTimeDate(timedate);
	filename.insert(0,"_");
	filename.insert(0,timedate);
	filename.insert(0,"/");
	filename.insert(0,filepath);
	//cout << filename << endl;

    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID + apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    lck2.unlock();
    std::thread writingThread (writeCaptureToFile);
    modifyVideo();
    writingThread.join(); //Not sure if needed
    cout << "Exit success" << endl;
    return 0;
}
