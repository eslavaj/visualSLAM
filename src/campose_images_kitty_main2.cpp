//============================================================================
// Name        : project_base.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <thread>
#include <mutex>
#include <memory>

#include "calibration_matrix.hpp"
#include "KpointProcessor.hpp"
#include "CameraPoseEstimator.hpp"
#include "FrameProvider.hpp"

using namespace cv;
using namespace std;

/*Global variables */
cv::Mat globalFrame;
mutex mtx;
bool frameAvailable = false;


int showCameraRaw(cv::Mat &camFrame)
{
	if( camFrame.empty() ) return -1; // end of video stream
	namedWindow( "Video input", WINDOW_NORMAL | WINDOW_KEEPRATIO ); // Create a window for display.
	imshow("Video input", camFrame);

	return 0;
}

int showImageRaw(cv::Mat &imageFrame, double fps_processing)
{
	namedWindow( "Image input", WINDOW_NORMAL | WINDOW_KEEPRATIO ); // Create a window for display.

	string fps_processing_text = std::to_string(fps_processing) + " fps";
	cv::putText(imageFrame, //target image
			fps_processing_text, //text
			cv::Point(10, imageFrame.rows / 10), //top-left position
			cv::FONT_HERSHEY_DUPLEX,
			1.0,
			CV_RGB(118, 185, 0), //font color
			2);

	imshow("Image input", imageFrame);
	cv::waitKey(1);
	return 0;
}


int main(int argc, char** argv)
{

	int num_args = argc;
	if(num_args!=2)
	{
		cout<<"image_folder: the name of your image folder"<<endl;
		return -1;
	}

	string imgBasePath = argv[1];

	/* INIT VARIABLES AND DATA STRUCTURES */
	int imgStartIndex = 1; // first file index to load (assumes Lidar and camera names have identical naming convention)
	int imgEndIndex = 443;   // last file index to load
    int frameBufferSize = 50;       // no. of frames which are held in memory (ring buffer) at the same time

    boost::circular_buffer<Frame> dataBuffer(frameBufferSize);

    kpproc::KpointProcessor kpprocessor(dataBuffer);
    Camera camera(A_calib, D_calib);
    campose::CameraPoseEstimator camposeestimator(camera, dataBuffer);

    //FrameProvider * frameProvider = new FrameProvider(mtx, frameAvailable, globalFrame);
    FrameProvider frameProvider(mtx, frameAvailable, globalFrame);
    thread frameProviderThread(&FrameProvider::captureFramesFromFolder, &frameProvider,imgBasePath, imgStartIndex, imgEndIndex);
    frameProviderThread.detach();

    while(cv::waitKey(2) != 27)
    {
    	cv::Mat visuRawFrame;
    	mtx.lock();

    	if(frameAvailable)
    	{
    		globalFrame.copyTo(visuRawFrame);
    		dataBuffer.push_back(Frame(globalFrame));
    	}
    	else
    	{
    		mtx.unlock();
    		continue;
    	}

    	mtx.unlock();

    	/*To measure fps*/
    	double t = (double)cv::getTickCount();
    	double fps_processing;

    	if( kpprocessor.process() )
    	{
    		if( camposeestimator.calcCameraPose( (dataBuffer.end() - 1)->refinedPointsPrev, (dataBuffer.end() - 1)->refinedPointsCurr,
    										 (dataBuffer.end() - 1)->rotationMatrix, (dataBuffer.end() - 1)->translationVector) )
    		{
    			t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    			fps_processing = 1.0/t;
    			camposeestimator.visualize();
    		}
    	}

    	showImageRaw(visuRawFrame, fps_processing);

    }

    return 0;
}
