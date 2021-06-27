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

#include "calibration_matrix.hpp"
#include "KpointProcessor.hpp"
#include "CameraPoseEstimator.hpp"

using namespace cv;
using namespace std;

int showCameraRaw(cv::Mat &camFrame)
{
	if( camFrame.empty() ) return -1; // end of video stream
	namedWindow( "Video input", WINDOW_NORMAL | WINDOW_KEEPRATIO ); // Create a window for display.
	imshow("Video input", camFrame);

	return 0;
}

int showImageRaw(cv::Mat &imageFrame)
{
	namedWindow( "Image input", WINDOW_NORMAL | WINDOW_KEEPRATIO ); // Create a window for display.
	imshow("Image input", imageFrame);
	cout<<"Press any key"<<endl;
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
	string imgFileType = ".png";
	int imgStartIndex = 1; // first file index to load (assumes Lidar and camera names have identical naming convention)
	int imgEndIndex = 130;   // last file index to load
	int imgFillWidth = 10;  // no. of digits which make up the file index (e.g. img-0001.png)
    int frameBufferSize = 10;       // no. of images which are held in memory (ring buffer) at the same time

    boost::circular_buffer<Frame> dataBuffer(frameBufferSize);

    kpproc::KpointProcessor kpprocessor(dataBuffer);
    Camera camera(A_calib, D_calib);
    campose::CameraPoseEstimator camposeestimator(camera, dataBuffer);

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
    	// assemble filenames for current index
    	ostringstream imgNumber;
    	imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex*1;
    	string imgFullFilename = imgBasePath + "/" + imgNumber.str() + imgFileType;
    	//cout <<"image full name: "<<imgFullFilename<< endl;

    	cv::Mat img;
    	img = cv::imread(imgFullFilename);
    	showImageRaw(img);

    	dataBuffer.push_back(Frame(img));

    	if( kpprocessor.process() )
    	{
    		if( camposeestimator.calcCameraPose( (dataBuffer.end() - 1)->refinedPointsPrev, (dataBuffer.end() - 1)->refinedPointsCurr,
    										 (dataBuffer.end() - 1)->rotationMatrix, (dataBuffer.end() - 1)->translationVector) )
    		{
    			camposeestimator.visualize();
    		}
    	}
    }

    return 0;
}
