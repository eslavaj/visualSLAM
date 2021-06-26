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
#include "KeypointProcessorGpu.hpp"
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
	//if( imageFrame.empty() ) return -1; // end of video stream

	namedWindow( "Image input", WINDOW_NORMAL | WINDOW_KEEPRATIO ); // Create a window for display.
	imshow("Image input", imageFrame);
	cout<<"Press any key"<<endl;
	cv::waitKey(1);
	return 0;
}


int main(int argc, char** argv)
{

	int num_args = argc;
	if(num_args!=5)
	{
		cout<<"Incorrect argument number"<<endl<<endl;
		cout<<"Usage: "<<endl;
		cout<<"      feature_processing_gpu <Match selector type> <image_folder>"<<endl;
		cout<<"Detector types: FAST , ORB"<<endl;
		cout<<"Selector types: SEL_NN , SEL_KNN"<<endl;
		cout<<"Match point refining strategy: FUND , HOMOGR, AUTO, NONE"<<endl;
		cout<<"image_folder: the name of your image folder"<<endl;
		return -1;
	}

	string detectorType = argv[1];
	string selectorType = argv[2];
	string mpointStrat = argv[3];
	string img_folder = argv[4];

	/* INIT VARIABLES AND DATA STRUCTURES */
	// data location
	string dataPath = "../";

	// camera
	string imgBasePath = dataPath + "images/";
	string imgPrefix = img_folder + "/"; // left camera, color
	string imgFileType = ".png";
	int imgStartIndex = 1; // first file index to load (assumes Lidar and camera names have identical naming convention)
	int imgEndIndex = 800;   // last file index to load
	int imgFillWidth = 6;  // no. of digits which make up the file index (e.g. img-0001.png)


    int dataBufferSize = 50;       // no. of images which are held in memory (ring buffer) at the same time

    // -------------------
    // Create a viz window
    cv::viz::Viz3d visualizer("Viz window");

    boost::circular_buffer<DataFrame> dataBuffer(dataBufferSize);
    KeypointProcessorGpu pointProcGPU(dataBuffer, detectorType, selectorType, true);
    CameraPoseEstimator camPoseEstimator(dataBuffer, A_calib, D_calib, visualizer);



    //cv::Mat frame, frame_vis;

    //int counter_viz = 0;

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
    	// assemble filenames for current index
    	ostringstream imgNumber;
    	imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex*1;
    	//imgNumber << imgStartIndex + imgIndex;
    	string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
    	cout <<"image full name: "<<imgFullFilename<< endl;

    	cv::Mat img;
    	img = cv::imread(imgFullFilename);
    	showImageRaw(img);

    	pointProcGPU.extractKpointDescriptors(img);
    	pointProcGPU.matchKpoints(mpointStrat);
    	//pointProcGPU.visualize(0);

    	camPoseEstimator.calcCameraPose();

    	//camPoseEstimator.visualize();
    	//camPoseEstimator.visualizeLastN(400);
    	camPoseEstimator.visualizeLastFrames();

    	//usleep(200000);
    	//cv::waitKey(0);

    }


/*
    while(cap.read(frame) && cv::waitKey(30) != 27)    // capture frame until ESC is pressed
    {

    	//if(showCameraRaw(frame) !=0) break;

        frame_vis = frame.clone();                     // refresh visualisation frame
        // MAIN ALGORITHM

    	pointProcGPU.extractKpointDescriptors(frame_vis);
    	pointProcGPU.matchKpoints(mpointStrat);
    	//pointProcGPU.visualize(0);

    	camPoseEstimator.calcCameraPose();
    	if(counter_viz%5000)
    	{
    		//camPoseEstimator.visualize();
    		camPoseEstimator.visualizeLast20();
    	}
    	counter_viz++;

    	usleep(00000);
    	//cv::waitKey(0);

    }
*/
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}
