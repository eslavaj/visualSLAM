/*
 * FrameProvider.cpp
 *
 *  Created on: Jun 28, 2021
 *      Author: user3
 */


#include "FrameProvider.hpp"
#include <unistd.h>


using namespace std;


void FrameProvider::captureFramesFromFolder(const std::string folderPath, size_t startIdx, size_t endIdx)
{

	size_t imgFillWidth = 10;
	string imgFileType = ".png";

	for (size_t imgIndex = 0; imgIndex <= endIdx - startIdx; imgIndex++)
	{
		ostringstream imgNumber;
		imgNumber << setfill('0') << setw(imgFillWidth) << startIdx + imgIndex*1;
		string imgFullFilename = folderPath + "/" + imgNumber.str() + imgFileType;
		//cout <<"image full name: "<<imgFullFilename<< endl;

		cv::Mat img;
		img = cv::imread(imgFullFilename);
        if( img.empty() ){
            //usleep(1000);
            continue;
        }
        m_mutex.lock();
        m_frameAvailable = false;
        img.copyTo(m_frame);
        m_frameAvailable = true;
        m_mutex.unlock();
        usleep(33340); /*30 fps approximately*/
	}

	std::cout << "################## END OF FRAMES ##########################" << std::endl;


}


void FrameProvider::captureFramesFromArgusCam()
{
    cv::VideoCapture *cap;
    cap = new cv::VideoCapture("nvarguscamerasrc ! "
   	        "video/x-raw(memory:NVMM), "
    	        "width=1280, height=720, "
    	        "format=NV12, framerate=20/1 ! "
    	        "nvvidconv flip-method=0 ! "
    	        "video/x-raw, width=640, height=480 ! "
    	        "videoconvert ! "
    	        "video/x-raw, format=BGR ! appsink"
    	        , cv::CAP_GSTREAMER);

    while(true)
    {
        m_mutex.lock();
        m_frameAvailable = false;
        *cap >> m_frame;

        if( m_frame.empty() ){
            //usleep(1000);
            continue;
        }
        //cv::resize(frame, frame, cv::Size(640, 480));
        m_frameAvailable = true;
        m_mutex.unlock();
        usleep(5000);
    }

    cap->release();

}
