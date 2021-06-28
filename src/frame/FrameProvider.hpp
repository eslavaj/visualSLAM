/*
 * FrameProvider.hpp
 *
 *  Created on: Jun 28, 2021
 *      Author: user3
 */

#ifndef FRAME_FRAMEPROVIDER_HPP_
#define FRAME_FRAMEPROVIDER_HPP_

#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"

#include <thread>
#include <mutex>



class FrameProvider{
/*TODO: need to implement end mechanism*/
public:
	FrameProvider(std::mutex & mtx, bool & frameAvail, cv::Mat & frame):
				  m_mutex(mtx), m_frameAvailable(frameAvail), m_frame(frame){};

	void captureFramesFromFolder(const std::string folderPath, size_t startIdx, size_t endIdx);
	void captureFramesFromArgusCam();


private:
	std::thread * m_captureFrameThread;
	std::mutex & m_mutex;
	bool & m_frameAvailable;
	cv::Mat & m_frame;

};


#endif /* FRAME_FRAMEPROVIDER_HPP_ */
