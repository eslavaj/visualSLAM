/*
 * FrameProvider.hpp
 *
 *  Created on: Jun 28, 2021
 *      Author: user3
 */

/**
 *
 * @file FrameProvider.hpp
 * @brief  Header file for frame provider class.
 *
 */

#ifndef FRAME_FRAMEPROVIDER_HPP_
#define FRAME_FRAMEPROVIDER_HPP_

#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"

#include <thread>
#include <mutex>


/**
 * Frame provider class, this class gets frames and serves them to other threads. It is useful to simulate a frame flow from a set of images
 *
 */
class FrameProvider{
/*TODO: need to implement terminate mechanism*/
public:
	/**
	 * @brief Frame provider constructor.
	 *
	 * @param mtx: mutex to control access to shared resources
	 * @param frameAvail: to tell a new frame is available
	 * @param frame: the shared frame
	 *
	 */
	FrameProvider(std::mutex & mtx, bool & frameAvail, cv::Mat & frame):
				  m_mutex(mtx), m_frameAvailable(frameAvail), m_frame(frame){};

	/**
	 * @brief retrieves images from a folder and serves them as a 30 fps flow.
	 *
	 * @param folderPath: folder containing the set of images
	 * @param startIdx: index of the first image
	 * @param endIdx: image of the last image
	 *
	 */
	void captureFramesFromFolder(const std::string folderPath, size_t startIdx, size_t endIdx);

	/**
	 * @brief retrieves images from an argus camera and serves them as a 30 fps flow - IN PROGRESS.
	 *
	 */
	void captureFramesFromArgusCam();


private:
	std::mutex & m_mutex;									/*!< mutex to control resources access */
	bool & m_frameAvailable;								/*!< to tell a frame is available */
	cv::Mat & m_frame;										/*!< the frame */

};


#endif /* FRAME_FRAMEPROVIDER_HPP_ */
