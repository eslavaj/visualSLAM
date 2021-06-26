/*
 * KeypointProcessorGpu.hpp
 *
 *  Created on: Jul 29, 2020
 *      Author: jeslava
 */

#ifndef KPOINTEXTRACTOR_HPP_
#define KPOINTEXTRACTOR_HPP_

#include <string>

#include <boost/circular_buffer.hpp>

#include <opencv2/core/cuda.hpp>

#include "dataStructures.h"

#include "Frame.hpp"

#include "conditCompOptions.h"


class KpointExtractor
{


public:
	KpointExtractor(bool visuEnable=false):m_visuEnable(visuEnable){};

	bool extractKpointDescriptors(const cv::Mat & inputImage);
	void getResults(std::vector<cv::KeyPoint> & resKeypoints, cv::Mat & resDescriptors, cv::Mat & resImage);

	/*
	void matchKpoints(std::string mpointStrategy="FUND");
	RefineReturnCode::RefineReturnCode refineMatches(const std::vector<cv::DMatch>& matches,
		                 std::vector<cv::KeyPoint>& keypoints1,
						 std::vector<cv::KeyPoint>& keypoints2,
					     std::vector<cv::DMatch>& outMatches,
						 std::string matchRefineStrategy);*/

	void visualize(double wait_uSec);


private:
	bool m_visuEnable;
	//boost::circular_buffer<DataFrame> dataBuffer(dataBufferSize);
    std::vector<cv::KeyPoint> m_keypoints; // 2D keypoints within camera image
    cv::Mat m_descriptors; // keypoint descriptors
    cv::Mat m_frameImg;



#if 0
	cv::Mat m_fundMatrix;
	cv::Mat m_homographyMatrix;
	double m_distToEpipLine = 1.0;
	double m_ransacConfid = 0.9;
	bool m_refineFund = true; /*Refine fundamental matrix*/
	bool m_refineMatches = true; /*Refine the matches*/
	bool m_calcRelVertDisp = false; /*If true then algorithm will calculate relative vertical displacement assuming
	 	 	 	 	 	 	 	 	 that camera orientation is constant between 2 consecutive images.
	 	 	 	 	 	 	 	 	 This function is useful only for tests under specific conditions (constant camera orientation)*/

	FrameSTS::FrameSTS previousFrameSts = FrameSTS::NOT_YET_PROCESSED;
#endif

};






#endif /* KPOINTEXTRACTOR_HPP_ */
