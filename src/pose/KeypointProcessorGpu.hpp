/*
 * KeypointProcessorGpu.hpp
 *
 *  Created on: Jul 29, 2020
 *      Author: jeslava
 */

#ifndef KEYPOINTPROCESSORGPU_HPP_
#define KEYPOINTPROCESSORGPU_HPP_

#include <string>

#include <boost/circular_buffer.hpp>

#include <opencv2/core/cuda.hpp>

#include "dataStructures.h"



namespace RefineReturnCode
{
enum RefineReturnCode
{
	OK = 0,
	NOT_ENOUGH_INLIERS = -1,

};
}



namespace ExtractReturnCode
{
enum ExtractReturnCode
{
	OK = 0,
	NOT_ENOUGH_KEYPOINTS = -1,

};
}

namespace FrameSTS
{
enum FrameSTS
{
	DISCARDED = -1,
	NOT_YET_PROCESSED = 0,
	PROCESSED = 1
};
}








class KeypointProcessorGpu
{


public:
	KeypointProcessorGpu(boost::circular_buffer<DataFrame> &dataFrameBuffer, std::string detectorType = "ORB", std::string selectorType="SEL_KNN", bool useGPUs=false, bool visuEnable=false):
						m_dataFrameBuffer(dataFrameBuffer),
						m_detectorType(detectorType),
						m_selectorType(selectorType),
						m_visuEnable(visuEnable),
						m_useGPUs(useGPUs)
						{};

	//virtual ~KeypointProcessorGpu();

	ExtractReturnCode::ExtractReturnCode extractKpointDescriptors(cv::Mat & newImage);
	void matchKpoints(std::string mpointStrategy="FUND");
	RefineReturnCode::RefineReturnCode refineMatches(const std::vector<cv::DMatch>& matches,
		                 std::vector<cv::KeyPoint>& keypoints1,
						 std::vector<cv::KeyPoint>& keypoints2,
					     std::vector<cv::DMatch>& outMatches,
						 std::string matchRefineStrategy);
	void visualize(double wait_uSec);


private:
	boost::circular_buffer<DataFrame> & m_dataFrameBuffer;
	std::string m_detectorType;
	std::string m_selectorType;
	bool m_visuEnable;
	bool m_useGPUs;
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




};






#endif /* KEYPOINTPROCESSORGPU_HPP_ */
