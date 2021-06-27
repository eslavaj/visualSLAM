/*
 * KpointMatcher.hpp
 *
 *  Created on: Jun 26, 2021
 *      Author: jeslava
 */

#ifndef POSE_KPOINTMATCHER_HPP_
#define POSE_KPOINTMATCHER_HPP_


#include <opencv2/core/cuda.hpp>

#include "dataStructures.h"

#include "Frame.hpp"

#include "conditCompOptions.h"


namespace kpproc
{

class KpointMatcher
{

public:

	static const unsigned int MIN_KPMATCHS_NBR = 32;
	static const unsigned int MIN_HOMOGR_INLIERS_NBR = 16;
	static constexpr double DIST_TO_EPILINE = 1.0;

	KpointMatcher(){};
	bool match(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors,
			   const std::vector<cv::KeyPoint>& queryKeypoints,
			   const std::vector<cv::KeyPoint>& trainKeypoints);

	void getResults(std::vector< cv::DMatch > & matches,
					std::vector<cv::Point2f> & refinedPointsPrev,
					std::vector<cv::Point2f> & refinedPointsCurr);

	std::vector< cv::DMatch > m_matches;
	std::vector<cv::Point2f> m_refinedPointsCurr;
	std::vector<cv::Point2f> m_refinedPointsPrev;

private:
	bool refineMatches(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors,
					   const std::vector<cv::KeyPoint>& queryKeypoints,
					   const std::vector<cv::KeyPoint>& trainKeypoints,
					   std::vector<cv::DMatch>& refinedMatches);

};



} /*namespace kpproc*/

#endif /* POSE_KPOINTMATCHER_HPP_ */
