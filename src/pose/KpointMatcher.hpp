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

	static const unsigned int MIN_KPMATCHS_NBR = 35;
	KpointMatcher();
	bool match(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors);

private:
	std::vector< cv::DMatch > m_matches;

};



} /*namespace kpproc*/

#endif /* POSE_KPOINTMATCHER_HPP_ */
