/*
 * KpointProcessor.hpp
 *
 *  Created on: Jun 25, 2021
 *      Author: user3
 */

#ifndef POSE_KPOINTPROCESSOR_HPP_
#define POSE_KPOINTPROCESSOR_HPP_

#include "KpointExtractor.hpp"
#include "KpointMatcher.hpp"


namespace kpproc
{

class KpointProcessor
{
public:
	KpointProcessor(bool visuEnable=false):m_visuEnable(visuEnable),m_extractor(visuEnable)
	{};

private:
	bool m_visuEnable;
	KpointExtractor m_extractor;
	KpointMatcher m_matcher;

};




}







#endif /* POSE_KPOINTPROCESSOR_HPP_ */
