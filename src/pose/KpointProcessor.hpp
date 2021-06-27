/*
 * KpointProcessor.hpp
 *
 *  Created on: Jun 25, 2021
 *      Author: user3
 */

#ifndef POSE_KPOINTPROCESSOR_HPP_
#define POSE_KPOINTPROCESSOR_HPP_


#include <boost/circular_buffer.hpp>

#include "KpointExtractor.hpp"
#include "KpointMatcher.hpp"
#include "Frame.hpp"


namespace kpproc
{

class KpointProcessor
{
public:
	KpointProcessor(boost::circular_buffer<Frame> & frameBuffer, bool visuEnable=false):
		m_visuEnable(visuEnable),
		m_extractor(visuEnable),
		m_matcher(), m_frameBuffer(frameBuffer)
	{

	};

	bool process();

	KpointExtractor m_extractor;
	KpointMatcher m_matcher;
	boost::circular_buffer<Frame> & m_frameBuffer;

private:
	bool m_visuEnable;


};




} /*namespace kpproc*/







#endif /* POSE_KPOINTPROCESSOR_HPP_ */
