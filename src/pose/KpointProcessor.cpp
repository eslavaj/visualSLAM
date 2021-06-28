/**
 *
 * @file KpointProcessor.cpp
 * @brief keypoint processor implementation file.
 *
 */

#include "KpointProcessor.hpp"

using namespace kpproc;


bool KpointProcessor::process()
{

	if( m_extractor.extractKpointDescriptors( (m_frameBuffer.end()-1)->cameraImg ) )
	{
		m_extractor.getResults((m_frameBuffer.end()-1)->keypoints, (m_frameBuffer.end()-1)->descriptors);
	}
	else
	{
		return false;
	}

	/*Wait until have 2 frames to do following processing*/
	if(m_frameBuffer.size()<2)
	{
		return false;
	}

	/*Do matching*/
	if( m_matcher.match( (m_frameBuffer.end()-2)->descriptors,
						 (m_frameBuffer.end()-1)->descriptors,
						 (m_frameBuffer.end()-2)->keypoints,
						 (m_frameBuffer.end()-1)->keypoints) )
	{
		m_matcher.getResults( (m_frameBuffer.end()-1)->matches, (m_frameBuffer.end()-1)->refinedPointsPrev, (m_frameBuffer.end()-1)->refinedPointsCurr );
		return true;
	}
	else
	{
		return false;
	}

}





