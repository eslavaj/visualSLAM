/*
 * KpointProcessor.hpp
 *
 *  Created on: Jun 25, 2021
 *      Author: user3
 */

#ifndef POSE_KPOINTPROCESSOR_HPP_
#define POSE_KPOINTPROCESSOR_HPP_

#include "KpointExtractor.hpp"


class KpointProcessor
{
public:
	KpointProcessor(bool visuEnable=false):m_visuEnable(visuEnable),extractor(visuEnable)
	{};

private:
	bool m_visuEnable;
	KpointExtractor extractor;


};





#endif /* POSE_KPOINTPROCESSOR_HPP_ */
