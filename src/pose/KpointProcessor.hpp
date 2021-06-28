/**
 *
 * @file KpointProcessor.hpp
 * @brief  Header file for keypoint processor.
 *
 */

#ifndef POSE_KPOINTPROCESSOR_HPP_
#define POSE_KPOINTPROCESSOR_HPP_

#include <boost/circular_buffer.hpp>

#include "KpointExtractor.hpp"
#include "KpointMatcher.hpp"
#include "Frame.hpp"

namespace kpproc
{

/**
 * Keypoint processor class: wrapper class for keypoints detection, description and matching
 *
 */
class KpointProcessor
{
public:

	/**
	 * @brief Keypoint processor constructor.
	 *
	 * @param frameBuffer: ring buffer containing previous and current frames
	 * @param visuEnable: enable/disable keypoints visualization
	 *
	 */
	KpointProcessor(boost::circular_buffer<Frame> & frameBuffer, bool visuEnable=false):
		m_visuEnable(visuEnable),
		m_extractor(visuEnable),
		m_matcher(), m_frameBuffer(frameBuffer){};

	/**
	 * @brief This functions trigger the process of two last frames: keypoints extraction, matching
	 *
	 * @return true on success.
	 */
	bool process();

	KpointExtractor m_extractor;							/*!< The keypoint extractor  */
	KpointMatcher m_matcher;								/*!< The keypoint matcher  */
	boost::circular_buffer<Frame> & m_frameBuffer;			/*!< The frame ring buffer  */

private:
	bool m_visuEnable;												/*!< To store visualization enable or disable status*/

};

} /*namespace kpproc*/


#endif /* POSE_KPOINTPROCESSOR_HPP_ */
