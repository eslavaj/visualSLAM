/**
 *
 * @file KpointExtractor.hpp
 * @brief  Header file for lass to extract keypoints.
 *
 */


#ifndef KPOINTEXTRACTOR_HPP_
#define KPOINTEXTRACTOR_HPP_



#include "dataStructures.h"

#include "Frame.hpp"

#include "conditCompOptions.h"



namespace kpproc
{

/**
 * Keypoint extraction class: given an image it extracts the keypoints and provide the corresponding descriptors
 *
 */
class KpointExtractor
{

public:

	/**
	 * @brief Keypoint extractor constructor.
	 *
	 * @param visuEnable: enable/disable keypoints visualization
	 *
	 */
	KpointExtractor(bool visuEnable=false):m_visuEnable(visuEnable){};

	/**
	 * @brief This functions detect keypoints and generate descriptors.
	 *
	 * @param inputImage: The image where the keypoints will be extracted from
	 *
	 * @return true on success.
	 */
	bool extractKpointDescriptors(const cv::Mat & inputImage);

	/**
	 * @brief This function retrieve keypoints and their descriptors.
	 *
	 * @param resKeypoints: where the keypoints will be stored
	 * @param resDescriptors: where the descriptors will be stored
	 *
	 * @return true on success.
	 */
	void getResults(std::vector<cv::KeyPoint> & resKeypoints, cv::Mat & resDescriptors);

	/**
	 * @brief To visualize keypoints - NOT IMPLEMENTED.
	 *
	 * @return void.
	 */
	void visualize(double wait_uSec);


	static const unsigned int MIN_KPOINT_NBR = 35;	/*!< Minimum number of keypoints to consider this frame or image is useful */
    cv::Mat m_frameImg; 							/*!< The image of this frame in cv::Mat format */

private:
	bool m_visuEnable;								/*!< To store visualization enable or disable status*/
    std::vector<cv::KeyPoint> m_keypoints; 			/*!< Keypoints within camera image */
    cv::Mat m_descriptors; 							/*!< keypoint descriptors associated to keypoints listed in \ref m_keypoints */

};


} /*namespace kpproc*/




#endif /* KPOINTEXTRACTOR_HPP_ */
