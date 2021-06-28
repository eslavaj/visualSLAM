/**
 *
 * @file KpointMatcher.hpp
 * @brief  Header file for class to match keypoints.
 *
 */



#ifndef POSE_KPOINTMATCHER_HPP_
#define POSE_KPOINTMATCHER_HPP_


#include "Frame.hpp"
#include "conditCompOptions.h"


namespace kpproc
{

/**
 * Keypoint matcher class: looks corresponding keypoints in two images
 *
 */
class KpointMatcher
{

public:

	/**
	 * @brief Keypoint matcher constructor.
	 *
	 */
	KpointMatcher(){};

	/**
	 * @brief This functions match the keypoints according their descriptors.
	 * Note this function matches considering descriptors first and then use homography to refine matches see \ref refineMatches
	 *
	 * @param queryDescriptors: the keypoints descriptors of the "previous" image
	 * @param trainDescriptors: the keypoints descriptors of the "current" image
	 * @param queryKeypoints: the keypoints of the "previous" image
	 * @param trainKeypoints: the keypoints of the "current" image
	 *
	 * @return true on success.
	 */
	bool match(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors,
			   const std::vector<cv::KeyPoint>& queryKeypoints,
			   const std::vector<cv::KeyPoint>& trainKeypoints);


	/**
	 * @brief This functions retrieve the matches and the refined coordinates of the keypoints.
	 *
	 * @param matches: to store the matches
	 * @param refinedPointsPrev: the refined coordinates for the keypoints of the previous image
	 * @param refinedPointsCurr: the refined coordinates for the keypoints of the current image
	 *
	 * @return void.
	 */
	void getResults(std::vector< cv::DMatch > & matches,
					std::vector<cv::Point2f> & refinedPointsPrev,
					std::vector<cv::Point2f> & refinedPointsCurr);


	static const unsigned int MIN_KPMATCHS_NBR = 16; 				/*!< Minimum number of keypoints matches to consider this frame or image is useful */
	static const unsigned int MIN_HOMOGR_INLIERS_NBR = 8;			/*!< Minimum number of inliers according to the selected homography to consider this frame or image is useful */
	static constexpr double DIST_TO_EPILINE = 3.0;					/*!< Maximal allowed distance to epipolar line to consider the point is in the epi. line  */


private:

	/**
	 * @brief This functions refines matches by finding the most probable Homography (RANSAC).
	 * Only the matches that are compliant with this Homography are kept.
	 *
	 * @param queryDescriptors: the keypoints descriptors of the "previous" image
	 * @param trainDescriptors: the keypoints descriptors of the "current" image
	 * @param queryKeypoints: the keypoints of the "previous" image
	 * @param trainKeypoints: the keypoints of the "current" image
	 * @param refinedMatches: to store the the refined matches
	 *
	 * @return true on success.
	 */
	bool refineMatches(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors,
					   const std::vector<cv::KeyPoint>& queryKeypoints,
					   const std::vector<cv::KeyPoint>& trainKeypoints,
					   std::vector<cv::DMatch>& refinedMatches);
	std::vector< cv::DMatch > m_matches;							/*!< The keypoint matches  */
	std::vector<cv::Point2f> m_refinedPointsCurr;					/*!< The refined coordinates for the keypoints of the previous image  */
	std::vector<cv::Point2f> m_refinedPointsPrev;					/*!< The refined coordinates for the keypoints of the current image  */

};



} /*namespace kpproc*/

#endif /* POSE_KPOINTMATCHER_HPP_ */
