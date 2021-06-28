/**
 *
 * @file Frame.hpp
 * @brief  Header file for Frame class.
 *
 */

#ifndef FRAME_FRAME_HPP_
#define FRAME_FRAME_HPP_

#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"

#include <boost/serialization/access.hpp>
#include "conditCompOptions.h"

/**
 * Frame class: this class is used to store frame attributes and frame processing results
 *
 */
class Frame
{
public:

	/**
	 * @brief Frame constructor.
	 *
	 * @param inputImage: the image of this frame
	 *
	 */
	Frame(cv::Mat inputImage){
		inputImage.copyTo(cameraImg);
		cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
		//diag.at<double>(2,2) = -1;
		diag.copyTo(rotationMatrix);
		cv::Mat orig(cv::Mat::zeros(3, 1, CV_64F));
		orig.copyTo(translationVector);
		m_pose = cv::Affine3d(rotationMatrix, translationVector);
	};

    cv::Mat cameraImg;  										/*!< camera image */
    double timestamp;											/*!< timestamp NOT USED YET*/
    std::vector<cv::KeyPoint> keypoints; 						/*!< 2D keypoints within camera image */
    std::vector< cv::DMatch > matches;							/*!< matches between this frame and the previous one */
    std::vector<cv::Point2f> refinedPointsCurr; 				/*!< refined points of this frame after refining according matching */
    cv::Mat descriptors; 										/*!<  keypoint descriptors */
    std::vector<cv::Point2f> refinedPointsPrev; 				/*!< refined points of the previous frame corresponding to refined points of this frame */
    std::vector<cv::Vec2d> undistortedPointsCurr; 				/*!< Undistorted points on current frame*/
    std::vector<cv::Vec2d> undistortedPointsPrev; 				/*!< Undistorted points on previous frame */
    std::vector<cv::Vec3d> points3D; 							/*!< 3D points obtained after triangulation using preious and current frame */


    cv::Mat essentialMatrix; 									/*!< Essential matrix relative to the previous frame */
    cv::Mat rotationMatrix; 									/*!< Rotation matrix relative to the previous frame */
    cv::Mat translationVector; 									/*!< Translation vector relative to the previous frame */
    cv::Mat projectionMatrix; 									/*!< Projection matrix that includes rotation and translation information */

    /*just for visualization*/
	#ifdef ENABLE_VISU
    cv::Affine3d m_pose;										/*!< camera relative pose to previous frame */
	#endif


    friend class boost::serialization::access;
    template<class Archive>
	/**
	 * @brief Function to provide serializing capabilities.
	 *
	 */
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & cameraImg;
        ar & timestamp;

        ar & keypoints;
        ar & refinedPointsCurr;
        ar & refinedPointsPrev;
        ar & undistortedPointsCurr;
        ar & undistortedPointsPrev;
        ar & points3D;

        ar & essentialMatrix;
        ar & rotationMatrix;
        ar & translationVector;
        ar & projectionMatrix;
    }

};

#endif /* FRAME_FRAME_HPP_ */
