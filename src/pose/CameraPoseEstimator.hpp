/**
 *
 * @file CameraPoseEstimator.hpp
 * @brief  Header file for camera pose estimator.
 *
 */

#ifndef CAMERAPOSEESTIMATOR_HPP_
#define CAMERAPOSEESTIMATOR_HPP_


#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"

#include <boost/circular_buffer.hpp>
#include <opencv2/viz.hpp>

#include "Camera.hpp"
#include "conditCompOptions.h"
#include "Frame.hpp"

namespace campose
{
/**
 * Camera pose estimator class, it estimates the relative pose between two frames
 *
 */
class CameraPoseEstimator
{
public:

	/**
	 * @brief Camera pose estimator constructor.
	 *
	 * @param camera: camera object \ref Camera
	 * @param frameBuffer: frame ring buffer containing last frames
	 *
	 */
	CameraPoseEstimator(const Camera & camera, boost::circular_buffer<Frame> & frameBuffer):m_camera(camera), m_frameBuffer(frameBuffer){};


	/**
	 * @brief This functions estimates the relative camera pose between two frames
	 *
	 * @param refinedPointsPrev: refined coordinates of previous keypoints
	 * @param refinedPointsPrev: refined coordinates of curent keypoints
	 * @param rotationMatrix: to store the estimated rotation matrix
	 * @param rotationMatrix: to store the estimated translation vector
	 *
	 * @return true on success.
	 */
	bool calcCameraPose(const std::vector<cv::Point2f> & refinedPointsPrev,
						const std::vector<cv::Point2f> & refinedPointsCurr,
						cv::Mat & rotationMatrix,
						cv::Mat & translationVector);

	/**
	 * @brief This functions manages pose visualization in 3D graph
	 *
	 * @return void.
	 */
	void visualize();


	static const unsigned int MIN_POSE_POINTS_NBR = 8;					/*!< Minimal number of inliers points to consider that pose estimation is acceptable */

private:

	boost::circular_buffer<Frame> & m_frameBuffer;						/*!< the frame ring buffer*/
	Camera m_camera;													/*!< the camera*/
	cv::Affine3d m_lastKnowPose;										/*!< to store temporarily the last know pose of the camera*/

	#ifdef ENABLE_VISU
	cv::viz::Viz3d m_visualizer;										/*!< the object to manage pose visualieation*/
	#endif
};


} /*namespace campose*/

//   ./vodo /home/jeslava/msata/cpp_work/img_processing/slamPoC/2011_09_26_2/2011_09_26_drive_0051_extract/image_00/data

#endif /* CAMERAPOSEESTIMATOR_HPP_ */
