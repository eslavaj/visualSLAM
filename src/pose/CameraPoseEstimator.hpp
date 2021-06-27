/*
 * CameraPoseEstimator.hpp
 *
 *  Created on: Aug 3, 2020
 *      Author: jeslava
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



class CameraPoseEstimator
{
public:

	static const unsigned int MIN_POSE_POINTS_NBR = 8;

	/*
	CameraPoseEstimator(boost::circular_buffer<DataFrame> &dataFrameBuffer, cv::Mat cameraIntriParam, cv::Mat cameraDistortParam, cv::viz::Viz3d & visualizer):m_dataFrameBuffer(dataFrameBuffer),
	m_A_calib(cameraIntriParam),
	m_A_calib33f(m_A_calib),
	m_D_calib(cameraDistortParam),
	m_visualizer(visualizer){ m_visualizer.setBackgroundColor(cv::viz::Color::white());};*/

	CameraPoseEstimator(const Camera & camera, boost::circular_buffer<Frame> & frameBuffer):m_camera(camera), m_frameBuffer(frameBuffer){};


	bool calcCameraPose(const std::vector<cv::Point2f> & refinedPointsPrev,
						const std::vector<cv::Point2f> & refinedPointsCurr,
						cv::Mat & rotationMatrix,
						cv::Mat & translationVector);



	void visualize();

	/*
	void visualizeCami(int i);
	void visualizeLastN(int n);
	void visualizeLastFrames();*/


private:
	//cv::Mat m_essentialMatrix;
	/*
	cv::Mat m_A_calib;
	cv::Mat m_D_calib;
	cv::Matx33f m_A_calib33f;
	boost::circular_buffer<DataFrame> & m_dataFrameBuffer;
	cv::viz::Viz3d m_visualizer;
	bool ena_transl_factor=false;*/

	boost::circular_buffer<Frame> & m_frameBuffer;

	Camera m_camera;

	cv::Affine3d m_lastKnowPose;

	#ifdef ENABLE_VISU
	cv::viz::Viz3d m_visualizer;
	#endif
};


} /*namespace campose*/



//   ./slamPoc /home/jeslava/msata/cpp_work/img_processing/slamPoC/2011_09_26_2/2011_09_26_drive_0051_extract/image_00/data



#endif /* CAMERAPOSEESTIMATOR_HPP_ */
