/*
 * CameraPose.hpp
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

#include "dataStructures.h"



#define UNDISTORTED_POINT_NBR (0)

class CameraPoseEstimator
{
public:
	CameraPoseEstimator(boost::circular_buffer<DataFrame> &dataFrameBuffer, cv::Mat cameraIntriParam, cv::Mat cameraDistortParam, cv::viz::Viz3d & visualizer):m_dataFrameBuffer(dataFrameBuffer),
	m_A_calib(cameraIntriParam),
	m_A_calib33f(m_A_calib),
	m_D_calib(cameraDistortParam),
	m_visualizer(visualizer){ m_visualizer.setBackgroundColor(cv::viz::Color::white());};

	void calcCameraPose();
	void visualize();
	void visualizeCami(int i);
	void visualizeLastN(int n);
	void visualizeLastFrames();


private:
	//cv::Mat m_essentialMatrix;
	cv::Mat m_A_calib;
	cv::Mat m_D_calib;
	cv::Matx33f m_A_calib33f;
	boost::circular_buffer<DataFrame> & m_dataFrameBuffer;
	cv::viz::Viz3d m_visualizer;
	bool ena_transl_factor=false;







};





#endif /* CAMERAPOSEESTIMATOR_HPP_ */
