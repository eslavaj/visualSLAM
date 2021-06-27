/*
 * Camera.hpp
 *
 *  Created on: Jun 27, 2021
 *      Author: jeslava
 */

#ifndef FRAME_CAMERA_HPP_
#define FRAME_CAMERA_HPP_


class Camera {

public:
	Camera(cv::Mat camMatrix, cv::Mat distorMatrix):m_camMatrix(camMatrix), m_distorMatrix(distorMatrix){};

	cv::Mat m_camMatrix;
	cv::Mat m_distorMatrix;

};





#endif /* FRAME_CAMERA_HPP_ */
