/**
 *
 * @file Camera.hpp
 * @brief  Header file for Camera class.
 *
 */

#ifndef FRAME_CAMERA_HPP_
#define FRAME_CAMERA_HPP_

/**
 * Camera class: this class is to model a camera
 *
 */
class Camera {

public:
	/**
	 * @brief Camera constructor.
	 *
	 * @param camMatrix: the camera matrix including focal distances and center
	 * @param distorMatrix: the camera distorsion parameters
	 *
	 */
	Camera(cv::Mat camMatrix, cv::Mat distorMatrix):m_camMatrix(camMatrix), m_distorMatrix(distorMatrix){};

	cv::Mat m_camMatrix;			/*!< camera matrix */
	cv::Mat m_distorMatrix;			/*!< camera distorsion matrix */

};

#endif /* FRAME_CAMERA_HPP_ */
