/**
 *
 * @file CameraPoseEstimator.cpp
 * @brief  Implementation file for camera pose estimator.
 *
 */

#include <string>
#include <iostream>
#include "opencv2/calib3d.hpp"
#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>
#include <opencv2/core/types.hpp>

#include "CameraPoseEstimator.hpp"

using namespace campose;
using namespace std;

bool CameraPoseEstimator::calcCameraPose(const vector<cv::Point2f> & refinedPointsPrev,
										 const vector<cv::Point2f> & refinedPointsCurr,
										 cv::Mat & rotationMatrix,
										 cv::Mat & translationVector)
{

	#if DEBUG_LATENCY
	/*To measure time*/
	double t = (double)cv::getTickCount();
	#endif

	cv::Mat inliers;
	cv::Mat essentialMatrix =
			cv::findEssentialMat(refinedPointsPrev, refinedPointsCurr,
			m_camera.m_camMatrix,	          // intrinsic parameters
			cv::RANSAC, 0.99, 1.0, // RANSAC method
			inliers);             // extracted inliers

	// recover relative camera pose from essential matrix
	cv::recoverPose(essentialMatrix,   // the essential matrix
			refinedPointsPrev, refinedPointsCurr,        // the matched keypoints
			m_camera.m_camMatrix,            // matrix of intrinsics
			rotationMatrix, translationVector,   // estimated motion
			//cv::noArray());                // inliers matches
			inliers);

	int numberOfPtsPose = cv::sum(inliers)[0];
	/*Check if camera has moved enough, if not then discard this frame*/
	if(numberOfPtsPose < MIN_POSE_POINTS_NBR)
	{
		/*
		cout << "##################### " << numberOfPtsPose << endl;
		m_frameBuffer.pop_back();
		cv::waitKey(1000);*/
		return false;
	}

	// to contain the inliers
	std::vector<cv::Vec2d> prevInlierPts;
	std::vector<cv::Vec2d> currInlierPts;

	// create inliers input point vector for triangulation
	for (int i = 0; i < inliers.rows; i++) {
		if (inliers.at<uchar>(i)) {
			prevInlierPts.push_back( cv::Vec2d(refinedPointsPrev[i].x, refinedPointsPrev[i].y) );
			currInlierPts.push_back( cv::Vec2d(refinedPointsCurr[i].x, refinedPointsCurr[i].y) );
		}
	}

	// undistort and normalize the image points
	std::vector<cv::Vec2d> points1u;
	cv::undistortPoints(prevInlierPts, points1u, m_camera.m_camMatrix, m_camera.m_distorMatrix);
	std::vector<cv::Vec2d> points2u;
	cv::undistortPoints(currInlierPts, points2u, m_camera.m_camMatrix, m_camera.m_distorMatrix);

	// compose projection matrix from R,T
	cv::Mat projectionM(3, 4, CV_64F);        // the 3x4 projection matrix
	rotationMatrix.copyTo(projectionM(cv::Rect(0, 0, 3, 3)));
	translationVector.copyTo(projectionM.colRange(3, 4));

	cv::Mat translVectorInv(cv::Mat::zeros(3, 1, CV_64F));
	translVectorInv.at<double>(0,0) = (m_frameBuffer.end() - 1)->translationVector.at<double>(0,0)*(1);
	translVectorInv.at<double>(0,1) = (m_frameBuffer.end() - 1)->translationVector.at<double>(0,1)*(-1);
	translVectorInv.at<double>(0,2) = (m_frameBuffer.end() - 1)->translationVector.at<double>(0,2)*(-1);
	cv::Affine3d pose_tmp((m_frameBuffer.end() - 1)->rotationMatrix, translVectorInv);
	(m_frameBuffer.end() - 1)->m_pose = pose_tmp.concatenate(m_lastKnowPose);
	m_lastKnowPose = (m_frameBuffer.end() - 1)->m_pose;

	#if DEBUG_LATENCY
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "#### CALC POSE DONE IN " << 1000 * t / 1.0 << " ms" << endl;
	#endif
}


void CameraPoseEstimator::visualize()
{

	m_visualizer.setBackgroundColor(cv::viz::Color::white());
	int bufferCurrSize = m_frameBuffer.size();

	static unsigned int i=0;

	cv::Matx33f  A_calib33f(m_camera.m_camMatrix);

	cv::viz::WCameraPosition cam2(A_calib33f,  // matrix of intrinsics
			(m_frameBuffer.end() - 1)->cameraImg,                             // image displayed on the plane
			1.0,                                // scale factor
			cv::viz::Color::black());

	string camiplus1 = "CameraC-" + std::to_string(-(-i+1));
	m_visualizer.showWidget(camiplus1, cam2);
	cv::Affine3d newPose = (m_frameBuffer.end() - 1)->m_pose;

	m_visualizer.setWidgetPose(camiplus1, newPose);
	i++;

	double rotViewer_elem[9] = {  -1.0000000,  0.0000000,  0.0000000,
			0.0000000,  0.0000000,  -1.0000000,
			0.0000000, -1.0000000,  0.0000000};

	cv::Mat rotViewer = cv::Mat(3, 3, CV_64F, rotViewer_elem);

	double translViewer_elem[3] = { -0, 60, 1}; //-90
	cv::Mat translViewer = cv::Mat(3, 1, CV_64F, translViewer_elem);
	double cumulTransl_elem[3] = {(m_frameBuffer.end() - 1)->m_pose.translation().val[0],
			(m_frameBuffer.end() - 1)->m_pose.translation().val[1],
			(m_frameBuffer.end() - 1)->m_pose.translation().val[2]};
	cv::Mat cumulTransl = cv::Mat(3, 1, CV_64F, cumulTransl_elem);
	translViewer = translViewer + cumulTransl;

	cv::Affine3d poseViewer(rotViewer, translViewer);
	m_visualizer.setViewerPose(poseViewer);
	m_visualizer.spinOnce(1,     // pause 1ms
						  false); // redraw
}


