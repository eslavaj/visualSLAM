/*
 * CameraPoseEstimator.cpp
 *
 *  Created on: Jun 27, 2021
 *      Author: jeslava
 */

#include "CameraPoseEstimator.hpp"


#include <string>
#include <iostream>
#include "opencv2/calib3d.hpp"
#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>
#include <opencv2/core/types.hpp>


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
			cv::RANSAC, 0.9, 1.0, // RANSAC method
			inliers);             // extracted inliers


	// recover relative camera pose from essential matrix
	//cv::Mat rotation, translation;
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
		cout << "##################### " << numberOfPtsPose << endl;
		m_frameBuffer.pop_back();
		//cv::waitKey(1000);
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

	#if DEBUG_LATENCY
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "#### CALC POSE DONE IN " << 1000 * t / 1.0 << " ms" << endl;
	#endif


}





void CameraPoseEstimator::visualizeCami(int i, boost::circular_buffer<Frame> & frameBuffer) /* 21 , 20 ****** 19, 18*/
{

	/*Generate random color*/
	srand (i);
	int rcolor = rand() % 200 + 50;
	srand (i+100);
	int gcolor = rand() % 200 + 50;
	srand (i+200);
	int bcolor = rand() % 200 +50;
	cv::viz::Color randomColor(bcolor, gcolor, rcolor);

	cv::Matx33f  A_calib33f(m_camera.m_camMatrix);

	/// Construct the scene
	// Create one virtual camera
	cv::viz::WCameraPosition cam1(A_calib33f,  // matrix of intrinsics
			(frameBuffer.end() - i)->cameraImg,                             // image displayed on the plane
			1.0,                                // scale factor
			cv::viz::Color::black());

	// Create a second virtual camera
	cv::viz::WCameraPosition cam2(A_calib33f,  // matrix of intrinsics
			(frameBuffer.end() - i + 1)->cameraImg,                             // image displayed on the plane
			1.0,                                // scale factor
			//cv::viz::Color::black());
			randomColor);



	// choose one point for visualization
	/*
	cv::Vec3d testPoint = triangulate((frameBuffer.end() - i)->projectionMatrix, (frameBuffer.end() - i + 1)->projectionMatrix,
			(frameBuffer.end() - i + 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR],
			(frameBuffer.end() - i + 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR]);*/

	//cv::viz::WSphere point3D(testPoint, 0.08, 10, randomColor);

	string labeliplus1 = "Image-" + std::to_string( -(-i+1) );


	//string labelTimestampplus1 = "Image-" + std::to_string( -(-i+1) ) + " " + std::to_string( (int)( (frameBuffer.end() - i + 1)->msTimestamp ) );

	cv::viz::WText3D labelCamiplus1(labeliplus1,cv::Point3_<double>(0, 0, 0), 0.03, true, randomColor);// cv::viz::Color::black());

	//cv::viz::WText3D labelTimestampCamiplus1(labelTimestampplus1,cv::Point3_<double>(0, 0, 0), 0.03, true, randomColor);

	// its associated line of projection
	double lenght(10.);
	/*
	cv::viz::WLine line1(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (frameBuffer.end() - i + 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](0) ),
			lenght*( (frameBuffer.end() - i + 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](1) ),lenght),
			cv::viz::Color::black());
	cv::viz::WLine line2(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (frameBuffer.end() - i + 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](0) ),
			lenght*( (frameBuffer.end() - i + 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](1) ), lenght),
					randomColor);*/


	// the reconstructed cloud of 3D points
	//cv::viz::WCloud cloud((frameBuffer.end() - i + 1)->points3D, randomColor);
	//cloud.setRenderingProperty(cv::viz::POINT_SIZE, 3.);

	// Add the virtual objects to the environment
	string cami = "CameraP-" + std::to_string(i);
	string camiplus1 = "CameraC-" + std::to_string(-(-i+1));
	//string cloudi = "CloudC-" + std::to_string(-(-i+1));
	//string linei = "LineP-" + std::to_string(i);
	//string lineiplus1 = "LineC-" + std::to_string(-(-i+1));
	//string triangulatedi = "Triangulated-" + std::to_string(-(-i+1));



	/*m_visualizer.showWidget(cami, cam1);*/
	m_visualizer.showWidget(camiplus1, cam2);



	//m_visualizer.showWidget(cloudi, cloud);
	//m_visualizer.showWidget(linei, line1);
	//m_visualizer.showWidget(lineiplus1, line2);
	//m_visualizer.showWidget(triangulatedi, point3D);

	//m_visualizer.showWidget(labeliplus1, labelCamiplus1);

	//m_visualizer.showWidget(labelTimestampplus1, labelTimestampCamiplus1);

	// Move the second camera
	//cv::Affine3d posePrev((m_dataFrameBuffer.end() - i)->rotationMatrix, (m_dataFrameBuffer.end() - i)->translationVector);

	/*
	cv::Affine3d pose((m_dataFrameBuffer.end() - i + 1)->rotationMatrix, (m_dataFrameBuffer.end() - i + 1)->translationVector);
	cv::Affine3d prevPose = (m_dataFrameBuffer.end() - i)->pose;
	cv::Affine3d newPose = pose.concatenate(prevPose);
	(m_dataFrameBuffer.end() - i + 1)->pose = newPose;*/
	/*cv::Affine3d pose((m_dataFrameBuffer.end() - i + 1)->rotationMatrix, cv::Vec3d(0, double(i)/100000.0, 0 ));*/

	cv::Affine3d newPose = (frameBuffer.end() - i + 1)->pose;

	cv::Affine3d poseTimestamp(cv::Mat::eye(3, 3, CV_64F), cv::Vec3d(10, double(i)/10.0, 10 ));

	m_visualizer.setWidgetPose(camiplus1, newPose);
	//m_visualizer.setWidgetPose(linei, posePrev);
	//m_visualizer.setWidgetPose(lineiplus1, pose);
	m_visualizer.setViewerPose(newPose);
	//m_visualizer.setWidgetPose(labeliplus1, newPose);
	//m_visualizer.setWidgetPose(labelTimestampplus1, poseTimestamp);
}






void CameraPoseEstimator::visualize()
{
	/*just for visualization*/
	cv::Mat translVectorInv(cv::Mat::zeros(3, 1, CV_64F));
	translVectorInv.at<double>(0,0) = (m_frameBuffer.end() - 1)->translationVector.at<double>(0,0)*(1);
	translVectorInv.at<double>(0,1) = (m_frameBuffer.end() - 1)->translationVector.at<double>(0,1)*(-1);
	translVectorInv.at<double>(0,2) = (m_frameBuffer.end() - 1)->translationVector.at<double>(0,2)*(-1);
	cv::Affine3d pose_tmp((m_frameBuffer.end() - 1)->rotationMatrix, translVectorInv);
	(m_frameBuffer.end() - 1)->pose = pose_tmp.concatenate((m_frameBuffer.end() - 2)->pose);

	//std::cout<< (frameBuffer.end() - 2)->pose.matrix<<std::endl;

	//m_visualizer.removeAllWidgets();
	m_visualizer.setBackgroundColor(cv::viz::Color::white());
	int bufferCurrSize = m_frameBuffer.size();
	//for(int i = 21; i>=2; i--)

	static unsigned int i=0;


/*
	for(int i = bufferCurrSize; i>=2; i--)
	{
		visualizeCami(i, frameBuffer);
	}
*/


	cv::Matx33f  A_calib33f(m_camera.m_camMatrix);

	cv::viz::WCameraPosition cam2(A_calib33f,  // matrix of intrinsics
			(m_frameBuffer.end() - 1)->cameraImg,                             // image displayed on the plane
			1.0,                                // scale factor
			//cv::viz::Color::black());
			cv::viz::Color::black());

	string camiplus1 = "CameraC-" + std::to_string(-(-i+1));
	m_visualizer.showWidget(camiplus1, cam2);
	cv::Affine3d newPose = (m_frameBuffer.end() - 1)->pose;

	m_visualizer.setWidgetPose(camiplus1, newPose);
	//m_visualizer.setViewerPose(newPose);
	i++;


	//cv::Affine3d poseViewer((m_dataFrameBuffer.end() - 1)->rotationMatrix, (m_dataFrameBuffer.end() - 1)->translationVector);
	//double rotViewer_elem[9] = { 1, 0, 0, 0, 0, -1, 0, 1, 0 };

	/*
	double rotViewer_elem[9] = {  1.0000000,  0.0000000,  0.0000000,
			0.0000000,  0.0000000,  1.0000000,
			0.0000000, -1.0000000,  0.0000000};
	double rotViewer_elem[9] = {  1.0000000,  0.0000000,  0.0000000,
			0.0000000,  0.0000000,  -1.0000000,
			0.0000000, 1.0000000,  0.0000000};*/

	double rotViewer_elem[9] = {  -1.0000000,  0.0000000,  0.0000000,
			0.0000000,  0.0000000,  -1.0000000,
			0.0000000, -1.0000000,  0.0000000};

	cv::Mat rotViewer = cv::Mat(3, 3, CV_64F, rotViewer_elem);

	//double translViewer_elem[3] = { -15, 45, -10}; //-90
	double translViewer_elem[3] = { -0, 60, 1}; //-90
	cv::Mat translViewer = cv::Mat(3, 1, CV_64F, translViewer_elem);
	double cumulTransl_elem[3] = {(m_frameBuffer.end() - 1)->pose.translation().val[0],
			(m_frameBuffer.end() - 1)->pose.translation().val[1],
			(m_frameBuffer.end() - 1)->pose.translation().val[2]};
	cv::Mat cumulTransl = cv::Mat(3, 1, CV_64F, cumulTransl_elem);
	translViewer = translViewer + cumulTransl;


	//cv::Mat rotViewer_tmp = ((m_dataFrameBuffer.end() - 1)->rotationMatrix)*rotViewer;
	//rotViewer_tmp.copyTo(rotViewer);

	cv::Affine3d poseViewer(rotViewer, translViewer);
	//poseViewer = (m_dataFrameBuffer.end() - 1)->pose.concatenate(poseViewer);
	//poseViewer.rotation(rotViewer);
	m_visualizer.setViewerPose(poseViewer);

	m_visualizer.spinOnce(1,     // pause 1ms
			false); // redraw
}


