/*
 * Triangulate.cpp
 *
 *  Created on: Aug 3, 2020
 *      Author: jeslava
 */

#include <string>
#include <iostream>


#include "CameraPoseEstimator.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>
#include <opencv2/core/types.hpp>

//#include "calibration_matrix.hpp"
#include "triangulate.h"

using namespace std;

void CameraPoseEstimator::calcCameraPose()
{
	if(m_dataFrameBuffer.size()>1)
	{
		double t = (double)cv::getTickCount();

		cv::Mat inliers;
		(m_dataFrameBuffer.end() - 1)->essentialMatrix =
				cv::findEssentialMat(
				(m_dataFrameBuffer.end() - 1)->refinedPointsPrev, (m_dataFrameBuffer.end() - 1)->refinedPointsCurr,
				m_A_calib,	          // intrinsic parameters
				cv::RANSAC, 0.9, 1.0, // RANSAC method
				inliers);             // extracted inliers

		int numberOfPts(cv::sum(inliers)[0]);
		std::cout << "#5 : Number of inliers according to Essential matrix: " << numberOfPts << std::endl;
		//std::cout << "Essential matrix: " << (m_dataFrameBuffer.end() - 1)->essentialMatrix << std::endl;

		// recover relative camera pose from essential matrix
		//cv::Mat rotation, translation;
		cv::recoverPose((m_dataFrameBuffer.end() - 1)->essentialMatrix,   // the essential matrix
				(m_dataFrameBuffer.end() - 1)->refinedPointsPrev, (m_dataFrameBuffer.end() - 1)->refinedPointsCurr,        // the matched keypoints
				m_A_calib,            // matrix of intrinsics
				(m_dataFrameBuffer.end() - 1)->rotationMatrix, (m_dataFrameBuffer.end() - 1)->translationVector,   // estimated motion
				//cv::noArray());                // inliers matches
				inliers);


		numberOfPts = cv::sum(inliers)[0];
		std::cout << "#6 : Number of inliers after recovering pose: " << numberOfPts << std::endl;
		cout << "#6 : Pose recovered between prev: "<< (m_dataFrameBuffer.end() - 2)->msTimestamp << " and current: "
		            		<< (m_dataFrameBuffer.end() - 1)->msTimestamp << endl;

		/*Check if camera has moved enough, if not then discard this frame*/
		if(numberOfPts<8)
		{
			m_dataFrameBuffer.pop_back();
			return;
		}

		// to contain the inliers
		std::vector<cv::Vec2d> inlierPts1;
		std::vector<cv::Vec2d> inlierPts2;

		// create inliers input point vector for triangulation
		for (int i = 0; i < inliers.rows; i++) {
			if (inliers.at<uchar>(i)) {
				inlierPts1.push_back(cv::Vec2d((m_dataFrameBuffer.end() - 1)->refinedPointsPrev[i].x, (m_dataFrameBuffer.end() - 1)->refinedPointsPrev[i].y));
				inlierPts2.push_back(cv::Vec2d((m_dataFrameBuffer.end() - 1)->refinedPointsCurr[i].x, (m_dataFrameBuffer.end() - 1)->refinedPointsCurr[i].y));
			}
		}

		// undistort and normalize the image points
		std::vector<cv::Vec2d> points1u;
		cv::undistortPoints(inlierPts1, points1u, m_A_calib, m_D_calib);
		std::vector<cv::Vec2d> points2u;
		cv::undistortPoints(inlierPts2, points2u, m_A_calib, m_D_calib);


		if(ena_transl_factor)
		{
			/*Calculate scale factor for translation*/
			double scaleTraslatFactor;
			vector<double> scaleFact_tmp;
			for(int i = 0; i < inlierPts1.size(); i++)
			{
				for(int j=i; j<inlierPts1.size(); j++)
				{
					double d2 = powf(points2u[i].val[0] - points2u[j].val[0], 2) + powf(points2u[i].val[1] - points2u[j].val[1], 2);
					if( d2 > 0.001)
					{
						double d1 = powf(points1u[i].val[0] - points1u[j].val[0], 2) + powf(points1u[i].val[1] - points1u[j].val[1], 2);
						scaleFact_tmp.push_back(sqrt(d1/d2));
						//cout<<"scaleTraslatFactor: "<< d1/d2 <<endl;
					}
				}
			}

			if(scaleFact_tmp.size()<5)
			{
				cout<<"Frame ignored, not able to compute scale factor";
				m_dataFrameBuffer.pop_back();
				return;
			}
			/*Calculate median*/
			std::sort(scaleFact_tmp.begin(), scaleFact_tmp.end());
			int scaleFact_nbr = scaleFact_tmp.size();
			int medianIdx = (int)(scaleFact_nbr/2);
			if(scaleFact_nbr%2 == 0)
			{
				scaleTraslatFactor = (scaleFact_tmp[ medianIdx ] + scaleFact_tmp[ medianIdx + 1 ])/2;
			}
			else
			{
				scaleTraslatFactor = scaleFact_tmp[ medianIdx + 1];
			}

			(m_dataFrameBuffer.end() - 1)->translationVector = ((m_dataFrameBuffer.end() - 1)->translationVector)*(1/scaleTraslatFactor);
		}


		// compose projection matrix from R,T
		cv::Mat projectionM(3, 4, CV_64F);        // the 3x4 projection matrix
		(m_dataFrameBuffer.end() - 1)->rotationMatrix.copyTo(projectionM(cv::Rect(0, 0, 3, 3)));
		(m_dataFrameBuffer.end() - 1)->translationVector.copyTo(projectionM.colRange(3, 4));

		projectionM.copyTo((m_dataFrameBuffer.end() - 1)->projectionMatrix);

		cout<<"rotationMatrix: " << (m_dataFrameBuffer.end() - 1)->rotationMatrix <<endl;
		cout<<"translationVector: " << (m_dataFrameBuffer.end() - 1)->translationVector <<endl;


		/*If the previous frame does not have projection matrix then assume a generic matrix*/
		/*
		if((m_dataFrameBuffer.end() - 2)->projectionMatrix.cols == 0)
		{
			cv::Mat projection_tmp(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
			cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
			diag.copyTo(projection_tmp(cv::Rect(0, 0, 3, 3)));
			projection_tmp.copyTo((m_dataFrameBuffer.end() - 2)->projectionMatrix);
		}*/


		// triangulation
		std::vector<cv::Vec3d> points3D;
		triangulate((m_dataFrameBuffer.end() - 2)->projectionMatrix, (m_dataFrameBuffer.end() - 1)->projectionMatrix, points1u, points2u, points3D);

		(m_dataFrameBuffer.end() - 1)->undistortedPointsCurr = points2u;
		(m_dataFrameBuffer.end() - 1)->undistortedPointsPrev = points1u;
		(m_dataFrameBuffer.end() - 1)->points3D = points3D;

		/*just for visualization*/
		cv::Mat translVectorInv(cv::Mat::zeros(3, 1, CV_64F));
		translVectorInv.at<double>(0,0) = (m_dataFrameBuffer.end() - 1)->translationVector.at<double>(0,0)*(1);
		translVectorInv.at<double>(0,1) = (m_dataFrameBuffer.end() - 1)->translationVector.at<double>(0,1)*(-1);
		translVectorInv.at<double>(0,2) = (m_dataFrameBuffer.end() - 1)->translationVector.at<double>(0,2)*(-1);
		cv::Affine3d pose_tmp((m_dataFrameBuffer.end() - 1)->rotationMatrix, translVectorInv);
		(m_dataFrameBuffer.end() - 1)->pose = pose_tmp.concatenate((m_dataFrameBuffer.end() - 2)->pose);

		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "#6 : Camera pose estimation done in " << 1000 * t / 1.0 << " ms" << endl;

	}
}



void CameraPoseEstimator::visualize()
{
	if(m_dataFrameBuffer.size()>1)
	{
		// -------------------
		// Create a viz window
		//cv::viz::Viz3d visualizer("Viz window");
		m_visualizer.setBackgroundColor(cv::viz::Color::white());

		/// Construct the scene
		// Create one virtual camera
		cv::viz::WCameraPosition cam1(m_A_calib33f,  // matrix of intrinsics
				(m_dataFrameBuffer.end() - 2)->cameraImg,                             // image displayed on the plane
				1.0,                                // scale factor
				cv::viz::Color::black());

		// Create a second virtual camera
		cv::viz::WCameraPosition cam2(m_A_calib33f,  // matrix of intrinsics
				(m_dataFrameBuffer.end() - 1)->cameraImg,                             // image displayed on the plane
				1.0,                                // scale factor
				cv::viz::Color::black());

		// choose one point for visualization
		cv::Vec3d testPoint = triangulate((m_dataFrameBuffer.end() - 2)->projectionMatrix, (m_dataFrameBuffer.end() - 1)->projectionMatrix,
				(m_dataFrameBuffer.end() - 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR],
				(m_dataFrameBuffer.end() - 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR]);

		cv::viz::WSphere point3D(testPoint, 0.05, 10, cv::viz::Color::red());

		// its associated line of projection
		double lenght(10.);
		cv::viz::WLine line1(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](0) ),
				lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](1) ),lenght),
				cv::viz::Color::green());
		cv::viz::WLine line2(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](0) ),
				lenght*( (m_dataFrameBuffer.end() - 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](1) ), lenght),
				cv::viz::Color::green());

		// the reconstructed cloud of 3D points
		cv::viz::WCloud cloud((m_dataFrameBuffer.end() - 1)->points3D, cv::viz::Color::blue());
		cloud.setRenderingProperty(cv::viz::POINT_SIZE, 3.);

		// Add the virtual objects to the environment
		m_visualizer.showWidget("Camera1", cam1);
		m_visualizer.showWidget("Camera2", cam2);
		m_visualizer.showWidget("Cloud", cloud);
		m_visualizer.showWidget("Line1", line1);
		m_visualizer.showWidget("Line2", line2);
		m_visualizer.showWidget("Triangulated", point3D);

		// Move the second camera
		cv::Affine3d pose((m_dataFrameBuffer.end() - 1)->rotationMatrix, (m_dataFrameBuffer.end() - 1)->translationVector);
		m_visualizer.setWidgetPose("Camera2", pose);
		m_visualizer.setWidgetPose("Line2", pose);


		m_visualizer.spinOnce(5000,     // pause 1ms
							true); // redraw

		//m_visualizer.spin();

	}
}




void CameraPoseEstimator::visualizeCami(int i) /* 21 , 20 ****** 19, 18*/
{

	/*Generate random color*/
	srand (i);
	int rcolor = rand() % 200 + 50;
	srand (i+100);
	int gcolor = rand() % 200 + 50;
	srand (i+200);
	int bcolor = rand() % 200 +50;
	cv::viz::Color randomColor(bcolor, gcolor, rcolor);

	/// Construct the scene
	// Create one virtual camera
	cv::viz::WCameraPosition cam1(m_A_calib33f,  // matrix of intrinsics
			(m_dataFrameBuffer.end() - i)->cameraImg,                             // image displayed on the plane
			1.0,                                // scale factor
			cv::viz::Color::black());

	// Create a second virtual camera
	cv::viz::WCameraPosition cam2(m_A_calib33f,  // matrix of intrinsics
			(m_dataFrameBuffer.end() - i + 1)->cameraImg,                             // image displayed on the plane
			1.0,                                // scale factor
			//cv::viz::Color::black());
			randomColor);

	// choose one point for visualization
	cv::Vec3d testPoint = triangulate((m_dataFrameBuffer.end() - i)->projectionMatrix, (m_dataFrameBuffer.end() - i + 1)->projectionMatrix,
			(m_dataFrameBuffer.end() - i + 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR],
			(m_dataFrameBuffer.end() - i + 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR]);

	cv::viz::WSphere point3D(testPoint, 0.08, 10, randomColor);

	string labeliplus1 = "Image-" + std::to_string( -(-i+1) );

	string labelTimestampplus1 = "Image-" + std::to_string( -(-i+1) ) + " " + std::to_string( (int)( (m_dataFrameBuffer.end() - i + 1)->msTimestamp ) );

	cv::viz::WText3D labelCamiplus1(labeliplus1,cv::Point3_<double>(0, 0, 0), 0.03, true, randomColor);// cv::viz::Color::black());

	cv::viz::WText3D labelTimestampCamiplus1(labelTimestampplus1,cv::Point3_<double>(0, 0, 0), 0.03, true, randomColor);

	// its associated line of projection
	double lenght(10.);
	cv::viz::WLine line1(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (m_dataFrameBuffer.end() - i + 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](0) ),
			lenght*( (m_dataFrameBuffer.end() - i + 1)->undistortedPointsPrev[UNDISTORTED_POINT_NBR](1) ),lenght),
			cv::viz::Color::black());
	cv::viz::WLine line2(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*( (m_dataFrameBuffer.end() - i + 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](0) ),
			lenght*( (m_dataFrameBuffer.end() - i + 1)->undistortedPointsCurr[UNDISTORTED_POINT_NBR](1) ), lenght),
					randomColor);

	// the reconstructed cloud of 3D points
	cv::viz::WCloud cloud((m_dataFrameBuffer.end() - i + 1)->points3D, randomColor);
	cloud.setRenderingProperty(cv::viz::POINT_SIZE, 3.);

	// Add the virtual objects to the environment
	string cami = "CameraP-" + std::to_string(i);
	string camiplus1 = "CameraC-" + std::to_string(-(-i+1));
	string cloudi = "CloudC-" + std::to_string(-(-i+1));
	string linei = "LineP-" + std::to_string(i);
	string lineiplus1 = "LineC-" + std::to_string(-(-i+1));
	string triangulatedi = "Triangulated-" + std::to_string(-(-i+1));

	/*m_visualizer.showWidget(cami, cam1);*/
	m_visualizer.showWidget(camiplus1, cam2);
	//m_visualizer.showWidget(cloudi, cloud);
	//m_visualizer.showWidget(linei, line1);
	//m_visualizer.showWidget(lineiplus1, line2);
	m_visualizer.showWidget(triangulatedi, point3D);
	m_visualizer.showWidget(labeliplus1, labelCamiplus1);

	m_visualizer.showWidget(labelTimestampplus1, labelTimestampCamiplus1);

	// Move the second camera
	//cv::Affine3d posePrev((m_dataFrameBuffer.end() - i)->rotationMatrix, (m_dataFrameBuffer.end() - i)->translationVector);

	/*
	cv::Affine3d pose((m_dataFrameBuffer.end() - i + 1)->rotationMatrix, (m_dataFrameBuffer.end() - i + 1)->translationVector);
	cv::Affine3d prevPose = (m_dataFrameBuffer.end() - i)->pose;
	cv::Affine3d newPose = pose.concatenate(prevPose);
	(m_dataFrameBuffer.end() - i + 1)->pose = newPose;*/
	/*cv::Affine3d pose((m_dataFrameBuffer.end() - i + 1)->rotationMatrix, cv::Vec3d(0, double(i)/100000.0, 0 ));*/
	cv::Affine3d newPose = (m_dataFrameBuffer.end() - i + 1)->pose;
	cv::Affine3d poseTimestamp(cv::Mat::eye(3, 3, CV_64F), cv::Vec3d(10, double(i)/10.0, 10 ));
	m_visualizer.setWidgetPose(camiplus1, newPose);
	//m_visualizer.setWidgetPose(linei, posePrev);
	//m_visualizer.setWidgetPose(lineiplus1, pose);

	m_visualizer.setViewerPose(newPose);
	m_visualizer.setWidgetPose(labeliplus1, newPose);
	m_visualizer.setWidgetPose(labelTimestampplus1, poseTimestamp);
}



void CameraPoseEstimator::visualizeLastN(int n)
{
	if(m_dataFrameBuffer.size()>=n)
	{
		m_visualizer.setBackgroundColor(cv::viz::Color::white());

		//for(int i = 21; i>=2; i--)
		for(int i = n; i>=2; i--)
		{
			visualizeCami(i);
		}


		cv::Affine3d poseViewer((m_dataFrameBuffer.end() - 1)->rotationMatrix, (m_dataFrameBuffer.end() - 1)->translationVector);
		//m_visualizer.setViewerPose((m_dataFrameBuffer.end() - 1)->pose);

		m_visualizer.spinOnce(150000,     // pause 1ms
							true); // redraw

		//m_visualizer.spin();

	}
}


void CameraPoseEstimator::visualizeLastFrames()
{
	int bufferCurrSize = m_dataFrameBuffer.size();

	if(bufferCurrSize>=3)
	{

		m_visualizer.removeAllWidgets();
		//for(int i = 21; i>=2; i--)
		for(int i = bufferCurrSize; i>=2; i--)
		{
			visualizeCami(i);
		}

		//cv::Affine3d poseViewer((m_dataFrameBuffer.end() - 1)->rotationMatrix, (m_dataFrameBuffer.end() - 1)->translationVector);
		//double rotViewer_elem[9] = { 1, 0, 0, 0, 0, -1, 0, 1, 0 };
		double rotViewer_elem[9] = {  1.0000000,  0.0000000,  0.0000000,
		   0.0000000,  0.0000000,  1.0000000,
		   0.0000000, -1.0000000,  0.0000000};
		cv::Mat rotViewer = cv::Mat(3, 3, CV_64F, rotViewer_elem);

		double translViewer_elem[3] = { -15, -90, -10}; //-90
		cv::Mat translViewer = cv::Mat(3, 1, CV_64F, translViewer_elem);
		double cumulTransl_elem[3] = {(m_dataFrameBuffer.end() - 1)->pose.translation().val[0],
								   (m_dataFrameBuffer.end() - 1)->pose.translation().val[1],
								   (m_dataFrameBuffer.end() - 1)->pose.translation().val[2]};
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

		//m_visualizer.spin();

	}
}



