#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core/cuda.hpp>

//#include "gtsam/geometry/Pose3.h"

struct DataFrame { // represents the available sensor information at the same time instance
    

	DataFrame()
	{
		cv::Mat projection1(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
		cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
		diag.at<double>(2,2) = -1;
		//float diag_elem[9] = {1, 0, 0, 0, 1, 0 , 0, 0, 1 };
		//cv::Mat diag = cv::Mat(3, 3, CV_64F, diag_elem);
		diag.copyTo(rotationMatrix);
		cv::Mat orig(cv::Mat::zeros(3, 1, CV_64F));
		orig.copyTo(translationVector);
		diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));
		projectionMatrix = projection1.clone();
		pose = cv::Affine3d(rotationMatrix, translationVector);

		/*
		gtsam::Rot3 R(rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2),
				rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2),
				rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2));
		gtsam::Point3 t(translationVector.at<double>(0,0),translationVector.at<double>(1,0), translationVector.at<double>(2,0));
		gtsamRelPose = gtsam::Pose3(R, t);
		*/
		graphPrevNode = 1;

	};

    cv::Mat cameraImg; // camera image
    cv::cuda::GpuMat gpu_cameraImg; // contains camera image on gpu
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    std::vector<cv::Point2f> refinedPointsCurr; //refined points of this frame after refining according matching
    std::vector<cv::Point2f> refinedPointsPrev; //refined points of the previous frame corresponding to refined points of this frame
    std::vector<cv::Vec2d> undistortedPointsCurr; //Undistorted points on current frame
    std::vector<cv::Vec2d> undistortedPointsPrev; //Undistorted points on previous frame
    std::vector<cv::Vec3d> points3D; //3D points obtained after triangulation using preious and current frame

    cv::cuda::GpuMat gpu_keypoints; // GPU 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    cv::cuda::GpuMat gpu_descriptors; // gpu keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    cv::cuda::GpuMat gpu_kptMatches; // gpu keypoint matches between previous and current frame

    cv::Mat essentialMatrix; //Rotation matrix relative to the previous frame
    cv::Mat rotationMatrix; //Rotation matrix relative to the previous frame
    cv::Mat translationVector; //Translation vector relative to the previous frame
    cv::Mat projectionMatrix; //Projection matrix that includes rotation and translation information

    double msTimestamp;

    /*just for visualization*/
    cv::Affine3d pose;

    /*for gtsam*/
    //gtsam::Pose3 gtsamRelPose;
    int graphPrevNode;

};



#if 0
struct DataFrame { // represents the available sensor information at the same time instance


	DataFrame()
	{
		cv::Mat projection1(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
		cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
		diag.at<double>(2,2) = -1;
		//float diag_elem[9] = {1, 0, 0, 0, 1, 0 , 0, 0, 1 };
		//cv::Mat diag = cv::Mat(3, 3, CV_64F, diag_elem);
		diag.copyTo(rotationMatrix);
		cv::Mat orig(cv::Mat::zeros(3, 1, CV_64F));
		orig.copyTo(translationVector);
		diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));
		projectionMatrix = projection1.clone();
		pose = cv::Affine3d(rotationMatrix, translationVector);

		/*
		gtsam::Rot3 R(rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2),
				rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2),
				rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2));
		gtsam::Point3 t(translationVector.at<double>(0,0),translationVector.at<double>(1,0), translationVector.at<double>(2,0));
		gtsamRelPose = gtsam::Pose3(R, t);
		*/
		graphPrevNode = 1;

	};

    cv::Mat cameraImg; // camera image
    //cv::cuda::GpuMat gpu_cameraImg; // contains camera image on gpu

    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    std::vector<cv::Point2f> refinedPointsCurr; //refined points of this frame after refining according matching
    std::vector<cv::Point2f> refinedPointsPrev; //refined points of the previous frame corresponding to refined points of this frame
    std::vector<cv::Vec2d> undistortedPointsCurr; //Undistorted points on current frame
    std::vector<cv::Vec2d> undistortedPointsPrev; //Undistorted points on previous frame
    std::vector<cv::Vec3d> points3D; //3D points obtained after triangulation using preious and current frame

    //cv::cuda::GpuMat gpu_keypoints; // GPU 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    //cv::cuda::GpuMat gpu_descriptors; // gpu keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    //cv::cuda::GpuMat gpu_kptMatches; // gpu keypoint matches between previous and current frame

    cv::Mat essentialMatrix; //Rotation matrix relative to the previous frame
    cv::Mat rotationMatrix; //Rotation matrix relative to the previous frame
    cv::Mat translationVector; //Translation vector relative to the previous frame
    cv::Mat projectionMatrix; //Projection matrix that includes rotation and translation information

    double msTimestamp;

    /*just for visualization*/
    cv::Affine3d pose;

    /*for gtsam*/
    //gtsam::Pose3 gtsamRelPose;
    int graphPrevNode;

};

#endif

#endif /* dataStructures_h */
