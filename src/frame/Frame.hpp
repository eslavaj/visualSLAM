/*
 * Frame.hpp
 *
 *  Created on: Jun 25, 2021
 *      Author: user3
 */

#ifndef FRAME_FRAME_HPP_
#define FRAME_FRAME_HPP_


#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core/cuda.hpp>
//#include <opencv2/core/types.hpp>

#include <boost/serialization/access.hpp>


class Frame
{

    cv::Mat cameraImg; // camera image
    double timestamp;
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    std::vector<cv::Point2f> refinedPointsCurr; //refined points of this frame after refining according matching
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::Point2f> refinedPointsPrev; //refined points of the previous frame corresponding to refined points of this frame
    std::vector<cv::Vec2d> undistortedPointsCurr; //Undistorted points on current frame
    std::vector<cv::Vec2d> undistortedPointsPrev; //Undistorted points on previous frame
    std::vector<cv::Vec3d> points3D; //3D points obtained after triangulation using preious and current frame


    cv::Mat essentialMatrix; //Rotation matrix relative to the previous frame
    cv::Mat rotationMatrix; //Rotation matrix relative to the previous frame
    cv::Mat translationVector; //Translation vector relative to the previous frame
    cv::Mat projectionMatrix; //Projection matrix that includes rotation and translation information


    friend class boost::serialization::access;
    template<class Archive>
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
