/*
 * KpointExtractor.cpp
 *
 *  Created on: Jun 25, 2021
 *      Author: user3
 */


#include "KpointExtractor.hpp"

using namespace std;
using namespace kpproc;





bool KpointExtractor::extractKpointDescriptors(const cv::Mat & inputImage)
{
	#if DEBUG_LATENCY
	/*To measure time*/
	double t = (double)cv::getTickCount();
	#endif

	/*Convert to grayscale*/
    cv::cvtColor(inputImage, m_frameImg, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::FeatureDetector> detector;
    detector = cv::ORB::create(300, 1.3f, 7, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    detector->detectAndCompute(m_frameImg, cv::Mat(), m_keypoints, m_descriptors);

    if(m_keypoints.size() < MIN_KPOINT_NBR)
    {
    	return false;
    }
    else
    {
		#if DEBUG_LATENCY
    	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    	cout << "#### KPOINT DESCRIPTOR EXTRACTION DONE " << m_keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
		#endif
    	return true;
    }
}


void KpointExtractor::getResults(std::vector<cv::KeyPoint> & resKeypoints, cv::Mat & resDescriptors, cv::Mat & resImage)
{

	resKeypoints = std::move(m_keypoints);
	resDescriptors = std::move(m_descriptors);
	resImage = std::move(m_frameImg);

}




