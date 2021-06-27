/*
 * KpointMatcher.cpp
 *
 *  Created on: Jun 26, 2021
 *      Author: jeslava
 */


#include "KpointMatcher.hpp"


using namespace std;
using namespace kpproc;

bool KpointMatcher::match(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors)
{

	#if DEBUG_LATENCY
	/*To measure time*/
	double t = (double)cv::getTickCount();
	#endif

	vector<vector<cv::DMatch>> knnMatches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
	matcher->knnMatch( queryDescriptors, trainDescriptors, knnMatches, 2, cv::noArray());
	double minDescDistRatio = 0.8;
	/*TODO: Optimize this matches filtering*/
	for(auto it = knnMatches.begin(); it!=knnMatches.end(); it++)
	{
		if((*it)[0].distance < minDescDistRatio*( (*it)[1].distance ) )
		{
			m_matches.push_back((*it)[0]);
		}
	}

    if(m_matches.size() < MIN_KPMATCHS_NBR)
    {
    	return false;
    }
    else
    {
		#if DEBUG_LATENCY
    	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    	cout << "#### KPOINT MATCHING DONE " << m_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
		#endif
    	return true;
    }

}
