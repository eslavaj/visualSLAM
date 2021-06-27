/*
 * KpointMatcher.cpp
 *
 *  Created on: Jun 26, 2021
 *      Author: jeslava
 */


#include "KpointMatcher.hpp"


using namespace std;
using namespace kpproc;

bool KpointMatcher::match(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors,
						  const std::vector<cv::KeyPoint>& queryKeypoints,
					      const std::vector<cv::KeyPoint>& trainKeypoints)
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
    	std::cout << "----------------- "<< m_matches.size() << std::endl;
    	return false;
    }
    else
    {

    	vector<cv::DMatch> refinedMatches;
    	if( refineMatches(queryDescriptors, trainDescriptors, queryKeypoints, trainKeypoints, refinedMatches) )
    	{
        	m_matches = move(refinedMatches);

    		#if DEBUG_LATENCY
        	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        	cout << "#### KPOINT MATCHING DONE " << m_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    		#endif
        	return true;
    	}else
    	{

    		return false;
    	}
    }

}


bool KpointMatcher::refineMatches(const cv::Mat & queryDescriptors, const cv::Mat & trainDescriptors,
		   	   	   	   	   	   	  const vector<cv::KeyPoint>& queryKeypoints,
								  const vector<cv::KeyPoint>& trainKeypoints,
								  vector<cv::DMatch>& refinedMatches)
{
	/*Convert keypoints into Point2f*/
	vector<cv::Point2f> queryPoints, trainPoints;
	cv::Mat outHomographMatrix;

	for (vector<cv::DMatch>::const_iterator it= m_matches.begin(); it!= m_matches.end(); it++)
	{
		/*
		if(it->queryIdx<0)
		{
			cout<< "it->queryIdx: "<< it->queryIdx<<endl;
			cout<< "it->trainIdx: "<< it->trainIdx<<endl;
		}*/

		// Get the position of previous (query) keypoints
		queryPoints.push_back(queryKeypoints[it->queryIdx].pt);
		// Get the position of current (train) keypoints
		trainPoints.push_back(trainKeypoints[it->trainIdx].pt);
	}


	vector<uchar> inliers(queryPoints.size(),0);
	outHomographMatrix = cv::findHomography(
			queryPoints,trainPoints, // corresponding points
			inliers,	     // outputed inliers matches
			cv::RANSAC,	     // RANSAC method
			DIST_TO_EPILINE);// max distance to reprojection point


	if(inliers.size() < MIN_HOMOGR_INLIERS_NBR)
	{
		/*Not enough inliers matches*/
		return false;
	}
	else
	{
		// extract the surviving (inliers) matches
		vector<uchar>::const_iterator itIn= inliers.begin();
		vector<cv::DMatch>::const_iterator itM= m_matches.begin();
		// for all inliers matches
		for ( ;itIn!= inliers.end(); ++itIn, ++itM)
		{
			if (*itIn)
			{ // it is a valid match
				refinedMatches.push_back(*itM);
				m_refinedPointsPrev.push_back(cv::Point_<float>( queryKeypoints[itM->queryIdx].pt.x, queryKeypoints[itM->queryIdx].pt.y ));
				m_refinedPointsCurr.push_back(cv::Point_<float>( trainKeypoints[itM->trainIdx].pt.x, trainKeypoints[itM->trainIdx].pt.y ));
			}
		}
	}

}


void KpointMatcher::getResults(std::vector< cv::DMatch > & matches,
							   std::vector<cv::Point2f> & refinedPointsPrev,
							   std::vector<cv::Point2f> & refinedPointsCurr)
{

	matches = move(m_matches);
	refinedPointsPrev = move(m_refinedPointsPrev);
	refinedPointsCurr = move(m_refinedPointsCurr);


}




