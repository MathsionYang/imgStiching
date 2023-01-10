#include "FeatureMatch.h"
#include "gms_matcher.h"
#include "SIFTGPU.hpp"
//ÌØÕ÷¼ì²âÆ÷
int minHessian = 400;
cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SURF::create(minHessian);
bool FeatureMatch::LoadImg(cv::Mat &img1, cv::Mat &img2)
{
	FirstImg = img1;
	SecondImg = img2;
	if (FirstImg.empty() || SecondImg.empty())
	{
		return false;
	}
	else
	{
		return true;
	}
	
}
bool FeatureMatch::DetectorKeyPoint(vector<cv::KeyPoint>& keypoints_1, vector<cv::KeyPoint>& keypoints_2)
{
	
	detector->detect(FirstImg, keypoints_1);
	detector->detect(SecondImg, keypoints_2);
	if (!keypoints_1.empty() && !keypoints_2.empty())
	{
		keypts_1 = keypoints_1;
		keypts_2 = keypoints_2;
		return true;
	}
	else 
	{ 
		return false; 
	}
}
int FeatureMatch::FeatMatch(vector<cv::DMatch>& matches_gms)
{
	vector<vector<cv::DMatch>>matchePoints;
	vector<cv::DMatch>_matches;
	cv::Mat descriptors_1, descriptors_2;
	detector->compute(FirstImg, keypts_1, descriptors_1);
	detector->compute(SecondImg, keypts_2, descriptors_2);
	cv::FlannBasedMatcher matcher;
	vector<cv::Mat> train_desc(1, descriptors_2);
	matcher.add(train_desc);
	matcher.train();
	matcher.knnMatch(descriptors_1, matchePoints, 2);
	//NNDRÉ¸Ñ¡
	_matches = NNSCNCheck(matchePoints,0.8);
	//GMSÉ¸Ñ¡
	gms_matcher gms(keypts_1, FirstImg.size(), keypts_2, SecondImg.size(), _matches);
	std::vector<bool>vbInliers;
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);

	for (size_t i = 0; i < vbInliers.size(); i++)
	{
		if (vbInliers[i] == true)
		{
			matches_gms.push_back(_matches[i]);
		}
	}


	cv::Mat img_matches__GMS;
	drawMatches(FirstImg, keypts_1, SecondImg, keypts_2, matches_gms, img_matches__GMS);
	int size = matches_gms.size();
	return size;
}
vector<cv::DMatch> FeatureMatch::NNSCNCheck(vector<vector<cv::DMatch> > Matches, float threshold) {
	vector<cv::DMatch> GoodMatchePoints;
	for (int i = 0; i < Matches.size(); i++)
	{
		if (Matches[i][0].distance < threshold * Matches[i][1].distance)
		{
			GoodMatchePoints.push_back(Matches[i][0]);
		}
	}
	return GoodMatchePoints;
}
void FeatureMatch::MatchBySIFTGPU(vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches_gms)
{
	SIFT_GPU fp;
	InitStatus initState = fp.create();
	if (initState != InitStatus::INIT_OK)
	{
		exit(0);
	}
	cv::Mat  des1, des2;
	
	vector<cv::DMatch> matches;
	kp1.clear();
	kp2.clear();
	matches.clear();
	
	fp.detectAndCompute(FirstImg, des1, kp1);
	fp.detectAndCompute(SecondImg, des2, kp2);
	fp.gpuMatch(des1, des2, matches);
	
	gms_matcher gms(kp1, FirstImg.size(), kp2, SecondImg.size(), matches);
	std::vector<bool>vbInliers;
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	for (size_t i = 0; i < vbInliers.size(); i++)
	{
		if (vbInliers[i] == true)
		{
			matches_gms.push_back(matches[i]);
		}
	}

}
