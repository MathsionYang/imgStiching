#pragma once
#include <opencv2/opencv.hpp>  
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/stitching.hpp>


using namespace std;
class FeatureMatch
{
public:
	bool DetectorKeyPoint( vector<cv::KeyPoint>& keypoints_1, vector<cv::KeyPoint>& keypoints_2);
	int FeatMatch(vector<cv::DMatch>& matches_gms);
	bool LoadImg(cv::Mat &img1, cv::Mat &img2);
	vector<cv::DMatch> NNSCNCheck(vector<vector<cv::DMatch> > Matches, float threshold);
	void MatchBySIFTGPU(vector<cv::KeyPoint> &kp1, vector<cv::KeyPoint> &kp2, vector<cv::DMatch> &matches);

private:
	cv::Mat FirstImg, SecondImg;
	vector<cv::KeyPoint> keypts_1, keypts_2;

};

