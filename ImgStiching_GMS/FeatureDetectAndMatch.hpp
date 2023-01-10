#ifndef _FeatureDetectAndMatch_hpp_
#define _FeatureDetectAndMatch_hpp_
#include "CommonHeade.h"
#include "gms_matcher.h"

Ptr<FeatureDetector> detector = SURF::create();

class FeatureDetectAndMatch
{
public:
	void GetImgs(vector<Mat> imgs);
	vector<ImageFeatures> GetFeauture();
	vector<MatchesInfo>GetMatches();
private:
	vector<Mat> _imgs;
	vector<ImageFeatures> _features;    //表示图像特征
	vector<MatchesInfo> PairMatches;
};
void FeatureDetectAndMatch::GetImgs(vector<Mat> imgs)
{
	for (int i =0;i<imgs.size();i++)
	{
		_imgs.push_back(imgs[i]);
	}
	cout << "文件个数:" << _imgs.size() << endl;
}
vector<ImageFeatures>FeatureDetectAndMatch::GetFeauture()
{
	int Num_images = _imgs.size();
	
	for (int img_idx = 0; img_idx < Num_images; img_idx++)
	{
		Size img_size = _imgs[img_idx].size();
		vector< KeyPoint > keypoint;
		detector->detect(_imgs[img_idx], keypoint);
		Mat desc;
		detector->compute(_imgs[img_idx], keypoint, desc);
		UMat umat;
		desc.copyTo(umat);
		_features.push_back({ img_idx ,img_size ,keypoint ,umat });
	}
	return _features;
}
vector<MatchesInfo>FeatureDetectAndMatch::GetMatches()
{
	for (int i = 0; i < _imgs.size(); i++)
	{
		Mat desc1;
		desc1.copyTo(_features[i].descriptors);
		vector<DMatch> matches_gms;
		vector<vector<DMatch> > matchePoints;
		double confidence;
		for (int k = 0; k < _imgs.size(); k++)
		{
			Mat desc2;
			desc2.copyTo(_features[k].descriptors);
			
			if (i != k)
			{
				FlannBasedMatcher matcher;
				vector<Mat> train_desc(1, desc1);
				matcher.add(train_desc);
				matcher.train();
				matcher.knnMatch(_features[i].descriptors, matchePoints, 2);

				vector<DMatch> GoodMatchePoints;
				for (int i = 0; i < matchePoints.size(); i++)
				{
					if (matchePoints[i][0].distance < 0.8 * matchePoints[i][1].distance)
					{
						GoodMatchePoints.push_back(matchePoints[i][0]);
					}
				}
				std::vector<uchar> inliers_mask(GoodMatchePoints.size(),0);
				gms_matcher gms(_features[i].keypoints, _imgs[i].size(), _features[k].keypoints, _imgs[k].size(), GoodMatchePoints);
				std::vector<bool>vbInliers;
				int num_inliers = gms.GetInlierMask(vbInliers, false, false);

				for (size_t n = 0; n < vbInliers.size(); n++)
				{
					if (vbInliers[i] == true)
					{
						inliers_mask[n] = 1;
						matches_gms.push_back(GoodMatchePoints[n]);
					}
				}
				if (matches_gms.size()>=6)
				{
					confidence = matches_gms.size() / (8 + 0.3*GoodMatchePoints.size());
				}
				else
				{
					confidence = 0.0;
				}
				Mat H;
				PairMatches.push_back({ i,k,matches_gms,inliers_mask,matches_gms.size(),H, confidence });
			}
			
		}
		
		matches_gms.clear();
	}
}
#endif
//int src_img_idx, dst_img_idx;       //!< Images indices (optional)
//std::vector<DMatch> matches;
//std::vector<uchar> inliers_mask;    //!< Geometrically consistent matches mask
//int num_inliers;                    //!< Number of geometrically consistent matches
//Mat H;                              //!< Estimated homography
//double confidence;                  //!< Confidence two images are from the same panorama