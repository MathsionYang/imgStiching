#ifndef _FeaturesMatcher1_hpp_
#define _FeaturesMatcher1_hpp_
#include "CommonHeade.hpp"
#include "gms_matcher.h"
typedef std::set<std::pair<int, int> > MatchesSet;
class FeaturesMatcher1
{
public:
	virtual ~FeaturesMatcher1() {}
	void operator ()(const ImageFeatures &features1, const ImageFeatures &features2,
		MatchesInfo& matches_info) {
		match(features1, features2, matches_info);
	}
	void operator ()(const std::vector<ImageFeatures> &features, std::vector<MatchesInfo> &pairwise_matches,
		const cv::UMat &mask = cv::UMat());
	bool isThreadSafe() const { return is_thread_safe_; }
	virtual void collectGarbage() {}

protected:
	FeaturesMatcher1(bool is_thread_safe = false) : is_thread_safe_(is_thread_safe) {}
	virtual void match(const ImageFeatures &features1, const ImageFeatures &features2,
		MatchesInfo& matches_info) = 0;
	bool is_thread_safe_;
};
struct MatchPairsBody1 : ParallelLoopBody
{
	MatchPairsBody1(FeaturesMatcher1 &_matcher, const std::vector<ImageFeatures> &_features,
		std::vector<MatchesInfo> &_pairwise_matches, std::vector<std::pair<int, int> > &_near_pairs)
		: matcher(_matcher), features(_features),
		pairwise_matches(_pairwise_matches), near_pairs(_near_pairs) {}

	void operator ()(const Range &r) const
	{
		cv::RNG rng = cv::theRNG(); // save entry rng state
		const int NUM_IMAGES = static_cast<int>(features.size());
		for (int i = r.start; i < r.end; ++i)
		{
			cv::theRNG() = cv::RNG(rng.state + i); // force "stable" RNG seed for each processed pair
			int from = near_pairs[i].first;
			int to = near_pairs[i].second;
			int pair_idx = from * NUM_IMAGES + to;

			matcher(features[from], features[to], pairwise_matches[pair_idx]);
			pairwise_matches[pair_idx].src_img_idx = from;
			pairwise_matches[pair_idx].dst_img_idx = to;

			size_t dual_pair_idx = to * NUM_IMAGES + from;

			pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
			pairwise_matches[dual_pair_idx].src_img_idx = to;
			pairwise_matches[dual_pair_idx].dst_img_idx = from;

			if (!pairwise_matches[pair_idx].H.empty())
				pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();

			for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
				std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
					pairwise_matches[dual_pair_idx].matches[j].trainIdx);
		}
	}

	FeaturesMatcher1 &matcher;
	const std::vector<ImageFeatures> &features;
	std::vector<MatchesInfo> &pairwise_matches;
	std::vector<std::pair<int, int> > &near_pairs;

private:

};

void FeaturesMatcher1::operator ()(const std::vector<ImageFeatures> &features, std::vector<MatchesInfo> &pairwise_matches,
	const UMat &mask)
{
	const int NUM_IMAGES = static_cast<int>(features.size());
	CV_Assert(mask.empty() || (mask.type() == CV_8U && mask.cols == NUM_IMAGES && mask.rows));
	Mat_<uchar> mask_(mask.getMat(ACCESS_READ));
	if (mask_.empty())
		mask_ = Mat::ones(NUM_IMAGES, NUM_IMAGES, CV_8U);
	std::vector<std::pair<int, int> > near_pairs;
	for (int i = 0; i < NUM_IMAGES - 1; ++i)
		for (int j = i + 1; j < NUM_IMAGES; ++j)
			if (features[i].keypoints.size() > 0 && features[j].keypoints.size() > 0 && mask_(i, j))
				near_pairs.push_back(std::make_pair(i, j));

	pairwise_matches.resize(NUM_IMAGES * NUM_IMAGES);
	MatchPairsBody1 body(*this, features, pairwise_matches, near_pairs);
	if (is_thread_safe_)
		parallel_for_(Range(0, static_cast<int>(near_pairs.size())), body);
	else
		body(Range(0, static_cast<int>(near_pairs.size())));

}


class CpuMatcher1 : public FeaturesMatcher1
{
public:
	CpuMatcher1(float match_conf) : FeaturesMatcher1(true), match_conf_(match_conf) {}
	void match(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo& matches_info);

private:
	float match_conf_;
};

void CpuMatcher1::match(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo& matches_info)
{

	//特征点的坐标信息和特征点的描述符信息如何一一对应。
	CV_Assert(features1.descriptors.type() == features2.descriptors.type());
	CV_Assert(features2.descriptors.depth() == CV_8U || features2.descriptors.depth() == CV_32F);
	matches_info.matches.clear();
	Ptr<cv::DescriptorMatcher> matcher;
	{
		Ptr<flann::IndexParams> indexParams = makePtr<flann::KDTreeIndexParams>();
		Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>();

		if (features2.descriptors.depth() == CV_8U)
		{
			indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
			searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
		}

		matcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
	}
	std::vector< std::vector<DMatch> > pair_matches;
	MatchesSet matches;

	// Find 1->2 matches
	matcher->knnMatch(features1.descriptors, features2.descriptors, pair_matches, 2);
	vector<DMatch>_matches1;
	int count = 0;
	for (size_t i = 0; i < pair_matches.size(); ++i)
	{
		if (pair_matches[i].size() < 2)
			continue;
		const DMatch& m0 = pair_matches[i][0];
		const DMatch& m1 = pair_matches[i][1];

		if (m0.distance < (1.f - match_conf_) * m1.distance)
		{
			count++;
			_matches1.push_back(m0);
		}

	}
	gms_matcher gms(features1.keypoints, features1.img_size, features2.keypoints, features2.img_size, _matches1);

	std::vector<bool>vbInliers;
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);

	for (size_t i = 0; i < vbInliers.size(); i++)
	{
		if (vbInliers[i] == true)
		{
			matches_info.matches.push_back(_matches1[i]);
		}
	}


}

class BestOf2NearestMatcher1 : public FeaturesMatcher1
{
public:
	BestOf2NearestMatcher1(bool try_use_gpu = false, float match_conf = 0.3f, int num_matches_thresh1 = 6,
		int num_matches_thresh2 = 6);

	//void collectGarbage();

protected:
	void match(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo &matches_info);

	int num_matches_thresh1_;
	int num_matches_thresh2_;
	Ptr<FeaturesMatcher1> impl_;
};

BestOf2NearestMatcher1::BestOf2NearestMatcher1(bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2)
{
	impl_ = makePtr<CpuMatcher1>(match_conf);
	is_thread_safe_ = impl_->isThreadSafe();
	num_matches_thresh1_ = num_matches_thresh1;
	num_matches_thresh2_ = num_matches_thresh2;
}

void BestOf2NearestMatcher1::match(const ImageFeatures &features1, const ImageFeatures &features2,
	MatchesInfo &matches_info)
{
	//CV_INSTRUMENT_REGION();
	(*impl_)(features1, features2, matches_info);
	// Check if it makes sense to find homography
	if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
		return;

	// Construct point-point correspondences for homography estimation
	Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
	Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
	for (size_t i = 0; i < matches_info.matches.size(); ++i)
	{
		const DMatch& m = matches_info.matches[i];

		Point2f p = features1.keypoints[m.queryIdx].pt;
		p.x -= features1.img_size.width * 0.5f;
		p.y -= features1.img_size.height * 0.5f;
		src_points.at<Point2f>(0, static_cast<int>(i)) = p;

		p = features2.keypoints[m.trainIdx].pt;
		p.x -= features2.img_size.width * 0.5f;
		p.y -= features2.img_size.height * 0.5f;
		dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
	}

	// Find pair-wise motion
	matches_info.H = findHomography(src_points, dst_points, matches_info.inliers_mask, RANSAC);
	if (matches_info.H.empty() || std::abs(determinant(matches_info.H)) < std::numeric_limits<double>::epsilon())
		return;

	// Find number of inliers
	matches_info.num_inliers = 0;
	for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
		if (matches_info.inliers_mask[i])
			matches_info.num_inliers++;

	// These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
	// using Invariant Features"
	matches_info.confidence = matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

	// Set zero confidence to remove matches between too close images, as they don't provide
	// additional information anyway. The threshold was set experimentally.
	matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;

	// Check if we should try to refine motion
	if (matches_info.num_inliers < num_matches_thresh2_)
		return;

	// Construct point-point correspondences for inliers only
	src_points.create(1, matches_info.num_inliers, CV_32FC2);
	dst_points.create(1, matches_info.num_inliers, CV_32FC2);
	int inlier_idx = 0;
	for (size_t i = 0; i < matches_info.matches.size(); ++i)
	{
		if (!matches_info.inliers_mask[i])
			continue;

		const DMatch& m = matches_info.matches[i];

		Point2f p = features1.keypoints[m.queryIdx].pt;
		p.x -= features1.img_size.width * 0.5f;
		p.y -= features1.img_size.height * 0.5f;
		src_points.at<Point2f>(0, inlier_idx) = p;

		p = features2.keypoints[m.trainIdx].pt;
		p.x -= features2.img_size.width * 0.5f;
		p.y -= features2.img_size.height * 0.5f;
		dst_points.at<Point2f>(0, inlier_idx) = p;

		inlier_idx++;
	}

	// Rerun motion estimation on inliers only
	matches_info.H = findHomography(src_points, dst_points, RANSAC);
}


#endif
