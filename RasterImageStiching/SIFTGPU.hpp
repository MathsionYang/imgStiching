#ifndef _SIFTGPU_H
#define _SIFTGPU_H
#include <Windows.h>
#include <gl\GL.h>
#include <opencv2\opencv.hpp>
#include <SiftGPU.h>

using namespace std;

enum InitStatus {
	INIT_OK,
	INIT_IS_NOT_SUPPORT,
	INIT_VERIFY_FAILED
};

class SIFT_GPU
{
public:
	SIFT_GPU() = default;
	~SIFT_GPU()
	{
		if (m_siftGpuDetector)
			delete m_siftGpuDetector;
		if (m_siftGpuMatcher)
			delete m_siftGpuMatcher;
	
	}
private:
	SiftGPU *m_siftGpuDetector;
	SiftMatchGPU *m_siftGpuMatcher;
	cv::Mat img_1, img_2;
	//m_maxMatch是进行匹配时，最多的匹配点的个数，默认为4096
	int m_maxMatch;
public:
	InitStatus create() {
		m_siftGpuDetector = new SiftGPU();

		char* myargv[4] = { "-fo","-1","-v","1" };
		m_siftGpuDetector->ParseParam(4, myargv);
		// Set edge threshold, dog threshold

		if (m_siftGpuDetector->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
		
			return InitStatus::INIT_IS_NOT_SUPPORT;
		}
		m_siftGpuMatcher = new SiftMatchGPU();
		m_siftGpuMatcher->VerifyContextGL();
		m_maxMatch = 10000;
		return INIT_OK;
	}

	void detectAndCompute(const cv::Mat &img, cv::Mat &descriptors, vector<cv::KeyPoint> &kpts) {

		//必须是RGB图像
		assert(img.channels() == 3);
		m_siftGpuDetector->RunSIFT(img.cols, img.rows, img.data, GL_RGB, GL_UNSIGNED_BYTE);
		auto num1 = m_siftGpuDetector->GetFeatureNum();
		vector<float> des(128 * num1);
		vector<SiftGPU::SiftKeypoint> keypoints(num1);
		m_siftGpuDetector->GetFeatureVector(&keypoints[0], &des[0]);

		// Trans to Mat
		cv::Mat m(des);
		descriptors = m.reshape(1, num1).clone();

		for (const SiftGPU::SiftKeypoint &kp : keypoints) {
			cv::KeyPoint t(kp.x, kp.y, kp.s, kp.o);
			kpts.push_back(t);
		}


	}
	void transToRootSift(const cv::Mat &siftFeature, cv::Mat &rootSiftFeature) {
		for (int i = 0; i < siftFeature.rows; i++) {
			// Conver to float type
			cv::Mat f;
			siftFeature.row(i).convertTo(f, CV_32FC1);
			normalize(f, f, 1, 0, cv::NORM_L1); // l1 normalize
			sqrt(f, f); // sqrt-root  root-sift
			rootSiftFeature.push_back(f);
		}
	}
	int gpuMatch(const cv::Mat &des1, const cv::Mat &des2, vector<cv::DMatch>& matches) {
		m_siftGpuMatcher->SetDescriptors(0, des1.rows, (float*)des1.data);
		m_siftGpuMatcher->SetDescriptors(1, des2.rows, (float*)des2.data);
		int(*match_buf)[2] = new int[m_maxMatch][2];
		auto matchNum = m_siftGpuMatcher->GetSiftMatch(m_maxMatch, match_buf);
		for (int i = 0; i < matchNum; i++) {
			cv::DMatch dm(match_buf[i][0], match_buf[i][1], 0);
			matches.push_back(dm);
		}
		delete[] match_buf;
		return matchNum;
	}
};
#endif