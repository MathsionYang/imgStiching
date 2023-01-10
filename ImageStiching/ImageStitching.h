#pragma once
//�˵���ͼ��ƴ��

#include<string>
#include<vector>
#include <gdal.h>
#include <gdal_priv.h>  
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp> 
#include<opencv2/xfeatures2d.hpp>
#include<opencv2/core/core.hpp>
#include<iostream>
#include<opencv2/stitching/warpers.hpp>


using namespace std;
using namespace cv::xfeatures2d;

enum Feature{SIFT, SURF, ORB, FERNS};


/*
loadImages����img1_path��img2_path:��������ͼƬ·��
setFeatureExtractor����������ȡ�ķ�ʽ
runPipeline������

saveImages����ԭʼ·�����������Ϊ5�����飬�����ÿһ����һ��ͼƬ��·����string���󣬷ֱ�Ϊ��
0��ԭʼ2��ͼƬ���Ұڷź��ͼƬ·��
1����ע���������ͼƬ·��
2��ȥ��outliner���ͼƬ·��
3��������ƥ����ͼƬ·��
4������ƴ�Ӻ��ͼƬ·��
*/
/*! byte */
typedef unsigned char   byte;
/*! 8U */
typedef unsigned char   DT_8U;
/*! 16U */
typedef unsigned short  DT_16U;
/*! 16S */
typedef short           DT_16S;
/*! 32U */
typedef unsigned int    DT_32U;
/*! 32S */
typedef int             DT_32S;
/*! 32F */
typedef float           DT_32F;
/*! 64F */
typedef double          DT_64F;

//����һ�����Ӱ����Ϣ��structure
typedef struct
{
	//GDALDataset *poDataset = NULL;
	int Xsize = 0;
	int Ysize = 0;
	int nbands = 0;
	double *tmpadfGeoTransform = new double[6]; //�洢����6����
	const char* proj = NULL;//�洢ͶӰ
	GDALDataType iDataType = GDT_Byte;
}MyStruct;

class ImageStitcher {
public:

	void loadImages(const cv::Mat& img1_path, const cv::Mat& img2_path, MyStruct &imgSt);
	void loadKeyPtandMatch(const vector<cv::KeyPoint>& keyp1, const vector<cv::KeyPoint>& keyp2, vector<cv::DMatch>& Fianllmatches);
	bool Mat2File(cv::Mat img, MyStruct &St, GDALDataset *poDataset);//const QString fileName


private:
	string left_img_path;
	string right_img_path;

	cv::Mat img1;
	cv::Mat img2;
	Feature feat;
	MyStruct img_St;
	vector<cv::KeyPoint> raw_kpts1;
	vector<cv::KeyPoint> raw_kpts2;

	vector<cv::KeyPoint> ransac_kpts1;
	vector<cv::KeyPoint> ransac_kpts2;

	vector<cv::DMatch> matches;

	//result images
	cv::Mat img_origin;
	cv::Mat img_detect_kpts;
	cv::Mat img_rm_outlier;
	cv::Mat img_after_ransac;
	cv::Mat img_stitched;
	cv::Mat img_postprocess;
	void imageTransform();
	void optimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst, int left_cover, int right_cover);
	void colorBalance();
	void AcE(int C = 3, int n = 2, float MaxCG = 7.5);

};



