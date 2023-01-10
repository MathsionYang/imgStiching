#pragma once
//端到端图像拼接

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
loadImages设置img1_path、img2_path:两张输入图片路径
setFeatureExtractor设置特征提取的方式
runPipeline计算结果

saveImages输入原始路径，输出长度为5的数组，数组的每一项是一张图片的路径的string对象，分别为：
0：原始2张图片左右摆放后的图片路径
1：标注了特征点的图片路径
2：去除outliner后的图片路径
3：特征点匹配后的图片路径
4：最终拼接后的图片路径
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

//创建一个存放影像信息的structure
typedef struct
{
	//GDALDataset *poDataset = NULL;
	int Xsize = 0;
	int Ysize = 0;
	int nbands = 0;
	double *tmpadfGeoTransform = new double[6]; //存储地理6参数
	const char* proj = NULL;//存储投影
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



