
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <gdal.h>
#include <gdal_priv.h>  
#include <iostream>
#include <string>
#include <vector>
#include "SIFTDetector.h"
#include "GDALread.h"
#include "GCPTransformer.h"
#include "DrawDelaunay.h"
#include "gms_matcher.h"
#include  "ImageStitching.h"
#include "GDALOpenCV.h"


using namespace std;
using namespace cv;

cv::Mat GDAL2Mat(GDALDataset *poDataset, MyStruct &St);
Mat imgchangeFormat(MyStruct St, Mat img);
int main()
{


	string  src_path = "E:\\daspatial\\lasdata\\Production_2000DOM_DSM_part_02_18.tif";
	//string  src_path1 = "img\\img02.tif";
	//注册驱动  
	MyStruct St, St2;
	GDALAllRegister();
	GDALDataset *poDataset = (GDALDataset *)GDALOpen(src_path.c_str(), GA_ReadOnly);
	//GDALDataset *poDataset1 = (GDALDataset *)GDALOpen(src_path1.c_str(), GA_ReadOnly);
	////影像1
	St.Xsize = poDataset->GetRasterXSize();
	St.Ysize = poDataset->GetRasterYSize();
	St.nbands = poDataset->GetRasterCount();
	poDataset->GetGeoTransform(St.tmpadfGeoTransform);
	St.proj = poDataset->GetProjectionRef();
	St.iDataType = poDataset->GetRasterBand(1)->GetRasterDataType();
	//////影像2
	//St2.Xsize = poDataset1->GetRasterXSize();
	//St2.Ysize = poDataset1->GetRasterYSize();
	//St2.nbands = poDataset1->GetRasterCount();
	//poDataset1->GetGeoTransform(St.tmpadfGeoTransform);
	//St2.proj = poDataset1->GetProjectionRef();
	//St2.iDataType = poDataset1->GetRasterBand(1)->GetRasterDataType();
	
	Mat img = GDAL2Mat(poDataset, St);
	//Mat img2 = GDAL2Mat(poDataset1, St2);
	Mat img_01, img_02;
	////RGB-BGR
	img_01 = imgchangeFormat(St, img);
	//img_02 = imgchangeFormat(St2, img2);
	////16位转8位
	Mat img_1, img_2;
	cv::normalize(img_01, img_1, 0, 255, cv::NORM_MINMAX, CV_8UC3);
	cv::normalize(img_02, img_2, 0, 255, cv::NORM_MINMAX, CV_8UC3);



	vector<KeyPoint> keypoints_1, keypoints_2;
	DetectorKeyPoint(img_1, img_2, keypoints_1, keypoints_2);
	Mat descriptors_1, descriptors_2;
	ComputeDescriptor(img_1, img_2, keypoints_1, keypoints_2, descriptors_1, descriptors_2);
	cout << "匹配计算开始" << endl;
	vector<vector<DMatch> > matchePoints;
	int size_ = FeatureMatch(matchePoints, descriptors_1, descriptors_2);
	vector<DMatch>_matches, matches_gms;
	//初始
	for (int i = 0; i < matchePoints.size(); i++)
	{
		_matches.push_back(matchePoints[i][0]);
	}
	///GMS
	cv::Mat output_line = img_1.clone();
	cv::Mat output_line1 = img_2.clone();
	gms_matcher gms(keypoints_1, output_line.size(), keypoints_2, output_line1.size(), _matches);
	std::vector<bool>vbInliers;
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);

	for (size_t i = 0; i < vbInliers.size(); i++)
	{
		if (vbInliers[i] == true)
		{
			matches_gms.push_back(_matches[i]);
		}
	}
	Mat img_matches__GMS;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches_gms, img_matches__GMS);
	ImageStitcher imgstiching;
	
	imgstiching.loadImages(img_1, img_2,St);
	imgstiching.loadKeyPtandMatch(keypoints_1, keypoints_2, matches_gms);
	GDALClose((GDALDatasetH)poDataset);
	//GDALClose((GDALDatasetH)poDataset1);
	return 0;
}

//GDAL数据转opencv的Mat格式
cv::Mat GDAL2Mat(GDALDataset *poDataset, MyStruct &St)  //const QString fileName
{
	//将GDAL的数据类型与Mat的数据类型对应起来
	auto MdataType = NULL;
	auto MdataTypes = NULL;
	if (St.iDataType == GDT_Byte)
	{
		MdataType = CV_MAKETYPE(CV_8U, 1);
		MdataTypes = CV_MAKETYPE(CV_8U, St.nbands);
	}
	if (St.iDataType == GDT_UInt16)
	{
		MdataType = CV_MAKETYPE(CV_16U, 1);
		MdataTypes = CV_MAKETYPE(CV_16U, St.nbands);
	}
	vector<Mat> imgMat;// 每个波段
	float *pafScan = new float[St.Xsize*St.Ysize];   // 存储数据

	for (int i = 0; i < St.nbands; i++)
	{
		GDALRasterBand *pBand = poDataset->GetRasterBand(i + 1);
		//pafScan = new float[tmpCols*tmpRows];
		pBand->RasterIO(GF_Read, 0, 0, St.Xsize, St.Ysize, pafScan,
			St.Xsize, St.Ysize, St.iDataType, 0, 0); //GDT_Float32
		cv::Mat tmpMat = cv::Mat(St.Ysize, St.Xsize, MdataType, pafScan);//CV_32FC1
		imgMat.push_back(tmpMat.clone());
	}
	delete[]pafScan;
	pafScan = NULL;

	cv::Mat img;
	img.create(St.Ysize, St.Xsize, MdataTypes); //CV_32FC(tmpBandSize)
	//cv::merge(imgMat.toStdVector(), img);
	cv::merge(imgMat, img);
	//释放内存
	imgMat.clear();
	//GDALClose((GDALDatasetH)poDataset);
	return img;
}

Mat imgchangeFormat(MyStruct St,Mat img)
{
	std::vector<cv::Mat> imgMat(St.nbands);
	std::vector<cv::Mat> tempMat(St.nbands);
	cv::split(img, imgMat);
	tempMat.at(0) = (imgMat.at(2));//BGR-->RGB
	tempMat.at(1) = (imgMat.at(1));
	tempMat.at(2) = (imgMat.at(0));
	Mat img_1;
	cv::merge(tempMat, img_1);
	imgMat.clear();
	tempMat.clear();
	return img_1;
}


