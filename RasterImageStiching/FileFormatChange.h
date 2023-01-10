#pragma once
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "CommonHeader.h"
#include <string>
using namespace std;




class FileFormatChange
{
public:
	cv::Mat GDALGetFileFromImg(const string  src_path, MyStruct &S);
	cv::Mat GDAL2Mat(GDALDataset *poDataset, MyStruct &St);
	cv::Mat imgColorCodeChange(MyStruct St, cv::Mat img);
};

