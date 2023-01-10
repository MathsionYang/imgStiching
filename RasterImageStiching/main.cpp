#include <opencv2/opencv.hpp>  
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/stitching.hpp>

#include "FileFormatChange.h"
#include "FeatureMatch.h"
#include "ImageStitching.h"

#include "CELLTimestamp.hpp"
using namespace std;

vector<string>getFileName(string filename);
int main()
{
	
	
	
	
	
	
	
	CELLTimestamp _tTime;//计时器

	string  src_path = "img\\img01.tif";
	string  src_path1 = "img\\1.tif";
	MyStruct St1, St2;
	//第一步：GDAL将影像转换为Mat
	FileFormatChange ffc;
	//cv::IMREAD_LOAD_GDAL | cv::IMREAD_COLOR | cv::IMREAD_ANYDEPTH
	cv::Mat	img1 = cv::imread(src_path1,0);

	cv::Mat img2 = ffc.GDALGetFileFromImg(src_path1, St2);
	

	//第二步：特征点提取与匹配
	FeatureMatch fm;
	if (fm.LoadImg(img1, img2) == false)
	{
		return 0;
	}
	vector<cv::KeyPoint> keypts_1, keypts_2;
	vector<cv::DMatch>matches;

	fm.DetectorKeyPoint(keypts_1, keypts_2);
	if (fm.FeatMatch(matches) <= 0)
	{
		return 0;
	}
	cout <<"SIFT耗时:"<< _tTime.getElapsedTimeInMilliSec()<< endl;
	////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<SIFT-GPU>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	/*vector<cv::KeyPoint> kp_1, kp_2;
	vector<cv::DMatch>matches_gpu;
	fm.MatchBySIFTGPU(kp_1, kp_2, matches_gpu);*/
	cv::Mat matchImg;
	drawMatches(img1, keypts_1, img2, keypts_2, matches, matchImg);
	//第三步：拼接
	//设置输出文件名"result\\result_r.tif";

	string stichingPath ="result\\" + getFileName(src_path)[0] + "_" + getFileName(src_path1)[0] + ".tif";
	ImageStitcher imgstich;
	imgstich.loadImages(img1, img2, St1, stichingPath);
	imgstich.loadKeyPtandMatch(keypts_1, keypts_2, matches);

	cout << "SIFTGPU耗时:" << _tTime.getElapsedTimeInMilliSec() << endl;
	getchar();
	return 0;

}
vector<string>getFileName(string filename)
{

	vector<string>fileinfo;
	//1.获取不带路径的文件名
	string::size_type iPos = filename.find_last_of('\\') + 1;
	string filename1 = filename.substr(iPos, filename.length() - iPos);

	//2.获取不带后缀的文件名
	string name = filename1.substr(0, filename1.rfind("."));

	fileinfo.push_back(name);
	//3.获取后缀名
	string suffix_str = filename1.substr(filename1.find_last_of('.') + 1);
	fileinfo.push_back(suffix_str);
	return fileinfo;
}
