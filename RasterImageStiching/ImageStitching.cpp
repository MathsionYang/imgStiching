#include "ImageStitching.h"
#include <Windows.h>
#include <string.h>


#pragma region 匹配阶段

//
//void ImageStitcher::loadImages()
//{
//	img1 = imread(left_img_path);
//	if (img1.data == NULL) {
//		char msg[200] = "Cannot find image ";
//		strcat(msg, left_img_path.c_str());
//		throw runtime_error(msg);
//	}
//
//	img2 = imread(right_img_path);
//	if (img2.data == NULL) {
//		char msg[200] = "Cannot find image ";
//		strcat(msg, right_img_path.c_str());
//		throw runtime_error(msg);
//	}
//}
//
//
//	img1 = imread(img1_path);
//	if (img1.data == NULL) {
//		char msg[200] = "Cannot find image ";
//		strcat(msg, img1_path.c_str());
//		throw runtime_error(msg);
//	}
//
//
//	img2 = imread(img2_path);
//	if (img2.data == NULL) {
//		char msg[200] = "Cannot find image ";
//		strcat(msg, img2_path.c_str());
//		throw runtime_error(msg);
//	}
//
//	drawMatches(img1, raw_kpts1, img2, raw_kpts2, vector<DMatch>(), img_origin);
//
//	//img1 = imread(img1_path);
//	//if (img1.data == NULL) return false;
//
//	//img2 = imread(img2_path);
//	//if (img2.data == NULL) return false;
//
//	//return true;
//}
//
//
//string ImageStitcher::getSavePath(const char * raw_path)
//{
//	cout << "raw_path: " << raw_path << endl;
//
//	string save_path;
//	if (strcmp(raw_path, "") == 0) {
//		char curr_path[MAX_PATH];
//		GetCurrentDirectoryA(MAX_PATH, curr_path);
//		cout << "curr_path: " << curr_path << endl;
//		save_path = string(curr_path);
//	}
//	else {
//		save_path = string(raw_path);
//	}
//	
//	switch (feat) {
//	case Feature::SIFT:
//		save_path += "/SIFT";
//		break;
//	case Feature::SURF:
//		save_path += "/SURF";
//		break;
//	case Feature::ORB:
//		save_path += "/ORB";
//		break;
//	default:
//		break;
//	}
//	return save_path;
//}
//
//
//vector<string> ImageStitcher::saveImages(const char* save_path)
//{
//	vector<string> res;
//
//	string save_dir = getSavePath(save_path);
//	cout << "save_path: " << save_dir << endl;
//	if (CreateDirectoryA(save_dir.c_str(), NULL) ||
//		GetLastError() == ERROR_ALREADY_EXISTS) {
//		try {
//			res.push_back(save_dir + string("/origin.jpg"));
//			imwrite(res.back(), img_origin);
//			res.push_back(save_dir + string("/detect_keypoints.jpg"));
//			imwrite(res.back(), img_detect_kpts);
//			res.push_back(save_dir + string("/remove_outlier.jpg"));
//			imwrite(res.back(), img_rm_outlier);
//			res.push_back(save_dir + string("/ransac_result.jpg"));
//			imwrite(res.back(), img_after_ransac);
//			//res.push_back(save_dir + string("/stitching_result.jpg"));
//			//imwrite(res.back(), img_stitched);
//			res.push_back(save_dir + string("/postprocess.jpg"));
//			imwrite(res.back(), img_postprocess);
//			return res;
//		}
//		catch (runtime_error& ex) {
//			cerr << "cannot save image: " << ex.what() << endl;
//			throw exception("Cannot save image.");
//		}
//	}
//	else {
//		throw exception("Save path error.");
//	}
//	
//	return vector<string>();
//}
//
//void ImageStitcher::setLeftImagePath(const string path)
//{
//	left_img_path = path;
//}
//
//void ImageStitcher::setRightImagePath(const string path)
//{
//	right_img_path = path;
//}
//
//void ImageStitcher::setFeatureExtractor(Feature feature)
//{
//	this->feat = feature;
//}
//
//
//void ImageStitcher::extractFeatures(Mat& descriptors1, Mat& descriptors2)
//{
//	Ptr<Feature2D> f2d;
//
//	switch (feat)
//	{
//	case Feature::SIFT:
//		f2d = SIFT::create();
//		break;
//	case Feature::SURF:
//		f2d = SURF::create();
//		break;
//	case Feature::ORB:
//		f2d = ORB::create();
//		break;
//	case Feature::FERNS:
//		//f2d = FERNS::create();
//		break;
//	default:
//		break;
//	}
//
//	//Detect the keypoints
//	double t0 = getTickCount();//当前滴答数
//	//vector<KeyPoint> keypoints_1, keypoints_2;
//	f2d->detect(img1, raw_kpts1);
//	f2d->detect(img2, raw_kpts2);
//	//cout << "The keypoints number of img1 is:" << keypoints1.size() << endl;
//	//cout << "The keypoints number of img2 is:" << keypoints2.size() << endl;
//
//	//Calculate descriptors (feature vectors)
//	f2d->compute(img1, raw_kpts1, descriptors1);
//	f2d->compute(img2, raw_kpts2, descriptors2);
//	double freq = getTickFrequency();
//	double tt = ((double)getTickCount() - t0) / freq;
//	cout << "Extract Features Time:" << tt << "ms" << endl;
//
//	drawMatches(img1, raw_kpts1, img2, raw_kpts2, vector<DMatch>(), img_detect_kpts);
//	//imshow("detect keypoints", img_detect_kpts);
//}
//
//
//void ImageStitcher::matchKeyPoints(Mat& descriptors1, Mat& descriptors2)
//{
//	double t0 = getTickCount();
//	//BFMatcher matcher;
//	vector<DMatch> bf_matches;
//	//FlannBasedMatcher matcher;
//	if (feat == Feature::ORB) {
//		BFMatcher().match(descriptors1, descriptors2, bf_matches);
//	}
//	else {
//		FlannBasedMatcher().match(descriptors1, descriptors2, bf_matches);
//	}
//	
//	cout << "The number of match:" << bf_matches.size() << endl;
//
//	//绘制匹配出的关键点
//	//Mat img_matches;
//	//drawMatches(img1, raw_kpts1, img2, raw_kpts2, matches, img_matches);
//
//	//imshow("Match image",img_matches);
//	//计算匹配结果中距离最大和距离最小值
//	double min_dist = bf_matches[0].distance, max_dist = bf_matches[0].distance;
//	for (int m = 0; m < bf_matches.size(); m++)
//	{
//		if (bf_matches[m].distance < min_dist)
//		{
//			min_dist = bf_matches[m].distance;
//		}
//		if (bf_matches[m].distance > max_dist)
//		{
//			max_dist = bf_matches[m].distance;
//		}
//	}
//	cout << "min dist=" << min_dist << endl;
//	cout << "max dist=" << max_dist << endl;
//	//筛选出较好的匹配点
//	vector<DMatch> goodMatches;
//	double ratio = (feat == Feature::ORB) ? 0.8 : 0.4;
//	for (int m = 0; m < bf_matches.size(); m++)
//	{
//		if (bf_matches[m].distance < ratio * max_dist)
//		{
//			goodMatches.push_back(bf_matches[m]);
//		}
//	}
//	cout << "The number of good matches:" << goodMatches.size() << endl;
//
//	
//
//	matches = goodMatches;
//	int ptCount = goodMatches.size();
//	if (ptCount < 100)
//	{
//		throw runtime_error("Lack of matched points after RANSAC.");
//	}
//	//坐标转换为float类型
//		//size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
//	for (::size_t i = 0; i < matches.size(); i++)
//	{
//		ransac_kpts1.push_back(raw_kpts1[goodMatches[i].queryIdx]);
//		ransac_kpts2.push_back(raw_kpts2[goodMatches[i].trainIdx]);
//	}
//
//	drawMatches(img1, ransac_kpts1, img2, ransac_kpts2, vector<DMatch>(), img_rm_outlier);
//
//	double freq = getTickFrequency();
//	double tt = ((double)getTickCount() - t0) / freq;
//	cout << "Match KeyPoints Time:" << tt << "ms" << endl;
//}

#pragma endregion
void ImageStitcher::loadImages(const cv::Mat& img1_path, const cv::Mat& img2_path, MyStruct &imgSt,string resultpath)
{
	dst_path = resultpath;
	img1 = img1_path;
	img2 = img2_path;
	img_St.nbands = imgSt.nbands;
	img_St.iDataType = imgSt.iDataType;
	img_St.proj = imgSt.proj;
	img_St.tmpadfGeoTransform = imgSt.tmpadfGeoTransform;


}
void ImageStitcher::loadKeyPtandMatch(const std::vector<cv::KeyPoint>& keyp1, const std::vector<cv::KeyPoint>& keyp2, std::vector<cv::DMatch>& Fianllmatches)
{
	matches = Fianllmatches;
	ransac_kpts1 = keyp1;
	ransac_kpts2 = keyp2;
	imageTransform();
}

void ImageStitcher::imageTransform()
{
	double t0 = cv::getTickCount();

	vector <cv::Point2f> p01, p02;
	for (::size_t i = 0; i < matches.size(); i++)
	{
		p01.push_back(ransac_kpts1[matches[i].queryIdx].pt);
		p02.push_back(ransac_kpts2[matches[i].trainIdx].pt);
	}

	cv::Mat m_homography;
	vector<uchar> m;
	//m_homography = findHomography(p01, p02, CV_RANSAC);
	m_homography = findHomography(p02, p01, CV_RANSAC);
	
	//求基础矩阵 Fundamental,3*3的基础矩阵
	vector<uchar> RansacStatus;
	cv::Mat Fundamental = findFundamentalMat(p02, p01, RansacStatus, cv::FM_RANSAC);

	//重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
	vector <cv::KeyPoint> RR_KP1, RR_KP2;
	vector <cv::DMatch> RR_matches;
	int index = 0;
	for (::size_t i = 0; i < matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			RR_KP1.push_back(ransac_kpts1[i]);
			RR_KP2.push_back(ransac_kpts2[i]);
			matches[i].queryIdx = index;
			matches[i].trainIdx = index;
			RR_matches.push_back(matches[i]);
			index++;
		}
	}
	//Mat img_RR_matches;
	drawMatches(img1, RR_KP1, img2, RR_KP2, RR_matches, img_after_ransac);
	//imshow("After RANSAC", img_after_ransac);

	vector<cv::Point2f> imagePoints1, imagePoints2;

	for (int i = 0; i < RR_matches.size(); i++)
	{
		imagePoints2.push_back(RR_KP1[RR_matches[i].trainIdx].pt);
		imagePoints1.push_back(RR_KP2[RR_matches[i].queryIdx].pt);
	}

	cv::Mat home;
	//invert(m_homography, home);
	home = m_homography;
	//cout << "invert:" << home << endl;

	std::vector<cv::Point2f> obj_corners(4);//定义右图的四个角
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(img2.cols, 0);
	obj_corners[2] = cvPoint(img2.cols, img2.rows);
	obj_corners[3] = cvPoint(0, img2.rows);

	std::vector<cv::Point2f> scene_corners(4);//定义左图的四个角
	perspectiveTransform(obj_corners, scene_corners, home);//将右图四角投影至左图



	cv::Mat imageTransform1, imageTransform1_move;
	warpPerspective(img2, imageTransform1, home, cv::Size(MAX(scene_corners[1].x, scene_corners[2].x), img1.rows));
	cv::Mat t_mat = cv::Mat::zeros(2, 3, CV_32FC1);

	//创建拼接后的图,需提前计算图的大小
	int dst_width = imageTransform1.cols;  //取最右点的长度为拼接图的长度
	int dst_height = img1.rows;

	cv::Mat img_stitched1= cv::Mat(dst_height, dst_width, CV_8UC3);
	img_stitched1.setTo(0);
	imageTransform1.copyTo(img_stitched1(cv::Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	img1.copyTo(img_stitched1(cv::Rect(0, 0, img1.cols, img1.rows)));
	//imwrite("stitched_image.tif", img_stitched);
	img_postprocess = cv::Mat(img_stitched1);
	ImageStitcher::optimizeSeam(img1, imageTransform1, img_postprocess, MIN(scene_corners[0].x, scene_corners[3].x), img1.cols);
	//imwrite("result\\stitched_image_after_optimize.tif", img_stitched1);

		//判断是否加载成功
	if (!img_stitched1.data) //或者image.empty()
	{
		cout << "  Faild" << endl;
	}
	

	//定义输出数据
	GDALDataset *poDataset2;   //GDAL数据集
	GDALDriver *poDriver;      //驱动，用于创建新的文件
	//poDriver = GetGDALDriverManager()->GetDriverByName("ENVI");
	poDriver = GetGDALDriverManager()->GetDriverByName("GTiff");

	if (poDriver == NULL) {
		return;
	}
	img_St.Xsize = dst_width;
	img_St.Ysize = dst_height;
	img_St.iDataType = GDT_Byte;
	poDataset2 = poDriver->Create(dst_path.c_str(), dst_width, dst_height, img_St.nbands,
		img_St.iDataType, NULL);//GDT_Float32
	////重新设置地理参考和投影系
	poDataset2->SetGeoTransform(img_St.tmpadfGeoTransform);
	poDataset2->SetProjection(img_St.proj);
	Mat2File(img_stitched1, img_St, poDataset2);

	GDALClose((GDALDatasetH)poDataset2);


}

void ImageStitcher::optimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst, int left_cover, int right_cover)
{
	int processWidth = right_cover-left_cover; 
	int start = left_cover+ processWidth/3*2;  

	int img1_row = img1.rows;
	int trans_row = trans.rows;
	int dst_row = dst.rows;
	int rows = MIN(MIN(img1_row, trans_row), dst_row);
	int cols = img1.cols; 
	double alpha = 1;
	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1;
			}
			else
			{
				alpha =(processWidth - (j - start)) / processWidth;
			}

			d[j * 3] =p[j * 3] * alpha + t[j * 3] * (1 - alpha);
			d[j * 3 + 1] =p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
			d[j * 3 + 2] =p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

		}
	}
}

void ImageStitcher::colorBalance()
{
	vector<cv::Mat> imageRGB;
	split(img_stitched, imageRGB);
	double R, G, B;
	B = cv::mean(imageRGB[0])[0];
	G = cv::mean(imageRGB[1])[0];
	R = cv::mean(imageRGB[2])[0];

	double KR, KG, KB;
	KB = (R + G + B) / (3 * B);
	KG = (R + G + B) / (3 * G);
	KR = (R + G + B) / (3 * R);

	imageRGB[0] = imageRGB[0] * KB;
	imageRGB[1] = imageRGB[1] * KG;
	imageRGB[2] = imageRGB[2] * KR;

	merge(imageRGB, img_stitched);
	//imshow("color balance", img_stitched);
}

cv::Mat matrixWiseMulti(cv::Mat &m1, cv::Mat &m2) {
	cv::Mat dst = m1.mul(m2);
	return dst;
}

bool ImageStitcher::Mat2File(cv::Mat img, MyStruct &St, GDALDataset *poDataset)//const QString fileName
{
	if (img.empty())    //    判断是否为空
		return 0;

	const int nBandCount = St.nbands;
	const int nImgSizeX = St.Xsize;
	const int nImgSizeY = St.Ysize;

	//    将通道分开
	//  imgMat每个通道数据连续
	std::vector<cv::Mat> imgMat(nBandCount);
	cv::split(img, imgMat);

	//  分波段写入文件
	GDALAllRegister();


	//  循环写入文件
	GDALRasterBand *pBand = NULL;
	float *ppafScan = new float[nImgSizeX*nImgSizeY];
	cv::Mat tmpMat;// = cv::Mat(nImgSizeY,nImgSizeX,CV_32FC1);

	for (int i = 1; i <= nBandCount; i++)
	{
		pBand = poDataset->GetRasterBand(i);
		tmpMat = imgMat.at(i - 1);
		if (tmpMat.isContinuous())
		{
			if (St.iDataType == GDT_Byte)
				memmove(ppafScan, (void*)tmpMat.ptr(0), sizeof(unsigned char)*nImgSizeX*nImgSizeY);//GDT_Byte
			if (St.iDataType == GDT_UInt16)
			{
				memmove(ppafScan, (void*)tmpMat.ptr(0), sizeof(unsigned short)*nImgSizeX*nImgSizeY);//GDT_Uint16

			}
		}
		else
			return false;

		CPLErr err = pBand->RasterIO(GF_Write, 0, 0, nImgSizeX, nImgSizeY, ppafScan,
			nImgSizeX, nImgSizeY, St.iDataType, 0, 0);
	}
	delete[] ppafScan;
	ppafScan = NULL;
	imgMat.clear();
	//GDALClose(poDataset);
	return 1;
}
